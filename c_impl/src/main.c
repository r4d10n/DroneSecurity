/**
 * @file main.c
 * @brief Main Drone-ID receiver application for PlutoSDR
 *
 * This is the complete receiver application integrating all modules:
 * - PlutoSDR interface (libiio)
 * - SpectrumCapture (packet detection)
 * - Packet processing (OFDM, ZC)
 * - QPSK demodulation
 * - Turbo decoding
 * - DroneID packet parsing
 * - Dual-core thread pool
 */

#include "common.h"
#include "pluto_iio.h"
#include "spectrum_capture.h"
#include "packet.h"
#include "qpsk.h"
#include "turbo.h"
#include "droneid_packet.h"
#include "threading.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>

/* Global state */
static volatile bool g_running = true;
static thread_pool_t g_thread_pool;
static droneid_stats_t g_stats;
static pthread_mutex_t g_stats_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Configuration */
typedef struct {
    const char* pluto_uri;
    uint64_t sample_rate;
    uint64_t bandwidth;
    int gain;
    bool use_agc;
    bool legacy;
    bool debug;
    int num_workers;
    packet_type_t packet_type;
    float capture_duration;
    const char* output_file;
} app_config_t;

/* Signal handler */
static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG_INFO("\nShutdown signal received...\n");
}

/* Print usage */
static void print_usage(const char* prog_name) {
    printf("Usage: %s [OPTIONS]\n\n", prog_name);
    printf("Drone-ID Receiver for PlutoSDR (DJI OcuSync 2.0)\n\n");
    printf("Options:\n");
    printf("  -h, --help              Show this help message\n");
    printf("  -u, --uri URI           PlutoSDR URI (default: ip:192.168.2.1)\n");
    printf("  -s, --sample-rate RATE  Sample rate in Hz (default: 50000000)\n");
    printf("  -b, --bandwidth BW      RF bandwidth in Hz (default: same as sample rate)\n");
    printf("  -g, --gain GAIN         RX gain in dB (-1 for AGC, default: AGC)\n");
    printf("  -l, --legacy            Support legacy drones (Mavic Pro, Mavic 2)\n");
    printf("  -d, --debug             Enable debug output\n");
    printf("  -w, --workers NUM       Number of worker threads (default: 2 for Zynq)\n");
    printf("  -p, --packet-type TYPE  Packet type: droneid, c2, beacon, video (default: droneid)\n");
    printf("  -t, --duration SECS     Capture duration per frequency (default: 1.3)\n");
    printf("  -o, --output FILE       Output file for decoded packets\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                                   # Run with default settings\n", prog_name);
    printf("  %s -g 40 -s 50000000                # Manual gain 40dB, 50 MHz\n", prog_name);
    printf("  %s -l -d                            # Legacy mode with debug\n", prog_name);
    printf("  %s --uri usb:1.2.5                  # Use specific USB device\n", prog_name);
    printf("\n");
}

/* Process captured samples */
static int process_capture(const complex_t* samples, int n_samples,
                           float sample_rate, const app_config_t* config) {
    int packets_decoded = 0;

    /* Initialize SpectrumCapture */
    spectrum_capture_t sc;
    int ret = spectrum_capture_init(&sc, samples, n_samples, sample_rate,
                                    config->packet_type, config->legacy, config->debug);
    if (ret != 0) {
        LOG_ERROR("SpectrumCapture init failed: %d\n", ret);
        return 0;
    }

    /* Detect packets */
    int num_packets = spectrum_capture_detect_packets(&sc);

    if (config->debug) {
        LOG_DEBUG("Found %d packet candidates\n", num_packets);
    }

    /* Process each detected packet */
    for (int pkt_idx = 0; pkt_idx < num_packets; pkt_idx++) {
        complex_buffer_t packet_buffer;

        /* Extract packet */
        ret = spectrum_capture_get_packet(&sc, pkt_idx, &packet_buffer);
        if (ret != 0) {
            continue;
        }

        /* Initialize packet processor */
        packet_t pkt;
        ret = packet_init(&pkt, packet_buffer.data, packet_buffer.size,
                         sample_rate, config->legacy, config->debug);
        if (ret != 0) {
            buffer_destroy(&packet_buffer);
            continue;
        }

        /* Detect ZC sequences */
        ret = packet_detect_zc(&pkt);
        if (ret != 0) {
            if (config->debug) {
                LOG_DEBUG("  Packet %d: ZC detection failed\n", pkt_idx);
            }
            packet_destroy(&pkt);
            buffer_destroy(&packet_buffer);
            continue;
        }

        /* Apply corrections */
        packet_apply_corrections(&pkt);

        /* Extract OFDM symbols */
        int num_symbols = packet_extract_symbols(&pkt);
        if (num_symbols < DATA_SYMBOLS) {
            if (config->debug) {
                LOG_DEBUG("  Packet %d: Insufficient symbols (%d)\n", pkt_idx, num_symbols);
            }
            packet_destroy(&pkt);
            buffer_destroy(&packet_buffer);
            continue;
        }

        /* Equalize */
        ret = packet_equalize(&pkt);
        if (ret != 0) {
            packet_destroy(&pkt);
            buffer_destroy(&packet_buffer);
            continue;
        }

        /* Get equalized symbols */
        complex_t symbols[DATA_SYMBOLS][NCARRIERS];
        packet_get_symbol_data(&pkt, symbols);

        /* Initialize QPSK decoder */
        qpsk_decoder_t qpsk;
        qpsk_decoder_init(&qpsk);

        /* Copy symbols to QPSK decoder */
        memcpy(qpsk.symbols, symbols, sizeof(symbols));

        /* Try all 4 phase rotations */
        bool decoded = false;
        for (int phase = 0; phase < 4; phase++) {
            /* Demodulate QPSK */
            qpsk_demodulate(&qpsk, phase);

            /* Decode payload */
            uint8_t payload_bytes[176];
            int payload_len;

            ret = qpsk_decode_payload(&qpsk, config->legacy, payload_bytes, &payload_len);
            if (ret != 0) continue;

            /* Parse DroneID packet */
            droneid_payload_t droneid;
            ret = droneid_parse_packet(payload_bytes, payload_len, &droneid);
            if (ret != 0) continue;

            /* Check CRC */
            if (droneid_check_crc(&droneid)) {
                /* Successfully decoded! */
                droneid_print_payload(&droneid);
                decoded = true;

                /* Update stats */
                pthread_mutex_lock(&g_stats_mutex);
                g_stats.frames_decoded++;
                g_stats.crc_ok++;
                pthread_mutex_unlock(&g_stats_mutex);

                packets_decoded++;

                /* Save to file if specified */
                if (config->output_file) {
                    FILE* fp = fopen(config->output_file, "ab");
                    if (fp) {
                        fwrite(payload_bytes, 1, payload_len, fp);
                        fclose(fp);
                    }
                }

                break;  /* Found correct phase */
            } else {
                /* CRC error */
                pthread_mutex_lock(&g_stats_mutex);
                g_stats.crc_error++;
                pthread_mutex_unlock(&g_stats_mutex);
            }
        }

        if (!decoded && config->debug) {
            LOG_DEBUG("  Packet %d: Decoding failed (all phases tried)\n", pkt_idx);
        }

        qpsk_decoder_destroy(&qpsk);
        packet_destroy(&pkt);
        buffer_destroy(&packet_buffer);
    }

    /* Update stats */
    pthread_mutex_lock(&g_stats_mutex);
    g_stats.frames_detected += num_packets;
    pthread_mutex_unlock(&g_stats_mutex);

    spectrum_capture_destroy(&sc);

    return packets_decoded;
}

/* Main receiver loop */
static int receiver_main(const app_config_t* config) {
    int ret;

    /* Initialize PlutoSDR */
    pluto_device_t pluto;
    ret = pluto_init(&pluto, config->pluto_uri);
    if (ret != 0) {
        LOG_ERROR("Failed to initialize PlutoSDR\n");
        return ret;
    }

    /* Get device info */
    char info[512];
    pluto_get_info(&pluto, info, sizeof(info));
    LOG_INFO("%s\n", info);

    /* Initialize thread pool for dual-core processing */
    ret = thread_pool_init(&g_thread_pool, config->num_workers);
    if (ret != 0) {
        LOG_ERROR("Failed to initialize thread pool\n");
        pluto_destroy(&pluto);
        return ret;
    }

    /* Calculate buffer size */
    size_t buffer_size = (size_t)(config->capture_duration * config->sample_rate);
    complex_t* sample_buffer = (complex_t*)malloc(buffer_size * sizeof(complex_t));
    if (!sample_buffer) {
        LOG_ERROR("Failed to allocate sample buffer\n");
        thread_pool_destroy(&g_thread_pool);
        pluto_destroy(&pluto);
        return DRONEID_ERROR_MEMORY;
    }

    /* Frequency hopping variables */
    int freq_idx = 0;
    int freq_lock_idx = -1;
    int no_detect_count = 0;

    LOG_INFO("\nStarting receiver...\n");
    LOG_INFO("  Sample rate: %.2f MHz\n", config->sample_rate / 1e6);
    LOG_INFO("  Bandwidth: %.2f MHz\n", config->bandwidth / 1e6);
    LOG_INFO("  Gain: %s%d dB\n", config->use_agc ? "AGC " : "", config->gain);
    LOG_INFO("  Workers: %d\n", config->num_workers);
    LOG_INFO("\nPress Ctrl+C to stop\n\n");

    /* Main loop */
    while (g_running) {
        /* Select frequency */
        uint64_t freq;
        if (freq_lock_idx >= 0) {
            /* Locked to specific frequency */
            freq = DRONEID_FREQUENCIES[freq_lock_idx];
        } else {
            /* Hop through frequencies */
            freq = DRONEID_FREQUENCIES[freq_idx];
            freq_idx = (freq_idx + 1) % DRONEID_NUM_FREQUENCIES;
        }

        /* Configure PlutoSDR */
        ret = pluto_configure(&pluto, config->sample_rate, freq,
                             config->bandwidth, config->use_agc ? -1 : config->gain);
        if (ret != 0) {
            LOG_ERROR("Failed to configure PlutoSDR\n");
            break;
        }

        LOG_INFO("Center freq: %.2f MHz\r", freq / 1e6);
        fflush(stdout);

        /* Start streaming */
        ret = pluto_start_streaming(&pluto, buffer_size);
        if (ret != 0) {
            LOG_ERROR("Failed to start streaming\n");
            break;
        }

        /* Receive samples */
        int samples_rcvd = pluto_receive(&pluto, sample_buffer, buffer_size);
        if (samples_rcvd <= 0) {
            LOG_WARN("No samples received\n");
            pluto_stop_streaming(&pluto);
            continue;
        }

        /* Stop streaming */
        pluto_stop_streaming(&pluto);

        /* Process samples */
        int packets_decoded = process_capture(sample_buffer, samples_rcvd,
                                             config->sample_rate, config);

        /* Frequency locking logic */
        if (packets_decoded > 0) {
            /* Found drone on this frequency, lock */
            freq_lock_idx = (freq_idx - 1 + DRONEID_NUM_FREQUENCIES) % DRONEID_NUM_FREQUENCIES;
            no_detect_count = 0;
            LOG_INFO("\nLocked to frequency: %.2f MHz (%d packets)\n",
                    freq / 1e6, packets_decoded);
        } else if (freq_lock_idx >= 0) {
            /* No detection on locked frequency */
            no_detect_count++;
            if (no_detect_count > 10) {
                /* Unlock after 10 empty captures */
                LOG_INFO("\nUnlocking frequency (no activity)\n");
                freq_lock_idx = -1;
                no_detect_count = 0;
            }
        }
    }

    /* Cleanup */
    LOG_INFO("\n\nShutting down...\n");

    free(sample_buffer);
    thread_pool_destroy(&g_thread_pool);
    pluto_destroy(&pluto);

    /* Print statistics */
    LOG_INFO("\n=== Statistics ===\n");
    LOG_INFO("  Frames detected: %u\n", g_stats.frames_detected);
    LOG_INFO("  Frames decoded: %u\n", g_stats.frames_decoded);
    LOG_INFO("  CRC OK: %u\n", g_stats.crc_ok);
    LOG_INFO("  CRC errors: %u\n", g_stats.crc_error);

    if (g_stats.frames_detected > 0) {
        float success_rate = 100.0f * g_stats.crc_ok / g_stats.frames_detected;
        LOG_INFO("  Success rate: %.1f%%\n", success_rate);
    }

    return DRONEID_SUCCESS;
}

/* Main entry point */
int main(int argc, char* argv[]) {
    /* Default configuration */
    app_config_t config = {
        .pluto_uri = "ip:192.168.2.1",
        .sample_rate = 50000000,  /* 50 MHz */
        .bandwidth = 50000000,
        .gain = 40,
        .use_agc = true,
        .legacy = false,
        .debug = false,
        .num_workers = 2,  /* Dual-core Zynq */
        .packet_type = PACKET_TYPE_DRONEID,
        .capture_duration = 1.3f,
        .output_file = NULL
    };

    /* Parse command line arguments */
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"uri", required_argument, 0, 'u'},
        {"sample-rate", required_argument, 0, 's'},
        {"bandwidth", required_argument, 0, 'b'},
        {"gain", required_argument, 0, 'g'},
        {"legacy", no_argument, 0, 'l'},
        {"debug", no_argument, 0, 'd'},
        {"workers", required_argument, 0, 'w'},
        {"packet-type", required_argument, 0, 'p'},
        {"duration", required_argument, 0, 't'},
        {"output", required_argument, 0, 'o'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "hu:s:b:g:ldw:p:t:o:", long_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                return 0;

            case 'u':
                config.pluto_uri = optarg;
                break;

            case 's':
                config.sample_rate = strtoull(optarg, NULL, 10);
                config.bandwidth = config.sample_rate;  /* Match bandwidth to sample rate */
                break;

            case 'b':
                config.bandwidth = strtoull(optarg, NULL, 10);
                break;

            case 'g':
                config.gain = atoi(optarg);
                config.use_agc = (config.gain < 0);
                break;

            case 'l':
                config.legacy = true;
                break;

            case 'd':
                config.debug = true;
                g_log_level = LOG_LEVEL_DEBUG;
                break;

            case 'w':
                config.num_workers = atoi(optarg);
                break;

            case 'p':
                if (strcmp(optarg, "c2") == 0) {
                    config.packet_type = PACKET_TYPE_C2;
                } else if (strcmp(optarg, "beacon") == 0) {
                    config.packet_type = PACKET_TYPE_BEACON;
                } else if (strcmp(optarg, "video") == 0) {
                    config.packet_type = PACKET_TYPE_VIDEO;
                } else {
                    config.packet_type = PACKET_TYPE_DRONEID;
                }
                break;

            case 't':
                config.capture_duration = atof(optarg);
                break;

            case 'o':
                config.output_file = optarg;
                break;

            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    /* Set up signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Print banner */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  DroneSecurity - Drone-ID Receiver for PlutoSDR             ║\n");
    printf("║  DJI OcuSync 2.0 Protocol Decoder                           ║\n");
    printf("║  Version %d.%d.%d (C Implementation with NEON)                 ║\n",
           DRONEID_VERSION_MAJOR, DRONEID_VERSION_MINOR, DRONEID_VERSION_PATCH);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* Run receiver */
    int ret = receiver_main(&config);

    return (ret == DRONEID_SUCCESS) ? 0 : 1;
}
