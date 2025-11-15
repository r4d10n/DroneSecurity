/**
 * @file spectrum_capture.c
 * @brief Spectrum capture and packet detection implementation
 */

#include "spectrum_capture.h"
#include "dsp_neon.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

int spectrum_capture_init(spectrum_capture_t* sc,
                          const complex_t* raw_data,
                          int n_samples,
                          float sample_rate,
                          packet_type_t packet_type,
                          bool legacy,
                          bool debug) {
    if (!sc || !raw_data || n_samples == 0) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    memset(sc, 0, sizeof(spectrum_capture_t));

    /* Copy parameters */
    sc->raw_data = (complex_t*)malloc(n_samples * sizeof(complex_t));
    if (!sc->raw_data) return DRONEID_ERROR_MEMORY;

    memcpy(sc->raw_data, raw_data, n_samples * sizeof(complex_t));
    sc->n_samples = n_samples;
    sc->sample_rate = sample_rate;
    sc->packet_type = packet_type;
    sc->legacy = legacy;
    sc->debug = debug;
    sc->num_packets = 0;

    /* Allocate PSD arrays */
    sc->psd_len = NFFT_WELCH;
    sc->psd = (float*)malloc(sc->psd_len * sizeof(float));
    sc->freq = (float*)malloc(sc->psd_len * sizeof(float));

    if (!sc->psd || !sc->freq) {
        spectrum_capture_destroy(sc);
        return DRONEID_ERROR_MEMORY;
    }

    /* Compute PSD */
    int ret = dsp_welch_psd(sc->raw_data, sc->n_samples, sc->sample_rate,
                            NFFT_WELCH, sc->psd, sc->freq);
    if (ret != 0) {
        spectrum_capture_destroy(sc);
        return ret;
    }

    /* Compute mean power */
    sc->mean_power = neon_mean(sc->psd, sc->psd_len);

    if (debug) {
        LOG_DEBUG("SpectrumCapture initialized: %d samples @ %.2f MHz\n",
                  n_samples, sample_rate / 1e6);
        LOG_DEBUG("  Mean power: %.2f\n", sc->mean_power);
    }

    return DRONEID_SUCCESS;
}

void spectrum_capture_destroy(spectrum_capture_t* sc) {
    if (!sc) return;

    if (sc->raw_data) {
        free(sc->raw_data);
        sc->raw_data = NULL;
    }

    if (sc->psd) {
        free(sc->psd);
        sc->psd = NULL;
    }

    if (sc->freq) {
        free(sc->freq);
        sc->freq = NULL;
    }

    /* Destroy packet buffers */
    for (int i = 0; i < sc->num_packets; i++) {
        buffer_destroy(&sc->packet_buffers[i]);
    }

    sc->num_packets = 0;
}

bool spectrum_capture_estimate_offset(spectrum_capture_t* sc,
                                      float* offset,
                                      float* bandwidth) {
    if (!sc || !offset || !bandwidth) return false;

    /* Add fake DC carrier to avoid detection of DC spike */
    int dc_idx = sc->psd_len / 2;
    for (int i = dc_idx - 10; i <= dc_idx + 10; i++) {
        if (i >= 0 && i < sc->psd_len) {
            sc->psd[i] = 1.1f * sc->mean_power;
        }
    }

    /* Find consecutive bins above mean power */
    bool in_band = false;
    int band_start = 0;
    int band_end = 0;
    bool found = false;

    for (int i = 0; i < sc->psd_len; i++) {
        if (sc->psd[i] > sc->mean_power) {
            if (!in_band) {
                band_start = i;
                in_band = true;
            }
            band_end = i;
        } else {
            if (in_band) {
                /* Band ended, check if it matches our criteria */
                float freq_start = sc->freq[band_start];
                float freq_end = sc->freq[band_end];
                float bw = fabsf(freq_end - freq_start);

                if (sc->debug) {
                    LOG_DEBUG("Candidate band: %.2f - %.2f MHz, BW: %.2f MHz\n",
                              freq_start / 1e6, freq_end / 1e6, bw / 1e6);
                }

                /* Check bandwidth criteria based on packet type */
                bool match = false;
                switch (sc->packet_type) {
                    case PACKET_TYPE_DRONEID:
                    case PACKET_TYPE_BEACON:
                    case PACKET_TYPE_PAIRING:
                        if (bw > 8e6 && bw < 11e6) match = true;
                        break;

                    case PACKET_TYPE_C2:
                        if (bw > 1.2e6 && bw < 1.95e6) match = true;
                        break;

                    case PACKET_TYPE_VIDEO:
                        if (bw > 18e6 && bw < 22e6) match = true;
                        break;
                }

                if (match) {
                    *offset = (freq_start + freq_end) / 2.0f;
                    *bandwidth = bw;
                    found = true;

                    if (sc->debug) {
                        LOG_DEBUG("  MATCH! Offset: %.2f kHz, BW: %.2f MHz\n",
                                  *offset / 1e3, *bandwidth / 1e6);
                    }
                    break;
                }

                in_band = false;
            }
        }
    }

    return found;
}

int spectrum_capture_detect_packets(spectrum_capture_t* sc) {
    if (!sc) return DRONEID_ERROR_INVALID_ARG;

    /* Time-domain packet detection using power threshold */
    float* power = (float*)malloc(sc->n_samples * sizeof(float));
    if (!power) return DRONEID_ERROR_MEMORY;

    /* Compute instantaneous power */
    neon_abs_squared(sc->raw_data, power, sc->n_samples);

    /* Simple threshold-based detection */
    float threshold = sc->mean_power * 2.0f; /* 3 dB above mean */

    bool in_packet = false;
    int packet_start = 0;
    sc->num_packets = 0;

    for (int i = 0; i < sc->n_samples; i++) {
        if (power[i] > threshold) {
            if (!in_packet) {
                packet_start = i;
                in_packet = true;
            }
        } else {
            if (in_packet) {
                int packet_end = i;
                int packet_len = packet_end - packet_start;

                /* Minimum packet length check (0.5 ms) */
                int min_len = (int)(0.0005f * sc->sample_rate);

                if (packet_len > min_len && sc->num_packets < MAX_PACKETS_PER_CAPTURE) {
                    /* Store packet info */
                    sc->packets[sc->num_packets].start_idx = packet_start;
                    sc->packets[sc->num_packets].end_idx = packet_end;
                    sc->packets[sc->num_packets].length = packet_len;

                    /* Estimate CFO for this packet */
                    float cfo, bw;
                    if (spectrum_capture_estimate_offset(sc, &cfo, &bw)) {
                        sc->packets[sc->num_packets].cfo = cfo;
                        sc->packets[sc->num_packets].bandwidth = bw;
                    } else {
                        sc->packets[sc->num_packets].cfo = 0.0f;
                        sc->packets[sc->num_packets].bandwidth = 0.0f;
                    }

                    /* Estimate SNR */
                    float signal_power = 0.0f;
                    for (int j = packet_start; j < packet_end; j++) {
                        signal_power += power[j];
                    }
                    signal_power /= packet_len;
                    sc->packets[sc->num_packets].snr = 10.0f * log10f(signal_power / sc->mean_power);

                    if (sc->debug) {
                        LOG_DEBUG("Packet #%d: start=%d, end=%d, len=%d, CFO=%.2f kHz, SNR=%.1f dB\n",
                                  sc->num_packets, packet_start, packet_end, packet_len,
                                  cfo / 1e3, sc->packets[sc->num_packets].snr);
                    }

                    sc->num_packets++;
                }

                in_packet = false;
            }
        }
    }

    free(power);

    LOG_INFO("Detected %d packet candidates\n", sc->num_packets);

    return sc->num_packets;
}

int spectrum_capture_get_packet(spectrum_capture_t* sc,
                                int packet_num,
                                complex_buffer_t* out_buffer) {
    if (!sc || !out_buffer || packet_num < 0 || packet_num >= sc->num_packets) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    packet_info_t* pkt = &sc->packets[packet_num];

    /* Extract packet samples */
    int packet_len = pkt->length;
    complex_t* packet_samples = &sc->raw_data[pkt->start_idx];

    /* Apply frequency shift to correct coarse CFO */
    complex_t* corrected = (complex_t*)malloc(packet_len * sizeof(complex_t));
    if (!corrected) return DRONEID_ERROR_MEMORY;

    if (fabsf(pkt->cfo) > 1000.0f) {  /* Only shift if CFO > 1 kHz */
        dsp_freq_shift(packet_samples, corrected, packet_len,
                      -pkt->cfo, sc->sample_rate);
    } else {
        memcpy(corrected, packet_samples, packet_len * sizeof(complex_t));
    }

    /* Resample to standard rate if needed (target: 50 MHz equivalent) */
    /* For now, just copy */
    int ret = buffer_init(out_buffer, packet_len);
    if (ret != 0) {
        free(corrected);
        return ret;
    }

    memcpy(out_buffer->data, corrected, packet_len * sizeof(complex_t));
    out_buffer->size = packet_len;

    free(corrected);

    return DRONEID_SUCCESS;
}
