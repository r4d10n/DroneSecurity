/**
 * @file pluto_iio.c
 * @brief PlutoSDR interface using libiio
 */

#include "pluto_iio.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* DJI OcuSync frequencies in Hz */
const uint64_t DRONEID_FREQUENCIES[] = {
    2414500000ULL, 2429500000ULL, 2434500000ULL,
    2444500000ULL, 2459500000ULL, 2474500000ULL,
    5721500000ULL, 5731500000ULL, 5741500000ULL,
    5756500000ULL, 5761500000ULL, 5771500000ULL,
    5786500000ULL, 5801500000ULL, 5816500000ULL,
    5831500000ULL
};

const int DRONEID_NUM_FREQUENCIES = sizeof(DRONEID_FREQUENCIES) / sizeof(DRONEID_FREQUENCIES[0]);

int pluto_init(pluto_device_t* dev, const char* uri) {
    if (!dev) return DRONEID_ERROR_INVALID_ARG;

    memset(dev, 0, sizeof(pluto_device_t));

#ifdef HAVE_LIBIIO
    /* Create IIO context */
    if (uri) {
        dev->ctx = iio_create_context_from_uri(uri);
    } else {
        dev->ctx = iio_create_default_context();
    }

    if (!dev->ctx) {
        LOG_ERROR("Failed to create IIO context\n");
        return DRONEID_ERROR_IO;
    }

    /* Get PHY device (ad9361-phy) */
    dev->phy = iio_context_find_device(dev->ctx, "ad9361-phy");
    if (!dev->phy) {
        LOG_ERROR("Failed to find ad9361-phy device\n");
        iio_context_destroy(dev->ctx);
        return DRONEID_ERROR_IO;
    }

    /* Get RX device (cf-ad9361-lpc) */
    dev->dev = iio_context_find_device(dev->ctx, "cf-ad9361-lpc");
    if (!dev->dev) {
        LOG_ERROR("Failed to find cf-ad9361-lpc device\n");
        iio_context_destroy(dev->ctx);
        return DRONEID_ERROR_IO;
    }

    /* Get RX channels */
    dev->rx_i = iio_device_find_channel(dev->dev, "voltage0", false);
    dev->rx_q = iio_device_find_channel(dev->dev, "voltage1", false);

    if (!dev->rx_i || !dev->rx_q) {
        LOG_ERROR("Failed to find RX channels\n");
        iio_context_destroy(dev->ctx);
        return DRONEID_ERROR_IO;
    }

    /* Enable channels */
    iio_channel_enable(dev->rx_i);
    iio_channel_enable(dev->rx_q);

    LOG_INFO("PlutoSDR initialized successfully\n");

    return DRONEID_SUCCESS;
#else
    LOG_ERROR("libiio not available, PlutoSDR support disabled\n");
    return DRONEID_ERROR_IO;
#endif
}

int pluto_configure(pluto_device_t* dev,
                    uint64_t sample_rate,
                    uint64_t center_freq,
                    uint64_t bandwidth,
                    int gain) {
    if (!dev) return DRONEID_ERROR_INVALID_ARG;

#ifdef HAVE_LIBIIO
    if (!dev->phy) return DRONEID_ERROR_IO;

    int ret;

    /* Set sample rate */
    ret = iio_channel_attr_write_longlong(
        iio_device_find_channel(dev->phy, "voltage0", false),
        "sampling_frequency", sample_rate);
    if (ret < 0) {
        LOG_ERROR("Failed to set sample rate: %d\n", ret);
        return DRONEID_ERROR_IO;
    }
    dev->sample_rate = sample_rate;

    /* Set RF bandwidth */
    ret = iio_channel_attr_write_longlong(
        iio_device_find_channel(dev->phy, "voltage0", false),
        "rf_bandwidth", bandwidth);
    if (ret < 0) {
        LOG_ERROR("Failed to set RF bandwidth: %d\n", ret);
        return DRONEID_ERROR_IO;
    }
    dev->bandwidth = bandwidth;

    /* Set center frequency */
    ret = pluto_set_frequency(dev, center_freq);
    if (ret != 0) return ret;

    /* Set gain */
    struct iio_channel* rx_chan = iio_device_find_channel(dev->phy, "voltage0", false);

    if (gain < 0) {
        /* Use AGC */
        ret = iio_channel_attr_write(rx_chan, "gain_control_mode", "slow_attack");
        if (ret < 0) {
            LOG_WARN("Failed to enable AGC, using manual gain\n");
            gain = 40;  /* Default manual gain */
        } else {
            dev->use_agc = true;
            LOG_INFO("AGC enabled\n");
        }
    }

    if (gain >= 0) {
        /* Manual gain */
        ret = iio_channel_attr_write(rx_chan, "gain_control_mode", "manual");
        if (ret < 0) {
            LOG_ERROR("Failed to set manual gain mode: %d\n", ret);
            return DRONEID_ERROR_IO;
        }

        ret = iio_channel_attr_write_longlong(rx_chan, "hardwaregain", gain);
        if (ret < 0) {
            LOG_ERROR("Failed to set gain: %d\n", ret);
            return DRONEID_ERROR_IO;
        }

        dev->gain = gain;
        dev->use_agc = false;
        LOG_INFO("Manual gain: %d dB\n", gain);
    }

    LOG_INFO("PlutoSDR configured:\n");
    LOG_INFO("  Sample rate: %lu Hz (%.2f MHz)\n", sample_rate, sample_rate / 1e6);
    LOG_INFO("  Bandwidth: %lu Hz (%.2f MHz)\n", bandwidth, bandwidth / 1e6);
    LOG_INFO("  Center freq: %lu Hz (%.2f MHz)\n", center_freq, center_freq / 1e6);

    return DRONEID_SUCCESS;
#else
    return DRONEID_ERROR_IO;
#endif
}

int pluto_set_frequency(pluto_device_t* dev, uint64_t freq) {
    if (!dev) return DRONEID_ERROR_INVALID_ARG;

#ifdef HAVE_LIBIIO
    if (!dev->phy) return DRONEID_ERROR_IO;

    struct iio_channel* rx_lo = iio_device_find_channel(dev->phy, "altvoltage0", true);
    if (!rx_lo) {
        LOG_ERROR("Failed to find LO channel\n");
        return DRONEID_ERROR_IO;
    }

    int ret = iio_channel_attr_write_longlong(rx_lo, "frequency", freq);
    if (ret < 0) {
        LOG_ERROR("Failed to set frequency: %d\n", ret);
        return DRONEID_ERROR_IO;
    }

    dev->center_freq = freq;

    return DRONEID_SUCCESS;
#else
    return DRONEID_ERROR_IO;
#endif
}

int pluto_start_streaming(pluto_device_t* dev, size_t buffer_size) {
    if (!dev) return DRONEID_ERROR_INVALID_ARG;

#ifdef HAVE_LIBIIO
    if (!dev->dev || dev->is_streaming) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    /* Create buffer */
    dev->rxbuf = iio_device_create_buffer(dev->dev, buffer_size, false);
    if (!dev->rxbuf) {
        LOG_ERROR("Failed to create RX buffer\n");
        return DRONEID_ERROR_IO;
    }

    dev->buffer_size = buffer_size;
    dev->is_streaming = true;

    LOG_INFO("Started streaming: buffer size = %zu samples\n", buffer_size);

    return DRONEID_SUCCESS;
#else
    return DRONEID_ERROR_IO;
#endif
}

void pluto_stop_streaming(pluto_device_t* dev) {
    if (!dev || !dev->is_streaming) return;

#ifdef HAVE_LIBIIO
    if (dev->rxbuf) {
        iio_buffer_destroy(dev->rxbuf);
        dev->rxbuf = NULL;
    }

    dev->is_streaming = false;

    LOG_INFO("Stopped streaming\n");
#endif
}

int pluto_receive(pluto_device_t* dev, complex_t* samples, size_t num_samples) {
    if (!dev || !samples || !dev->is_streaming) {
        return DRONEID_ERROR_INVALID_ARG;
    }

#ifdef HAVE_LIBIIO
    /* Refill buffer */
    ssize_t nbytes = iio_buffer_refill(dev->rxbuf);
    if (nbytes < 0) {
        LOG_ERROR("Buffer refill failed: %ld\n", nbytes);
        return DRONEID_ERROR_IO;
    }

    /* Get buffer pointers */
    void* p_dat = iio_buffer_first(dev->rxbuf, dev->rx_i);
    if (!p_dat) {
        LOG_ERROR("Failed to get buffer data\n");
        return DRONEID_ERROR_IO;
    }

    /* Convert from 12-bit IQ to complex float */
    int16_t* samples_i16 = (int16_t*)p_dat;
    size_t samples_available = nbytes / 4;  /* 2 bytes I + 2 bytes Q */

    size_t to_copy = (num_samples < samples_available) ? num_samples : samples_available;

    for (size_t i = 0; i < to_copy; i++) {
        float i_val = (float)samples_i16[i * 2] / 2048.0f;
        float q_val = (float)samples_i16[i * 2 + 1] / 2048.0f;
        samples[i] = i_val + I * q_val;
    }

    return (int)to_copy;
#else
    return DRONEID_ERROR_IO;
#endif
}

void pluto_destroy(pluto_device_t* dev) {
    if (!dev) return;

#ifdef HAVE_LIBIIO
    pluto_stop_streaming(dev);

    if (dev->ctx) {
        iio_context_destroy(dev->ctx);
        dev->ctx = NULL;
    }

    LOG_INFO("PlutoSDR destroyed\n");
#endif

    memset(dev, 0, sizeof(pluto_device_t));
}

void pluto_get_info(pluto_device_t* dev, char* info, size_t max_len) {
    if (!dev || !info || max_len == 0) return;

#ifdef HAVE_LIBIIO
    if (dev->ctx) {
        const char* name = iio_context_get_name(dev->ctx);
        const char* desc = iio_context_get_description(dev->ctx);

        snprintf(info, max_len,
                "PlutoSDR Info:\n"
                "  Context: %s\n"
                "  Description: %s\n"
                "  Sample rate: %lu Hz\n"
                "  Center freq: %lu Hz\n"
                "  Bandwidth: %lu Hz\n"
                "  Gain: %s%d dB\n",
                name ? name : "unknown",
                desc ? desc : "unknown",
                (unsigned long)dev->sample_rate,
                (unsigned long)dev->center_freq,
                (unsigned long)dev->bandwidth,
                dev->use_agc ? "AGC " : "",
                dev->gain);
    } else {
        strncpy(info, "PlutoSDR not initialized\n", max_len);
    }
#else
    strncpy(info, "libiio not available\n", max_len);
#endif
}
