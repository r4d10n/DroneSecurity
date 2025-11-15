/**
 * @file packet.c
 * @brief OFDM packet processing implementation
 */

#include "packet.h"
#include "dsp_neon.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

int packet_init(packet_t* pkt,
                const complex_t* packet_data,
                int packet_len,
                float sample_rate,
                bool legacy,
                bool debug) {
    if (!pkt || !packet_data || packet_len == 0) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    memset(pkt, 0, sizeof(packet_t));

    /* Copy packet data */
    pkt->packet_data = (complex_t*)malloc(packet_len * sizeof(complex_t));
    if (!pkt->packet_data) return DRONEID_ERROR_MEMORY;

    memcpy(pkt->packet_data, packet_data, packet_len * sizeof(complex_t));
    pkt->packet_len = packet_len;
    pkt->sample_rate = sample_rate;
    pkt->legacy = legacy;
    pkt->debug = debug;

    /* Initialize FFT */
    pkt->fft_cfg = fft_init(NFFT);
    if (!pkt->fft_cfg) {
        free(pkt->packet_data);
        return DRONEID_ERROR_MEMORY;
    }

    /* Generate ZC sequences */
    dsp_generate_zc_sequence(pkt->zc_seq_600, ZC_ROOT_1, ZC_LEN);
    dsp_generate_zc_sequence(pkt->zc_seq_147, ZC_ROOT_2, ZC_LEN);

    if (debug) {
        LOG_DEBUG("Packet initialized: %d samples @ %.2f MHz\n",
                  packet_len, sample_rate / 1e6);
    }

    return DRONEID_SUCCESS;
}

void packet_destroy(packet_t* pkt) {
    if (!pkt) return;

    if (pkt->packet_data) {
        free(pkt->packet_data);
        pkt->packet_data = NULL;
    }

    if (pkt->fft_cfg) {
        fft_destroy(pkt->fft_cfg);
        pkt->fft_cfg = NULL;
    }
}

int packet_detect_zc(packet_t* pkt) {
    if (!pkt) return DRONEID_ERROR_INVALID_ARG;

    /* Detect ZC sequences by correlation */
    int peak_idx_600, peak_idx_147;
    complex_t peak_val_600, peak_val_147;

    int ret = dsp_detect_zc(pkt->packet_data, pkt->packet_len,
                            pkt->zc_seq_600, ZC_LEN,
                            &peak_idx_600, &peak_val_600);
    if (ret != 0) return ret;

    ret = dsp_detect_zc(pkt->packet_data, pkt->packet_len,
                        pkt->zc_seq_147, ZC_LEN,
                        &peak_idx_147, &peak_val_147);
    if (ret != 0) return ret;

    /* Store ZC locations */
    const int* zc_indices = pkt->legacy ? ZC_SYMBOL_IDX_LEGACY : ZC_SYMBOL_IDX;
    pkt->zc_indices[0] = peak_idx_600;
    pkt->zc_indices[1] = peak_idx_147;
    pkt->zc_roots[0] = ZC_ROOT_1;
    pkt->zc_roots[1] = ZC_ROOT_2;

    /* Estimate frequency offset from ZC phase */
    pkt->freq_offset = complex_phase(peak_val_600) * pkt->sample_rate / (2.0f * M_PI * ZC_LEN);

    /* Estimate time offset from ZC peak position */
    int expected_pos = zc_indices[0] * (NFFT + CP_LENGTHS[zc_indices[0]]);
    pkt->time_offset = (float)(peak_idx_600 - expected_pos);

    if (pkt->debug) {
        LOG_DEBUG("ZC Detection:\n");
        LOG_DEBUG("  ZC-600 at index %d (mag: %.2f)\n", peak_idx_600, complex_mag(peak_val_600));
        LOG_DEBUG("  ZC-147 at index %d (mag: %.2f)\n", peak_idx_147, complex_mag(peak_val_147));
        LOG_DEBUG("  Freq offset: %.2f Hz\n", pkt->freq_offset);
        LOG_DEBUG("  Time offset: %.2f samples\n", pkt->time_offset);
    }

    return DRONEID_SUCCESS;
}

int packet_apply_corrections(packet_t* pkt) {
    if (!pkt) return DRONEID_ERROR_INVALID_ARG;

    /* Apply frequency correction */
    if (fabsf(pkt->freq_offset) > 100.0f) {
        complex_t* corrected = (complex_t*)malloc(pkt->packet_len * sizeof(complex_t));
        if (!corrected) return DRONEID_ERROR_MEMORY;

        dsp_freq_shift(pkt->packet_data, corrected, pkt->packet_len,
                      -pkt->freq_offset, pkt->sample_rate);

        memcpy(pkt->packet_data, corrected, pkt->packet_len * sizeof(complex_t));
        free(corrected);

        if (pkt->debug) {
            LOG_DEBUG("Applied freq correction: %.2f Hz\n", pkt->freq_offset);
        }
    }

    /* Time offset correction is implicit in symbol extraction */

    return DRONEID_SUCCESS;
}

int packet_extract_symbols(packet_t* pkt) {
    if (!pkt) return DRONEID_ERROR_INVALID_ARG;

    const int* cp_lengths = pkt->legacy ? CP_LENGTHS_LEGACY : CP_LENGTHS;
    int num_symbols = pkt->legacy ? 8 : MAX_SYMBOLS;

    /* Calculate symbol start positions */
    int symbol_start = (int)pkt->time_offset;

    pkt->num_symbols = 0;

    for (int sym = 0; sym < num_symbols && pkt->num_symbols < MAX_OFDM_SYMBOLS; sym++) {
        int cp_len = cp_lengths[sym];
        int symbol_len = NFFT + cp_len;

        /* Check bounds */
        if (symbol_start + symbol_len > pkt->packet_len) {
            break;
        }

        /* Remove cyclic prefix and extract symbol */
        complex_t* symbol_data = &pkt->packet_data[symbol_start + cp_len];

        /* Copy to symbol buffer */
        memcpy(pkt->symbols_time[pkt->num_symbols], symbol_data,
               NFFT * sizeof(complex_t));

        /* FFT to frequency domain */
        complex_t fft_out[NFFT];
        fft_forward(pkt->fft_cfg, pkt->symbols_time[pkt->num_symbols],
                   fft_out, NFFT);

        /* Extract carriers */
        dsp_extract_carriers(fft_out, pkt->symbols_freq[pkt->num_symbols]);

        pkt->num_symbols++;
        symbol_start += symbol_len;
    }

    if (pkt->debug) {
        LOG_DEBUG("Extracted %d OFDM symbols\n", pkt->num_symbols);
    }

    return pkt->num_symbols;
}

int packet_equalize(packet_t* pkt) {
    if (!pkt || pkt->num_symbols < 2) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    const int* zc_indices = pkt->legacy ? ZC_SYMBOL_IDX_LEGACY : ZC_SYMBOL_IDX;

    /* Use first ZC symbol for channel estimation */
    int zc_sym_idx = zc_indices[0];
    if (zc_sym_idx >= pkt->num_symbols) {
        LOG_ERROR("ZC symbol index out of range\n");
        return DRONEID_ERROR_DECODE;
    }

    /* Channel estimate: H = Y / X (received / known) */
    complex_t* zc_rx = pkt->symbols_freq[zc_sym_idx];
    complex_t* zc_known = pkt->zc_seq_600;

    for (int k = 0; k < NCARRIERS; k++) {
        float mag_sq = complex_mag_sq(zc_known[k]);
        if (mag_sq > 1e-6f) {
            pkt->channel_est[k] = zc_rx[k] * conjf(zc_known[k]) / mag_sq;
        } else {
            pkt->channel_est[k] = 1.0f + 0.0fI;
        }
    }

    /* Average channel estimates from both ZC symbols if available */
    if (zc_indices[1] < pkt->num_symbols) {
        complex_t* zc_rx2 = pkt->symbols_freq[zc_indices[1]];
        complex_t* zc_known2 = pkt->zc_seq_147;

        for (int k = 0; k < NCARRIERS; k++) {
            float mag_sq = complex_mag_sq(zc_known2[k]);
            if (mag_sq > 1e-6f) {
                complex_t H2 = zc_rx2[k] * conjf(zc_known2[k]) / mag_sq;
                pkt->channel_est[k] = (pkt->channel_est[k] + H2) * 0.5f;
            }
        }
    }

    /* Equalize data symbols */
    int data_sym_idx = 0;
    for (int sym = 0; sym < pkt->num_symbols && data_sym_idx < DATA_SYMBOLS; sym++) {
        /* Skip ZC symbols */
        bool is_zc = false;
        for (int i = 0; i < 2; i++) {
            if (sym == zc_indices[i]) {
                is_zc = true;
                break;
            }
        }

        if (!is_zc) {
            /* Equalize: Y_eq = Y / H */
            for (int k = 0; k < NCARRIERS; k++) {
                float mag_sq = complex_mag_sq(pkt->channel_est[k]);
                if (mag_sq > 1e-6f) {
                    pkt->data_symbols[data_sym_idx][k] =
                        pkt->symbols_freq[sym][k] * conjf(pkt->channel_est[k]) / mag_sq;
                } else {
                    pkt->data_symbols[data_sym_idx][k] = pkt->symbols_freq[sym][k];
                }
            }
            data_sym_idx++;
        }
    }

    if (pkt->debug) {
        LOG_DEBUG("Equalized %d data symbols\n", data_sym_idx);
    }

    return DRONEID_SUCCESS;
}

int packet_get_symbol_data(packet_t* pkt, complex_t symbols[DATA_SYMBOLS][NCARRIERS]) {
    if (!pkt || !symbols) return DRONEID_ERROR_INVALID_ARG;

    memcpy(symbols, pkt->data_symbols, DATA_SYMBOLS * NCARRIERS * sizeof(complex_t));

    return DRONEID_SUCCESS;
}
