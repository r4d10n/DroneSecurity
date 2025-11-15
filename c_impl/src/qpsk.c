/**
 * @file qpsk.c
 * @brief QPSK demodulation and decoding implementation
 */

#include "qpsk.h"
#include <stdlib.h>
#include <string.h>

/* QPSK quadrant to bits mapping for phase rotations */
const uint8_t QPSK_TO_BITS[4][4] = {
    {2, 3, 1, 0},  /* 0 degrees */
    {0, 2, 3, 1},  /* 90 degrees */
    {1, 0, 2, 3},  /* 180 degrees */
    {3, 1, 0, 2}   /* 270 degrees */
};

/* Data symbol indices (skip ZC symbols) */
const int DATA_SYMBOL_IDX[DATA_SYMBOLS] = {0, 1, 2, 4, 6, 7, 8};

/* Rate matching permutation from 3GPP */
const int RM_PERM_TURBO[32] = {
    0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30,
    1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31
};

int qpsk_decoder_init(qpsk_decoder_t* decoder) {
    if (!decoder) return DRONEID_ERROR_INVALID_ARG;

    memset(decoder, 0, sizeof(qpsk_decoder_t));

    /* Allocate descrambled bits buffer (max size) */
    decoder->descrambled_bits = (uint8_t*)malloc(DATA_SYMBOLS * NCARRIERS * 2);
    if (!decoder->descrambled_bits) {
        return DRONEID_ERROR_MEMORY;
    }

    decoder->n_descrambled = 0;

    return DRONEID_SUCCESS;
}

void qpsk_decoder_destroy(qpsk_decoder_t* decoder) {
    if (!decoder) return;

    if (decoder->descrambled_bits) {
        free(decoder->descrambled_bits);
        decoder->descrambled_bits = NULL;
    }

    decoder->n_descrambled = 0;
}

uint8_t qpsk_get_symbol_bits(complex_t symbol, int phase_correction) {
    if (phase_correction < 0 || phase_correction >= 4) {
        return 0;
    }

    /* Determine quadrant */
    float re = crealf(symbol);
    float im = cimagf(symbol);

    int quadrant;
    if (re >= 0.0f && im >= 0.0f) {
        quadrant = 0;  /* Q1 */
    } else if (re >= 0.0f && im < 0.0f) {
        quadrant = 1;  /* Q4 */
    } else if (re < 0.0f && im < 0.0f) {
        quadrant = 2;  /* Q3 */
    } else {
        quadrant = 3;  /* Q2 */
    }

    return QPSK_TO_BITS[phase_correction][quadrant];
}

void qpsk_demodulate(qpsk_decoder_t* decoder, int phase_correction) {
    if (!decoder) return;

    for (int sym = 0; sym < DATA_SYMBOLS; sym++) {
        for (int carr = 0; carr < NCARRIERS; carr++) {
            decoder->symbol_bits[sym][carr] =
                qpsk_get_symbol_bits(decoder->symbols[sym][carr], phase_correction);
        }
    }
}

/* Linear Feedback Shift Register for Gold sequence */
static uint32_t lfsr_step(uint32_t state, uint32_t taps) {
    uint32_t feedback = state & taps;

    /* Count bits */
    uint32_t count = 0;
    while (feedback) {
        count ^= (feedback & 1);
        feedback >>= 1;
    }

    return (state >> 1) | (count << 31);
}

void gold_sequence_generate(int n1, int n2, uint32_t seed, uint8_t* out) {
    if (!out) return;

    /* Simple LFSR-based Gold sequence generator */
    uint32_t state = seed;
    uint32_t taps = 0x80200003;  /* Polynomial taps */

    for (int i = 0; i < n2; i++) {
        out[i] = state & 1;
        state = lfsr_step(state, taps);
    }
}

int qpsk_descramble(qpsk_decoder_t* decoder, bool legacy) {
    if (!decoder) return DRONEID_ERROR_INVALID_ARG;

    /* Convert symbol bits to bit array */
    uint8_t* bits = (uint8_t*)malloc(DATA_SYMBOLS * NCARRIERS * 2);
    if (!bits) return DRONEID_ERROR_MEMORY;

    int bit_idx = 0;

    /* Remove DC carrier (index 300) and convert 2-bit symbols to bits */
    for (int sym = 0; sym < DATA_SYMBOLS; sym++) {
        for (int carr = 0; carr < NCARRIERS; carr++) {
            if (carr == 300) continue;  /* Skip DC carrier */

            uint8_t symbol_val = decoder->symbol_bits[sym][carr];

            /* Extract 2 bits from symbol */
            bits[bit_idx++] = (symbol_val & 2) >> 1;
            bits[bit_idx++] = (symbol_val & 1);
        }
    }

    int total_bits = bit_idx;

    /* Generate Gold sequence */
    uint8_t* gold_seq;
    int gold_len;

    if (legacy || total_bits <= 7200) {
        /* Legacy mode: shorter sequence */
        gold_len = total_bits;
    } else {
        /* Standard mode: check first symbol against Gold */
        gold_len = 1200;

        gold_seq = (uint8_t*)malloc(gold_len);
        if (!gold_seq) {
            free(bits);
            return DRONEID_ERROR_MEMORY;
        }

        gold_sequence_generate(1600, gold_len, 0x12345678, gold_seq);

        /* Verify first symbol (optional validation) */
        int gold_errors = 0;
        for (int i = 0; i < gold_len; i++) {
            if (bits[i] != gold_seq[i]) {
                gold_errors++;
            }
        }

        LOG_DEBUG("Gold sequence check: %d errors out of %d bits\n",
                  gold_errors, gold_len);

        free(gold_seq);

        /* Skip first symbol for descrambling */
        bit_idx = gold_len;
        total_bits -= gold_len;
    }

    /* Generate scrambling sequence for remaining bits */
    gold_len = total_bits;
    gold_seq = (uint8_t*)malloc(gold_len);
    if (!gold_seq) {
        free(bits);
        return DRONEID_ERROR_MEMORY;
    }

    gold_sequence_generate(1600, gold_len, 0x12345678, gold_seq);

    /* XOR with Gold sequence to descramble */
    for (int i = 0; i < gold_len; i++) {
        decoder->descrambled_bits[i] = bits[bit_idx + i] ^ gold_seq[i];
    }

    decoder->n_descrambled = gold_len;

    free(bits);
    free(gold_seq);

    return DRONEID_SUCCESS;
}

int qpsk_rm_turbo_rx(const uint8_t* bits_in, int n_in, uint8_t* bits_out) {
    if (!bits_in || !bits_out) return DRONEID_ERROR_INVALID_ARG;

    int ncols = 32;
    int nrows = (n_in + 31) / ncols;
    int n_dummy = (ncols * nrows) - n_in;

    /* Allocate matrix */
    int8_t* matrix = (int8_t*)calloc(nrows * ncols, sizeof(int8_t));
    if (!matrix) return DRONEID_ERROR_MEMORY;

    /* Initialize with -1 (dummy bits) */
    for (int i = 0; i < nrows * ncols; i++) {
        matrix[i] = -1;
    }

    /* De-interleave */
    int p = 0;
    for (int col = 0; col < ncols; col++) {
        int perm_col = RM_PERM_TURBO[col];

        if (perm_col < n_dummy) {
            /* This column starts with dummy bit */
            for (int row = 1; row < nrows && p < n_in; row++) {
                matrix[row * ncols + perm_col] = bits_in[p++];
            }
        } else {
            /* Full column */
            for (int row = 0; row < nrows && p < n_in; row++) {
                matrix[row * ncols + perm_col] = bits_in[p++];
            }
        }
    }

    /* Extract output (skip dummy bits) */
    int out_idx = 0;
    for (int i = 0; i < nrows * ncols; i++) {
        if (matrix[i] != -1) {
            bits_out[out_idx++] = matrix[i];
        }
    }

    free(matrix);

    return out_idx;
}

int qpsk_decode_payload(qpsk_decoder_t* decoder, bool legacy,
                        uint8_t* out_bytes, int* out_len) {
    if (!decoder || !out_bytes || !out_len) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    /* Descramble bits */
    int ret = qpsk_descramble(decoder, legacy);
    if (ret != 0) return ret;

    /* Extract systematic stream from cyclic buffer */
    int offset = 4148;
    int sys_len = 1412;

    if (offset + sys_len > decoder->n_descrambled) {
        LOG_ERROR("Not enough descrambled bits for systematic extraction\n");
        return DRONEID_ERROR_DECODE;
    }

    uint8_t* systematic = &decoder->descrambled_bits[offset];

    /* Apply rate matching de-interleaving */
    uint8_t* rm_output = (uint8_t*)malloc(sys_len);
    if (!rm_output) return DRONEID_ERROR_MEMORY;

    int decoded_len = qpsk_rm_turbo_rx(systematic, sys_len, rm_output);

    if (decoded_len <= 0) {
        free(rm_output);
        return DRONEID_ERROR_DECODE;
    }

    /* Convert bits to bytes */
    *out_len = decoded_len / 8;
    for (int i = 0; i < *out_len; i++) {
        out_bytes[i] = 0;
        for (int bit = 0; bit < 8; bit++) {
            out_bytes[i] |= (rm_output[i * 8 + bit] << (7 - bit));
        }
    }

    free(rm_output);

    return DRONEID_SUCCESS;
}
