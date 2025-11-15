/**
 * @file dsp_neon.c
 * @brief DSP functions with NEON optimization
 *
 * This implementation uses ARM NEON intrinsics for SIMD acceleration
 * and falls back to scalar code when NEON is unavailable.
 */

#include "dsp_neon.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Ne10 library for optimized FFT */
#ifdef HAVE_NE10
#include <NE10.h>
#endif

/* FFT configuration structure */
struct fft_config {
    int nfft;
#ifdef HAVE_NE10
    ne10_fft_cfg_float32_t ne10_cfg;
#else
    void* generic_cfg;
#endif
};

fft_config_t fft_init(int nfft) {
    fft_config_t cfg = (fft_config_t)malloc(sizeof(struct fft_config));
    if (!cfg) return NULL;

    cfg->nfft = nfft;

#ifdef HAVE_NE10
    cfg->ne10_cfg = ne10_fft_alloc_c2c_float32_neon(nfft);
    if (!cfg->ne10_cfg) {
        free(cfg);
        return NULL;
    }
#else
    LOG_WARN("Ne10 not available, FFT will be slow\n");
    cfg->generic_cfg = NULL;
#endif

    return cfg;
}

void fft_destroy(fft_config_t cfg) {
    if (!cfg) return;

#ifdef HAVE_NE10
    if (cfg->ne10_cfg) {
        NE10_FREE(cfg->ne10_cfg);
    }
#endif

    free(cfg);
}

void fft_forward(fft_config_t cfg, const complex_t* in, complex_t* out, int nfft) {
    if (!cfg || !in || !out) return;

#ifdef HAVE_NE10
    ne10_fft_c2c_1d_float32_neon(
        (ne10_fft_cpx_float32_t*)out,
        (ne10_fft_cpx_float32_t*)in,
        cfg->ne10_cfg,
        0  /* Forward FFT */
    );
#else
    /* Fallback: naive DFT (very slow!) */
    for (int k = 0; k < nfft; k++) {
        out[k] = 0;
        for (int n = 0; n < nfft; n++) {
            float angle = -2.0f * M_PI * k * n / nfft;
            out[k] += in[n] * (cosf(angle) + I * sinf(angle));
        }
    }
#endif
}

void fft_inverse(fft_config_t cfg, const complex_t* in, complex_t* out, int nfft) {
    if (!cfg || !in || !out) return;

#ifdef HAVE_NE10
    ne10_fft_c2c_1d_float32_neon(
        (ne10_fft_cpx_float32_t*)out,
        (ne10_fft_cpx_float32_t*)in,
        cfg->ne10_cfg,
        1  /* Inverse FFT */
    );

    /* Ne10 doesn't normalize, do it manually */
    float scale = 1.0f / nfft;
    neon_scale(out, out, scale, nfft);
#else
    /* Fallback: naive IDFT */
    for (int k = 0; k < nfft; k++) {
        out[k] = 0;
        for (int n = 0; n < nfft; n++) {
            float angle = 2.0f * M_PI * k * n / nfft;
            out[k] += in[n] * (cosf(angle) + I * sinf(angle));
        }
        out[k] /= nfft;
    }
#endif
}

/* Welch's PSD */
int dsp_welch_psd(const complex_t* samples, int n_samples, float sample_rate,
                  int nfft, float* psd, float* freq) {
    if (!samples || !psd || !freq || n_samples < nfft) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    /* Initialize FFT */
    fft_config_t fft_cfg = fft_init(nfft);
    if (!fft_cfg) return DRONEID_ERROR_MEMORY;

    /* Welch parameters */
    int overlap = nfft / 2;
    int hop = nfft - overlap;
    int num_windows = (n_samples - nfft) / hop + 1;

    /* Allocate temporary buffers */
    complex_t* fft_in = (complex_t*)malloc(nfft * sizeof(complex_t));
    complex_t* fft_out = (complex_t*)malloc(nfft * sizeof(complex_t));
    float* window = (float*)malloc(nfft * sizeof(float));

    if (!fft_in || !fft_out || !window) {
        free(fft_in);
        free(fft_out);
        free(window);
        fft_destroy(fft_cfg);
        return DRONEID_ERROR_MEMORY;
    }

    /* Hann window */
    for (int i = 0; i < nfft; i++) {
        window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (nfft - 1)));
    }

    /* Initialize PSD to zero */
    memset(psd, 0, nfft * sizeof(float));

    /* Compute PSD for each window */
    for (int w = 0; w < num_windows; w++) {
        int start = w * hop;

        /* Apply window */
        for (int i = 0; i < nfft; i++) {
            fft_in[i] = samples[start + i] * window[i];
        }

        /* Compute FFT */
        fft_forward(fft_cfg, fft_in, fft_out, nfft);

        /* Accumulate power */
        for (int i = 0; i < nfft; i++) {
            psd[i] += complex_mag_sq(fft_out[i]);
        }
    }

    /* Average and normalize */
    float scale = 1.0f / num_windows;
    for (int i = 0; i < nfft; i++) {
        psd[i] *= scale;
    }

    /* FFT shift (move DC to center) */
    float* psd_shifted = (float*)malloc(nfft * sizeof(float));
    int half = nfft / 2;
    for (int i = 0; i < half; i++) {
        psd_shifted[i] = psd[half + i];
        psd_shifted[half + i] = psd[i];
    }
    memcpy(psd, psd_shifted, nfft * sizeof(float));

    /* Compute frequency array */
    for (int i = 0; i < nfft; i++) {
        freq[i] = (i - half) * sample_rate / nfft;
    }

    /* Cleanup */
    free(fft_in);
    free(fft_out);
    free(window);
    free(psd_shifted);
    fft_destroy(fft_cfg);

    return DRONEID_SUCCESS;
}

/* Correlation */
int dsp_correlate(const complex_t* x, int nx, const complex_t* y, int ny, complex_t* out) {
    if (!x || !out) return DRONEID_ERROR_INVALID_ARG;

    const complex_t* y_actual = y ? y : x;
    int ny_actual = y ? ny : nx;
    int n_out = nx + ny_actual - 1;

    /* Full correlation */
    for (int lag = 0; lag < n_out; lag++) {
        out[lag] = 0;

        int start_x = (lag >= ny_actual) ? lag - ny_actual + 1 : 0;
        int end_x = (lag < nx) ? lag + 1 : nx;

        for (int i = start_x; i < end_x; i++) {
            int j = lag - i;
            if (j >= 0 && j < ny_actual) {
                out[lag] += x[i] * conjf(y_actual[j]);
            }
        }
    }

    /* Return only second half (as in Python version) */
    int half = n_out / 2;
    memmove(out, &out[half], (n_out - half) * sizeof(complex_t));

    return n_out - half;
}

/* Frequency shift */
void dsp_freq_shift(const complex_t* in, complex_t* out, int n,
                    float freq_offset, float sample_rate) {
    if (!in || !out || n == 0) return;

    float dt = 1.0f / sample_rate;

    for (int i = 0; i < n; i++) {
        float t = i * dt;
        float phase = 2.0f * M_PI * freq_offset * t;
        complex_t phasor = cosf(phase) + I * sinf(phase);
        out[i] = in[i] * phasor;
    }
}

/* Resample (linear interpolation) */
void dsp_resample(const complex_t* in, int n_in, complex_t* out, int n_out,
                  float fs_in, float fs_out) {
    if (!in || !out || n_in == 0 || n_out == 0) return;

    float ratio = fs_in / fs_out;

    for (int i = 0; i < n_out; i++) {
        float src_idx = i * ratio;
        int idx0 = (int)src_idx;
        int idx1 = idx0 + 1;

        if (idx1 >= n_in) {
            out[i] = in[n_in - 1];
            continue;
        }

        float frac = src_idx - idx0;
        out[i] = in[idx0] * (1.0f - frac) + in[idx1] * frac;
    }
}

/* NEON-optimized complex multiplication */
void neon_complex_mult(const complex_t* a, const complex_t* b, complex_t* out, int n) {
    if (!a || !b || !out) return;

#ifdef HAVE_NEON
    int i = 0;
    /* Process 2 complex numbers at a time (4 floats) */
    for (; i < n - 1; i += 2) {
        float32x4_t va = vld1q_f32((const float*)&a[i]); /* [a0.r, a0.i, a1.r, a1.i] */
        float32x4_t vb = vld1q_f32((const float*)&b[i]);

        /* Real: a.r * b.r - a.i * b.i */
        float32x4_t real_part = vmulq_f32(va, vb);
        float32x4_t imag_swap = vrev64q_f32(va); /* [a0.i, a0.r, a1.i, a1.r] */
        float32x4_t cross = vmulq_f32(imag_swap, vb);

        /* Subtract imaginary cross product */
        float32x2_t r0 = vget_low_f32(real_part);
        float32x2_t r1 = vget_high_f32(real_part);
        float32x2_t c0 = vget_low_f32(cross);
        float32x2_t c1 = vget_high_f32(cross);

        float result_r0 = vget_lane_f32(r0, 0) - vget_lane_f32(c0, 1);
        float result_i0 = vget_lane_f32(r0, 1) + vget_lane_f32(c0, 0);
        float result_r1 = vget_lane_f32(r1, 0) - vget_lane_f32(c1, 1);
        float result_i1 = vget_lane_f32(r1, 1) + vget_lane_f32(c1, 0);

        out[i] = result_r0 + I * result_i0;
        out[i + 1] = result_r1 + I * result_i1;
    }

    /* Handle remainder */
    for (; i < n; i++) {
        out[i] = a[i] * b[i];
    }
#else
    /* Scalar fallback */
    for (int i = 0; i < n; i++) {
        out[i] = a[i] * b[i];
    }
#endif
}

/* NEON-optimized complex mult by conjugate */
void neon_complex_mult_conj(const complex_t* a, const complex_t* b, complex_t* out, int n) {
    if (!a || !b || !out) return;

    for (int i = 0; i < n; i++) {
        out[i] = a[i] * conjf(b[i]);
    }
}

/* NEON-optimized magnitude squared */
void neon_abs_squared(const complex_t* in, float* out, int n) {
    if (!in || !out) return;

#ifdef HAVE_NEON
    int i = 0;
    for (; i < n - 1; i += 2) {
        float32x4_t v = vld1q_f32((const float*)&in[i]); /* [r0, i0, r1, i1] */
        float32x4_t sq = vmulq_f32(v, v);                 /* [r0^2, i0^2, r1^2, i1^2] */

        /* Pairwise add to get |z|^2 */
        float32x2_t low = vget_low_f32(sq);  /* [r0^2, i0^2] */
        float32x2_t high = vget_high_f32(sq); /* [r1^2, i1^2] */

        float32x2_t sum = vpadd_f32(low, high); /* [r0^2+i0^2, r1^2+i1^2] */

        out[i] = vget_lane_f32(sum, 0);
        out[i + 1] = vget_lane_f32(sum, 1);
    }

    /* Remainder */
    for (; i < n; i++) {
        out[i] = complex_mag_sq(in[i]);
    }
#else
    for (int i = 0; i < n; i++) {
        out[i] = complex_mag_sq(in[i]);
    }
#endif
}

/* NEON-optimized magnitude */
void neon_abs(const complex_t* in, float* out, int n) {
    neon_abs_squared(in, out, n);

    /* Take square root */
    for (int i = 0; i < n; i++) {
        out[i] = sqrtf(out[i]);
    }
}

/* Find argmax of absolute values */
int neon_argmax_abs(const complex_t* in, int n) {
    if (!in || n == 0) return -1;

    float max_val = 0.0f;
    int max_idx = 0;

    for (int i = 0; i < n; i++) {
        float mag = complex_mag(in[i]);
        if (mag > max_val) {
            max_val = mag;
            max_idx = i;
        }
    }

    return max_idx;
}

/* Find argmax of float array */
int neon_argmax_float(const float* in, int n) {
    if (!in || n == 0) return -1;

    float max_val = in[0];
    int max_idx = 0;

    for (int i = 1; i < n; i++) {
        if (in[i] > max_val) {
            max_val = in[i];
            max_idx = i;
        }
    }

    return max_idx;
}

/* Compute mean */
float neon_mean(const float* in, int n) {
    if (!in || n == 0) return 0.0f;

    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += in[i];
    }

    return sum / n;
}

/* Complex conjugate */
void neon_conj(const complex_t* in, complex_t* out, int n) {
    for (int i = 0; i < n; i++) {
        out[i] = conjf(in[i]);
    }
}

/* Add */
void neon_add(const complex_t* a, const complex_t* b, complex_t* out, int n) {
    for (int i = 0; i < n; i++) {
        out[i] = a[i] + b[i];
    }
}

/* Subtract */
void neon_sub(const complex_t* a, const complex_t* b, complex_t* out, int n) {
    for (int i = 0; i < n; i++) {
        out[i] = a[i] - b[i];
    }
}

/* Scale */
void neon_scale(const complex_t* in, complex_t* out, float scale, int n) {
    for (int i = 0; i < n; i++) {
        out[i] = in[i] * scale;
    }
}

/* Extract OFDM carriers */
void dsp_extract_carriers(const complex_t* fft_out, complex_t* carriers) {
    int half_carriers = NCARRIERS / 2;

    /* Copy negative frequencies */
    memcpy(&carriers[0], &fft_out[NFFT - half_carriers], half_carriers * sizeof(complex_t));

    /* Copy positive frequencies + DC */
    memcpy(&carriers[half_carriers], &fft_out[0], (half_carriers + 1) * sizeof(complex_t));
}

/* Insert OFDM carriers */
void dsp_insert_carriers(const complex_t* carriers, complex_t* fft_in) {
    int half_carriers = NCARRIERS / 2;

    /* Zero out FFT input */
    memset(fft_in, 0, NFFT * sizeof(complex_t));

    /* Insert negative frequencies */
    memcpy(&fft_in[NFFT - half_carriers], &carriers[0], half_carriers * sizeof(complex_t));

    /* Insert positive frequencies + DC */
    memcpy(&fft_in[0], &carriers[half_carriers], (half_carriers + 1) * sizeof(complex_t));
}

/* Generate Zadoff-Chu sequence */
void dsp_generate_zc_sequence(complex_t* out, int root, int len) {
    for (int n = 0; n < len; n++) {
        float phase = -M_PI * root * n * (n + 1) / len;
        out[n] = cosf(phase) + I * sinf(phase);
    }
}

/* Detect ZC sequence */
int dsp_detect_zc(const complex_t* signal, int n_signal,
                  const complex_t* zc_seq, int zc_len,
                  int* peak_idx, complex_t* peak_val) {
    if (!signal || !zc_seq || !peak_idx || !peak_val) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    /* Compute correlation */
    int n_corr = n_signal + zc_len - 1;
    complex_t* corr = (complex_t*)malloc(n_corr * sizeof(complex_t));
    if (!corr) return DRONEID_ERROR_MEMORY;

    int corr_len = dsp_correlate(signal, n_signal, zc_seq, zc_len, corr);

    /* Find peak */
    *peak_idx = neon_argmax_abs(corr, corr_len);
    *peak_val = corr[*peak_idx];

    free(corr);

    return DRONEID_SUCCESS;
}
