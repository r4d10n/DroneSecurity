# DroneSecurity C Implementation Architecture

## Overview

This document describes the C implementation of the DroneSecurity Drone-ID receiver, optimized for ARM Cortex-A9 with NEON SIMD extensions, specifically targeting the Xilinx Zynq 7000 SoC onboard the PlutoSDR.

## Goals

1. **Performance**: Achieve real-time processing on Zynq 7000 ARM Cortex-A9 @ 666 MHz
2. **Efficiency**: Utilize NEON SIMD for compute-intensive operations
3. **Portability**: Support both ARM (cross-compiled) and x86_64 (native) builds
4. **Standalone**: Run directly on PlutoSDR without external PC
5. **Compatibility**: Match Python implementation's functionality and accuracy

## System Architecture

### Hardware Platform

**Target: Xilinx Zynq 7000 (PlutoSDR)**
- **CPU**: Dual ARM Cortex-A9 @ 666 MHz
- **SIMD**: NEON (128-bit vector engine)
- **Memory**: 512 MB DDR3
- **RF**: AD9363 transceiver
- **OS**: Linux (Analog Devices custom distribution)

### Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                        libiio Interface                          │
│  (RF Frontend: AD9363 via IIO, Sample Acquisition, Buffering)   │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Complex samples (IQ data)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Spectrum Capture Module                       │
│  • Coarse frequency offset estimation (Welch's method)          │
│  • Packet detection (power threshold, bandwidth matching)       │
│  • Frame extraction and resampling                              │
│  • NEON: FFT, PSD computation, signal statistics                │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Individual frames (time domain)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Packet Processing Module                     │
│  • Zadoff-Chu sequence detection (correlation)                  │
│  • Fine time/frequency offset correction                        │
│  • OFDM symbol splitting (CP removal)                           │
│  • FFT per OFDM symbol (1024-point)                             │
│  • Channel equalization using ZC pilots                         │
│  • NEON: Correlation, FFT, complex arithmetic                   │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Equalized subcarriers (601 symbols × 7 OFDM)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    QPSK Demodulation Module                      │
│  • QPSK symbol decision (quadrant mapping)                      │
│  • Phase rotation brute-force (0°, 90°, 180°, 270°)            │
│  • Bit extraction from symbols                                  │
│  • Gold sequence descrambling                                   │
│  • NEON: Parallel symbol processing                             │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Scrambled bits
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Turbo Decoder Module                          │
│  • Rate matching de-interleaving                                │
│  • Turbo decoding (MAX-Log-MAP algorithm)                       │
│  • Integration with TurboFEC library (SIMD optimized)           │
│  • NEON: Soft decision operations                               │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Decoded payload bytes
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Drone-ID Packet Parser                          │
│  • Unpack bitstream into structured data                        │
│  • CRC validation                                               │
│  • Drone model identification                                   │
│  • GPS coordinate parsing, telemetry extraction                 │
└─────────────────────────────────────────────────────────────────┘
```

## Module Descriptions

### 1. DSP Library (`dsp_neon.c/h`)

Core digital signal processing functions optimized with NEON intrinsics.

**Functions**:
- `neon_cfft_f32()` - Complex FFT (1024, 2048 point)
- `neon_correlate()` - Complex correlation
- `neon_abs_squared()` - Magnitude squared (power)
- `neon_frequency_shift()` - Frequency domain shift
- `neon_resample()` - Arbitrary ratio resampling
- `neon_welch_psd()` - Power spectral density estimation
- `neon_complex_mult()` - Element-wise complex multiplication
- `neon_complex_conj()` - Complex conjugate

**NEON Optimization Strategy**:
- Process 4 complex float32 values per iteration (128-bit NEON register)
- Use intrinsics: `vld1q_f32`, `vmulq_f32`, `vaddq_f32`, etc.
- Leverage NEON pipelined FPU for maximum throughput
- Minimize memory transfers, maximize register usage

**Dependencies**:
- Ne10 library (ARM's NEON-optimized DSP library)
- Or CMSIS-DSP (alternative)
- Custom NEON intrinsics for specialized operations

### 2. SpectrumCapture Module (`spectrum_capture.c/h`)

Detects and extracts Drone-ID frames from wideband captures.

**Key Algorithms**:
1. **Welch's Method PSD** (NEON-accelerated):
   - 2048-point FFT on overlapping windows
   - Average power across windows
   - Identify frequency bands above noise floor

2. **Bandwidth Detection**:
   - Find consecutive bins > mean power
   - Match bandwidth: 8-11 MHz (Drone-ID), 1.2-1.95 MHz (C2)
   - Calculate center frequency offset

3. **Frame Extraction**:
   - Apply frequency shift to compensate coarse CFO
   - Resample to target rate (NEON interpolation)
   - Extract individual packet candidates

**Memory Optimization**:
- Ring buffers for streaming operation
- In-place FFT when possible
- Fixed-size packet queues

### 3. Packet Processing Module (`packet.c/h`)

Performs OFDM demodulation and channel equalization.

**Zadoff-Chu Detection**:
```c
// Correlate with known ZC sequences (roots 600, 147)
for (int symbol = 0; symbol < num_symbols; symbol++) {
    complex_t* zc_corr = neon_correlate(symbol_data, zc_seq_600, seq_len);
    // Find peak correlation index (NEON argmax)
    int peak_idx = neon_argmax_abs(zc_corr, corr_len);
    // Timing offset = peak position
    // Frequency offset = phase of correlation peak
}
```

**OFDM Symbol Processing**:
1. Remove cyclic prefix (CP lengths per LTE spec)
2. 1024-point FFT (Ne10 `ne10_fft_c2c_1d_float32_neon`)
3. Extract 601 data carriers (center bins)
4. Equalize using ZC symbol channel estimates

**Channel Equalization**:
```c
// H_est = ZC_received / ZC_known
complex_t H[601];
for (int k = 0; k < 601; k++) {
    H[k] = complex_div(zc_rx[k], zc_known[k]);
}

// Equalize data symbols
for (int sym = 0; sym < 7; sym++) {
    for (int k = 0; k < 601; k++) {
        data_eq[sym][k] = complex_div(data_rx[sym][k], H[k]);
    }
}
```

### 4. QPSK Decoder Module (`qpsk.c/h`)

Demodulates QPSK symbols and prepares bits for Turbo decoding.

**QPSK Decision (NEON Vectorized)**:
```c
// Process 4 symbols in parallel
float32x4_t real_part = vld1q_f32(&symbols[i].real);
float32x4_t imag_part = vld1q_f32(&symbols[i].imag);

uint32x4_t real_pos = vcgtq_f32(real_part, vzeroq_f32());
uint32x4_t imag_pos = vcgtq_f32(imag_part, vzeroq_f32());

// Map to quadrants (0-3) based on real/imag signs
uint32x4_t quadrant = ...; // Bit manipulation
```

**Phase Rotation Brute-Force**:
- Try all 4 QPSK orientations (0°, 90°, 180°, 270°)
- Demodulate, descramble, decode for each
- Select orientation with valid CRC

**Gold Sequence Descrambling**:
- Generate Gold sequence (LFSR with seed 0x12345678)
- XOR with demodulated bits
- Extract systematic bits from turbo code cyclic buffer

### 5. Turbo Decoder Module (`turbo.c/h`)

Implements 3GPP LTE turbo decoding.

**Integration Options**:

**Option A: TurboFEC Library** (Recommended)
- GitHub: ttsou/turbofec
- SIMD-optimized (SSE/AVX/NEON)
- LTE-compliant turbo decoder
- Proven performance

**Option B: Custom Implementation**
- MAX-Log-MAP algorithm
- Parallel SISO (soft-input soft-output) decoders
- Interleaving/de-interleaving
- More complex but full control

**Rate Matching**:
```c
// De-interleave per 3GPP spec (RM_PERM_TURBO)
const int RM_PERM_TURBO[32] = {0, 16, 8, 24, 4, 20, ...};

void rm_turbo_rx(uint8_t* bits_in, int len, uint8_t* bits_out) {
    int ncols = 32;
    int nrows = (len + 31) / ncols;
    int n_dummy = (ncols * nrows) - len;

    // De-interleave into matrix
    // Remove dummy bits
    // Flatten back to array
}
```

### 6. Drone-ID Parser Module (`droneid_packet.c/h`)

Unpacks decoded payload into structured telemetry data.

**Data Structure**:
```c
typedef struct {
    uint16_t pkt_len;
    uint8_t version;
    uint16_t sequence_number;
    uint16_t state_info;
    char serial_number[16];
    double longitude;
    double latitude;
    float altitude;
    float height;
    int16_t v_north, v_east, v_up;
    uint64_t gps_time;
    double app_lat, app_lon;
    double home_lat, home_lon;
    char device_type[32];
    uint16_t crc_packet;
    uint16_t crc_calculated;
} droneid_payload_t;
```

**CRC Validation**:
- CRC-16 over payload
- Computed using `crcmod` algorithm equivalent in C

### 7. libiio Interface Module (`pluto_iio.c/h`)

Interfaces with PlutoSDR AD9363 transceiver via Industrial I/O (IIO) framework.

**Initialization**:
```c
struct iio_context* ctx = iio_create_default_context();
struct iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
struct iio_device* dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

// Configure RX parameters
iio_channel_attr_write_longlong(phy_rx, "frequency", 2414500000); // 2.4 GHz
iio_channel_attr_write_longlong(phy_rx, "sampling_frequency", 50000000); // 50 MHz
iio_channel_attr_write_longlong(phy_rx, "rf_bandwidth", 50000000);
iio_channel_attr_write(phy_rx, "gain_control_mode", "slow_attack");
```

**Sample Acquisition**:
```c
struct iio_buffer* rxbuf = iio_device_create_buffer(dev, buffer_size, false);

while (running) {
    ssize_t nbytes = iio_buffer_refill(rxbuf);

    // Process IQ samples
    int16_t* p_dat = (int16_t*)iio_buffer_first(rxbuf, rx_i);

    for (int i = 0; i < buffer_size; i++) {
        complex_t sample;
        sample.real = (float)p_dat[i*2] / 2048.0f;     // I
        sample.imag = (float)p_dat[i*2+1] / 2048.0f;   // Q

        process_sample(sample);
    }
}
```

## NEON Optimization Details

### Register Allocation

NEON provides 32 × 128-bit vector registers (Q0-Q15, or D0-D31 for 64-bit).

**Complex float32 packing**:
- Q0 = [real0, imag0, real1, imag1] (4 × float32)
- Process 2 complex samples per register

**Common NEON Intrinsics**:
```c
#include <arm_neon.h>

// Load 4 floats
float32x4_t vld1q_f32(const float32_t* ptr);

// Multiply
float32x4_t vmulq_f32(float32x4_t a, float32x4_t b);

// Add
float32x4_t vaddq_f32(float32x4_t a, float32x4_t b);

// Store
void vst1q_f32(float32_t* ptr, float32x4_t val);
```

### Complex Arithmetic with NEON

**Complex Multiplication**: `(a + bi) × (c + di) = (ac - bd) + (ad + bc)i`

```c
void neon_complex_mult(complex_t* a, complex_t* b, complex_t* out, int n) {
    for (int i = 0; i < n; i += 2) {
        float32x4_t va = vld1q_f32((float*)&a[i]); // [a0.r, a0.i, a1.r, a1.i]
        float32x4_t vb = vld1q_f32((float*)&b[i]);

        // Real part: ac - bd
        float32x4_t ac = vmulq_f32(va, vb);
        float32x4_t bd = vmulq_f32(
            vrev64q_f32(va),  // Swap real/imag
            vb
        );
        float32x4_t real = vsubq_f32(ac, bd);

        // ... (similar for imaginary part)

        vst1q_f32((float*)&out[i], result);
    }
}
```

### FFT Acceleration

Use Ne10 library's NEON-optimized FFT:

```c
#include <NE10.h>

ne10_fft_cfg_float32_t cfg = ne10_fft_alloc_c2c_float32_neon(1024);

void compute_fft(complex_t* in, complex_t* out) {
    ne10_fft_c2c_1d_float32_neon(
        (ne10_fft_cpx_float32_t*)out,
        (ne10_fft_cpx_float32_t*)in,
        cfg,
        0  // Forward FFT
    );
}
```

**Performance**: ~2-4x faster than scalar C implementation

## Memory Management

### Buffer Strategy

1. **Pre-allocated Buffers**: Avoid malloc/free in processing loop
2. **Ring Buffers**: For sample streaming
3. **Object Pools**: Reuse packet structures

```c
// Example: Packet pool
typedef struct {
    packet_t packets[MAX_PACKETS];
    int free_list[MAX_PACKETS];
    int free_count;
} packet_pool_t;

packet_t* packet_alloc(packet_pool_t* pool) {
    if (pool->free_count == 0) return NULL;
    int idx = pool->free_list[--pool->free_count];
    return &pool->packets[idx];
}
```

### Stack vs Heap

- **Small buffers**: Stack (< 1 KB)
- **Large buffers**: Heap, but pre-allocated
- **Streaming data**: mmap or shared memory

## Threading Model

### Multi-core Utilization (Zynq 7000 dual-core)

**Option 1: Pipeline Parallelism**
```
Core 0: libiio RX → SpectrumCapture → Queue
Core 1: Packet → QPSK → Turbo → Output
```

**Option 2: Worker Pool**
```
Core 0: Main thread, libiio, dispatcher
Core 1: Worker processing packet queue
```

**Synchronization**: Lock-free queues (atomic operations)

## Build System

### CMake Configuration

Target configurations:
1. **ARM Cross-compile**: Zynq 7000 (Cortex-A9)
2. **Native x86_64**: Development and testing
3. **ARM Native**: Compile directly on PlutoSDR (slow)

### Dependencies

**Required**:
- libiio (>= 0.21)
- Ne10 library (or CMSIS-DSP)
- pthreads (POSIX threads)

**Optional**:
- TurboFEC (for turbo decoding)
- libm (math library)

### Compiler Flags

```cmake
# ARM Cortex-A9 with NEON
set(CMAKE_C_FLAGS "-mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -O3 -ffast-math")

# NEON intrinsics
add_definitions(-DHAVE_NEON)

# Link Ne10
target_link_libraries(droneid_receiver NE10 iio pthread m)
```

## Performance Targets

### Baseline: Python on x86_64 @ 3 GHz

- **SpectrumCapture**: ~200 ms / frame
- **Packet processing**: ~50 ms / frame
- **QPSK + Turbo**: ~30 ms / frame
- **Total**: ~280 ms / frame

### Target: C on ARM Cortex-A9 @ 666 MHz

- **SpectrumCapture**: < 100 ms / frame (NEON FFT)
- **Packet processing**: < 30 ms / frame (NEON correlation)
- **QPSK + Turbo**: < 20 ms / frame (TurboFEC SIMD)
- **Total**: < 150 ms / frame

**Goal**: **Real-time processing** (> 7 frames/second)

## Testing Strategy

### Unit Tests

- DSP functions: Compare against NumPy/SciPy outputs
- QPSK decoder: Known test vectors
- Turbo decoder: 3GPP compliance test vectors

### Integration Tests

- Python vs C output comparison on same input files
- Sample file decoding (mini2_sm, mavic_air_2)
- CRC validation rate matching

### Hardware Tests

- PlutoSDR live captures
- Performance profiling (gprof, perf)
- Power consumption measurement

## Development Roadmap

1. **Phase 1**: Core DSP library + Ne10 integration
2. **Phase 2**: SpectrumCapture module
3. **Phase 3**: Packet processing (OFDM, ZC)
4. **Phase 4**: QPSK decoder
5. **Phase 5**: Turbo decoder integration
6. **Phase 6**: libiio interface
7. **Phase 7**: Optimization and profiling
8. **Phase 8**: PlutoSDR deployment and validation

## References

- **Ne10 Library**: https://projectne10.github.io/Ne10/
- **CMSIS-DSP**: https://github.com/ARM-software/CMSIS-DSP
- **libiio**: https://analogdevicesinc.github.io/libiio/
- **TurboFEC**: https://github.com/ttsou/turbofec
- **3GPP TS 36.212**: LTE Turbo Coding specification
- **ARM NEON Programmer's Guide**: https://developer.arm.com/architectures/instruction-sets/simd-isas/neon
