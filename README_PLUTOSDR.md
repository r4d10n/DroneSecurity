# PlutoSDR Support for Drone-ID Receiver

This document describes how to use the Drone-ID receiver with Analog Devices PlutoSDR.

## Overview

The PlutoSDR port enables reception of DJI Drone-ID signals using the affordable ADALM-PLUTO SDR. This implementation uses `pyadi-iio` instead of UHD, making it accessible to users with PlutoSDR hardware.

## Hardware Requirements

### PlutoSDR Specifications

- **Model**: ADALM-PLUTO (AD9363 or AD9364)
- **Frequency Range**:
  - Standard (AD9363): 325 MHz - 3.8 GHz
  - Extended (AD9364 mode): 70 MHz - 6 GHz
- **Bandwidth**: Up to 20 MHz (official), 56 MHz (with modifications)
- **Sample Rate**: Up to 61.44 MSPS
- **Interface**: USB 2.0
- **Processor**: Zynq 7000 (ARM Cortex-A9 + FPGA)

### Frequency Band Support

**2.4 GHz Band** (Fully Supported):
- All frequencies from 2414.5 MHz to 2474.5 MHz
- Works with standard PlutoSDR (AD9363)

**5.8 GHz Band** (Requires Extended Mode):
- Frequencies from 5721.5 MHz to 5831.5 MHz
- Requires AD9364 mode or hardware AD9364 chip
- Out of specification for AD9363

## Installation

### 1. Install System Dependencies

For Debian/Ubuntu:
```bash
sudo apt install libiio-utils libiio-dev
```

### 2. Install Python Dependencies

```bash
# Install all requirements including pyadi-iio
pip3 install -r requirements.txt
```

### 3. Verify PlutoSDR Connection

Connect your PlutoSDR via USB and verify it's detected:

```bash
# List IIO devices
iio_info -s

# Expected output:
# Library version: 0.x (git tag: ...)
# Available contexts:
#   0: 0456:b673 (Analog Devices Inc. PlutoSDR (ADALM-PLUTO)), serial=...
```

### 4. Configure PlutoSDR Network

By default, PlutoSDR creates a network interface at `192.168.2.1`:

```bash
# Test connectivity
ping 192.168.2.1

# SSH access (password: analog)
ssh root@192.168.2.1
```

## Quick Start

### Running the PlutoSDR Receiver

Basic usage with automatic gain control:

```bash
./src/droneid_receiver_pluto.py
```

### Common Options

```bash
# With manual gain control (30 dB)
./src/droneid_receiver_pluto.py -g 30

# With reduced sample rate for continuous streaming
./src/droneid_receiver_pluto.py -s 20e6

# Enable debug output
./src/droneid_receiver_pluto.py -d

# Support legacy drones (Mavic Pro, Mavic 2)
./src/droneid_receiver_pluto.py -l

# Specify custom PlutoSDR IP
./src/droneid_receiver_pluto.py --uri ip:192.168.2.1

# Combine multiple options
./src/droneid_receiver_pluto.py -g 40 -s 50e6 -d
```

### All Available Options

```
-g, --gain GAIN          RX gain in dB (default: AGC)
-s, --sample_rate RATE   Sample rate in Hz (default: 50 MHz)
-w, --workers NUM        Number of worker threads (default: 2)
-l, --legacy             Support legacy drones
-d, --debug              Enable debug output
-t, --duration SECS      Capture duration per band (default: 1.3s)
-p, --packettype TYPE    Packet type: droneid, c2, beacon, video
--uri URI                PlutoSDR URI (default: ip:192.168.2.1)
--buffer-size SIZE       RX buffer size in samples (default: auto)
```

## Configuration Utility

The `pluto_config.py` utility helps configure and test your PlutoSDR:

### List Available Presets

```bash
./src/pluto_config.py --list-presets
```

Available presets:
- **default**: 50 MHz burst mode (highest quality)
- **continuous**: 20 MHz continuous streaming
- **low_bandwidth**: 10 MHz for reduced processing
- **high_gain**: 50 MHz with 60 dB gain for weak signals

### Check PlutoSDR Capabilities

```bash
./src/pluto_config.py --check --uri ip:192.168.2.1
```

Output includes:
- Hardware model (AD9363/AD9364)
- Firmware version
- Current configuration
- Serial number

### Enable AD9364 Mode (Extended Range)

**WARNING**: This operates the hardware outside official specifications.

```bash
./src/pluto_config.py --enable-ad9364 --uri ip:192.168.2.1
```

This enables:
- Extended frequency range: 70 MHz - 6 GHz
- Higher bandwidth: up to 56 MHz
- 5.8 GHz band support for Drone-ID

## Sample Rate Considerations

### Burst Mode (Default: 50 MHz)

The receiver captures in bursts (1.3 seconds per frequency), allowing high sample rates:

**Advantages**:
- Better signal quality
- Higher bandwidth coverage
- Compatible with original signal processing

**Limitations**:
- USB 2.0 buffer may fill during capture
- Works well for burst captures but not continuous streaming

### Continuous Mode (20 MHz)

For continuous operation without dropped samples:

```bash
./src/droneid_receiver_pluto.py -s 20e6
```

**Advantages**:
- Reliable continuous streaming
- USB 2.0 safe
- Lower CPU usage

**Limitations**:
- Reduced bandwidth
- May miss weak signals

## Performance Optimization

### USB Buffer Tuning

Increase USB buffer size for better performance:

```bash
# On host system
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### CPU Governor

Set CPU to performance mode:

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Multi-threading

Adjust worker threads based on CPU cores:

```bash
# For quad-core CPU
./src/droneid_receiver_pluto.py -w 4
```

## Troubleshooting

### Connection Issues

**Problem**: "Error connecting to PlutoSDR"

**Solutions**:
1. Check USB connection: `lsusb | grep Analog`
2. Verify network: `ping 192.168.2.1`
3. Check IIO devices: `iio_info -s`
4. Try different URI: `--uri usb:1.2.5` (adjust based on lsusb)

### Sample Loss

**Problem**: "ERROR_CODE_TIMEOUT" or dropped samples

**Solutions**:
1. Reduce sample rate: `-s 20e6`
2. Increase USB buffer size (see USB Buffer Tuning)
3. Reduce capture duration: `-t 1.0`
4. Close other USB-intensive applications

### 5.8 GHz Not Working

**Problem**: Cannot tune to 5.8 GHz frequencies

**Solutions**:
1. Enable AD9364 mode: `./src/pluto_config.py --enable-ad9364`
2. Verify frequency range: `./src/pluto_config.py --check`
3. Update PlutoSDR firmware

### Low Signal Quality

**Problem**: Many CRC errors or no packets decoded

**Solutions**:
1. Increase gain: `-g 60`
2. Switch to manual gain: `-g 40` instead of AGC
3. Improve antenna connection
4. Use directional antenna
5. Reduce distance to drone

## Comparison: PlutoSDR vs USRP B205

| Feature | PlutoSDR | USRP B205 |
|---------|----------|-----------|
| **Price** | ~$150 | ~$700 |
| **Frequency Range** | 325-3800 MHz (70-6000 MHz extended) | 70 MHz - 6 GHz |
| **Bandwidth** | 20 MHz (56 MHz extended) | 56 MHz |
| **Sample Rate** | 61.44 MSPS | 61.44 MSPS |
| **Interface** | USB 2.0 | USB 3.0 |
| **Streaming** | ~5 MHz continuous | 56 MHz continuous |
| **Burst Capture** | Up to 61 MSPS | Up to 61 MSPS |
| **Processing** | Zynq 7000 onboard | External PC |
| **Portability** | Very portable | Portable |
| **Power** | USB powered | USB powered |

**Recommendation**: PlutoSDR is excellent for this application due to burst capture nature and 2.4 GHz focus.

## Advanced: Running on PlutoSDR's ARM Processor

The PlutoSDR contains a Zynq 7000 SoC with dual ARM Cortex-A9 cores. For maximum efficiency, see the C implementation documentation for running the decoder directly on the PlutoSDR's processor.

Benefits:
- No USB bottleneck
- Lower latency
- Standalone operation
- NEON SIMD acceleration

See: `README_C_IMPLEMENTATION.md` for details.

## Known Limitations

1. **USB 2.0 Bandwidth**: Continuous streaming limited to ~5 MHz sample rate
2. **5.8 GHz Band**: Requires AD9364 mode or hardware AD9364 chip
3. **Buffer Size**: Large buffers may cause USB timeout issues
4. **Processing Power**: May need powerful host PC for 50 MHz real-time processing

## References

- [PlutoSDR Official Documentation](https://wiki.analog.com/university/tools/pluto)
- [pyadi-iio Documentation](https://analogdevicesinc.github.io/pyadi-iio/)
- [PlutoSDR Modifications](https://wiki.analog.com/university/tools/pluto/users/customizing)

## Support

For PlutoSDR-specific issues:
1. Check PlutoSDR wiki: https://wiki.analog.com/university/tools/pluto
2. Analog Devices forums: https://ez.analog.com/
3. This project's issues: https://github.com/r4d10n/DroneSecurity/issues

For general Drone-ID receiver questions, see main README.md.
