#!/usr/bin/python3
"""
PlutoSDR Configuration Helper
Provides utilities for configuring PlutoSDR for Drone-ID reception
"""

import sys
try:
    import adi
except ImportError:
    print("Warning: pyadi-iio not installed. Install with: pip install pyadi-iio")

class PlutoConfig:
    """Configuration profiles for PlutoSDR"""

    # Preset configurations for different use cases
    PRESETS = {
        "default": {
            "name": "Default (50 MHz)",
            "sample_rate": 50e6,
            "rf_bandwidth": 50e6,
            "buffer_size": int(1.3 * 50e6),
            "gain_mode": "slow_attack",
            "description": "Burst mode, highest quality for short captures"
        },
        "continuous": {
            "name": "Continuous (20 MHz)",
            "sample_rate": 20e6,
            "rf_bandwidth": 20e6,
            "buffer_size": int(1.3 * 20e6),
            "gain_mode": "slow_attack",
            "description": "Continuous streaming mode, USB 2.0 safe"
        },
        "low_bandwidth": {
            "name": "Low Bandwidth (10 MHz)",
            "sample_rate": 10e6,
            "rf_bandwidth": 10e6,
            "buffer_size": int(1.3 * 10e6),
            "gain_mode": "slow_attack",
            "description": "Reduced bandwidth for lower processing requirements"
        },
        "high_gain": {
            "name": "High Gain (50 MHz, 60 dB)",
            "sample_rate": 50e6,
            "rf_bandwidth": 50e6,
            "buffer_size": int(1.3 * 50e6),
            "gain_mode": "manual",
            "gain": 60,
            "description": "Maximum sensitivity for weak signals"
        }
    }

    @staticmethod
    def get_preset(preset_name):
        """Get configuration preset by name"""
        if preset_name not in PlutoConfig.PRESETS:
            raise ValueError(f"Unknown preset: {preset_name}. Available: {list(PlutoConfig.PRESETS.keys())}")
        return PlutoConfig.PRESETS[preset_name].copy()

    @staticmethod
    def list_presets():
        """Print all available presets"""
        print("\nAvailable PlutoSDR Configuration Presets:")
        print("=" * 70)
        for name, config in PlutoConfig.PRESETS.items():
            print(f"\n{name}:")
            print(f"  Name: {config['name']}")
            print(f"  Sample Rate: {config['sample_rate']/1e6:.1f} MHz")
            print(f"  RF Bandwidth: {config['rf_bandwidth']/1e6:.1f} MHz")
            print(f"  Gain Mode: {config['gain_mode']}")
            if 'gain' in config:
                print(f"  Gain: {config['gain']} dB")
            print(f"  Description: {config['description']}")
        print("=" * 70)

    @staticmethod
    def enable_ad9364_mode(sdr):
        """
        Enable AD9364 mode on PlutoSDR for extended frequency range
        This is the "hack" to unlock 70 MHz - 6 GHz operation

        WARNING: This operates the hardware outside official specifications
        """
        try:
            # This tells the firmware to operate as AD9364 instead of AD9363
            sdr._ctrl.debug_attrs["adi,frequency-division-duplex-mode-enable"].value = "1"
            sdr._ctrl.debug_attrs["adi,ensm-enable-txnrx-control-enable"].value = "0"

            # Reboot required for changes to take effect
            print("AD9364 mode enabled. PlutoSDR will reboot...")
            print("After reboot, frequency range: 70 MHz - 6 GHz")
            print("Max bandwidth: 56 MHz")
            return True
        except Exception as e:
            print(f"Error enabling AD9364 mode: {e}")
            return False

    @staticmethod
    def check_pluto_capabilities(sdr):
        """
        Check PlutoSDR capabilities and print information
        """
        try:
            print("\n" + "=" * 70)
            print("PlutoSDR Information")
            print("=" * 70)

            # Get hardware info
            phy = sdr._ctrl.find_device('ad9361-phy')
            print(f"Hardware Model: {phy.attrs['model'].value if 'model' in phy.attrs else 'Unknown'}")

            # Get context attributes
            ctx = sdr._ctx
            if 'fw_version' in ctx.attrs:
                print(f"Firmware Version: {ctx.attrs['fw_version'].value}")
            if 'serial' in ctx.attrs:
                print(f"Serial Number: {ctx.attrs['serial'].value}")
            if 'local,kernel' in ctx.attrs:
                print(f"Kernel Version: {ctx.attrs['local,kernel'].value}")

            # Check frequency range
            try:
                # Try to read the frequency range
                print(f"\nCurrent Configuration:")
                print(f"  RX Sample Rate: {sdr.sample_rate / 1e6:.2f} MHz")
                print(f"  RX RF Bandwidth: {sdr.rx_rf_bandwidth / 1e6:.2f} MHz")
                print(f"  RX LO Frequency: {sdr.rx_lo / 1e6:.2f} MHz")
                print(f"  RX Gain Mode: {sdr.gain_control_mode_chan0}")
                if sdr.gain_control_mode_chan0 == "manual":
                    print(f"  RX Hardware Gain: {sdr.rx_hardwaregain_chan0} dB")
            except Exception as e:
                print(f"Could not read current configuration: {e}")

            print("=" * 70)

        except Exception as e:
            print(f"Error checking PlutoSDR capabilities: {e}")

    @staticmethod
    def validate_frequency(freq_hz, extended_mode=False):
        """
        Validate if frequency is in supported range

        Args:
            freq_hz: Frequency in Hz
            extended_mode: True if AD9364 mode is enabled

        Returns:
            (is_valid, message)
        """
        freq_mhz = freq_hz / 1e6

        if extended_mode:
            # AD9364 mode: 70 MHz - 6 GHz
            if 70 <= freq_mhz <= 6000:
                return True, "OK"
            else:
                return False, f"Frequency {freq_mhz:.2f} MHz outside AD9364 range (70-6000 MHz)"
        else:
            # Standard AD9363 mode: 325 MHz - 3800 MHz
            if 325 <= freq_mhz <= 3800:
                return True, "OK"
            else:
                return False, f"Frequency {freq_mhz:.2f} MHz outside AD9363 range (325-3800 MHz)"

    @staticmethod
    def validate_sample_rate(sample_rate, continuous=False):
        """
        Validate sample rate and check USB 2.0 limitations

        Args:
            sample_rate: Sample rate in Hz
            continuous: True if continuous streaming (USB 2.0 limited)

        Returns:
            (is_valid, message)
        """
        rate_mhz = sample_rate / 1e6

        # Hardware maximum
        if rate_mhz > 61.44:
            return False, f"Sample rate {rate_mhz:.2f} MHz exceeds hardware max (61.44 MHz)"

        # USB 2.0 limitation for continuous streaming
        if continuous and rate_mhz > 5:
            return True, f"WARNING: {rate_mhz:.2f} MHz may exceed USB 2.0 bandwidth for continuous streaming (recommended: <5 MHz)"

        return True, "OK"

    @staticmethod
    def optimize_for_droneid():
        """
        Get optimized settings for Drone-ID reception

        Returns:
            dict with recommended settings
        """
        return {
            "sample_rate": 50e6,  # High sample rate for burst captures
            "rf_bandwidth": 50e6,  # Match sample rate
            "gain_mode": "slow_attack",  # AGC with slow attack
            "buffer_size": int(1.3 * 50e6),  # 1.3 seconds
            "frequencies": [
                # 2.4 GHz band (all supported by AD9363)
                2414.5e6, 2429.5e6, 2434.5e6, 2444.5e6, 2459.5e6, 2474.5e6,
                # 5.8 GHz band (needs extended mode or AD9364)
                # Commented out for standard PlutoSDR - uncomment if using AD9364 mode
                # 5721.5e6, 5731.5e6, 5741.5e6, 5756.5e6, 5761.5e6, 5771.5e6,
                # 5786.5e6, 5801.5e6, 5816.5e6, 5831.5e6
            ],
            "notes": "For 5.8 GHz band, enable AD9364 mode or use hardware AD9364 chip"
        }

def main():
    """Test and configuration utility"""
    import argparse

    parser = argparse.ArgumentParser(description='PlutoSDR Configuration Utility')
    parser.add_argument('--list-presets', action='store_true',
                       help='List all configuration presets')
    parser.add_argument('--check', action='store_true',
                       help='Check PlutoSDR capabilities')
    parser.add_argument('--enable-ad9364', action='store_true',
                       help='Enable AD9364 mode (CAUTION: out of spec)')
    parser.add_argument('--uri', default='ip:192.168.2.1',
                       help='PlutoSDR URI (default: ip:192.168.2.1)')

    args = parser.parse_args()

    if args.list_presets:
        PlutoConfig.list_presets()
        return

    if args.check or args.enable_ad9364:
        try:
            print(f"Connecting to PlutoSDR at {args.uri}...")
            sdr = adi.Pluto(uri=args.uri)
            print("Connected!")

            if args.check:
                PlutoConfig.check_pluto_capabilities(sdr)

            if args.enable_ad9364:
                confirm = input("\nWARNING: Enabling AD9364 mode operates hardware outside specifications.\n"
                              "Continue? (yes/no): ")
                if confirm.lower() == 'yes':
                    PlutoConfig.enable_ad9364_mode(sdr)
                else:
                    print("Cancelled.")

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
