#!/usr/bin/python3
"""
Drone-ID Receiver for PlutoSDR using pyadi-iio
This is a port of the UHD-based receiver to work with Analog Devices PlutoSDR
Supports both standard PlutoSDR and AD9364-mode (extended frequency range)
"""

import queue
import numpy as np
import signal
import SpectrumCapture as SC
from Packet import Packet
from qpsk import Decoder
from droneid_packet import DroneIDPacket
from datetime import datetime
import argparse
import matplotlib.pyplot as plt
import threading
import multiprocessing as mp
import time
import sys

try:
    import adi
except ImportError:
    print("Error: pyadi-iio not installed. Install with: pip install pyadi-iio")
    sys.exit(1)

import warnings
warnings.filterwarnings("ignore")

# Global variables
queue = mp.Queue()
exit_event = threading.Event()
sdr = None
db_filename = None
sample_rate = None
args = None
coords = []
lat_list = []
lon_list = []
raw_droneid_bits = []
fixed_runs = 0
c_freq = 0
num_decoded = 0
interesting_freq = 0
crc_err = 0
correct_pkt = 0
total_num_pkt = 0
recv_thread = None
worker = None

def signal_handler(sig, frame):
    global exit_event
    exit_event.set()

def clean_up():
    global exit_event, recv_thread, workers
    # Stop Stream
    print("\n\n######### Stopping Threads, please wait #########\n\n")
    while recv_thread.is_alive():
        recv_thread.join(timeout=10)

    print("Receiver stopped")

    for worker in workers:
        print("Send stop message to thread:", worker.name)
        queue.put((None, None))

def decoded_to_file(raw_bits):
    if len(raw_bits) > 0:
        with open(db_filename, "ab") as fd:
            fd.write(raw_bits)

def set_sdr(sdr, sample_rate=50e6, gain=None, bandwidth=None):
    """
    Configure PlutoSDR for reception

    Args:
        sdr: adi.Pluto() object
        sample_rate: Sample rate in Hz (default 50 MHz, will be limited by USB/hardware)
        gain: RX gain in dB (None for AGC)
        bandwidth: RF bandwidth in Hz (None for auto)

    Returns:
        Configured SDR ready for reception
    """
    try:
        # Set sample rate - PlutoSDR can handle up to 61.44 MSPS
        # but USB 2.0 limits continuous streaming to ~4-5 MHz
        # For burst captures (our use case), higher rates work
        sdr.sample_rate = int(sample_rate)

        # Configure gain control
        if gain is not None:
            sdr.gain_control_mode_chan0 = "manual"
            sdr.rx_hardwaregain_chan0 = float(gain)
        else:
            # Use AGC (Automatic Gain Control)
            sdr.gain_control_mode_chan0 = "slow_attack"

        # Set RF bandwidth (default to same as sample rate if not specified)
        if bandwidth is not None:
            sdr.rx_rf_bandwidth = int(bandwidth)
        else:
            # Auto-bandwidth based on sample rate
            sdr.rx_rf_bandwidth = int(sample_rate)

        # Set buffer size for reception
        sdr.rx_buffer_size = int(sample_rate * 1.3)  # 1.3 seconds of data

        # Enable RX channel
        sdr.rx_enabled_channels = [0]

        print(f"PlutoSDR configured:")
        print(f"  Sample rate: {sdr.sample_rate / 1e6:.2f} MHz")
        print(f"  RF bandwidth: {sdr.rx_rf_bandwidth / 1e6:.2f} MHz")
        print(f"  Gain mode: {sdr.gain_control_mode_chan0}")
        if gain is not None:
            print(f"  RX gain: {sdr.rx_hardwaregain_chan0} dB")
        print(f"  Buffer size: {sdr.rx_buffer_size} samples")

        return sdr

    except Exception as e:
        print(f"Error configuring PlutoSDR: {e}")
        raise

def run_demod(samples, Fs, debug=False, legacy=False):
    global correct_pkt, crc_err, total_num_pkt
    chunk_samples = int(500e-3 * Fs)  # in seconds
    found = False

    chunks = len(samples) // chunk_samples

    for i in range(chunks):
        capture = SC.SpectrumCapture(
            raw_data=samples[i*chunk_samples:(i+1)*chunk_samples],
            Fs=Fs,
            debug=debug,
            p_type=args.packettype,
            legacy=legacy
        )
        if debug:
            print("Found %i Drone-ID RF frames in spectrum capture." % len(capture.packets))

        total_num_pkt += len(capture.packets)
        for packet_num, _ in enumerate(capture.packets):

            with open("ext_drone_id_" + str(sample_rate), "ab") as f:
                f.write(_)

            # get a Drone ID frame, resampled and with coarse center frequency correction.
            packet_data = capture.get_packet_samples(pktnum=packet_num, debug=debug)

            try:
                packet = Packet(packet_data, debug=debug, legacy=legacy)
            except:
                if debug:
                    print("Could not decode packet.")
                continue

            # perform RF corrections, OFDM and stuff
            symbols = packet.get_symbol_data(skip_zc=True)
            decoder = Decoder(symbols)

            # brute force QPSK alignment
            for phase_corr in range(4):
                decoder.raw_data_to_symbol_bits(phase_corr)
                droneid_duml = decoder.magic()
                if not droneid_duml:
                    # decoding failed
                    continue

                # save bits to file
                decoded_to_file(droneid_duml)

                try:
                    payload = DroneIDPacket(droneid_duml)
                except:
                    print("error decoding packet")
                    continue
                print(payload)
                found = True

                if not payload.check_crc():
                    # CRC check failed
                    crc_err += 1
                    continue
                correct_pkt += 1
                break

    return found

def receive_samples(sdr, num_samps):
    """
    Receive samples from PlutoSDR

    Args:
        sdr: Configured adi.Pluto() object
        num_samps: Number of samples to receive

    Returns:
        numpy array of complex samples or None on error
    """
    try:
        # Receive samples - pyadi-iio handles buffering internally
        samples = sdr.rx()

        # Ensure we have the right amount of samples
        if len(samples) < num_samps:
            # Pad with zeros if we got less samples
            samples = np.pad(samples, (0, int(num_samps - len(samples))), 'constant')
        elif len(samples) > num_samps:
            # Truncate if we got more
            samples = samples[:int(num_samps)]

        return samples

    except Exception as e:
        print(f"Error receiving samples: {e}")
        return None

def receive_thread(sdr, sample_rate, duration, gain, queue):
    """
    Thread for receiving samples and frequency hopping
    """
    global interesting_freq

    # DJI OcuSync 2.0 frequencies (in MHz)
    # 2.4 GHz and 5.8 GHz ISM bands
    frequencies = [
        2414.5, 2429.502441, 2434.5, 2444.5, 2459.5, 2474.5,  # 2.4 GHz
        5721.5, 5731.5, 5741.5, 5756.5, 5761.5, 5771.5,       # 5.8 GHz
        5786.5, 5801.5, 5816.5, 5831.5                        # 5.8 GHz
    ]

    num_samps = int(duration * sample_rate)

    # Configure SDR
    try:
        set_sdr(sdr, sample_rate, gain)
    except Exception as e:
        print(f"Failed to configure SDR: {e}")
        return

    while True:
        for c_freq in frequencies:
            c_freq_hz = c_freq * 1e6

            # Frequency locking: if we found a drone, stay on that frequency
            if interesting_freq == 0:
                cnt_freq = c_freq_hz
            else:
                cnt_freq = interesting_freq

            # Set center frequency
            try:
                sdr.rx_lo = int(cnt_freq)
                print(f"Center Freq: {cnt_freq/1e6:.2f} MHz @ {sample_rate/1e6:.2f} MSPS")
            except Exception as e:
                print(f"Unable to set center freq: {e}")
                continue

            # Receive samples
            samples = receive_samples(sdr, num_samps)
            if samples is None:
                continue

            # Convert to complex64 if needed
            samples = samples.astype(np.complex64)

            # Put samples in queue for processing
            queue.put((samples.copy(), cnt_freq))

            if exit_event.is_set():
                break

        if exit_event.is_set():
            print("Receiver Thread: Stopped")
            break

def process_samples(sample_rate, queue):
    """
    Thread for processing received samples
    """
    global interesting_freq, fixed_runs

    while True:
        samples, cnt_freq = queue.get()

        if samples is None and cnt_freq is None:
            break

        # Save raw samples for debugging
        with open("receive_test_pluto.raw", 'ab') as f:
            f.write(samples)

        # Run demodulation
        if run_demod(samples, sample_rate, debug=args.debug, legacy=args.legacy):
            interesting_freq = cnt_freq
            print(f"Locking Frequency to {cnt_freq/1e6:.2f} MHz")
            fixed_runs = 0
        else:
            fixed_runs += 1

        # If we haven't found anything in 10 runs, unlock frequency
        if fixed_runs > 10:
            interesting_freq = 0
            fixed_runs = 0

        if exit_event.is_set():
            print("Process Thread: Stopped")
            break

def main():
    global db_filename, sdr, recv_thread, args, workers

    parser = argparse.ArgumentParser(
        description='Drone-ID Receiver for PlutoSDR',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default settings (AGC, 50 MHz sample rate)
  ./droneid_receiver_pluto.py

  # Run with manual gain control
  ./droneid_receiver_pluto.py -g 30

  # Run with reduced sample rate for continuous streaming
  ./droneid_receiver_pluto.py -s 20e6

  # Connect to remote PlutoSDR
  ./droneid_receiver_pluto.py --uri ip:192.168.2.1

  # Enable debug output and legacy drone support
  ./droneid_receiver_pluto.py -d -l
        """
    )

    parser.add_argument('-g', '--gain', default=None, type=float,
                       help="RX gain in dB (default: AGC)")
    parser.add_argument('-s', '--sample_rate', default=50e6, type=float,
                       help="Sample rate in Hz (default: 50 MHz)")
    parser.add_argument('-w', '--workers', default=2, type=int,
                       help="Number of worker threads for processing")
    parser.add_argument('-l', '--legacy', default=False, action="store_true",
                       help="Support legacy drones (Mavic Pro, Mavic 2)")
    parser.add_argument('-d', '--debug', default=False, action="store_true",
                       help="Enable debug output")
    parser.add_argument('-t', '--duration', default=1.3, type=float,
                       help="Time of receiving samples per band (seconds)")
    parser.add_argument('-p', '--packettype', default="droneid", type=str,
                       help="Packet type: droneid, c2, beacon, video")
    parser.add_argument('--uri', default="ip:192.168.2.1", type=str,
                       help="PlutoSDR URI (default: ip:192.168.2.1)")
    parser.add_argument('--buffer-size', default=None, type=int,
                       help="RX buffer size in samples (default: auto)")

    args = parser.parse_args()

    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize PlutoSDR
    print(f"Connecting to PlutoSDR at {args.uri}...")
    try:
        sdr = adi.Pluto(uri=args.uri)
        print(f"Connected to PlutoSDR")

        # Try to get device info if available
        try:
            if hasattr(sdr, '_ctx') and sdr._ctx:
                ctx_attrs = sdr._ctx.attrs
                if 'fw_version' in ctx_attrs:
                    print(f"  Firmware: {ctx_attrs['fw_version'].value}")
                if 'hw_model' in ctx_attrs:
                    print(f"  Hardware: {ctx_attrs['hw_model'].value}")
        except:
            # If we can't get device info, just continue
            pass

    except Exception as e:
        print(f"Error connecting to PlutoSDR: {e}")
        print("Make sure PlutoSDR is connected and accessible at the specified URI")
        sys.exit(1)

    duration = args.duration
    sample_rate = args.sample_rate

    # Generate timestamped filename for decoded bits
    dt = datetime.now()
    db_filename = f"decoded_bits_{dt.day:02d}{dt.month:02d}_{dt.hour:02d}{dt.minute:02d}.bin"

    # Start receiving thread
    print("Start receiving...")
    recv_thread = threading.Thread(
        target=receive_thread,
        args=(sdr, sample_rate, duration, args.gain, queue)
    )
    recv_thread.start()

    # Start worker threads for processing
    num_workers = args.workers
    workers = []
    for i in range(num_workers):
        proc_thread = mp.Process(
            target=process_samples,
            args=(sample_rate, queue)
        )
        proc_thread.start()
        workers.append(proc_thread)

    # Main loop
    while True:
        if exit_event.is_set():
            clean_up()
            exit_event.clear()

        workers_alive = 0
        for worker in workers:
            if worker.is_alive():
                workers_alive += 1

        if workers_alive == 0:
            print("No more workers alive!\nExiting...")
            break

    # Print statistics
    print(f"\n\nSuccessfully decoded {correct_pkt} / {total_num_pkt} packets")
    print(f"{crc_err} Packets with CRC error")

if __name__ == "__main__":
    main()
