"""
Continuous live monitor for the photodiode TIA + thermistor bridge ADCs.

Uses the compiled driver (drivers/ads1220_driver.py + libads1220_driver.so)
rather than talking SPI/pigpio directly — build it first:
    bash altairfc/drivers/build_ads1220.sh
(requires libgpiod headers: sudo apt install -y libgpiod-dev)

Two identical ADS1220 breakouts, each with both a photodiode TIA input
(AIN0) and a thermistor Wheatstone bridge input (AIN2+/AIN3-):

    ADC1: CS = GPIO17
    ADC2: CS = GPIO4

Prints a continuously-updating line per board: photodiode voltage, bridge
differential voltage, derived thermistor resistance, and derived
temperature in Celsius. Ctrl+C to stop.

Usage:
    python tests/monitor_photodiode_adc.py
    python tests/monitor_photodiode_adc.py --interval 0.5
    python tests/monitor_photodiode_adc.py --only 1
    python tests/monitor_photodiode_adc.py --csv log.csv     # also append each row to a CSV file
"""

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.ads1220_driver import Ads1220Driver  # noqa: E402


def open_boards(only, spi_dev, gpiochip, cs1, cs2):
    boards = []
    if only in (None, 1):
        boards.append((f"ADC1(CS=GPIO{cs1})", Ads1220Driver(spi_dev, gpiochip, cs1)))
    if only in (None, 2):
        boards.append((f"ADC2(CS=GPIO{cs2})", Ads1220Driver(spi_dev, gpiochip, cs2)))
    return boards


def format_row(name, pd_v, bridge):
    if pd_v is None or bridge is None:
        return f"{name}: [READ ERROR]"
    return (f"{name}: PD={pd_v:+.6f} V  Bridge={bridge.volts:+.6f} V  "
            f"TH1={bridge.resistance_ohm:8.1f} ohm  T={bridge.temperature_c:6.2f} C")


def main():
    parser = argparse.ArgumentParser(description="Continuous photodiode + thermistor bridge monitor")
    parser.add_argument("--spi-dev", default="/dev/spidev0.0", help="SPI device node (shared by both boards)")
    parser.add_argument("--gpiochip", default="gpiochip0", help="gpiod chip name for CS lines")
    parser.add_argument("--cs1", type=int, default=17, help="BCM pin for ADC1 CS")
    parser.add_argument("--cs2", type=int, default=4, help="BCM pin for ADC2 CS")
    parser.add_argument("--only", type=int, choices=[1, 2], default=None,
                         help="Monitor only ADC1 (--cs1) or only ADC2 (--cs2)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between samples")
    parser.add_argument("--csv", default=None, help="Optional path to append CSV rows to")
    args = parser.parse_args()

    try:
        boards = open_boards(args.only, args.spi_dev, args.gpiochip, args.cs1, args.cs2)
    except OSError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    csv_writer = None
    csv_file = None
    if args.csv:
        is_new = not Path(args.csv).exists()
        csv_file = open(args.csv, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if is_new:
            header = ["timestamp"]
            for name, _ in boards:
                header += [f"{name}_pd_v", f"{name}_bridge_v", f"{name}_th1_ohm", f"{name}_temp_c"]
            csv_writer.writerow(header)

    print(f"Monitoring {len(boards)} board(s), interval={args.interval}s, Ctrl+C to stop\n")
    try:
        while True:
            ts = time.time()
            row_parts = []
            csv_row = [f"{ts:.3f}"]

            for name, adc in boards:
                pd_v = adc.read_photodiode()
                bridge = adc.read_bridge()
                row_parts.append(format_row(name, pd_v, bridge))

                if bridge is not None and pd_v is not None:
                    csv_row += [f"{pd_v:.6f}", f"{bridge.volts:.6f}",
                                f"{bridge.resistance_ohm:.1f}", f"{bridge.temperature_c:.2f}"]
                else:
                    csv_row += ["", "", "", ""]

            print("   |   ".join(row_parts))

            if csv_writer:
                csv_writer.writerow(csv_row)
                csv_file.flush()

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for _, adc in boards:
            adc.close()
        if csv_file:
            csv_file.close()
        print("Done")


if __name__ == "__main__":
    main()
