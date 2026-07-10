"""
MCP4728 channel A DAC-code-to-voltage calibration sweep.

Steps MCP4728 channel A from code 0 upward by a fixed increment (default 1),
pauses at each step for you to measure the actual voltage across the load
(a 4 ohm resistor) with a multimeter, prompts for that measured voltage, and
logs code/measured-voltage pairs to a CSV file as it goes. Ctrl+C at any
prompt stops early and keeps everything logged so far.

LDAC (BCM 20 / physical pin 38) is driven low via pigpio so each write
reaches VOUT immediately, same as tests/test_mcp4728.py.

Usage:
    python tests/calibrate_mcp4728.py
    python tests/calibrate_mcp4728.py --increment 16

Output: mcp4728_calibration.csv in the current directory (code, voltage_mv).
"""

import argparse
import csv
import sys

DEFAULT_ADDR = 0x60
MAX_CODE = 4095
LDAC_PIN = 20  # BCM numbering, physical pin 38
CHANNEL = 0    # channel A
CSV_PATH = "mcp4728_calibration.csv"


class Ldac:
    """Drives the LDAC pin via pigpio so DAC writes reach VOUT immediately."""

    def __init__(self, pin):
        import pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.write(pin, 0)  # idle low: outputs track input registers directly
        self._pin = pin

    def close(self):
        self._pi.write(self._pin, 0)
        self._pi.stop()


def multi_write_channel(bus, addr, channel, code):
    """Multi-Write one channel, forcing Vref=Vdd, gain=1x."""
    code = max(0, min(MAX_CODE, code))
    cmd = 0x40 | (channel << 1)
    upper = (0 << 7) | (0 << 5) | (0 << 4) | ((code >> 8) & 0x0F)  # vref=Vdd, gain=1x
    lower = code & 0xFF
    bus.write_i2c_block_data(addr, cmd, [upper, lower])


def fast_write_channel(bus, addr, channel, code, other_codes):
    """Fast Write all 4 channels in one transaction, only changing `channel`."""
    codes = list(other_codes)
    codes[channel] = max(0, min(MAX_CODE, code))
    payload = []
    for c in codes:
        payload.extend([(c >> 8) & 0x0F, c & 0xFF])
    bus.write_i2c_block_data(addr, payload[0], payload[1:])


def prompt_voltage_mv():
    """Ask for the measured voltage in mV; empty input or Ctrl+C/Ctrl+D stops the sweep."""
    try:
        raw = input("  Measured voltage (mV), Enter/Ctrl+C to stop: ").strip()
    except (EOFError, KeyboardInterrupt):
        return None
    if raw == "":
        return None
    try:
        return float(raw)
    except ValueError:
        print("  [!] Not a number, try again.")
        return prompt_voltage_mv()


def main():
    parser = argparse.ArgumentParser(description="MCP4728 channel A DAC calibration sweep")
    parser.add_argument("--increment", type=int, default=1,
                         help="DAC code step size (default: 1)")
    args = parser.parse_args()

    if args.increment < 1:
        print("[FAIL] --increment must be >= 1")
        sys.exit(1)

    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)

    try:
        ldac = Ldac(LDAC_PIN)
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    try:
        bus = smbus2.SMBus(1)
    except Exception as e:
        print(f"[FAIL] Could not open /dev/i2c-1: {e}")
        ldac.close()
        sys.exit(1)

    other_codes = [0, 0, 0, 0]
    multi_write_channel(bus, DEFAULT_ADDR, CHANNEL, 0)

    print(f"=== MCP4728 channel A calibration sweep (increment={args.increment}) ===")
    print(f"Logging to {CSV_PATH}. Measure VOUT across the 4 ohm load at each step.\n")

    csv_file = open(CSV_PATH, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["code", "voltage_mv"])

    code = 0
    try:
        while code <= MAX_CODE:
            fast_write_channel(bus, DEFAULT_ADDR, CHANNEL, code, other_codes)
            print(f"Code {code}/{MAX_CODE}")
            voltage_mv = prompt_voltage_mv()
            if voltage_mv is None:
                print("\nStopped early.")
                break

            csv_writer.writerow([code, voltage_mv])
            csv_file.flush()
            code += args.increment
    finally:
        fast_write_channel(bus, DEFAULT_ADDR, CHANNEL, 0, other_codes)
        csv_file.close()
        ldac.close()
        bus.close()
        print(f"\nDone. Results saved to {CSV_PATH}")


if __name__ == "__main__":
    main()
