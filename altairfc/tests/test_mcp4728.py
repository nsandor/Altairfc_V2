"""
MCP4728 hardware verification script.

Runs directly on the Pi with the MCP4728 quad DAC wired at I2C address 0x60.
Uses smbus2 directly — no driver module required.

Usage:
    python tests/test_mcp4728.py [--bus /dev/i2c-1] [--address 0x60] [--vdd 3.3]
    python tests/test_mcp4728.py --sweep 1              # sweep channel 1, Ctrl+C to stop

Checks performed:
    1. Device responds at the given address
    2. Fast-Write each channel to a known code, then read back the
       input registers via the device's sequential-read protocol and
       verify the code round-trips
    3. Set all channels to mid-scale (Vout ~= Vdd/2) for a voltmeter check

Notes:
    - Channels are 0-indexed here (A=0, B=1, C=2, D=3) to match register order.
    - DAC codes are 12-bit (0-4095). Output is Vdd * code / 4095 in the
      default Vref=Vdd, gain=1x configuration used by this script.
"""

import argparse
import sys
import time

DEFAULT_ADDR = 0x60
DEFAULT_BUS  = 1
MAX_CODE     = 4095

# Fast-Write command: 2 bytes per channel, sent in one block starting at
# channel A. Upper byte bits: 0 0 VREF PD1 PD0 D11 D10 D9, lower byte: D7-D0.
_FAST_WRITE_HEADER = 0x00


def fast_write_all(bus, addr, codes):
    """Write all 4 channels in one transaction via Fast Write mode."""
    if len(codes) != 4:
        raise ValueError("codes must have exactly 4 entries (A, B, C, D)")

    payload = []
    for code in codes:
        code = max(0, min(MAX_CODE, code))
        upper = (code >> 8) & 0x0F      # VREF=0 (Vdd), PD=00 (normal), D11-D8
        lower = code & 0xFF
        payload.extend([upper, lower])

    bus.write_i2c_block_data(addr, payload[0], payload[1:])


def read_input_registers(bus, addr):
    """
    Sequential read of all 4 channels' DAC/EEPROM registers.

    Each channel returns 6 bytes (3 for DAC reg, 3 for EEPROM reg);
    we only care about the DAC register (first 3 bytes of each group).
    Byte layout: [status], [upper: xx VREF PD1 PD0 GX D11 D10 D9], [lower: D7-D0]
    """
    raw = bus.read_i2c_block_data(addr, 0x00, 24)
    codes = []
    for ch in range(4):
        base = ch * 6
        upper = raw[base + 1]
        lower = raw[base + 2]
        code = ((upper & 0x0F) << 8) | lower
        codes.append(code)
    return codes


def check_device_present(bus, addr):
    try:
        bus.read_byte(addr)
        print(f"[OK] Device responds at 0x{addr:02X}")
        return True
    except OSError:
        print(f"[FAIL] No device at 0x{addr:02X} — check wiring and I2C bus")
        return False


def check_roundtrip(bus, addr):
    """Write distinct known codes to each channel and verify readback."""
    test_codes = [512, 1024, 2048, 4095]
    fast_write_all(bus, addr, test_codes)
    time.sleep(0.01)

    readback = read_input_registers(bus, addr)
    ok = readback == test_codes
    for ch, (want, got) in enumerate(zip(test_codes, readback)):
        flag = "OK" if want == got else "FAIL"
        print(f"  [{flag}] CH{chr(ord('A') + ch)}: wrote {want:4d}, read {got:4d}")

    if ok:
        print("[OK] All 4 channels round-tripped correctly")
    else:
        print("[FAIL] One or more channels did not round-trip")
    return ok


def check_midscale(bus, addr, vdd):
    """Set all channels to mid-scale so voltages can be checked with a meter."""
    mid_code = MAX_CODE // 2
    fast_write_all(bus, addr, [mid_code] * 4)
    expected_v = vdd * mid_code / MAX_CODE
    print(f"[INFO] All channels set to code {mid_code} "
          f"(~{expected_v:.3f} V with Vdd={vdd} V, Vref=Vdd, gain=1x)")
    print("       Verify with a multimeter on VOUTA-D if available.")
    return True


def sweep_channel(bus, addr, channel, vdd):
    """Continuously ramp one channel 0 -> max -> 0 until Ctrl+C."""
    codes = [0, 0, 0, 0]
    print(f"Sweeping CH{chr(ord('A') + channel)} at 0x{addr:02X}, Ctrl+C to stop")
    try:
        while True:
            for code in list(range(0, MAX_CODE, 64)) + list(range(MAX_CODE, 0, -64)):
                codes[channel] = code
                fast_write_all(bus, addr, codes)
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        codes[channel] = 0
        fast_write_all(bus, addr, codes)
        print("\nDone")


def main():
    parser = argparse.ArgumentParser(description="MCP4728 hardware verification")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--address", default=hex(DEFAULT_ADDR), help="I2C address (e.g. 0x60)")
    parser.add_argument("--vdd", default=3.3, type=float, help="Supply voltage for Vout estimate")
    parser.add_argument("--sweep", type=int, choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, continuously sweep one channel (0-3 = A-D)")
    args = parser.parse_args()

    addr = int(args.address, 0)

    try:
        import smbus2
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        sys.exit(1)

    if args.sweep is not None:
        sweep_channel(bus, addr, args.sweep, args.vdd)
        bus.close()
        sys.exit(0)

    print(f"=== MCP4728 verification at 0x{addr:02X} on {args.bus} ===\n")

    results = []
    results.append(check_device_present(bus, addr))
    if not results[-1]:
        print("\nDevice not found, aborting.")
        bus.close()
        sys.exit(1)

    results.append(check_roundtrip(bus, addr))
    results.append(check_midscale(bus, addr, args.vdd))

    bus.close()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("MCP4728 verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
