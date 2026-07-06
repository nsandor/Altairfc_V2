"""
MCP4728 hardware verification script.

Runs directly on the Pi with the MCP4728 quad DAC wired at I2C address 0x60.
Uses smbus2 directly — no driver module required.

LDAC (BCM 20 / physical pin 38) must be driven LOW for I2C writes to reach
the analog outputs immediately; while LDAC is HIGH, writes only update the
DAC input registers and VOUT keeps holding its previously latched value.
This script drives LDAC itself via pigpio (see --ldac-pin / --no-ldac).

NOTE: tests/test_led_on.py and tests/test_leds.py previously also drove BCM 20
for an LDD-700LS LED driver. That LED driver has since been removed from the
hardware, so BCM 20 is now dedicated to LDAC.

Usage:
    python tests/test_mcp4728.py [--bus /dev/i2c-1] [--address 0x60] [--vdd 3.3]
    python tests/test_mcp4728.py --sweep 1              # sweep channel 1, Ctrl+C to stop
    python tests/test_mcp4728.py --no-ldac              # don't touch LDAC (e.g. if tied to GND)

Checks performed:
    1. Device responds at the given address
    2. Multi-Write each channel to a known code (forcing Vref=Vdd, gain=1x),
       then read back the input registers via the device's sequential-read
       protocol and verify the code round-trips
    3. Set all channels to mid-scale (Vout ~= Vdd/2) for a voltmeter check,
       and read back Vref/gain/PD flags to confirm the reference actually
       in use

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
DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

# Fast-Write command: 2 bytes per channel. Upper byte is only
# [0][0][D11][D10][D9][D8] — Fast Write has NO Vref/PD/gain bits, so it
# cannot change reference/gain; those stay whatever was last set (by
# Multi-Write, EEPROM, or power-on default).
_FAST_WRITE_HEADER = 0x00

# Multi-Write command byte per channel: 0b01000_AAA_0 where AAA selects
# the channel (0=A..3=D); the trailing bit is UDAC (0 = update immediately).
_MULTIWRITE_CMD = 0x40


class Ldac:
    """
    Drives the LDAC pin via pigpio so writes actually reach VOUT.

    LDAC is active-low: pull it low to latch input registers to the
    analog outputs, leave it high to hold the previous output regardless
    of new I2C writes.
    """

    def __init__(self, pin, enabled):
        self._pin = pin
        self._enabled = enabled
        self._pi = None
        if not enabled:
            return

        import pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.write(pin, 0)  # idle low: outputs track input registers directly
        print(f"[INFO] LDAC (BCM {pin}) held LOW — outputs will update immediately on write")

    def pulse(self):
        """Toggle LDAC high->low to force-latch, for use if it's normally held high elsewhere."""
        if not self._enabled:
            return
        self._pi.write(self._pin, 1)
        time.sleep(0.001)
        self._pi.write(self._pin, 0)

    def close(self):
        if self._enabled and self._pi is not None:
            self._pi.write(self._pin, 0)
            self._pi.stop()


def fast_write_all(bus, addr, codes):
    """
    Write all 4 channels' DAC codes in one transaction via Fast Write mode.

    NOTE: Fast Write cannot set Vref/PD/gain — it only updates the 12-bit
    code. Use multi_write_all() at least once first to force Vref=Vdd,
    gain=1x, otherwise VOUT follows whatever reference was last configured
    (e.g. internal 2.048V ref from EEPROM/power-on default).
    """
    if len(codes) != 4:
        raise ValueError("codes must have exactly 4 entries (A, B, C, D)")

    payload = []
    for code in codes:
        code = max(0, min(MAX_CODE, code))
        upper = (code >> 8) & 0x0F      # D11-D8 only; no Vref/PD/gain bits exist here
        lower = code & 0xFF
        payload.extend([upper, lower])

    bus.write_i2c_block_data(addr, payload[0], payload[1:])


def multi_write_all(bus, addr, codes, vref_vdd=True, gain=1):
    """
    Write all 4 channels via Multi-Write, explicitly setting Vref/PD/gain.

    Unlike Fast Write, this can force Vref=Vdd (vref_vdd=True) so VOUT
    tracks the supply rail instead of the internal 2.048V reference.
    gain only applies when using the internal reference (vref_vdd=False);
    it is ignored by the hardware when Vref=Vdd.
    """
    if len(codes) != 4:
        raise ValueError("codes must have exactly 4 entries (A, B, C, D)")

    vref_bit = 0 if vref_vdd else 1
    gain_bit = 1 if gain == 2 else 0

    for ch, code in enumerate(codes):
        code = max(0, min(MAX_CODE, code))
        cmd   = _MULTIWRITE_CMD | (ch << 1)
        upper = (vref_bit << 7) | (0 << 5) | (gain_bit << 4) | ((code >> 8) & 0x0F)
        lower = code & 0xFF
        bus.write_i2c_block_data(addr, cmd, [upper, lower])


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


def read_channel_flags(bus, addr):
    """
    Decode VREF/PD/GAIN bits from the DAC (input) register of each channel,
    to diagnose whether Fast Write actually selected Vdd vs internal Vref.

    Upper byte of the DAC register: [RDY/BSY][POR][x][x][VREF][PD1][PD0][GX]
    Note: DAC register upper byte layout (read form) differs slightly from
    the write-form byte — bit 7/6 are RDY/BSY and POR flags when reading.
    """
    raw = bus.read_i2c_block_data(addr, 0x00, 24)
    flags = []
    for ch in range(4):
        base = ch * 6
        upper = raw[base + 1]
        vref = (upper >> 3) & 0x01
        pd   = (upper >> 1) & 0x03
        gain = upper & 0x01
        flags.append({"vref": vref, "pd": pd, "gain": gain})
    return flags


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
    # Multi-Write (not Fast Write) so Vref=Vdd/gain=1x is actually forced,
    # rather than leaving whatever reference was last configured.
    multi_write_all(bus, addr, test_codes, vref_vdd=True, gain=1)
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
    multi_write_all(bus, addr, [mid_code] * 4, vref_vdd=True, gain=1)
    expected_v = vdd * mid_code / MAX_CODE
    print(f"[INFO] All channels set to code {mid_code} "
          f"(~{expected_v:.3f} V with Vdd={vdd} V, Vref=Vdd, gain=1x)")
    print("       Verify with a multimeter on VOUTA-D if available.")

    flags = read_channel_flags(bus, addr)
    for ch, f in enumerate(flags):
        ref_name  = "internal (2.048V)" if f["vref"] else "Vdd"
        gain_name = "2x" if (f["vref"] and f["gain"]) else "1x"
        pd_name   = "normal" if f["pd"] == 0 else f"powered-down (mode {f['pd']})"
        print(f"  CH{chr(ord('A') + ch)}: Vref={ref_name}, gain={gain_name}, {pd_name}")
        if f["vref"]:
            print(f"       [WARN] Requested Vdd reference but the chip reports "
                  f"internal reference — measured VOUT will follow 2.048V x gain, not Vdd.")
    return True


def sweep_channel(bus, addr, channel, vdd):
    """Continuously ramp one channel 0 -> max -> 0 until Ctrl+C."""
    codes = [0, 0, 0, 0]
    # Force Vref=Vdd/gain=1x once via Multi-Write so the sweep's voltage
    # actually tracks Vdd; Fast Write below only updates the 12-bit code.
    multi_write_all(bus, addr, codes, vref_vdd=True, gain=1)
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


def _open_ldac(args):
    try:
        return Ldac(args.ldac_pin, enabled=not args.no_ldac)
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="MCP4728 hardware verification")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--address", default=hex(DEFAULT_ADDR), help="I2C address (e.g. 0x60)")
    parser.add_argument("--vdd", default=3.3, type=float, help="Supply voltage for Vout estimate")
    parser.add_argument("--sweep", type=int, choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, continuously sweep one channel (0-3 = A-D)")
    parser.add_argument("--ldac-pin", type=int, default=DEFAULT_LDAC_PIN,
                         help="BCM pin driving LDAC (default: 20 / physical pin 38)")
    parser.add_argument("--no-ldac", action="store_true",
                         help="Don't drive LDAC (use if it's hardwired to GND)")
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

    ldac = _open_ldac(args)

    if args.sweep is not None:
        sweep_channel(bus, addr, args.sweep, args.vdd)
        ldac.close()
        bus.close()
        sys.exit(0)

    print(f"=== MCP4728 verification at 0x{addr:02X} on {args.bus} ===\n")

    results = []
    results.append(check_device_present(bus, addr))
    if not results[-1]:
        print("\nDevice not found, aborting.")
        ldac.close()
        bus.close()
        sys.exit(1)

    results.append(check_roundtrip(bus, addr))
    results.append(check_midscale(bus, addr, args.vdd))

    ldac.close()
    bus.close()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("MCP4728 verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
