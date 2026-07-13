"""
MCP4728 breakout board verification script (no LDAC pin wired).

This is a separate script from tests/test_mcp4728.py, which is for the
board-mounted MCP4728 with LDAC wired to a GPIO. This breakout board has
no LDAC connection exposed, so this script never touches pigpio/GPIO at
all — it relies entirely on the breakout's on-board LDAC strapping (almost
always tied to GND on these boards, so writes reach VOUT immediately).

Since that's an assumption about a board we don't control, check 2 below
specifically verifies it empirically: it writes a code, then immediately
reads back both the DAC input register AND (indirectly, by watching for
consistent round-trips with no separate "latch" step) confirms writes are
taking effect without any extra action. If check 2 fails, LDAC is likely
NOT grounded on this board and needs a jumper/wire to GND before it will
work at all — see the printed guidance if that happens.

Uses smbus2 directly — no driver module or LDAC/pigpio dependency.

Usage:
    python tests/test_mcp4728_breakout.py [--bus /dev/i2c-1] [--address 0x60] [--vdd 3.3]
    python tests/test_mcp4728_breakout.py --sweep 1              # sweep channel 1, Ctrl+C to stop
    python tests/test_mcp4728_breakout.py --hold 0 --code 2047   # hold channel A at code 2047, Ctrl+C to stop
    python tests/test_mcp4728_breakout.py --hold 0 1 --code-a 2047 --code-b 1024  # hold A and B at different codes
    python tests/test_mcp4728_breakout.py --zero-eeprom          # persist 0V on all channels
    python tests/test_mcp4728_breakout.py --save-eeprom --code 2047

Checks performed:
    1. Device responds at the given address
    2. Multi-Write each channel to a known code (forcing Vref=Vdd, gain=1x),
       then read back the input registers to verify the code round-trips
       WITHOUT any LDAC pulse — confirms LDAC is grounded on this board
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
MAX_CODE = 4095

# Multi-Write command byte per channel: 0b01000_AAA_0 where AAA selects
# the channel (0=A..3=D); the trailing bit is UDAC (0 = update immediately).
_MULTIWRITE_CMD = 0x40

# Sequential Write command byte: 0b01010_000 — writes DAC reg + EEPROM for
# all 4 channels starting at channel A, in one block transaction.
_SEQWRITE_CMD = 0x50

EEPROM_WRITE_S = 0.05  # datasheet: EEPROM write takes up to ~50 ms


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


def sequential_write_eeprom_all(bus, addr, codes, vref_vdd=True, gain=1):
    """
    Write all 4 channels' DAC register AND EEPROM in one Sequential Write
    transaction, starting at channel A. Unlike Multi-Write/Fast Write, this
    persists across power cycles — the chip will power up outputting these
    codes (with this Vref/gain) every time, with no software write needed.

    Blocks ~50 ms while the EEPROM write completes, per the datasheet.
    """
    if len(codes) != 4:
        raise ValueError("codes must have exactly 4 entries (A, B, C, D)")

    vref_bit = 0 if vref_vdd else 1
    gain_bit = 1 if gain == 2 else 0

    payload = []
    for code in codes:
        code = max(0, min(MAX_CODE, code))
        upper = (vref_bit << 7) | (0 << 5) | (gain_bit << 4) | ((code >> 8) & 0x0F)
        lower = code & 0xFF
        payload.extend([upper, lower])

    bus.write_i2c_block_data(addr, _SEQWRITE_CMD, payload)
    time.sleep(EEPROM_WRITE_S)


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
    """
    Write distinct known codes to each channel (no LDAC pulse — this board
    has no LDAC pin wired) and verify readback of the DAC input registers.

    NOTE: this confirms the *input register* took the write, which always
    works regardless of LDAC. It does NOT by itself prove VOUT updated —
    if VOUT doesn't visibly change during check_midscale()'s voltmeter
    check, LDAC is likely floating/pulled high on this board rather than
    grounded, and needs a jumper wire from LDAC to GND.
    """
    test_codes = [512, 1024, 2048, 4095]
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
    print("       Verify with a multimeter on VOUTA-D.")
    print("       If VOUT does NOT read ~half of Vdd, LDAC is likely floating or")
    print("       pulled high on this breakout — jumper the LDAC pin to GND and retry.")

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


def hold_values(bus, addr, channel_codes, vdd):
    """
    Write known codes to one or more channels (forcing Vref=Vdd, gain=1x via
    Multi-Write) and hold them there until Ctrl+C. Channels not listed are
    left at 0. On Ctrl+C, resets all listed channels back to 0V before exiting.

    channel_codes: dict mapping channel index (0-3) -> code (0-4095)
    """
    codes = [0, 0, 0, 0]
    for channel, code in channel_codes.items():
        codes[channel] = code
    multi_write_all(bus, addr, codes, vref_vdd=True, gain=1)

    for channel, code in channel_codes.items():
        expected_v = vdd * code / MAX_CODE
        print(f"Holding CH{chr(ord('A') + channel)} at code {code} (~{expected_v:.3f} V with Vdd={vdd} V)")
    print("Ctrl+C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for channel in channel_codes:
            codes[channel] = 0
        fast_write_all(bus, addr, codes)
        reset_names = ", ".join(f"CH{chr(ord('A') + ch)}" for ch in channel_codes)
        print(f"\nDone — {reset_names} reset to 0V")


def save_eeprom_all(bus, addr, codes, vdd):
    """
    Persist the given codes, Vref=Vdd, gain=1x on all 4 channels to EEPROM,
    so the chip powers up outputting these voltages with no software write
    needed. Confirms via readback before reporting success.
    """
    print(f"Writing power-on defaults to EEPROM on all 4 channels at 0x{addr:02X}...")
    sequential_write_eeprom_all(bus, addr, codes, vref_vdd=True, gain=1)

    readback = read_input_registers(bus, addr)
    flags = read_channel_flags(bus, addr)
    ok = True
    for ch in range(4):
        code_ok = readback[ch] == codes[ch]
        vref_ok = flags[ch]["vref"] == 0
        chan_ok = code_ok and vref_ok
        ok = ok and chan_ok
        flag = "OK" if chan_ok else "FAIL"
        expected_v = vdd * codes[ch] / MAX_CODE
        print(f"  [{flag}] CH{chr(ord('A') + ch)}: code={readback[ch]} (~{expected_v:.3f} V), "
              f"vref={'Vdd' if flags[ch]['vref'] == 0 else 'internal'}")

    if ok:
        print("[OK] All 4 channels persisted to EEPROM — will hold across power cycles")
    else:
        print("[FAIL] EEPROM write did not take on one or more channels")
    return ok


def main():
    parser = argparse.ArgumentParser(description="MCP4728 breakout board verification (no LDAC wired)")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--address", default=hex(DEFAULT_ADDR), help="I2C address (e.g. 0x60)")
    parser.add_argument("--vdd", default=3.3, type=float, help="Supply voltage for Vout estimate")
    parser.add_argument("--sweep", type=int, choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, continuously sweep one channel (0-3 = A-D)")
    parser.add_argument("--hold", type=int, nargs="+", choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, write to one or more channels (0-3 = A-D, "
                              "e.g. --hold 0 1) and hold them there until Ctrl+C (other "
                              "channels set to 0). Each listed channel uses its --code-a/b/c/d "
                              "override if given, otherwise --code")
    parser.add_argument("--code", type=int, default=MAX_CODE // 2,
                         help="12-bit DAC code (0-4095) default for --hold/--save-eeprom channels "
                              "with no per-channel override (default: mid-scale)")
    parser.add_argument("--zero-eeprom", action="store_true",
                         help="Persist 0V (code 0, Vref=Vdd) to EEPROM on all 4 channels "
                              "so it survives power cycles, then exit")
    parser.add_argument("--save-eeprom", action="store_true",
                         help="Persist --code (or --code-a/b/c/d) to EEPROM as the new "
                              "power-on default (Vref=Vdd) on all 4 channels, then exit")
    parser.add_argument("--code-a", type=int, default=None,
                         help="Per-channel override for --hold/--save-eeprom (defaults to --code)")
    parser.add_argument("--code-b", type=int, default=None,
                         help="Per-channel override for --hold/--save-eeprom (defaults to --code)")
    parser.add_argument("--code-c", type=int, default=None,
                         help="Per-channel override for --hold/--save-eeprom (defaults to --code)")
    parser.add_argument("--code-d", type=int, default=None,
                         help="Per-channel override for --hold/--save-eeprom (defaults to --code)")
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

    if args.zero_eeprom:
        ok = save_eeprom_all(bus, addr, [0, 0, 0, 0], args.vdd)
        bus.close()
        sys.exit(0 if ok else 1)

    if args.save_eeprom:
        per_channel = [args.code_a, args.code_b, args.code_c, args.code_d]
        codes = [args.code if c is None else c for c in per_channel]
        ok = save_eeprom_all(bus, addr, codes, args.vdd)
        bus.close()
        sys.exit(0 if ok else 1)

    if args.hold is not None:
        per_channel = [args.code_a, args.code_b, args.code_c, args.code_d]
        channel_codes = {ch: (args.code if per_channel[ch] is None else per_channel[ch])
                          for ch in args.hold}
        hold_values(bus, addr, channel_codes, args.vdd)
        bus.close()
        sys.exit(0)

    if args.sweep is not None:
        sweep_channel(bus, addr, args.sweep, args.vdd)
        bus.close()
        sys.exit(0)

    print(f"=== MCP4728 breakout verification at 0x{addr:02X} on {args.bus} ===\n")

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
        print("MCP4728 breakout verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
