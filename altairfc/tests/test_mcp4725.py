"""
MCP4725 hardware verification script.

Runs directly on the Pi with the MCP4725 single-channel DAC wired at I2C
address 0x62 (A0 pin tied low) or 0x63 (A0 tied high). Uses smbus2 directly
— no driver module required.

Unlike the MCP4728, the MCP4725 has no LDAC pin (writes always reach VOUT
immediately), no selectable reference (Vref is always Vdd), and no gain
setting. It does have onboard EEPROM for a persistent power-on default,
and 4 power-down modes (normal, plus 3 different pulldown resistances to
GND while powered down).

Usage:
    python tests/test_mcp4725.py [--bus /dev/i2c-1] [--address 0x62] [--vdd 3.3]
    python tests/test_mcp4725.py --hold 2047                # hold code 2047, Ctrl+C to stop
    python tests/test_mcp4725.py --sweep                    # ramp 0->max->0, Ctrl+C to stop
    python tests/test_mcp4725.py --save-eeprom --code 2047  # persist as power-on default
    python tests/test_mcp4725.py --power-down 1              # power down with 1k pulldown

Checks performed (default, no flags):
    1. Device responds at the given address
    2. Write a known code via Write-DAC-Register (not EEPROM), then read
       it back via the 5-byte read format and verify it round-trips
    3. Set the output to mid-scale (Vout ~= Vdd/2) for a voltmeter check,
       and report the EEPROM-stored power-on default alongside it

Notes:
    - DAC codes are 12-bit (0-4095). Output is Vdd * code / 4096 — note
      the divisor is 4096 (2^12), not 4095, per the datasheet formula
      (differs slightly from the MCP4728's Vdd * code / 4095 convention
      used elsewhere in this repo).
    - Vref is always Vdd here; there is no internal-reference option and
      no gain setting, unlike the MCP4728.
"""

import argparse
import sys
import time

DEFAULT_ADDR = 0x62   # A0 pin tied low; use 0x63 if A0 is tied high
MAX_CODE = 4095
CODE_DIVISOR = 4096.0  # Vout = Vdd * code / 4096, per datasheet (not 4095)

# Write DAC Register command byte: C2 C1 C0 = 010, then PD1 PD0, then 0
# i.e. 0b010_00_PD_0 -> 0x40 with PD=00 (normal operation)
_CMD_WRITE_DAC = 0x40

# Write DAC Register AND EEPROM command byte: C2 C1 C0 = 011
# i.e. 0b011_00_PD_0 -> 0x60 with PD=00 (normal operation)
_CMD_WRITE_DAC_EEPROM = 0x60

EEPROM_WRITE_S = 0.05  # datasheet: EEPROM write takes typ 25ms, max 50ms


def _pd_bits(power_down):
    """PD1:PD0 — 0=normal, 1=1k pulldown, 2=100k pulldown, 3=500k pulldown."""
    if not 0 <= power_down <= 3:
        raise ValueError("power_down must be 0-3")
    return power_down


def fast_write(bus, addr, code, power_down=0):
    """
    Fast Mode Write: 2-byte transaction, DAC register only (not EEPROM).

    Byte1: 0 0 PD1 PD0 D11 D10 D9 D8
    Byte2: D7 D6 D5 D4 D3 D2 D1 D0
    """
    code = max(0, min(MAX_CODE, code))
    pd = _pd_bits(power_down)
    byte1 = (pd << 4) | ((code >> 8) & 0x0F)
    byte2 = code & 0xFF
    bus.write_i2c_block_data(addr, byte1, [byte2])


def write_dac_register(bus, addr, code, power_down=0):
    """
    Write DAC Register command: 3-byte transaction, DAC register only.

    Command byte: 010 0 0 PD1 PD0 0  (0x40 with PD=00)
    Data byte1:   D11..D4 (upper 8 bits)
    Data byte2:   D3..D0 0 0 0 0 (lower 4 bits in top nibble)
    """
    code = max(0, min(MAX_CODE, code))
    pd = _pd_bits(power_down)
    cmd = _CMD_WRITE_DAC | (pd << 1)
    data1 = (code >> 4) & 0xFF
    data2 = (code << 4) & 0xF0
    bus.write_i2c_block_data(addr, cmd, [data1, data2])


def write_dac_and_eeprom(bus, addr, code, power_down=0):
    """
    Write DAC Register AND EEPROM command: persists across power cycles.

    Same data layout as write_dac_register(), but C1:C0=11 instead of 10,
    selecting the EEPROM-persisting variant of the command.
    Blocks ~50ms (datasheet max) for the EEPROM write to complete.
    """
    code = max(0, min(MAX_CODE, code))
    pd = _pd_bits(power_down)
    cmd = _CMD_WRITE_DAC_EEPROM | (pd << 1)
    data1 = (code >> 4) & 0xFF
    data2 = (code << 4) & 0xF0
    bus.write_i2c_block_data(addr, cmd, [data1, data2])
    time.sleep(EEPROM_WRITE_S)


def read_all(bus, addr):
    """
    5-byte read: DAC register (live), power-down state, and EEPROM-stored
    power-on default, plus EEPROM write-ready status.

    Byte0: RDY/BSY POR 0 0 0 PD1 PD0 0  (status)
    Byte1: D11..D4  (DAC register, upper 8 bits)
    Byte2: D3..D0 0 0 0 0  (DAC register, lower 4 bits)
    Byte3: x PD1 PD0 x D11..D8  (EEPROM-stored value, upper bits + PD)
    Byte4: D7..D0  (EEPROM-stored value, lower 8 bits)
    """
    raw = bus.read_i2c_block_data(addr, 0x00, 5)

    status = raw[0]
    eeprom_ready = bool(status & 0x80)
    por = bool(status & 0x40)
    dac_pd = (status >> 1) & 0x03

    dac_code = ((raw[1] << 4) | (raw[2] >> 4)) & 0xFFF

    eeprom_pd = (raw[3] >> 4) & 0x03
    eeprom_code = ((raw[3] & 0x0F) << 8) | raw[4]

    return {
        "eeprom_ready": eeprom_ready,
        "por": por,
        "dac_code": dac_code,
        "dac_power_down": dac_pd,
        "eeprom_code": eeprom_code,
        "eeprom_power_down": eeprom_pd,
    }


def check_device_present(bus, addr):
    try:
        bus.read_byte(addr)
        print(f"[OK] Device responds at 0x{addr:02X}")
        return True
    except OSError:
        print(f"[FAIL] No device at 0x{addr:02X} — check wiring, A0 strapping, and I2C bus")
        return False


def check_roundtrip(bus, addr):
    """Write a known code via Write-DAC-Register (not EEPROM) and verify readback."""
    test_code = 2345
    write_dac_register(bus, addr, test_code, power_down=0)
    time.sleep(0.01)

    state = read_all(bus, addr)
    ok = state["dac_code"] == test_code and state["dac_power_down"] == 0
    flag = "OK" if ok else "FAIL"
    print(f"  [{flag}] wrote code={test_code}, read code={state['dac_code']} "
          f"(power_down={state['dac_power_down']})")
    return ok


def check_midscale(bus, addr, vdd):
    """Set the output to mid-scale so voltage can be checked with a meter."""
    mid_code = MAX_CODE // 2
    write_dac_register(bus, addr, mid_code, power_down=0)
    expected_v = vdd * mid_code / CODE_DIVISOR
    print(f"[INFO] DAC set to code {mid_code} (~{expected_v:.3f} V with Vdd={vdd} V)")
    print("       Verify with a multimeter on VOUT if available.")

    state = read_all(bus, addr)
    print(f"  Live DAC register: code={state['dac_code']}, power_down={state['dac_power_down']}")
    print(f"  EEPROM power-on default: code={state['eeprom_code']}, "
          f"power_down={state['eeprom_power_down']}, ready={state['eeprom_ready']}")
    return True


def sweep(bus, addr, vdd):
    """Continuously ramp 0 -> max -> 0 until Ctrl+C."""
    print(f"Sweeping at 0x{addr:02X}, Ctrl+C to stop")
    try:
        while True:
            for code in list(range(0, MAX_CODE, 32)) + list(range(MAX_CODE, 0, -32)):
                fast_write(bus, addr, code, power_down=0)
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        fast_write(bus, addr, 0, power_down=0)
        print("\nDone — reset to 0V")


def hold_value(bus, addr, code, vdd):
    """Write a single known code and hold it there until Ctrl+C, then reset to 0V."""
    write_dac_register(bus, addr, code, power_down=0)
    expected_v = vdd * code / CODE_DIVISOR
    print(f"Holding code {code} (~{expected_v:.3f} V with Vdd={vdd} V), Ctrl+C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        fast_write(bus, addr, 0, power_down=0)
        print("\nDone — reset to 0V")


def save_eeprom(bus, addr, code, vdd):
    """Persist the given code (normal power mode) to EEPROM as the new power-on default."""
    print(f"Writing power-on default (code {code}) to EEPROM at 0x{addr:02X}...")
    write_dac_and_eeprom(bus, addr, code, power_down=0)

    state = read_all(bus, addr)
    ok = state["eeprom_code"] == code and state["eeprom_ready"]
    flag = "OK" if ok else "FAIL"
    expected_v = vdd * code / CODE_DIVISOR
    print(f"  [{flag}] EEPROM code={state['eeprom_code']} (~{expected_v:.3f} V), "
          f"ready={state['eeprom_ready']}")

    if ok:
        print("[OK] Power-on default persisted to EEPROM — will hold across power cycles")
    else:
        print("[FAIL] EEPROM write did not take, or is still in progress")
    return ok


def set_power_down(bus, addr, power_down):
    """Set the power-down mode immediately (Fast Write), holding code at 0."""
    names = {0: "normal operation", 1: "powered down, 1k pulldown",
             2: "powered down, 100k pulldown", 3: "powered down, 500k pulldown"}
    fast_write(bus, addr, 0, power_down=power_down)
    print(f"[OK] Set power-down mode {power_down} ({names[power_down]})")
    return True


def main():
    parser = argparse.ArgumentParser(description="MCP4725 hardware verification")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--address", default=hex(DEFAULT_ADDR),
                         help="I2C address (0x62 if A0 tied low, 0x63 if A0 tied high)")
    parser.add_argument("--vdd", default=3.3, type=float, help="Supply voltage for Vout estimate")
    parser.add_argument("--hold", type=int, default=None,
                         help="Instead of checks, write this 12-bit code (0-4095) and hold it "
                              "until Ctrl+C, then reset to 0V")
    parser.add_argument("--sweep", action="store_true",
                         help="Instead of checks, continuously ramp 0->max->0 until Ctrl+C")
    parser.add_argument("--save-eeprom", action="store_true",
                         help="Persist --code as the new power-on default (normal power mode), then exit")
    parser.add_argument("--code", type=int, default=MAX_CODE // 2,
                         help="12-bit DAC code (0-4095) to use with --save-eeprom (default: mid-scale)")
    parser.add_argument("--power-down", type=int, choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, set power-down mode immediately and exit "
                              "(0=normal, 1=1k pulldown, 2=100k, 3=500k)")
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

    if args.power_down is not None:
        ok = set_power_down(bus, addr, args.power_down)
        bus.close()
        sys.exit(0 if ok else 1)

    if args.save_eeprom:
        ok = save_eeprom(bus, addr, args.code, args.vdd)
        bus.close()
        sys.exit(0 if ok else 1)

    if args.hold is not None:
        hold_value(bus, addr, args.hold, args.vdd)
        bus.close()
        sys.exit(0)

    if args.sweep:
        sweep(bus, addr, args.vdd)
        bus.close()
        sys.exit(0)

    print(f"=== MCP4725 verification at 0x{addr:02X} on {args.bus} ===\n")

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
        print("MCP4725 verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
