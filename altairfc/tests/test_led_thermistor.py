"""
Thermistor cross-check: ADS1115 differential bridge read (AIN2-AIN3) vs.
single-ended voltage-divider read (AIN2 midpoint), streamed side by side.

The existing AIN2-AIN3 differential bridge read (see tests/test_LED_system.py)
reported ~25C at a point where an external thermocouple read ~31.2C, with
Vdiff sitting only ~0.6mV off true balance. Rather than trust that
differential reading blindly, this script also reads AIN2 single-ended and
treats it as the midpoint of a simple two-resistor divider:

    Vexc ---- TH1 (high side, thermistor) ---- AIN2 (midpoint) ---- R_low (10k) ---- GND

    V_AIN2 = Vexc * R_low / (TH1 + R_low)
    TH1    = R_low * (Vexc - V_AIN2) / V_AIN2

This divider read is independent of the AIN2/AIN3 differential polarity
question that came up earlier, and uses the ADS1115's full single-ended
range rather than a small differential signal — useful for sanity-checking
whether the bridge reading, the polarity fix, or the bridge wiring itself
is the source of the mismatch against the thermocouple.

No MCP4728/LED driving here — this is a read-only diagnostic focused
purely on the thermistor measurement path.

Usage:
    python tests/test_led_thermistor.py
    python tests/test_led_thermistor.py --interval 0.5 --therm-gain 16
"""

import argparse
import math
import sys
import time

ADS1115_ADDR = 0x4A  # ADDR pin strapped to SDA
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG = 0x01
_ADS1115_MUX_SINGLE = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}
_ADS1115_MUX_DIFF_2_3 = 0b011  # AIN2-AIN3
_ADS1115_DR_128SPS = 0b100
_ADS1115_COMP_QUE_DISABLE = 0b11
_ADS1115_GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),  # reset default
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}

# Bridge / divider / thermistor constants (same physical TH1 as test_LED_system.py)
BRIDGE_R = 10000.0
DIVIDER_R_LOW = 10000.0
VEXC = 3.3
THERM_R25 = 10000.0
THERM_B = 3380.0
T0_KELVIN = 298.15


def bridge_volts_to_resistance(vdiff, r=BRIDGE_R, vexc=VEXC):
    """TH1 = R * (Vexc/2 + Vdiff) / (Vexc/2 - Vdiff)"""
    half_vexc = vexc / 2.0
    denom = half_vexc - vdiff
    if denom == 0:
        return float("inf")
    return r * (half_vexc + vdiff) / denom


def divider_volts_to_resistance(v_ain2, r_low=DIVIDER_R_LOW, vexc=VEXC):
    """TH1 = R_low * (Vexc - V_AIN2) / V_AIN2, for TH1 on the high side, AIN2 at the midpoint."""
    if v_ain2 <= 0:
        return float("inf")
    return r_low * (vexc - v_ain2) / v_ain2


def resistance_to_celsius(r, r25=THERM_R25, b=THERM_B, t0=T0_KELVIN):
    if r <= 0:
        return float("nan")
    t_kelvin = 1.0 / (1.0 / t0 + (1.0 / b) * math.log(r / r25))
    return t_kelvin - 273.15


def _ads1115_to_int16(raw):
    val = raw & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val


def ads1115_one_shot_read_mux(bus, addr, mux_bits, gain=2, data_rate=_ADS1115_DR_128SPS):
    """Trigger a single-shot conversion on an arbitrary ADS1115 MUX setting and block until ready."""
    pga_bits, _fsr = _ADS1115_GAIN_FSR[gain]

    cfg = 0
    cfg |= 1 << 15  # OS: start conversion
    cfg |= (mux_bits & 0x07) << 12
    cfg |= (pga_bits & 0x07) << 9
    cfg |= 1 << 8  # MODE: single-shot
    cfg |= (data_rate & 0x07) << 5
    cfg |= _ADS1115_COMP_QUE_DISABLE & 0x03

    bus.write_i2c_block_data(addr, ADS1115_REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])

    time.sleep((1.0 / 128) * 1.5)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        raw = bus.read_i2c_block_data(addr, ADS1115_REG_CONFIG, 2)
        status = (raw[0] << 8) | raw[1]
        if status & 0x8000:  # OS=1 means conversion complete / ADC idle
            break
        time.sleep(0.001)
    else:
        raise TimeoutError("ADS1115 conversion did not complete in time")

    raw = bus.read_i2c_block_data(addr, ADS1115_REG_CONVERSION, 2)
    code = _ads1115_to_int16((raw[0] << 8) | raw[1])
    return code


def ads1115_code_to_volts(code, gain=2):
    _pga_bits, fsr = _ADS1115_GAIN_FSR[gain]
    return code * fsr / 32768.0


def main():
    parser = argparse.ArgumentParser(
        description="Stream the AIN2-AIN3 differential bridge reading and the AIN2 single-ended "
                    "divider reading side by side for cross-checking TH1 temperature")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--ads1115-addr", default=hex(ADS1115_ADDR), help="ADS1115 I2C address")
    parser.add_argument("--therm-gain", type=int, choices=sorted(_ADS1115_GAIN_FSR), default=2,
                         help="ADS1115 PGA gain for the AIN2-AIN3 differential bridge read")
    parser.add_argument("--divider-gain", type=int, choices=sorted(_ADS1115_GAIN_FSR), default=1,
                         help="ADS1115 PGA gain for the AIN2 single-ended divider read "
                              "(default gain=1, +-4.096V, to safely cover the full 0-Vexc range)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between samples")
    args = parser.parse_args()

    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)

    addr = int(args.ads1115_addr, 0)

    try:
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        sys.exit(1)

    print(f"=== TH1 cross-check at 0x{addr:02X} on {args.bus} ===")
    print("Bridge: AIN2-AIN3 differential (raw polarity — no sign correction applied)")
    print("Divider: AIN2 single-ended, TH1 high side / 10k low side to GND\n")
    print(f"interval={args.interval}s, Ctrl+C to stop\n")

    try:
        while True:
            try:
                bridge_code = ads1115_one_shot_read_mux(bus, addr, _ADS1115_MUX_DIFF_2_3,
                                                         gain=args.therm_gain)
                vdiff = ads1115_code_to_volts(bridge_code, gain=args.therm_gain)
                r_bridge = bridge_volts_to_resistance(vdiff)
                t_bridge = resistance_to_celsius(r_bridge)
            except (OSError, TimeoutError) as e:
                print(f"[FAIL] Bridge read error: {e}")
                time.sleep(args.interval)
                continue

            try:
                divider_code = ads1115_one_shot_read_mux(bus, addr, _ADS1115_MUX_SINGLE[2],
                                                          gain=args.divider_gain)
                v_ain2 = ads1115_code_to_volts(divider_code, gain=args.divider_gain)
                r_divider = divider_volts_to_resistance(v_ain2)
                t_divider = resistance_to_celsius(r_divider)
            except (OSError, TimeoutError) as e:
                print(f"[FAIL] Divider read error: {e}")
                time.sleep(args.interval)
                continue

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] BRIDGE:  Vdiff={vdiff:+.6f} V  R={r_bridge:8.1f} ohm  T={t_bridge:6.2f} C  |  "
                  f"DIVIDER: AIN2={v_ain2:.4f} V  R={r_divider:8.1f} ohm  T={t_divider:6.2f} C")

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bus.close()
        print("Done")


if __name__ == "__main__":
    main()
