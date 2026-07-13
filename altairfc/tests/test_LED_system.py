"""
LED system integration test: MCP4728 DAC drive + ADS1113 thermistor bridge
+ ADS1115 single-ended channel monitor, all streamed live.

Sets one MCP4728 output channel (0-3 = A-D) to a fixed DAC code and holds
it there, then continuously streams three things per LED channel:

  1. The thermistor bridge temperature via the ADS1113 (same Wheatstone
     bridge/NTC as tests/test_led_driver_thermistor.py and
     tests/test_photodiode_adc.py — differential AIN0-AIN1, fixed
     +-2.048V FSR, no MUX/PGA on that chip).
  2. The live single-ended voltage on the ADS1115 channel matching the
     MCP4728 channel index (MCP4728 channel 0 -> ADS1115 AIN0, etc.) —
     see tests/test_ads1115.py for the ADS1115 register-level driver this
     reuses.

LDAC (BCM 20 / physical pin 38) must be driven LOW for MCP4728 writes to
reach VOUT immediately, same as tests/test_led_driver_thermistor.py; pass
--no-ldac if it's hardwired to GND on this board.

Usage:
    python tests/test_LED_system.py --channel 0 --code 2047
    python tests/test_LED_system.py --channel 2 --code 4095 --interval 0.5
    python tests/test_LED_system.py --channel 0 --code 0 --no-ldac
"""

import argparse
import math
import sys
import time

MCP4728_ADDR = 0x60
MCP4728_MAX_CODE = 4095

ADS1113_ADDR = 0x4B  # thermistor bridge, ADDR pin tied to SCL
ADS1113_REG_CONVERSION = 0x00
ADS1113_REG_CONFIG = 0x01
# Reset value already has OS=1 (start conversion), MODE=1 (single-shot),
# DR=100b (128 SPS) — doubles as the "start a conversion" write as-is.
ADS1113_CONFIG_START = 0x8583
ADS1113_FSR_V = 2.048
ADS1113_LSB_V = ADS1113_FSR_V / 32768.0  # 62.5 uV, 16-bit signed two's complement

ADS1115_ADDR = 0x4A  # ADDR pin strapped to SDA
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG = 0x01
_ADS1115_MUX_SINGLE = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}
_ADS1115_DR_128SPS = 0b100
_ADS1115_COMP_QUE_DISABLE = 0b11
_ADS1115_GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),  # reset default, matches Vdd=3.3V single-ended range
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

# Bridge / thermistor constants (same bridge as test_led_driver_thermistor.py)
BRIDGE_R = 10000.0
VEXC = 5.0
THERM_R25 = 10000.0
THERM_B = 3380.0
T0_KELVIN = 298.15


class Ldac:
    """Drives the MCP4728 LDAC pin via pigpio so DAC writes reach VOUT immediately."""

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
        print(f"[INFO] LDAC (BCM {pin}) held LOW — DAC writes will update VOUT immediately")

    def close(self):
        if self._enabled and self._pi is not None:
            self._pi.write(self._pin, 0)
            self._pi.stop()


def mcp4728_multi_write_channel(bus, addr, channel, code, vref_vdd=True, gain=1):
    """Write one MCP4728 channel via Multi-Write, forcing Vref/gain so VOUT tracks Vdd."""
    code = max(0, min(MCP4728_MAX_CODE, code))
    vref_bit = 0 if vref_vdd else 1
    gain_bit = 1 if gain == 2 else 0

    cmd = 0x40 | (channel << 1)
    upper = (vref_bit << 7) | (0 << 5) | (gain_bit << 4) | ((code >> 8) & 0x0F)
    lower = code & 0xFF
    bus.write_i2c_block_data(addr, cmd, [upper, lower])


def ads1113_read_single_shot(bus, addr, settle_s=None):
    """Trigger a single-shot conversion and read back the signed 16-bit differential result."""
    if settle_s is None:
        settle_s = (1.0 / 128) * 1.5  # ~1.5x period for 128 SPS default, generous margin

    config_bytes = [(ADS1113_CONFIG_START >> 8) & 0xFF, ADS1113_CONFIG_START & 0xFF]
    bus.write_i2c_block_data(addr, ADS1113_REG_CONFIG, config_bytes)
    time.sleep(settle_s)

    raw = bus.read_i2c_block_data(addr, ADS1113_REG_CONVERSION, 2)
    code = (raw[0] << 8) | raw[1]
    if code & 0x8000:
        code -= 1 << 16
    return code


def ads1113_code_to_volts(code):
    return code * ADS1113_LSB_V


def bridge_volts_to_resistance(vdiff, r=BRIDGE_R, vexc=VEXC):
    """TH1 = R * (Vexc/2 + Vdiff) / (Vexc/2 - Vdiff)"""
    half_vexc = vexc / 2.0
    denom = half_vexc - vdiff
    if denom == 0:
        return float("inf")
    return r * (half_vexc + vdiff) / denom


def resistance_to_celsius(r, r25=THERM_R25, b=THERM_B, t0=T0_KELVIN):
    if r <= 0:
        return float("nan")
    t_kelvin = 1.0 / (1.0 / t0 + (1.0 / b) * math.log(r / r25))
    return t_kelvin - 273.15


def _ads1115_to_int16(raw):
    val = raw & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val


def ads1115_one_shot_read(bus, addr, channel, gain=2, data_rate=_ADS1115_DR_128SPS):
    """Trigger a single-ended single-shot conversion on one ADS1115 input and block until ready."""
    pga_bits, _fsr = _ADS1115_GAIN_FSR[gain]
    mux_bits = _ADS1115_MUX_SINGLE[channel]

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
        description="Hold an MCP4728 output channel at a fixed code while streaming the "
                    "ADS1113 thermistor bridge temperature and the matching ADS1115 channel voltage")
    parser.add_argument("--channel", type=int, choices=[0, 1, 2, 3], required=True,
                         help="MCP4728 output channel (0-3 = A-D); also selects the ADS1115 "
                              "input channel to monitor (0->AIN0, 1->AIN1, etc.)")
    parser.add_argument("--code", type=int, required=True,
                         help="12-bit DAC code (0-4095) to hold on the selected MCP4728 channel")
    parser.add_argument("--bus", default="/dev/i2c-1", help="Shared I2C device node for all three chips")
    parser.add_argument("--mcp4728-addr", default=hex(MCP4728_ADDR), help="MCP4728 I2C address")
    parser.add_argument("--ads1113-addr", default=hex(ADS1113_ADDR), help="ADS1113 I2C address")
    parser.add_argument("--ads1115-addr", default=hex(ADS1115_ADDR), help="ADS1115 I2C address")
    parser.add_argument("--ads1115-gain", type=int, choices=sorted(_ADS1115_GAIN_FSR), default=2,
                         help="ADS1115 PGA gain setting (selects full-scale range)")
    parser.add_argument("--ldac-pin", type=int, default=DEFAULT_LDAC_PIN,
                         help="BCM pin driving LDAC (default: 20 / physical pin 38)")
    parser.add_argument("--no-ldac", action="store_true",
                         help="Don't drive LDAC (use if it's hardwired to GND)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between samples")
    args = parser.parse_args()

    if not 0 <= args.code <= MCP4728_MAX_CODE:
        print(f"[FAIL] --code must be 0-{MCP4728_MAX_CODE}, got {args.code}")
        sys.exit(1)

    try:
        import smbus2
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)

    try:
        ldac = Ldac(args.ldac_pin, enabled=not args.no_ldac)
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)

    mcp4728_addr = int(args.mcp4728_addr, 0)
    ads1113_addr = int(args.ads1113_addr, 0)
    ads1115_addr = int(args.ads1115_addr, 0)

    try:
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        ldac.close()
        sys.exit(1)

    try:
        mcp4728_multi_write_channel(bus, mcp4728_addr, args.channel, args.code,
                                     vref_vdd=True, gain=1)
    except OSError as e:
        print(f"[FAIL] Could not write MCP4728 at 0x{mcp4728_addr:02X}: {e}")
        ldac.close()
        bus.close()
        sys.exit(1)

    ch_letter = chr(ord('A') + args.channel)
    print(f"[OK] MCP4728 channel {ch_letter} set to code {args.code}/{MCP4728_MAX_CODE}")
    print(f"Monitoring ADS1113 at 0x{ads1113_addr:02X} (thermistor bridge, AIN0-AIN1) and "
          f"ADS1115 at 0x{ads1115_addr:02X} (AIN{args.channel}, single-ended), "
          f"interval={args.interval}s, Ctrl+C to stop\n")

    try:
        while True:
            try:
                therm_code = ads1113_read_single_shot(bus, ads1113_addr)
                vdiff = ads1113_code_to_volts(therm_code)
                r = bridge_volts_to_resistance(vdiff)
                t_c = resistance_to_celsius(r)
            except OSError as e:
                print(f"[FAIL] ADS1113 read error: {e}")
                time.sleep(args.interval)
                continue

            try:
                led_code = ads1115_one_shot_read(bus, ads1115_addr, args.channel, gain=args.ads1115_gain)
                led_v = ads1115_code_to_volts(led_code, gain=args.ads1115_gain)
            except (OSError, TimeoutError) as e:
                print(f"[FAIL] ADS1115 read error: {e}")
                time.sleep(args.interval)
                continue

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] CH{ch_letter} code={args.code:4d}  "
                  f"TH1: Vdiff={vdiff:+.6f} V R={r:8.1f} ohm T={t_c:6.2f} C  |  "
                  f"AIN{args.channel}: {led_v:.4f} V")

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ldac.close()
        bus.close()
        print("Done")


if __name__ == "__main__":
    main()
