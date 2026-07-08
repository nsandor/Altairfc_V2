"""
LED driver (MCP4728 DAC) control + thermistor bridge (ADS1113) monitor.

Sets MCP4728 channel A (the LED driver's control input) to a DAC code given
on the command line, then continuously reads the ADS1113 at I2C address
0x4B (ADDR pin tied to SCL) and prints the derived thermistor temperature
over time. Uses smbus2 directly — no compiled driver required.

The ADS1113 is wired differentially across a Wheatstone bridge (same bridge
used with the ADS1220 boards): R3/R4/R5 = 10k fixed, TH1 = NCU18XH103F60RB
NTC thermistor.

    +5V
     |
    R3(10k) --- TH1 (thermistor)
     |               |
  NTC_DIFF-      NTC_DIFF+  -> AIN0
     |               |
    R4(10k) --- R5(10k)
     |
    GND

  NTC_DIFF- -> AIN1
  NTC_DIFF+ -> AIN0

ADS1113 has no MUX/PGA — it always reports AIN0 minus AIN1 at a fixed
+-2.048V full-scale range, 16-bit right-justified two's complement.

LDAC (BCM 20 / physical pin 38) must be driven LOW for MCP4728 writes to
reach VOUT immediately; this script drives it via pigpio like
tests/test_mcp4728.py does (see --no-ldac if it's hardwired to GND).

Usage:
    python tests/test_led_driver_thermistor.py --dac 2047
    python tests/test_led_driver_thermistor.py --dac 4095 --interval 0.5
    python tests/test_led_driver_thermistor.py --dac 0 --no-ldac
"""

import argparse
import math
import sys
import time

MCP4728_ADDR = 0x60
MCP4728_MAX_CODE = 4095
LED_DAC_CHANNEL = 0  # MCP4728 channel A drives the LED driver input

ADS1113_ADDR = 0x4B  # 1001011b, ADDR pin tied to SCL
ADS1113_REG_CONVERSION = 0x00
ADS1113_REG_CONFIG = 0x01

# Config register: OS=1 (start single conversion), MODE=1 (single-shot),
# DR=100b (128 SPS), reserved bits left at their datasheet reset pattern.
# The ADS1113 reset value (0x8583) already has MODE=1 and DR=100b, and its
# bit 15 (OS) is already 1, so the reset value doubles as the "start a
# conversion" write — verified bit-by-bit against the datasheet register
# map, not assumed from the ADS1115's MUX/PGA-bearing layout.
ADS1113_CONFIG_START = 0x8583  # = datasheet reset value; OS/MODE/DR already correct as-is
ADS1113_FSR_V = 2.048
ADS1113_LSB_V = ADS1113_FSR_V / 32768.0  # 62.5 uV, 16-bit signed two's complement

DEFAULT_LDAC_PIN = 20  # BCM numbering, physical pin 38

# Bridge / thermistor constants (same bridge as tests/test_photodiode_adc.py)
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
    """
    Write one MCP4728 channel via Multi-Write, explicitly setting Vref/gain
    so VOUT tracks Vdd rather than whatever reference was last configured
    (matches the fix from tests/test_mcp4728.py).
    """
    code = max(0, min(MCP4728_MAX_CODE, code))
    vref_bit = 0 if vref_vdd else 1
    gain_bit = 1 if gain == 2 else 0

    cmd = 0x40 | (channel << 1)  # Multi-Write command byte for this channel
    upper = (vref_bit << 7) | (0 << 5) | (gain_bit << 4) | ((code >> 8) & 0x0F)
    lower = code & 0xFF
    bus.write_i2c_block_data(addr, cmd, [upper, lower])


def ads1113_read_single_shot(bus, addr, settle_s=None):
    """
    Trigger a single-shot conversion and read back the signed 16-bit result.

    NOTE: not OS-bit-polled — writes the config register to start a
    conversion, sleeps a fixed margin sized for the 128 SPS default data
    rate, then reads the conversion register once. If samples look stale,
    poll the OS bit (bit 15 of the config register reads 0 while
    converting, 1 when done) instead of relying on the fixed delay.
    """
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


def code_to_volts(code):
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


def main():
    parser = argparse.ArgumentParser(
        description="Set MCP4728 LED driver DAC output and monitor ADS1113 thermistor bridge temperature")
    parser.add_argument("--dac", type=int, required=True,
                         help="12-bit DAC code (0-4095) to write to MCP4728 channel A (LED driver input)")
    parser.add_argument("--mcp4728-bus", default="/dev/i2c-1", help="I2C device node for the MCP4728")
    parser.add_argument("--mcp4728-addr", default=hex(MCP4728_ADDR), help="MCP4728 I2C address")
    parser.add_argument("--ads1113-bus", default="/dev/i2c-1", help="I2C device node for the ADS1113")
    parser.add_argument("--ads1113-addr", default=hex(ADS1113_ADDR), help="ADS1113 I2C address")
    parser.add_argument("--ldac-pin", type=int, default=DEFAULT_LDAC_PIN,
                         help="BCM pin driving LDAC (default: 20 / physical pin 38)")
    parser.add_argument("--no-ldac", action="store_true",
                         help="Don't drive LDAC (use if it's hardwired to GND)")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between temperature samples")
    args = parser.parse_args()

    if not 0 <= args.dac <= MCP4728_MAX_CODE:
        print(f"[FAIL] --dac must be 0-{MCP4728_MAX_CODE}, got {args.dac}")
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

    try:
        mcp4728_bus = smbus2.SMBus(int(args.mcp4728_bus.replace("/dev/i2c-", "")))
        ads1113_bus = (mcp4728_bus if args.ads1113_bus == args.mcp4728_bus
                        else smbus2.SMBus(int(args.ads1113_bus.replace("/dev/i2c-", ""))))
    except Exception as e:
        print(f"[FAIL] Could not open I2C bus: {e}")
        ldac.close()
        sys.exit(1)

    try:
        mcp4728_multi_write_channel(mcp4728_bus, mcp4728_addr, LED_DAC_CHANNEL, args.dac,
                                     vref_vdd=True, gain=1)
    except OSError as e:
        print(f"[FAIL] Could not write MCP4728 at 0x{mcp4728_addr:02X}: {e}")
        ldac.close()
        sys.exit(1)

    print(f"[OK] MCP4728 channel A (LED driver) set to code {args.dac}/{MCP4728_MAX_CODE}")
    print(f"Monitoring ADS1113 at 0x{ads1113_addr:02X} (AIN0-AIN1 differential), "
          f"interval={args.interval}s, Ctrl+C to stop\n")

    try:
        while True:
            try:
                code = ads1113_read_single_shot(ads1113_bus, ads1113_addr)
            except OSError as e:
                print(f"[FAIL] ADS1113 read error: {e}")
                time.sleep(args.interval)
                continue

            vdiff = code_to_volts(code)
            r = bridge_volts_to_resistance(vdiff)
            t_c = resistance_to_celsius(r)
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] Vdiff={vdiff:+.6f} V  TH1={r:8.1f} ohm  T={t_c:6.2f} C")

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ldac.close()
        mcp4728_bus.close()
        if ads1113_bus is not mcp4728_bus:
            ads1113_bus.close()
        print("Done")


if __name__ == "__main__":
    main()
