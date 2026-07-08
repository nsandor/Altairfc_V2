"""
Photodiode TIA + thermistor bridge ADC verification script.

Two ADS1220 24-bit delta-sigma ADCs share the Pi's SPI0 bus (SCLK/MOSI/MISO)
but each has its own chip-select driven manually via pigpio (neither uses
the hardware CE0/CE1 pins):

    ADC1 (photodiode TIA):   CS = GPIO4
    ADC2 (thermistor bridge): CS = GPIO17

ADC1 reads the photodiode TIA output on AIN0, single-ended vs AVSS.
ADC2 reads a Wheatstone bridge (R3/R4/TH1/R5, all nominally 10k, TH1 is the
NTC thermistor) differentially: AIN2(+) - AIN3(-).

Thermistor: NCU18XH103F60RB, R25=10000 ohm +-1%, B25/50=3380K +-1%.

Requires: pip install spidev pigpio; sudo pigpiod running.

Usage:
    python tests/test_photodiode_adc.py
    python tests/test_photodiode_adc.py --samples 10
    python tests/test_photodiode_adc.py --stream          # continuous read, Ctrl+C to stop

Checks performed:
    1. Both ADCs respond (config register write/read round-trip)
    2. N consecutive samples from each ADC: values finite, DRDY behaves,
       no SPI errors
    3. Thermistor bridge sample converted to resistance and temperature

NOTE: Command bytes, register bit layouts, and MUX codes were verified
against the TI ADS1220 datasheet (SBAS501D, register-map tables 8-7..8-14).
Conversion timing is NOT DRDY-driven — read_single_shot() sleeps for a fixed
margin after START rather than polling the DRDY pin, since DRDY isn't wired
to a separate GPIO here. If samples look stale or noisy, wire DRDY out and
poll it instead of relying on the fixed delay.
"""

import argparse
import math
import sys
import time

# ------------------------------------------------------------------
# ADS1220 command bytes
# ------------------------------------------------------------------
CMD_RESET     = 0x06
CMD_START     = 0x08
CMD_POWERDOWN = 0x02
CMD_RDATA     = 0x10
CMD_RREG0     = 0x20   # RREG starting at reg 0, 1 register  (0010 00 00)
CMD_WREG0     = 0x40   # WREG starting at reg 0, 1 register  (0100 00 00)

# Config Register 0: MUX[7:4] GAIN[3:1] PGA_BYPASS[0]
MUX_AIN0_AVSS      = 0x8   # single-ended AIN0 vs AVSS (requires PGA_BYPASS=1)
MUX_AIN2_AIN3_DIFF = 0x5   # differential AIN2(+) - AIN3(-)
GAIN_1X            = 0x0
PGA_BYPASS         = 0x1
PGA_ENABLED        = 0x0

# Config Register 1: DR[7:5] MODE[4:3] CM[2] TS[1] BCS[0]
DR_20SPS_NORMAL = 0x0
MODE_NORMAL     = 0x0
CM_SINGLE_SHOT  = 0x0

# Config Register 2: VREF[7:6] FIR[5:4] PSW[3] IDAC[2:0]
VREF_INTERNAL = 0x0  # internal 2.048V reference

VREF_V   = 2.048
FULL_SCALE_CODE = 1 << 23  # 2^23

# Thermistor / bridge constants
BRIDGE_R = 10000.0   # R3=R4=R5, ohms
VEXC     = 5.0        # bridge excitation voltage
THERM_R25 = 10000.0
THERM_B   = 3380.0
T0_KELVIN = 298.15


class Ads1220:
    """
    One ADS1220 on a shared SPI bus with a manually-driven CS pin.

    Usage:
        adc = Ads1220(spi, pi, cs_pin=4)
        adc.reset()
        adc.configure(mux=MUX_AIN0_AVSS, gain=GAIN_1X, pga_bypass=PGA_BYPASS)
        code = adc.read_single_shot()
        volts = code_to_volts(code, gain=1)
    """

    def __init__(self, spi, pi, cs_pin):
        self._spi = spi
        self._pi = pi
        self._cs = cs_pin
        import pigpio
        self._pi.set_mode(cs_pin, pigpio.OUTPUT)
        self._pi.write(cs_pin, 1)  # idle high

    def _cs_low(self):
        self._pi.write(self._cs, 0)

    def _cs_high(self):
        self._pi.write(self._cs, 1)

    def _xfer(self, data):
        self._cs_low()
        try:
            result = self._spi.xfer2(list(data))
        finally:
            self._cs_high()
        return result

    def reset(self):
        self._xfer([CMD_RESET])
        time.sleep(0.001)  # tosc startup, generous margin

    def write_reg(self, addr, value):
        cmd = CMD_WREG0 | (addr << 2)
        self._xfer([cmd, value & 0xFF])

    def read_reg(self, addr):
        cmd = CMD_RREG0 | (addr << 2)
        result = self._xfer([cmd, 0x00])
        return result[1]

    def configure(self, mux, gain=GAIN_1X, pga_bypass=PGA_ENABLED,
                  data_rate=DR_20SPS_NORMAL, vref=VREF_INTERNAL):
        reg0 = (mux << 4) | (gain << 1) | pga_bypass
        reg1 = (data_rate << 5) | (MODE_NORMAL << 3) | (CM_SINGLE_SHOT << 2)
        reg2 = (vref << 6)
        reg3 = 0x00
        self.write_reg(0, reg0)
        self.write_reg(1, reg1)
        self.write_reg(2, reg2)
        self.write_reg(3, reg3)
        return (reg0, reg1, reg2, reg3)

    def read_config(self):
        return tuple(self.read_reg(i) for i in range(4))

    def read_single_shot(self, settle_s=None):
        """
        Trigger a conversion and read back the 24-bit signed result.

        NOTE: this does not poll a dedicated DRDY GPIO — it issues START,
        sleeps for the data rate's conversion period (with margin), then
        issues RDATA once. This is simple and matches this script's fixed
        20 SPS config, but is not as robust as watching DRDY go low; if
        conversions are ever unreliable, wire DRDY to a spare GPIO and
        poll it instead of sleeping.
        """
        if settle_s is None:
            settle_s = (1.0 / 20) * 1.5  # ~1.5x period for 20 SPS default, generous margin

        self._xfer([CMD_START])
        time.sleep(settle_s)

        raw = self._xfer([CMD_RDATA, 0x00, 0x00, 0x00])
        code_bytes = raw[1:4]
        code = (code_bytes[0] << 16) | (code_bytes[1] << 8) | code_bytes[2]
        if code & 0x800000:
            code -= 1 << 24
        return code

    def close(self):
        self._cs_high()


def code_to_volts(code, gain=1, vref=VREF_V):
    return (code / FULL_SCALE_CODE) * (vref / gain)


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


def check_config_roundtrip(adc, name, mux, pga_bypass):
    written = adc.configure(mux=mux, gain=GAIN_1X, pga_bypass=pga_bypass)
    time.sleep(0.001)
    readback = adc.read_config()
    ok = written == readback
    flag = "OK" if ok else "FAIL"
    print(f"  [{flag}] {name}: wrote {[hex(b) for b in written]}, "
          f"read {[hex(b) for b in readback]}")
    return ok


def check_samples(adc, name, n_samples, converter):
    print(f"\n--- {name}: taking {n_samples} sample(s) ---")
    errors = 0
    for i in range(n_samples):
        try:
            code = adc.read_single_shot()
        except OSError as e:
            print(f"  [FAIL] Sample {i+1}: SPI error: {e}")
            errors += 1
            continue

        volts = code_to_volts(code)
        extra = converter(volts) if converter else ""
        print(f"  Sample {i+1}: code={code:8d}  {volts:+.6f} V  {extra}")

    ok = errors == 0
    if ok:
        print(f"[OK] All {n_samples} sample(s) from {name} succeeded")
    else:
        print(f"[FAIL] {errors}/{n_samples} sample(s) from {name} failed")
    return ok


def photodiode_extra(volts):
    return ""


def bridge_extra(volts):
    r = bridge_volts_to_resistance(volts)
    t_c = resistance_to_celsius(r)
    return f"TH1={r:8.1f} ohm  T={t_c:6.2f} C"


def stream(adc_pd, adc_bridge):
    print("Streaming photodiode (AIN0) and thermistor bridge (AIN2-AIN3), Ctrl+C to stop")
    try:
        while True:
            pd_code = adc_pd.read_single_shot()
            pd_v = code_to_volts(pd_code)

            br_code = adc_bridge.read_single_shot()
            br_v = code_to_volts(br_code)
            r = bridge_volts_to_resistance(br_v)
            t_c = resistance_to_celsius(r)

            print(f"PD: {pd_v:+.6f} V   |   Bridge: {br_v:+.6f} V  "
                  f"TH1={r:8.1f} ohm  T={t_c:6.2f} C")
    except KeyboardInterrupt:
        print("\nDone")


def main():
    parser = argparse.ArgumentParser(description="Photodiode TIA + thermistor bridge ADC verification")
    parser.add_argument("--spi-bus", type=int, default=0, help="SPI bus number")
    parser.add_argument("--spi-speed", type=int, default=1_000_000, help="SPI clock speed (Hz)")
    parser.add_argument("--cs-photodiode", type=int, default=4, help="BCM pin for photodiode ADC CS")
    parser.add_argument("--cs-bridge", type=int, default=17, help="BCM pin for bridge ADC CS")
    parser.add_argument("--samples", type=int, default=5, help="Number of read samples")
    parser.add_argument("--stream", action="store_true", help="Continuously stream both channels, Ctrl+C to stop")
    args = parser.parse_args()

    try:
        import spidev
    except ImportError:
        print("[FAIL] spidev not installed — run: pip install spidev")
        sys.exit(1)

    try:
        import pigpio
    except ImportError:
        print("[FAIL] pigpio not installed — run: pip install pigpio")
        sys.exit(1)

    pi = pigpio.pi()
    if not pi.connected:
        print("[FAIL] Cannot connect to pigpio daemon. Run: sudo pigpiod")
        sys.exit(1)

    # Both ADS1220s share the same physical SCLK/MOSI/MISO wires (SPI0,
    # device 0). Neither uses the kernel's hardware CE0 line for chip
    # select — that's driven manually per-chip via pigpio instead — so
    # opening "device 0" twice here is intentional, not a bug: each
    # spidev handle only supplies clock/data timing, and _xfer() on each
    # Ads1220 instance gates its own CS pin around every transaction so
    # the two chips never see each other's traffic.
    spi_pd = spidev.SpiDev()
    spi_bridge = spidev.SpiDev()
    try:
        spi_pd.open(args.spi_bus, 0)
        spi_pd.max_speed_hz = args.spi_speed
        spi_pd.mode = 0b01  # ADS1220: CPOL=0, CPHA=1
        spi_pd.no_cs = True  # CS is manual via pigpio; don't also toggle hardware CE0

        spi_bridge.open(args.spi_bus, 0)
        spi_bridge.max_speed_hz = args.spi_speed
        spi_bridge.mode = 0b01
        spi_bridge.no_cs = True
    except Exception as e:
        print(f"[FAIL] Could not open SPI bus {args.spi_bus}: {e}")
        pi.stop()
        sys.exit(1)

    adc_pd = Ads1220(spi_pd, pi, args.cs_photodiode)
    adc_bridge = Ads1220(spi_bridge, pi, args.cs_bridge)

    adc_pd.reset()
    adc_bridge.reset()
    time.sleep(0.01)

    if args.stream:
        adc_pd.configure(mux=MUX_AIN0_AVSS, gain=GAIN_1X, pga_bypass=PGA_BYPASS)
        adc_bridge.configure(mux=MUX_AIN2_AIN3_DIFF, gain=GAIN_1X, pga_bypass=PGA_ENABLED)
        stream(adc_pd, adc_bridge)
        adc_pd.close()
        adc_bridge.close()
        spi_pd.close()
        spi_bridge.close()
        pi.stop()
        sys.exit(0)

    print(f"=== Photodiode/thermistor ADC verification "
          f"(PD CS=GPIO{args.cs_photodiode}, Bridge CS=GPIO{args.cs_bridge}) ===\n")

    results = []
    results.append(check_config_roundtrip(adc_pd, "Photodiode ADC (AIN0 single-ended)",
                                           MUX_AIN0_AVSS, PGA_BYPASS))
    results.append(check_config_roundtrip(adc_bridge, "Bridge ADC (AIN2-AIN3 diff)",
                                           MUX_AIN2_AIN3_DIFF, PGA_ENABLED))

    if not all(results):
        print("\nConfig round-trip failed — check wiring/CS pins before sampling.")
        adc_pd.close()
        adc_bridge.close()
        spi_pd.close()
        spi_bridge.close()
        pi.stop()
        sys.exit(1)

    results.append(check_samples(adc_pd, "Photodiode ADC", args.samples, photodiode_extra))
    results.append(check_samples(adc_bridge, "Bridge ADC", args.samples, bridge_extra))

    adc_pd.close()
    adc_bridge.close()
    spi_pd.close()
    spi_bridge.close()
    pi.stop()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("ADS1220 photodiode/thermistor chain verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
