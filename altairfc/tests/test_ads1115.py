"""
ADS1115 hardware verification script.

Runs directly on the Pi with the ADS1115 4-channel 16-bit ADC breakout
wired at I2C address 0x4A (ADDR strapped to SDA — the datasheet default
0x48 is ADDR-to-GND). Uses smbus2 directly — no driver module required.

The ADS1115 has one Config register and one Conversion register, selected
via a register-pointer byte, plus Lo_thresh/Hi_thresh registers used here
only to enable the comparator-disabled ("traditional" but non-latching,
default) mode when in continuous-conversion. All multi-byte register
values are big-endian (MSB first), unlike this repo's other I2C DAC
scripts which are little-endian at the wire-format level in places.

Usage:
    python tests/test_ads1115.py [--bus /dev/i2c-1] [--vdd 3.3]
    python tests/test_ads1115.py --channel 0                # single-ended AIN0, one-shot
    python tests/test_ads1115.py --channel 0 --continuous    # stream AIN0 until Ctrl+C
    python tests/test_ads1115.py --diff 0 1                  # differential AIN0-AIN1
    python tests/test_ads1115.py --gain 1                    # +-4.096V FSR (see GAIN_FSR)

Checks performed (default, no flags):
    1. Device responds at the given address
    2. Config register round-trips a known value (write then read back)
    3. One-shot conversion completes (OS bit clears then sets) and returns
       a plausible code on each single-ended input channel

Notes:
    - Default PGA is +-2.048V FSR (gain=2), which is the reset default and
      safe for a 3.3V supply on single-ended inputs.
    - Conversion codes are signed 16-bit; single-ended inputs never return
      negative codes in practice (input is 0..Vdd), but the register is
      still interpreted as a two's-complement int16.
    - Data rate defaults to 128 SPS (DR=100), the reset default.
"""

import argparse
import sys
import time

DEFAULT_ADDR = 0x4A   # ADDR pin strapped to SDA (datasheet default 0x48 is ADDR-to-GND)

_REG_CONVERSION = 0x00
_REG_CONFIG = 0x01
_REG_LO_THRESH = 0x02
_REG_HI_THRESH = 0x03

# Config register bit layout (16 bits, MSB first on the wire):
# [15]    OS      : 1=start single conversion (write), 0=converting / 1=ready (read)
# [14:12] MUX     : input multiplexer
# [11:9]  PGA     : programmable gain amplifier / full-scale range
# [8]     MODE    : 1=single-shot (default), 0=continuous
# [7:5]   DR      : data rate
# [4]     COMP_MODE: 0=traditional
# [3]     COMP_POL: 0=active low
# [2]     COMP_LAT: 0=non-latching
# [1:0]   COMP_QUE: 11=disable comparator (default)

_MUX_DIFF = {
    (0, 1): 0b000,
    (0, 3): 0b001,
    (1, 3): 0b010,
    (2, 3): 0b011,
}
_MUX_SINGLE = {0: 0b100, 1: 0b101, 2: 0b110, 3: 0b111}

# gain setting -> (PGA bits, full-scale range in volts)
GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),  # reset default
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}

_DR_128SPS = 0b100
_COMP_QUE_DISABLE = 0b11

_CONFIG_DEFAULT = 0x8583  # reset value per datasheet (OS=1, MUX=000, PGA=010, MODE=1, DR=100, COMP_QUE=11
_CONVERSION_WAIT_S = 0.01  # >> 1/128s conversion time at default data rate


def _to_int16(raw):
    val = raw & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val


def read_reg(bus, addr, reg):
    raw = bus.read_i2c_block_data(addr, reg, 2)
    return (raw[0] << 8) | raw[1]


def write_reg(bus, addr, reg, value):
    bus.write_i2c_block_data(addr, reg, [(value >> 8) & 0xFF, value & 0xFF])


def build_config(mux_bits, gain=2, mode_single_shot=True, data_rate=_DR_128SPS, start=True):
    pga_bits, _fsr = GAIN_FSR[gain]
    cfg = 0
    cfg |= (1 if start else 0) << 15
    cfg |= (mux_bits & 0x07) << 12
    cfg |= (pga_bits & 0x07) << 9
    cfg |= (1 if mode_single_shot else 0) << 8
    cfg |= (data_rate & 0x07) << 5
    cfg |= (_COMP_QUE_DISABLE & 0x03)
    return cfg


def code_to_volts(code, gain=2):
    _pga_bits, fsr = GAIN_FSR[gain]
    return code * fsr / 32768.0


def one_shot_read(bus, addr, mux_bits, gain=2, data_rate=_DR_128SPS):
    """Trigger a single-shot conversion and block until it completes."""
    cfg = build_config(mux_bits, gain=gain, mode_single_shot=True, data_rate=data_rate, start=True)
    write_reg(bus, addr, _REG_CONFIG, cfg)

    time.sleep(_CONVERSION_WAIT_S)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        status = read_reg(bus, addr, _REG_CONFIG)
        if status & 0x8000:  # OS=1 means conversion complete / ADC idle
            break
        time.sleep(0.001)
    else:
        raise TimeoutError("ADS1115 conversion did not complete in time")

    raw = read_reg(bus, addr, _REG_CONVERSION)
    return _to_int16(raw)


def check_device_present(bus, addr):
    try:
        bus.read_byte(addr)
        print(f"[OK] Device responds at 0x{addr:02X}")
        return True
    except OSError:
        print(f"[FAIL] No device at 0x{addr:02X} — check wiring, ADDR strapping, and I2C bus")
        return False


def check_config_roundtrip(bus, addr):
    """Write a known config (continuous mode, channel 0, disabled comparator) and read it back."""
    test_cfg = build_config(_MUX_SINGLE[0], gain=2, mode_single_shot=False, start=False)
    write_reg(bus, addr, _REG_CONFIG, test_cfg)
    time.sleep(0.01)
    readback = read_reg(bus, addr, _REG_CONFIG)

    # OS bit reads back as conversion status, not the written value — mask it off for comparison
    ok = (readback & 0x7FFF) == (test_cfg & 0x7FFF)
    flag = "OK" if ok else "FAIL"
    print(f"  [{flag}] wrote config=0x{test_cfg:04X}, read config=0x{readback:04X} (OS bit ignored)")

    # restore single-shot default so subsequent one-shot reads behave normally
    write_reg(bus, addr, _REG_CONFIG, _CONFIG_DEFAULT)
    return ok


def check_single_ended_channels(bus, addr, vdd, gain):
    ok = True
    for ch in range(4):
        try:
            code = one_shot_read(bus, addr, _MUX_SINGLE[ch], gain=gain)
        except TimeoutError as e:
            print(f"  [FAIL] AIN{ch}: {e}")
            ok = False
            continue
        volts = code_to_volts(code, gain=gain)
        plausible = -0.1 <= volts <= (vdd + 0.1)
        flag = "OK" if plausible else "WARN"
        print(f"  [{flag}] AIN{ch}: code={code:6d}  ~{volts:.4f} V")
        ok = ok and plausible
    return ok


def stream_channel(bus, addr, ch, gain, diff=None):
    """Continuously trigger single-shot conversions on one input until Ctrl+C."""
    mux_bits = _MUX_DIFF[diff] if diff is not None else _MUX_SINGLE[ch]
    label = f"AIN{diff[0]}-AIN{diff[1]}" if diff is not None else f"AIN{ch}"
    print(f"Streaming {label} (gain={gain}), Ctrl+C to stop")
    try:
        while True:
            code = one_shot_read(bus, addr, mux_bits, gain=gain)
            volts = code_to_volts(code, gain=gain)
            print(f"\r{label}: code={code:6d}  ~{volts:.4f} V   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nDone")


def main():
    parser = argparse.ArgumentParser(description="ADS1115 hardware verification")
    parser.add_argument("--bus", default="/dev/i2c-1", help="I2C device node")
    parser.add_argument("--vdd", default=3.3, type=float, help="Supply voltage for plausibility check")
    parser.add_argument("--gain", type=int, choices=sorted(GAIN_FSR), default=2,
                         help="PGA gain setting (selects full-scale range, see GAIN_FSR)")
    parser.add_argument("--channel", type=int, choices=[0, 1, 2, 3], default=None,
                         help="Instead of checks, read a single-ended input channel")
    parser.add_argument("--diff", type=int, nargs=2, metavar=("POS", "NEG"), default=None,
                         help="Instead of checks, read a differential pair, e.g. --diff 0 1")
    parser.add_argument("--continuous", action="store_true",
                         help="With --channel/--diff, stream readings until Ctrl+C instead of one read")
    args = parser.parse_args()

    addr = DEFAULT_ADDR

    try:
        import smbus2
        bus = smbus2.SMBus(int(args.bus.replace("/dev/i2c-", "")))
    except ImportError:
        print("[FAIL] smbus2 not installed — run: pip install smbus2")
        sys.exit(1)
    except Exception as e:
        print(f"[FAIL] Could not open {args.bus}: {e}")
        sys.exit(1)

    if args.diff is not None:
        pair = tuple(args.diff)
        if pair not in _MUX_DIFF:
            print(f"[FAIL] Unsupported differential pair {pair}. Supported: {sorted(_MUX_DIFF)}")
            bus.close()
            sys.exit(1)
        if args.continuous:
            stream_channel(bus, addr, None, args.gain, diff=pair)
        else:
            code = one_shot_read(bus, addr, _MUX_DIFF[pair], gain=args.gain)
            volts = code_to_volts(code, gain=args.gain)
            print(f"AIN{pair[0]}-AIN{pair[1]}: code={code}  ~{volts:.4f} V")
        bus.close()
        sys.exit(0)

    if args.channel is not None:
        if args.continuous:
            stream_channel(bus, addr, args.channel, args.gain)
        else:
            code = one_shot_read(bus, addr, _MUX_SINGLE[args.channel], gain=args.gain)
            volts = code_to_volts(code, gain=args.gain)
            print(f"AIN{args.channel}: code={code}  ~{volts:.4f} V")
        bus.close()
        sys.exit(0)

    print(f"=== ADS1115 verification at 0x{addr:02X} on {args.bus} (gain={args.gain}) ===\n")

    results = []
    results.append(check_device_present(bus, addr))
    if not results[-1]:
        print("\nDevice not found, aborting.")
        bus.close()
        sys.exit(1)

    print("Config register round-trip:")
    results.append(check_config_roundtrip(bus, addr))

    print("Single-ended channel readings:")
    results.append(check_single_ended_channels(bus, addr, args.vdd, args.gain))

    bus.close()

    print(f"\n=== Results: {sum(results)}/{len(results)} checks passed ===")
    if all(results):
        print("ADS1115 verified OK")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
