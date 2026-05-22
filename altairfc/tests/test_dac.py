#!/usr/bin/env python3
"""
MCP4725 DAC Sweep Test
======================
Initializes I2C connection to an MCP4725 12-bit DAC at address 0x60
and performs a continuous triangle-wave sweep from 0 to 4095 and back.

Wiring (Pi 4B):
  SDA  -> GPIO 2  (pin 3)
  SCL  -> GPIO 3  (pin 5)
  VCC  -> 3.3V or 5V (pin 1 or 2)
  GND  -> GND     (pin 6)

Dependencies:
  pip install smbus2
  (or: sudo apt install python3-smbus)

Usage:
  python3 mcp4725_sweep.py [--bus 1] [--addr 0x60] [--step 8] [--delay 0.001]
"""

import time
import argparse
import sys

try:
    import smbus2
except ImportError:
    print("[ERROR] smbus2 not found. Install with: pip install smbus2")
    sys.exit(1)

# MCP4725 register/command constants
MCP4725_CMD_WRITEDAC        = 0x40  # Write to DAC register (volatile, no EEPROM)
MCP4725_CMD_WRITEDAC_EEPROM = 0x60  # Write to DAC + EEPROM (survives power cycle)
MCP4725_MAX_VALUE           = 4095  # 12-bit DAC full scale
MCP4725_POWERDOWN_NORMAL    = 0x00  # Normal operation (no power-down)


class MCP4725:
    def __init__(self, bus_number: int, address: int):
        """
        Open I2C bus and verify the MCP4725 is present.

        Args:
            bus_number: I2C bus index (1 for Pi's default /dev/i2c-1)
            address:    7-bit I2C address (0x60 or 0x61 depending on A0 pin)
        """
        self.address = address
        try:
            self.bus = smbus2.SMBus(bus_number)
        except FileNotFoundError:
            print(f"[ERROR] I2C bus {bus_number} not found.")
            print("  Enable I2C with: sudo raspi-config -> Interface Options -> I2C")
            sys.exit(1)

        # Attempt a read to confirm the device is present
        try:
            self.bus.read_byte(self.address)
            print(f"[OK]  MCP4725 found at address 0x{self.address:02X} on bus {bus_number}")
        except OSError as e:
            print(f"[ERROR] No device responded at 0x{self.address:02X}: {e}")
            print("  Check wiring and confirm I2C is enabled (sudo i2cdetect -y 1)")
            self.bus.close()
            sys.exit(1)

    def set_voltage_raw(self, value: int, persist: bool = False) -> None:
        """
        Write a 12-bit value (0–4095) to the DAC output.

        The MCP4725 fast-write protocol packs the command, PD bits, and the
        upper 8 bits of the 12-bit value into byte 0, and the lower 4 bits
        (shifted to bits 7:4) into byte 1.

        Wire format (fast mode, no EEPROM):
          Byte 0: [C1 C0 PD1 PD0 D11 D10 D9 D8]
          Byte 1: [D7  D6  D5  D4  D3  D2 D1 D0]

        Args:
            value:   12-bit DAC code, 0 = 0 V, 4095 = VCC
            persist: If True, also write to EEPROM (survives power cycle).
                     Avoid calling with persist=True in a tight loop —
                     EEPROM has a limited write endurance (~1 million cycles).
        """
        value = max(0, min(MCP4725_MAX_VALUE, int(value)))  # clamp

        if persist:
            cmd = MCP4725_CMD_WRITEDAC_EEPROM
        else:
            cmd = MCP4725_CMD_WRITEDAC

        # Pack 12-bit value into two bytes
        byte0 = cmd | MCP4725_POWERDOWN_NORMAL | ((value >> 8) & 0x0F)
        byte1 = value & 0xFF

        self.bus.write_i2c_block_data(self.address, byte0, [byte1])

    def read_status(self) -> dict:
        """
        Read the 3-byte status from the MCP4725:
          Byte 0: status flags
          Byte 1: DAC register upper byte (D11:D4)
          Byte 2: DAC register lower byte (D3:D0 in bits 7:4)

        Returns a dict with:
          ready:      True if device is not busy writing EEPROM
          por:        Power-On Reset flag
          dac_value:  Current DAC register value (0–4095)
        """
        data = self.bus.read_i2c_block_data(self.address, 0x00, 3)
        ready   = bool(data[0] & 0x80)
        por     = bool(data[0] & 0x40)
        dac_val = ((data[1] << 4) | (data[2] >> 4)) & 0x0FFF
        return {"ready": ready, "por": por, "dac_value": dac_val}

    def close(self):
        """Zero the DAC output and release the I2C bus."""
        self.set_voltage_raw(0)
        self.bus.close()
        print("[OK]  DAC zeroed and bus closed.")


def sweep(dac: MCP4725, step: int, delay: float) -> None:
    """
    Triangle-wave sweep: ramp up 0 → 4095 then ramp down 4095 → 0, repeat.

    Args:
        dac:   Initialised MCP4725 instance
        step:  Increment/decrement per write (larger = faster but coarser)
        delay: Seconds to sleep between writes (0 for max speed)
    """
    ramp_time = (MCP4725_MAX_VALUE / step) * delay
    print(f"[INFO] Sweeping: step={step}, delay={delay:.6f}s, ramp time≈{ramp_time:.2f}s  (Ctrl-C to stop)\n")

    cycle = 0
    try:
        while True:
            # Ramp up
            for v in range(0, MCP4725_MAX_VALUE + 1, step):
                dac.set_voltage_raw(v)
                if delay > 0:
                    time.sleep(delay)

            # Ensure we hit exactly 4095 if step doesn't divide evenly
            dac.set_voltage_raw(MCP4725_MAX_VALUE)

            # Ramp down
            for v in range(MCP4725_MAX_VALUE, -1, -step):
                dac.set_voltage_raw(v)
                if delay > 0:
                    time.sleep(delay)

            dac.set_voltage_raw(0)

            cycle += 1
            print(f"\r  Cycles completed: {cycle}", end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped after {cycle} full cycle(s).")


def main():
    parser = argparse.ArgumentParser(description="MCP4725 DAC sweep test")
    parser.add_argument("--bus",   type=int,   default=1,     help="I2C bus number (default: 1)")
    parser.add_argument("--addr",  type=lambda x: int(x, 0),
                                               default=0x60,  help="I2C address (default: 0x60)")
    parser.add_argument("--step",  type=int,   default=1,        help="DAC step size per write (default: 1)")
    parser.add_argument("--delay", type=float, default=0.000610, help="Delay between writes in seconds (default: 0.000610 → ~2.5s per ramp)")
    parser.add_argument("--status", action="store_true",      help="Print device status and exit")
    args = parser.parse_args()

    print("=" * 50)
    print("  MCP4725 DAC Sweep Test")
    print("=" * 50)

    dac = MCP4725(bus_number=args.bus, address=args.addr)

    status = dac.read_status()
    print(f"[INFO] Device status: ready={status['ready']}, "
          f"POR={status['por']}, current_DAC={status['dac_value']}")

    if args.status:
        dac.close()
        return

    try:
        sweep(dac, step=args.step, delay=args.delay)
    finally:
        dac.close()


if __name__ == "__main__":
    main()