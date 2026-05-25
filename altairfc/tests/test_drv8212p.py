#!/usr/bin/env python3
"""
DRV8212P H-bridge interactive test.

Assumes the MCP23017 is at its default I2C address (0x24) on bus 1.

Wiring:
  Driver 1: MCP23017 GPB2 (pin 10) -> DRV1 IN1
            MCP23017 GPB3 (pin 11) -> DRV1 IN2
  Driver 2: MCP23017 GPB0 (pin  8) -> DRV2 IN1
            MCP23017 GPB1 (pin  9) -> DRV2 IN2

Usage:
  python3 tests/test_drv8212p.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.mcp23017 import MCP23017
from drivers.drv8212p import DRV8212P

COMMANDS = {
    "1f": (0, "forward"),
    "1r": (0, "reverse"),
    "1b": (0, "brake"),
    "1c": (0, "coast"),
    "2f": (1, "forward"),
    "2r": (1, "reverse"),
    "2b": (1, "brake"),
    "2c": (1, "coast"),
    "ca": (None, "coast_all"),
}

HELP = """\
Commands:
  1f / 2f   driver 1/2 forward
  1r / 2r   driver 1/2 reverse
  1b / 2b   driver 1/2 brake
  1c / 2c   driver 1/2 coast
  ca        coast all
  q         quit
"""


def main() -> None:
    print("=" * 45)
    print(" DRV8212P interactive test")
    print("=" * 45)
    print()

    try:
        io = MCP23017()
    except Exception as e:
        print(f"[ERROR] Failed to open MCP23017: {e}")
        sys.exit(1)

    motors = DRV8212P(io)
    print("[OK] MCP23017 and DRV8212P initialised")
    print()
    print(HELP)

    try:
        while True:
            try:
                cmd = input(">> ").strip().lower()
            except EOFError:
                break

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("h", "help", "?"):
                print(HELP)
            elif cmd in COMMANDS:
                driver, action = COMMANDS[cmd]
                if action == "coast_all":
                    motors.coast_all()
                    print("Both coasted")
                else:
                    getattr(motors, action)(driver)
                    print(f"Driver {driver + 1}: {action}")
            elif cmd == "":
                continue
            else:
                print(f"Unknown command: '{cmd}'. Type 'h' for help.")
    except KeyboardInterrupt:
        print()
    finally:
        motors.coast_all()
        io.close()
        print("[OK] Outputs coasted, I2C bus closed.")


if __name__ == "__main__":
    main()
