#!/usr/bin/env python3
"""
DRV8212P H-bridge driver test script.

Tests both drivers through all four states: forward, reverse, brake, coast.
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
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.mcp23017 import MCP23017
from drivers.drv8212p import DRV8212P

HOLD_S = 1.5   # seconds to hold each state before moving on
PAUSE_S = 0.5  # pause between tests


def run_state(motors: DRV8212P, driver: int, state: str) -> None:
    print(f"  Driver {driver}: {state:<8}", end="  ", flush=True)
    getattr(motors, state)(driver)
    time.sleep(HOLD_S)
    motors.coast(driver)
    print("ok")


def test_all_states(motors: DRV8212P, driver: int) -> None:
    print(f"--- Driver {driver} ---")
    for state in ("forward", "reverse", "brake", "coast"):
        run_state(motors, driver, state)
    time.sleep(PAUSE_S)


def test_simultaneous(motors: DRV8212P) -> None:
    print("--- Both drivers forward simultaneously ---")
    motors.forward(0)
    motors.forward(1)
    time.sleep(HOLD_S)
    motors.coast_all()
    print("  ok")
    time.sleep(PAUSE_S)

    print("--- Both drivers reverse simultaneously ---")
    motors.reverse(0)
    motors.reverse(1)
    time.sleep(HOLD_S)
    motors.coast_all()
    print("  ok")
    time.sleep(PAUSE_S)

    print("--- Opposing directions (driver 0 forward, driver 1 reverse) ---")
    motors.forward(0)
    motors.reverse(1)
    time.sleep(HOLD_S)
    motors.coast_all()
    print("  ok")
    time.sleep(PAUSE_S)


def main() -> None:
    print("=" * 45)
    print(" DRV8212P motor driver test")
    print("=" * 45)
    print()

    try:
        io = MCP23017()
    except Exception as e:
        print(f"[ERROR] Failed to open MCP23017: {e}")
        sys.exit(1)

    motors = DRV8212P(io)
    print("[OK] MCP23017 and DRV8212P initialised\n")

    try:
        test_all_states(motors, 0)
        test_all_states(motors, 1)
        test_simultaneous(motors)
        print("\nAll tests passed.")
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Ctrl+C detected.")
    finally:
        motors.coast_all()
        io.close()
        print("[OK] Outputs coasted, I2C bus closed.")


if __name__ == "__main__":
    main()
