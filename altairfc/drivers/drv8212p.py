from __future__ import annotations

from drivers.mcp23017 import MCP23017, HIGH, LOW

# MCP23017 pin assignments (pins 8-15 = GPB0-GPB7)
# Driver 1: IN1=GPB2 (pin 10), IN2=GPB3 (pin 11)
# Driver 2: IN1=GPB0 (pin 8),  IN2=GPB1 (pin 9)
_PINS: tuple[tuple[int, int], tuple[int, int]] = (
    (10, 11),  # driver 1: (IN1, IN2)
    (8,  9),   # driver 2: (IN1, IN2)
)


class DRV8212P:
    """
    Controls two DRV8212P H-bridge motor drivers via an MCP23017 I/O expander.

    Truth table (IN1, IN2):
        forward : (H, L)
        reverse : (L, H)
        brake   : (H, H)
        coast    : (L, L)

    driver_index is 0 or 1.
    """

    def __init__(self, io: MCP23017) -> None:
        self._io = io
        for in1, in2 in _PINS:
            io.set_output(in1)
            io.set_output(in2)
            io.set(in1, LOW)
            io.set(in2, LOW)

    def forward(self, driver: int) -> None:
        in1, in2 = self._pins(driver)
        self._io.set(in1, HIGH)
        self._io.set(in2, LOW)

    def reverse(self, driver: int) -> None:
        in1, in2 = self._pins(driver)
        self._io.set(in1, LOW)
        self._io.set(in2, HIGH)

    def brake(self, driver: int) -> None:
        in1, in2 = self._pins(driver)
        self._io.set(in1, HIGH)
        self._io.set(in2, HIGH)

    def coast(self, driver: int) -> None:
        in1, in2 = self._pins(driver)
        self._io.set(in1, LOW)
        self._io.set(in2, LOW)

    def coast_all(self) -> None:
        for i in range(len(_PINS)):
            self.coast(i)

    @staticmethod
    def _pins(driver: int) -> tuple[int, int]:
        if driver not in (0, 1):
            raise ValueError(f"driver must be 0 or 1, got {driver}")
        return _PINS[driver]
