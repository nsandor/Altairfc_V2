from __future__ import annotations

import logging

import smbus2

logger = logging.getLogger(__name__)

# Register addresses
_IODIRA = 0x00
_IODIRB = 0x01
_GPIOA = 0x12
_GPIOB = 0x13

HIGH = 1
LOW = 0

DEFAULT_ADDR = 0x20
DEFAULT_BUS = 1


class MCP23017:
    """
    Driver for the MCP23017 16-bit I/O expander.

    Pins 0–7  → Port A (GPA0–GPA7)
    Pins 8–15 → Port B (GPB0–GPB7)

    Usage:
        io = MCP23017()
        io.set_output(0)          # configure pin 0 as output
        io.set(0, HIGH)           # drive pin 0 high
        io.set(0, LOW)            # drive pin 0 low
    """

    def __init__(self, address: int = DEFAULT_ADDR, bus: int = DEFAULT_BUS) -> None:
        self._addr = address
        self._bus = smbus2.SMBus(bus)
        # Shadow registers so we can do read-modify-write without an extra I2C read
        self._iodir = [
            self._bus.read_byte_data(address, _IODIRA),
            self._bus.read_byte_data(address, _IODIRB),
        ]
        self._gpio = [
            self._bus.read_byte_data(address, _GPIOA),
            self._bus.read_byte_data(address, _GPIOB),
        ]
        logger.info("MCP23017: opened bus %d address 0x%02X", bus, address)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_output(self, pin: int) -> None:
        """Configure a pin as an output (clears the IODIR bit)."""
        port, bit, mask = self._decode(pin)
        iodir_reg = _IODIRA if port == 0 else _IODIRB
        self._iodir[port] &= ~mask
        self._bus.write_byte_data(self._addr, iodir_reg, self._iodir[port])

    def set_input(self, pin: int) -> None:
        """Configure a pin as an input (sets the IODIR bit)."""
        port, bit, mask = self._decode(pin)
        iodir_reg = _IODIRA if port == 0 else _IODIRB
        self._iodir[port] |= mask
        self._bus.write_byte_data(self._addr, iodir_reg, self._iodir[port])

    def set(self, pin: int, value: int) -> None:
        """Drive an output pin HIGH (1) or LOW (0)."""
        port, bit, mask = self._decode(pin)
        gpio_reg = _GPIOA if port == 0 else _GPIOB
        if value:
            self._gpio[port] |= mask
        else:
            self._gpio[port] &= ~mask
        self._bus.write_byte_data(self._addr, gpio_reg, self._gpio[port])

    def get(self, pin: int) -> int:
        """Read the current logic level of a pin. Returns HIGH or LOW."""
        port, bit, mask = self._decode(pin)
        gpio_reg = _GPIOA if port == 0 else _GPIOB
        raw = self._bus.read_byte_data(self._addr, gpio_reg)
        return HIGH if (raw & mask) else LOW

    def close(self) -> None:
        self._bus.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _decode(pin: int) -> tuple[int, int, int]:
        if not 0 <= pin <= 15:
            raise ValueError(f"pin must be 0–15, got {pin}")
        port = pin >> 3  # 0 for pins 0-7, 1 for pins 8-15
        bit = pin & 0x07
        mask = 1 << bit
        return port, bit, mask
