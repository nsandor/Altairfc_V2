from __future__ import annotations

import ctypes
import logging
from enum import IntEnum
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libdac5311_driver.so"


class PowerDownMode(IntEnum):
    NORMAL = 0x00
    OUTPUT_1K_TO_GND = 0x01
    OUTPUT_100K_TO_GND = 0x02
    HIGH_Z = 0x03


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))

    lib.dac5311_open.restype = ctypes.c_void_p
    lib.dac5311_open.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]

    lib.dac5311_write_value.restype = ctypes.c_int
    lib.dac5311_write_value.argtypes = [ctypes.c_void_p, ctypes.c_uint8]

    lib.dac5311_power_down.restype = ctypes.c_int
    lib.dac5311_power_down.argtypes = [ctypes.c_void_p, ctypes.c_uint8]

    lib.dac5311_close.restype = None
    lib.dac5311_close.argtypes = [ctypes.c_void_p]
    return lib


class dac5311Driver:
    """
    Thin Python wrapper around libdac5311_driver.so.

    Usage:
        dac = dac5311Driver("/dev/spidev0.0", "gpiochip0", cs_offset=17)
        dac.set_value(128) # mid-scale for 8-bit DAC
        dac.power_down(PowerDownMode.HIGH_Z)
        dac.close()
    """

    def __init__(
        self, spi_dev: str, gpiochip: str, cs_offset: int, v_ref: float = 5.1
    ) -> None:
        self.v_ref = v_ref
        self._lib = _load_lib()
        self._handle = self._lib.dac5311_open(
            spi_dev.encode(), gpiochip.encode(), cs_offset
        )
        if not self._handle:
            raise OSError(
                f"dac5311_open failed on {spi_dev} (CS {gpiochip}:{cs_offset}) — "
                "check SPI bus, gpiochip name, and CS line number"
            )
        logger.info("dac5311Driver: opened %s CS=%s:%d", spi_dev, gpiochip, cs_offset)

    def set_value(self, value: int) -> None:
        """
        Write an 8-bit value (0-255) to the DAC in Normal Operation mode.
        """
        if not (0 <= value <= 255):
            raise ValueError("DAC5311 value must be between 0 and 255")
        if self._lib.dac5311_write_value(self._handle, value) != 0:
            logger.warning("dac5311Driver: set_value SPI error")

    def set_voltage(self, volts: float) -> float:
        """
        Set the DAC output to the specified voltage.
        Returns the actual voltage set (rounded to the nearest valid DAC code).
        """
        code = round((volts / self.v_ref) * 256)
        code = max(0, min(255, code))
        self.set_value(code)
        actual_volts = (code / 256) * self.v_ref
        return actual_volts

    def power_down(self, mode: PowerDownMode) -> None:
        """
        Enter one of the power-down modes.
        """
        if self._lib.dac5311_power_down(self._handle, mode) != 0:
            logger.warning("dac5311Driver: power_down SPI error")

    def close(self) -> None:
        if self._handle:
            self._lib.dac5311_close(self._handle)
            self._handle = None
