from __future__ import annotations

import ctypes
import logging
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libads124s08_driver.so"

MUX_AIN0_AVSS = 0x8       # single-ended AIN0 vs AVSS (photodiode TIA)
MUX_AIN2_AIN3_DIFF = 0x5  # differential AIN2(+) - AIN3(-) (thermistor bridge)
PGA_BYPASS_ON = 0x1
PGA_BYPASS_OFF = 0x0


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))

    lib.ads124s08_open.restype  = ctypes.c_void_p
    lib.ads124s08_open.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]

    lib.ads124s08_reset.restype  = ctypes.c_int
    lib.ads124s08_reset.argtypes = [ctypes.c_void_p]

    lib.ads124s08_configure.restype  = ctypes.c_int
    lib.ads124s08_configure.argtypes = [
        ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8 * 4),
    ]

    lib.ads124s08_read_config.restype  = ctypes.c_int
    lib.ads124s08_read_config.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8 * 5)]

    lib.ads124s08_read_single_shot.restype  = ctypes.c_int
    lib.ads124s08_read_single_shot.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int32)]

    lib.ads124s08_code_to_volts.restype  = ctypes.c_float
    lib.ads124s08_code_to_volts.argtypes = [ctypes.c_int32]

    lib.ads124s08_thermistor_volts_to_resistance.restype  = ctypes.c_float
    lib.ads124s08_thermistor_volts_to_resistance.argtypes = [ctypes.c_float]

    lib.ads124s08_resistance_to_celsius.restype  = ctypes.c_float
    lib.ads124s08_resistance_to_celsius.argtypes = [ctypes.c_float]

    lib.ads124s08_close.restype  = None
    lib.ads124s08_close.argtypes = [ctypes.c_void_p]
    return lib


@dataclass
class ThermistorReading:
    volts: float
    resistance_ohm: float
    temperature_c: float


class ads124s08Driver:
    """
    Thin Python wrapper around libads124s08_driver.so.

    One instance per physical ads124s08 breakout. Each breakout has both a
    photodiode TIA input (AIN0, single-ended) and a thermistor Wheatstone
    bridge input (AIN2+/AIN3-, differential) — switch between them with
    read_photodiode() / read_bridge(), which reconfigure the mux before
    each conversion.

    Usage:
        adc = ads124s08Driver("/dev/spidev0.0", "gpiochip0", cs_offset=17)
        pd_v = adc.read_photodiode()
        bridge = adc.read_bridge()  # BridgeReading(volts, resistance_ohm, temperature_c)
        adc.close()

    Note: CS is driven internally via libgpiod — no external GPIO handling
    needed, unlike the MCP4728 driver's LDAC pin.
    """

    def __init__(self, spi_dev: str, gpiochip: str, cs_offset: int) -> None:
        self._lib = _load_lib()
        self._handle = self._lib.ads124s08_open(
            spi_dev.encode(), gpiochip.encode(), cs_offset
        )
        if not self._handle:
            raise OSError(
                f"ads124s08_open failed on {spi_dev} (CS {gpiochip}:{cs_offset}) — "
                "check SPI bus, gpiochip name, and CS line number"
            )
        logger.info("ads124s08Driver: opened %s CS=%s:%d", spi_dev, gpiochip, cs_offset)
        if self._lib.ads124s08_reset(self._handle) != 0:
            logger.warning("ads124s08Driver: reset SPI error")

    def _configure(self, mux: int, pga_bypass: int) -> tuple[int, int, int, int] | None:
        out = (ctypes.c_uint8 * 4)()
        ret = self._lib.ads124s08_configure(self._handle, mux, pga_bypass, ctypes.byref(out))
        if ret != 0:
            logger.warning("ads124s08Driver: configure SPI error")
            return None
        return tuple(out)

    def read_config(self) -> tuple[int, int, int, int] | None:
        out = (ctypes.c_uint8 * 4)()
        ret = self._lib.ads124s08_read_config(self._handle, ctypes.byref(out))
        if ret != 0:
            logger.warning("ads124s08Driver: read_config SPI error")
            return None
        return tuple(out)

    def _read_single_shot_volts(self, mux: int, pga_bypass: int) -> float | None:
        if self._configure(mux, pga_bypass) is None:
            return None
        code = ctypes.c_int32()
        ret = self._lib.ads124s08_read_single_shot(self._handle, ctypes.byref(code))
        if ret != 0:
            logger.warning("ads124s08Driver: read_single_shot SPI error")
            return None
        return self._lib.ads124s08_code_to_volts(code.value)

    def read_photodiode(self) -> float | None:
        """Switch mux to AIN0 single-ended and take one reading, in volts."""
        return self._read_single_shot_volts(MUX_AIN0_AVSS, PGA_BYPASS_ON)

    def read_bridge(self) -> BridgeReading | None:
        """Switch mux to AIN2-AIN3 differential and take one reading."""
        volts = self._read_single_shot_volts(MUX_AIN2_AIN3_DIFF, PGA_BYPASS_OFF)
        if volts is None:
            return None
        resistance = self._lib.ads124s08_bridge_volts_to_resistance(volts)
        temperature = self._lib.ads124s08_resistance_to_celsius(resistance)
        return BridgeReading(volts=volts, resistance_ohm=resistance, temperature_c=temperature)

    def close(self) -> None:
        if self._handle:
            self._lib.ads124s08_close(self._handle)
            self._handle = None
