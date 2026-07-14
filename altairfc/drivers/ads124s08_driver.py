import math
from __future__ import annotations

import ctypes
import logging
from enum import IntEnum
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libads124s08_driver.so"


class Mux(IntEnum):
    VGND = 0x20  # Differential read, AIN2 (Virtual Ground)(+) AIN0 (2.5V ref)(-)
    IVC = 0x30  # Differential read, AIN3(IVC level shift output)(+) AIN0 (2.5V ref)(-)
    ACF = 0x40  # Differential read, AIN4(ACF level shift output)(+) AIN0 (2.5V ref)(-)
    TIA = 0x50  # Differential read, AIN5(TIA output)(+) AIN0 (2.5V ref)(-)
    BOARD_TMP = 0x60  # Differential read, AIN6(Board Temp Sensor)(+) AIN0 (2.5V ref)(-)
    PD_TMP = 0x70  # Differential read, AIN7(PD Temp Sensor)(+) AIN0 (2.5V ref)(-)


class DataRate(IntEnum):
    SPS_2_5 = 0x00
    SPS_5 = 0x01
    SPS_10 = 0x02
    SPS_16_6 = 0x03
    SPS_20 = 0x04
    SPS_50 = 0x05
    SPS_60 = 0x06
    SPS_100 = 0x07
    SPS_200 = 0x08
    SPS_400 = 0x09
    SPS_800 = 0x0A
    SPS_1000 = 0x0B
    SPS_2000 = 0x0C
    SPS_4000 = 0x0D


class Relay(IntEnum):
    ACF = 0x01
    IVC = 0x02
    TIA = 0x04
    TIA_LOWGAIN = 0x0C


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))

    try:
        open_with_drdy_start = lib.ads124s08_open_with_drdy_start
    except AttributeError as exc:
        raise RuntimeError(
            "libads124s08_driver.so does not support hardware DRDY/START; rebuild it with "
            "drivers/build_ads124s08.sh"
        ) from exc
    open_with_drdy_start.restype = ctypes.c_void_p
    open_with_drdy_start.argtypes = [
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_uint,
        ctypes.c_uint,
        ctypes.c_uint,
    ]

    lib.ads124s08_reset.restype = ctypes.c_int
    lib.ads124s08_reset.argtypes = [ctypes.c_void_p]

    lib.ads124s08_configure.restype = ctypes.c_int
    lib.ads124s08_configure.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8 * 5),
    ]

    lib.ads124s08_read_config.restype = ctypes.c_int
    lib.ads124s08_read_config.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8 * 5),
    ]

    lib.ads124s08_read_register.restype = ctypes.c_int
    lib.ads124s08_read_register.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8),
    ]

    lib.ads124s08_write_register.restype = ctypes.c_int
    lib.ads124s08_write_register.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint8,
        ctypes.c_uint8,
    ]

    lib.ads124s08_read_single_shot.restype = ctypes.c_int
    lib.ads124s08_read_single_shot.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int32),
    ]

    lib.ads124s08_switch_relays.restype = ctypes.c_int
    lib.ads124s08_switch_relays.argtypes = [ctypes.c_void_p, ctypes.c_uint8]

    lib.ads124s08_code_to_volts.restype = ctypes.c_float
    lib.ads124s08_code_to_volts.argtypes = [ctypes.c_int32]

    lib.ads124s08_thermistor_volts_to_resistance.restype = ctypes.c_float
    lib.ads124s08_thermistor_volts_to_resistance.argtypes = [ctypes.c_float]

    lib.ads124s08_resistance_to_celsius.restype = ctypes.c_float
    lib.ads124s08_resistance_to_celsius.argtypes = [ctypes.c_float]

    lib.ads124s08_close.restype = None
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
    The V26C Stack has two of these, each with the following inputs:
    Ain2 : Board virtual ground level (4.9V)
    Ain3 : IVC integrator input
    Ain4 : ACF integrator input
    Ain5 : TIA integrator input
    Ain6 : Board temperature sensor
    Ain7 : Photodiode temperature sensor

    switch between them with
    read_voltage() / read_thermistor(), which reconfigure the mux before
    each conversion.

    Usage:
        adc = ads124s08Driver(
            "/dev/spidev0.0",
            "gpiochip0",
            cs_offset=13,
            drdy_offset=22,
            start_offset=25,
        )
        adc.configure()
        pd_v = adc.read_voltage()
        board_tmp = adc.read_board_thermistor()
        adc.close()

    Note: CS is driven internally via libgpiod — no external GPIO handling
    needed, unlike the MCP4728 driver's LDAC pin.
    """

    def __init__(
        self,
        spi_dev: str,
        gpiochip: str,
        cs_offset: int,
        drdy_offset: int,
        start_offset: int,
    ) -> None:
        self._lib = _load_lib()
        self._handle = self._lib.ads124s08_open_with_drdy_start(
            spi_dev.encode(),
            gpiochip.encode(),
            cs_offset,
            drdy_offset,
            start_offset,
        )
        if not self._handle:
            raise OSError(
                f"ads124s08_open_with_drdy_start failed on {spi_dev} "
                f"(CS {gpiochip}:{cs_offset}, DRDY {gpiochip}:{drdy_offset}, "
                f"START {gpiochip}:{start_offset}) — "
                "check the SPI bus and GPIO line assignments"
            )
        logger.info(
            "ads124s08Driver: opened %s CS=%s:%d DRDY=%s:%d START=%s:%d",
            spi_dev,
            gpiochip,
            cs_offset,
            gpiochip,
            drdy_offset,
            gpiochip,
            start_offset,
        )
        if self._lib.ads124s08_reset(self._handle) != 0:
            logger.warning("ads124s08Driver: reset SPI error")

    def reset(self) -> bool:
        if self._lib.ads124s08_reset(self._handle) != 0:
            logger.warning("ads124s08Driver: reset SPI error")
            return False
        return True

    def _configure(self, mux: int, dr: int) -> tuple[int, int, int, int, int] | None:
        out = (ctypes.c_uint8 * 5)()
        ret = self._lib.ads124s08_configure(self._handle, mux, dr, ctypes.byref(out))
        if ret != 0:
            logger.warning("ads124s08Driver: configure SPI error")
            return None
        return tuple(out)

    def read_config(self) -> tuple[int, int, int, int, int] | None:
        out = (ctypes.c_uint8 * 5)()
        ret = self._lib.ads124s08_read_config(self._handle, ctypes.byref(out))
        if ret != 0:
            logger.warning("ads124s08Driver: read_config SPI error")
            return None
        return tuple(out)

    def read_register(self, addr: int) -> int | None:
        out = ctypes.c_uint8()
        ret = self._lib.ads124s08_read_register(self._handle, addr, ctypes.byref(out))
        if ret != 0:
            logger.warning("ads124s08Driver: read_register SPI error")
            return None
        return out.value

    def write_register(self, addr: int, value: int) -> bool:
        ret = self._lib.ads124s08_write_register(self._handle, addr, value)
        if ret != 0:
            logger.warning("ads124s08Driver: write_register SPI error")
            return False
        return True

    def _read_single_shot_raw(self) -> int | None:
        code = ctypes.c_int32()
        ret = self._lib.ads124s08_read_single_shot(self._handle, ctypes.byref(code))
        if ret == -2:
            logger.warning("ads124s08Driver: timed out waiting for DRDY")
            return None
        if ret != 0:
            logger.warning("ads124s08Driver: read_single_shot SPI/GPIO error")
            return None
        return code.value

    def _read_single_shot_volts(self) -> float | None:
        code = ctypes.c_int32()
        ret = self._lib.ads124s08_read_single_shot(self._handle, ctypes.byref(code))
        if ret == -2:
            logger.warning("ads124s08Driver: timed out waiting for DRDY")
            return None
        if ret != 0:
            logger.warning("ads124s08Driver: read_single_shot SPI/GPIO error")
            return None
        return self._lib.ads124s08_code_to_volts(code.value)

    def set_relays(self, relays: int) -> None:
        if self._lib.ads124s08_switch_relays(self._handle, relays) != 0:
            logger.warning("ads124s08Driver: switch_relays SPI error")

    def read_voltage(self) -> float | None:
        """Switch mux to AIN0 single-ended and take one reading, in volts."""
        return self._read_single_shot_volts()

    def read_board_thermistor(self) -> ThermistorReading | None:
        """Switch mux to board temp sensor and take one reading."""
        if self._configure(Mux.BOARD_TMP, DataRate.SPS_100) is None:
            return None
        volts = self._read_single_shot_volts()
        if volts is None:
            return None
        resistance = self._lib.ads124s08_thermistor_volts_to_resistance(volts)
        temperature = self._lib.ads124s08_resistance_to_celsius(resistance)
        return ThermistorReading(
            volts=volts, resistance_ohm=resistance, temperature_c=temperature
        )

    def read_pd_thermistor(self) -> ThermistorReading | None:
        """Switch mux to photodiode temp sensor and take one reading."""
        if self._configure(Mux.PD_TMP, DataRate.SPS_100) is None:
            return None
        volts = self._read_single_shot_volts()
        if volts is None:
            return None
        resistance = self._lib.ads124s08_thermistor_volts_to_resistance(volts)
        # pd thermistor has different beta of 3950. convert resistance to temp here with our own calculation, not the c driver
        temperature_k = 1.0 / (1.0 / 3950.0 + math.log(resistance / 10000.0) / 3950.0)
        temperature_c = temperature_k - 273.15
        return ThermistorReading(
            volts=volts, resistance_ohm=resistance, temperature_c=temperature_c
        )

    def close(self) -> None:
        if self._handle:
            self._lib.ads124s08_close(self._handle)
            self._handle = None
