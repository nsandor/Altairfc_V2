from __future__ import annotations

import ctypes
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

_SO_PATH = Path(__file__).parent / "libmcp4728_driver.so"

NUM_CHANNELS = 4
MAX_CODE = 4095


class MCP4728State(ctypes.Structure):
    _fields_ = [
        ("code", ctypes.c_uint16 * NUM_CHANNELS),
        ("vref_vdd", ctypes.c_uint8 * NUM_CHANNELS),
        ("gain2x", ctypes.c_uint8 * NUM_CHANNELS),
        ("powered_down", ctypes.c_uint8 * NUM_CHANNELS),
    ]


def _load_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(str(_SO_PATH))
    lib.mcp4728_open.restype  = ctypes.c_int
    lib.mcp4728_open.argtypes = [ctypes.c_char_p]

    lib.mcp4728_multi_write.restype  = ctypes.c_int
    lib.mcp4728_multi_write.argtypes = [ctypes.c_int, ctypes.POINTER(MCP4728State)]

    lib.mcp4728_fast_write.restype  = ctypes.c_int
    lib.mcp4728_fast_write.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint16 * NUM_CHANNELS)]

    lib.mcp4728_read.restype  = ctypes.c_int
    lib.mcp4728_read.argtypes = [ctypes.c_int, ctypes.POINTER(MCP4728State)]

    lib.mcp4728_write_eeprom.restype  = ctypes.c_int
    lib.mcp4728_write_eeprom.argtypes = [ctypes.c_int, ctypes.POINTER(MCP4728State)]

    lib.mcp4728_ping.restype  = ctypes.c_int
    lib.mcp4728_ping.argtypes = [ctypes.c_int]

    lib.mcp4728_close.restype  = None
    lib.mcp4728_close.argtypes = [ctypes.c_int]
    return lib


class MCP4728Driver:
    """
    Thin Python wrapper around libmcp4728_driver.so.

    Channels (0-indexed): 0=A, 1=B, 2=C, 3=D.

    Usage:
        dac = MCP4728Driver()
        dac.set_vdd_reference([2047, 2047, 2047, 2047])   # one-time Multi-Write
        dac.set_codes([1000, 2000, 3000, 4095])           # fast updates after that
        dac.close()

    Note: LDAC must be held low (or pulsed) externally for writes to reach
    VOUT immediately — this driver does not control the LDAC GPIO pin.
    """

    def __init__(self, i2c_dev: str = "/dev/i2c-1") -> None:
        self._lib = _load_lib()
        self._fd = self._lib.mcp4728_open(i2c_dev.encode())
        if self._fd < 0:
            raise OSError(
                f"mcp4728_open failed on {i2c_dev} — "
                "check I2C bus and address 0x60"
            )
        logger.info("MCP4728Driver: opened %s (fd=%d)", i2c_dev, self._fd)

    def set_vdd_reference(self, codes: list[int]) -> bool:
        """Multi-Write all 4 channels, forcing Vref=Vdd and gain=1x."""
        state = MCP4728State(
            code=(ctypes.c_uint16 * NUM_CHANNELS)(*codes),
            vref_vdd=(ctypes.c_uint8 * NUM_CHANNELS)(*([1] * NUM_CHANNELS)),
            gain2x=(ctypes.c_uint8 * NUM_CHANNELS)(*([0] * NUM_CHANNELS)),
            powered_down=(ctypes.c_uint8 * NUM_CHANNELS)(*([0] * NUM_CHANNELS)),
        )
        ret = self._lib.mcp4728_multi_write(self._fd, ctypes.byref(state))
        if ret != 0:
            logger.warning("MCP4728Driver: multi_write I2C error")
        return ret == 0

    def set_codes(self, codes: list[int]) -> bool:
        """Fast Write all 4 channels' DAC codes (Vref/gain unchanged)."""
        buf = (ctypes.c_uint16 * NUM_CHANNELS)(*codes)
        ret = self._lib.mcp4728_fast_write(self._fd, ctypes.byref(buf))
        if ret != 0:
            logger.warning("MCP4728Driver: fast_write I2C error")
        return ret == 0

    def read(self) -> MCP4728State | None:
        state = MCP4728State()
        ret = self._lib.mcp4728_read(self._fd, ctypes.byref(state))
        if ret == 0:
            return state
        logger.warning("MCP4728Driver: read I2C error")
        return None

    def write_eeprom(self, codes: list[int]) -> bool:
        """Persist all 4 channels (Vref=Vdd, gain=1x) to EEPROM. Blocks ~50 ms."""
        state = MCP4728State(
            code=(ctypes.c_uint16 * NUM_CHANNELS)(*codes),
            vref_vdd=(ctypes.c_uint8 * NUM_CHANNELS)(*([1] * NUM_CHANNELS)),
            gain2x=(ctypes.c_uint8 * NUM_CHANNELS)(*([0] * NUM_CHANNELS)),
            powered_down=(ctypes.c_uint8 * NUM_CHANNELS)(*([0] * NUM_CHANNELS)),
        )
        ret = self._lib.mcp4728_write_eeprom(self._fd, ctypes.byref(state))
        if ret != 0:
            logger.warning("MCP4728Driver: write_eeprom I2C error")
        return ret == 0

    def ping(self) -> bool:
        return self._lib.mcp4728_ping(self._fd) == 0

    def close(self) -> None:
        if self._fd >= 0:
            self._lib.mcp4728_close(self._fd)
            self._fd = -1
