"""High-level driver for the UVIC photodiode readout (PDRO) board.

The PDRO contains two independent readout paths (Sergeant and Soldier), each
with an ADS124S08 ADC and DAC5311 bias DAC.  An MCP23017 and the switched
integrators are shared by both paths.  This module owns those lower-level
drivers so applications do not need to coordinate their initialization,
operation, or shutdown directly.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum, IntFlag
from typing import Iterable

from drivers.ads124s08_driver import (
    DataRate,
    Mux,
    Relay,
    ThermistorReading,
    ads124s08Driver,
)
from drivers.dac5311_driver import dac5311Driver
from drivers.integrator_driver import IntegratorDriver
from drivers.mcp23017 import DEFAULT_ADDR, DEFAULT_BUS, MCP23017

logger = logging.getLogger(__name__)

__all__ = [
    "DataRate",
    "Input",
    "IntegrationReading",
    "Readout",
    "ReadoutPinout",
    "SignalPath",
    "ThermistorReading",
    "UVICPDRO",
]


class Readout(Enum):
    """One of the two photodiode readout paths on the PDRO board."""

    SERGEANT = "sergeant"
    SOLDIER = "soldier"


class Input(Enum):
    """Analog inputs available on each PDRO readout path."""

    VGND = "virtual_ground"
    TIA = "tia"
    TIA_LOW_GAIN = "tia_low_gain"
    IVC = "ivc"
    ACF = "acf"


class SignalPath(IntFlag):
    """Relay-controlled signal paths on a readout.

    Values may be combined for board diagnostics.  Normal measurements use
    :meth:`UVICPDRO.read_voltage`, which selects the required path itself.
    """

    NONE = 0
    ACF = Relay.ACF.value
    IVC = Relay.IVC.value
    TIA = Relay.TIA.value
    TIA_LOW_GAIN = Relay.TIA_LOWGAIN.value


@dataclass(frozen=True)
class ReadoutPinout:
    """Raspberry Pi GPIO assignments for one PDRO readout path."""

    adc_cs: int
    adc_drdy: int
    adc_start: int
    dac_cs: int


@dataclass(frozen=True)
class IntegrationReading:
    """Result and measured timing of an integrate-and-hold conversion."""

    voltage_v: float | None
    requested_time_us: float
    actual_time_us: float


DEFAULT_PINOUTS = {
    Readout.SERGEANT: ReadoutPinout(
        adc_cs=13,
        adc_drdy=22,
        adc_start=25,
        dac_cs=12,
    ),
    Readout.SOLDIER: ReadoutPinout(
        adc_cs=19,
        adc_drdy=24,
        adc_start=8,
        dac_cs=6,
    ),
}

_INPUT_MUX = {
    Input.VGND: Mux.VGND,
    Input.TIA: Mux.TIA,
    Input.TIA_LOW_GAIN: Mux.TIA,
    Input.IVC: Mux.IVC,
    Input.ACF: Mux.ACF,
}

_INPUT_PATH = {
    Input.VGND: SignalPath.NONE,
    Input.TIA: SignalPath.TIA,
    Input.TIA_LOW_GAIN: SignalPath.TIA_LOW_GAIN,
    Input.IVC: SignalPath.IVC,
    Input.ACF: SignalPath.ACF,
}


@dataclass
class _ReadoutHardware:
    adc: ads124s08Driver
    dac: dac5311Driver


class UVICPDRO:
    """Own and operate all electronics on one UVIC PDRO board.

    By default both readout paths are opened using the flight-computer pinout.
    A subset can be selected for isolated hardware testing.
    """

    MAX_BIAS_V = 5.1
    RELAY_SETTLE_S = 0.05

    def __init__(
        self,
        readouts: Iterable[Readout] | None = None,
        *,
        spi_dev: str = "/dev/spidev0.0",
        gpiochip: str = "gpiochip0",
        io_address: int = DEFAULT_ADDR,
        io_bus: int = DEFAULT_BUS,
        pinouts: dict[Readout, ReadoutPinout] | None = None,
    ) -> None:
        requested_readouts = tuple(Readout) if readouts is None else readouts
        selected = tuple(Readout(readout) for readout in requested_readouts)
        if not selected:
            raise ValueError("at least one PDRO readout must be selected")
        if len(set(selected)) != len(selected):
            raise ValueError("a PDRO readout may only be selected once")

        configured_pinouts = DEFAULT_PINOUTS if pinouts is None else pinouts
        missing = [
            readout.value for readout in selected if readout not in configured_pinouts
        ]
        if missing:
            raise ValueError(f"missing pinout for: {', '.join(missing)}")

        self._hardware: dict[Readout, _ReadoutHardware] = {}
        self._io: MCP23017 | None = None
        self._integrator: IntegratorDriver | None = None
        self._closed = False

        try:
            # IntegratorDriver releases and verifies the shared ADC enable line.
            # It must be initialized before either ADC issues an SPI command.
            self._io = MCP23017(address=io_address, bus=io_bus)
            self._integrator = IntegratorDriver(self._io)

            for readout in selected:
                pins = configured_pinouts[readout]
                adc = ads124s08Driver(
                    spi_dev,
                    gpiochip,
                    pins.adc_cs,
                    pins.adc_drdy,
                    pins.adc_start,
                )
                try:
                    dac = dac5311Driver(spi_dev, gpiochip, pins.dac_cs)
                except Exception:
                    adc.close()
                    raise
                self._hardware[readout] = _ReadoutHardware(adc=adc, dac=dac)
                adc.set_relays(0)
                dac.set_voltage(0.0)
        except Exception:
            self._close_resources()
            self._closed = True
            raise

    @property
    def readouts(self) -> tuple[Readout, ...]:
        """Readout paths opened by this driver."""

        return tuple(self._hardware)

    def _get_hardware(self, readout: Readout) -> _ReadoutHardware:
        if self._closed:
            raise RuntimeError("UVIC PDRO driver is closed")
        try:
            return self._hardware[Readout(readout)]
        except KeyError as exc:
            raise ValueError(f"{Readout(readout).value} readout is not open") from exc

    def configure_input(
        self,
        readout: Readout,
        input_channel: Input,
        data_rate: DataRate = DataRate.SPS_100,
    ) -> tuple[int, int, int, int, int] | None:
        """Configure a readout ADC for a semantic PDRO input."""

        hardware = self._get_hardware(readout)
        return hardware.adc._configure(
            _INPUT_MUX[Input(input_channel)], DataRate(data_rate)
        )

    def read_voltage(
        self,
        readout: Readout,
        input_channel: Input,
        data_rate: DataRate = DataRate.SPS_100,
        *,
        select_signal_path: bool = True,
    ) -> float | None:
        """Configure an input and take one voltage measurement.

        Relay selection is handled automatically.  Board diagnostics may pass
        ``select_signal_path=False`` to inspect nodes without changing the
        current signal path.
        """

        input_channel = Input(input_channel)
        if select_signal_path:
            self.set_signal_paths(readout, _INPUT_PATH[input_channel])
        if self.configure_input(readout, input_channel, data_rate) is None:
            return None
        return self._get_hardware(readout).adc.read_voltage()

    def read_board_thermistor(
        self,
        readout: Readout,
        data_rate: DataRate = DataRate.SPS_100,
    ) -> ThermistorReading | None:
        """Read the temperature sensor mounted on a PDRO readout path."""

        return self._get_hardware(readout).adc.read_board_thermistor(data_rate)

    def read_photodiode_thermistor(
        self,
        readout: Readout,
        data_rate: DataRate = DataRate.SPS_100,
    ) -> ThermistorReading | None:
        """Read the photodiode temperature sensor on a readout path."""

        return self._get_hardware(readout).adc.read_pd_thermistor(data_rate)

    def set_bias_voltage(self, readout: Readout, volts: float) -> float:
        """Set a readout's photodiode bias and return the quantized voltage."""

        if not 0.0 <= volts <= self.MAX_BIAS_V:
            raise ValueError(
                f"bias voltage must be between 0 and {self.MAX_BIAS_V} V"
            )
        return self._get_hardware(readout).dac.set_voltage(volts)

    def set_signal_paths(
        self,
        readout: Readout,
        paths: SignalPath | Iterable[SignalPath],
    ) -> None:
        """Actuate one or more PDRO signal-path relays."""

        if isinstance(paths, SignalPath):
            selected = paths
        else:
            selected = SignalPath.NONE
            for path in paths:
                selected |= SignalPath(path)
        self._get_hardware(readout).adc.set_relays(selected.value)

    def integrate_and_read(
        self,
        readout: Readout,
        input_channel: Input,
        time_us: float,
        data_rate: DataRate = DataRate.SPS_100,
    ) -> IntegrationReading:
        """Integrate an IVC or ACF input, hold it, and take one conversion."""

        input_channel = Input(input_channel)
        if input_channel not in (Input.IVC, Input.ACF):
            raise ValueError("only IVC and ACF inputs support integration")
        if time_us <= 0:
            raise ValueError("integration time must be greater than zero")

        hardware = self._get_hardware(readout)
        if self.configure_input(readout, input_channel, data_rate) is None:
            raise OSError(f"failed to configure {input_channel.value} input")
        self.set_signal_paths(readout, _INPUT_PATH[input_channel])
        time.sleep(self.RELAY_SETTLE_S)

        assert self._integrator is not None
        try:
            started, ended = self._integrator.integrate_and_hold(
                time_us, print_timing=False
            )
            voltage = hardware.adc.read_voltage()
        finally:
            self._integrator.reset()

        return IntegrationReading(
            voltage_v=voltage,
            requested_time_us=time_us,
            actual_time_us=(ended - started) * 1_000_000.0,
        )

    # ADC register methods are intentionally diagnostic-only.  They keep the
    # production test from reaching through this driver to a component object.
    def read_adc_config(
        self, readout: Readout
    ) -> tuple[int, int, int, int, int] | None:
        return self._get_hardware(readout).adc.read_config()

    def read_adc_register(self, readout: Readout, address: int) -> int | None:
        if not 0 <= address <= 0x1F:
            raise ValueError("ADC register address must be between 0 and 0x1F")
        return self._get_hardware(readout).adc.read_register(address)

    def write_adc_register(
        self, readout: Readout, address: int, value: int
    ) -> bool:
        if not 0 <= address <= 0x1F:
            raise ValueError("ADC register address must be between 0 and 0x1F")
        if not 0 <= value <= 0xFF:
            raise ValueError("ADC register value must be between 0 and 0xFF")
        return self._get_hardware(readout).adc.write_register(address, value)

    def reset_adc(self, readout: Readout) -> bool:
        return self._get_hardware(readout).adc.reset()

    def _close_resources(self) -> None:
        """Best-effort safe shutdown, including partially constructed state."""

        if self._integrator is not None:
            try:
                self._integrator.reset()
            except Exception:
                logger.exception("failed to reset PDRO integrators during shutdown")

        for readout, hardware in self._hardware.items():
            try:
                hardware.adc.set_relays(0)
            except Exception:
                logger.exception("failed to open %s PDRO relays", readout.value)
            try:
                hardware.dac.set_voltage(0.0)
            except Exception:
                logger.exception("failed to zero %s PDRO bias", readout.value)
            try:
                hardware.adc.close()
            except Exception:
                logger.exception("failed to close %s PDRO ADC", readout.value)
            try:
                hardware.dac.close()
            except Exception:
                logger.exception("failed to close %s PDRO DAC", readout.value)
        self._hardware.clear()

        if self._io is not None:
            try:
                self._io.close()
            except Exception:
                logger.exception("failed to close PDRO I/O expander")
            self._io = None
        self._integrator = None

    def close(self) -> None:
        """Return the PDRO to a safe state and release all device handles."""

        if not self._closed:
            self._close_resources()
            self._closed = True

    def __enter__(self) -> UVICPDRO:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
