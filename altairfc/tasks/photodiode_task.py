from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)


class PhotodiodeTask(BaseTask):
    """Collect the six fixed UVIC PDRO test-campaign measurements.

    The low-gain TIA measurements use ``signal_data_rate``. Both temperature
    measurements use ``temperature_data_rate``. A complete six-value pass is
    published only after every conversion succeeds.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        *,
        signal_data_rate: str = "SPS_1000",
        temperature_data_rate: str = "SPS_100",
        bias_voltage_v: float = 0.0,
        pdro_factory: Callable[..., Any] | None = None,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._signal_data_rate_name = signal_data_rate
        self._temperature_data_rate_name = temperature_data_rate
        self._bias_voltage_v = bias_voltage_v
        self._pdro_factory = pdro_factory
        self._pdro = None
        self._signal_data_rate = None
        self._temperature_data_rate = None
        self._readouts = ()

    @staticmethod
    def _parse_data_rate(data_rate, name: str):
        try:
            return data_rate[name.strip().upper()]
        except KeyError as exc:
            choices = ", ".join(data_rate.__members__)
            raise ValueError(
                f"unknown PDRO data rate {name!r}; choose from {choices}"
            ) from exc

    def setup(self) -> None:
        # Keep hardware imports lazy so a disabled task does not load Pi-only
        # dependencies during flight-computer startup.
        from drivers.uvic_pdro import DataRate, Readout, UVICPDRO

        self.datastore.write("system.photodiode_connected", 0)
        signal_rate = self._parse_data_rate(
            DataRate, self._signal_data_rate_name
        )
        temperature_rate = self._parse_data_rate(
            DataRate, self._temperature_data_rate_name
        )
        readouts = (Readout.SERGEANT, Readout.SOLDIER)
        factory = self._pdro_factory or UVICPDRO
        pdro = factory(readouts=readouts)
        try:
            for readout in readouts:
                pdro.set_bias_voltage(readout, self._bias_voltage_v)
        except Exception:
            try:
                pdro.close()
            except Exception:
                logger.exception("PhotodiodeTask: cleanup after setup failure failed")
            raise

        self._pdro = pdro
        self._signal_data_rate = signal_rate
        self._temperature_data_rate = temperature_rate
        self._readouts = readouts
        self.datastore.write("system.photodiode_connected", 1)
        logger.info(
            "PhotodiodeTask: UVIC PDRO ready (signal=%s, temperature=%s, "
            "bias=%.3f V)",
            signal_rate.name,
            temperature_rate.name,
            self._bias_voltage_v,
        )

    @staticmethod
    def _require_value(value, measurement: str) -> float:
        if value is None:
            raise OSError(f"UVIC PDRO conversion failed for {measurement}")
        return float(value)

    @classmethod
    def _require_temperature(cls, reading, measurement: str) -> float:
        value = None if reading is None else reading.temperature_c
        return cls._require_value(value, measurement)

    def execute(self) -> None:
        if (
            self._pdro is None
            or self._signal_data_rate is None
            or self._temperature_data_rate is None
        ):
            raise RuntimeError("UVIC PDRO is not initialized")

        from drivers.uvic_pdro import Input

        sergeant, soldier = self._readouts
        sergeant_tia = self._pdro.read_voltage(
            sergeant, Input.TIA_LOW_GAIN, self._signal_data_rate
        )
        soldier_tia = self._pdro.read_voltage(
            soldier, Input.TIA_LOW_GAIN, self._signal_data_rate
        )
        sergeant_board = self._pdro.read_board_thermistor(
            sergeant, self._temperature_data_rate
        )
        soldier_board = self._pdro.read_board_thermistor(
            soldier, self._temperature_data_rate
        )
        sergeant_photodiode = self._pdro.read_photodiode_thermistor(
            sergeant, self._temperature_data_rate
        )
        soldier_photodiode = self._pdro.read_photodiode_thermistor(
            soldier, self._temperature_data_rate
        )

        ordered_values = (
            (
                "photodiode.sergeant.tia_low_gain",
                self._require_value(sergeant_tia, "sergeant TIA low gain"),
            ),
            (
                "photodiode.soldier.tia_low_gain",
                self._require_value(soldier_tia, "soldier TIA low gain"),
            ),
            (
                "photodiode.sergeant.board_temperature",
                self._require_temperature(
                    sergeant_board,
                    "sergeant board temperature",
                ),
            ),
            (
                "photodiode.soldier.board_temperature",
                self._require_temperature(
                    soldier_board,
                    "soldier board temperature",
                ),
            ),
            (
                "photodiode.sergeant.photodiode_temperature",
                self._require_temperature(
                    sergeant_photodiode,
                    "sergeant photodiode temperature",
                ),
            ),
            (
                "photodiode.soldier.photodiode_temperature",
                self._require_temperature(
                    soldier_photodiode,
                    "soldier photodiode temperature",
                ),
            ),
        )
        timestamp = time.monotonic()
        for key, value in ordered_values:
            self.datastore.write(key, value, timestamp=timestamp)

    def teardown(self) -> None:
        pdro = self._pdro
        self._pdro = None
        self._signal_data_rate = None
        self._temperature_data_rate = None
        self._readouts = ()
        try:
            if pdro is not None:
                pdro.close()
        finally:
            self.datastore.write("system.photodiode_connected", 0)
        logger.info("PhotodiodeTask: UVIC PDRO closed")
