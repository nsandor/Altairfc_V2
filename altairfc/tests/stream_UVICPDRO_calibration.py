"""Stream UVIC PDRO calibration data to the terminal and a CSV file.

Examples (run from ``altairfc``):

    python tests/stream_UVICPDRO_calibration.py -b both -r SPS_100 -c TIA board_temp -v 2.5
    python tests/stream_UVICPDRO_calibration.py -b both -r SPS_1000 -r2 SPS_100 -c TIA VGND board_temp -v 2.5
    python tests/stream_UVICPDRO_calibration.py -b sgt -r SPS_1000 -c ACF IVC -i 500 -v 3.0

Sampling continues until Ctrl+C.  Every measurement is printed and flushed to
the CSV immediately so that data already collected survives an interrupted run.
Select ``bridge_temp`` and/or ``current`` with ``-c`` to stream the ADS1115
AIN2-AIN3 thermistor bridge temperature and AIN0 current through the 2.2 ohm
current-monitoring resistor alongside the UVIC PDRO channels.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.ads124s08_driver import (  # noqa: E402
    DataRate,
    Mux,
    Relay,
    ads124s08Driver,
)
from drivers.dac5311_driver import dac5311Driver  # noqa: E402

if TYPE_CHECKING:
    from drivers.integrator_driver import IntegratorDriver


SPI_DEVICE = "/dev/spidev0.0"
GPIO_CHIP = "gpiochip0"
BOARD_CONFIG = {
    # name, ADC chip select, ADC DRDY, ADC START, bias DAC chip select
    "sgt": ("sgt", 13, 22, 25, 12),
    "sol": ("sol", 19, 24, 8, 6),
}
DEFAULT_INTEGRATION_US = 1000.0

# LED-driver thermistor bridge (ADS1115 AIN2-AIN3).  The bridge has three
# fixed 10 kohm legs and one 10 kohm NTC, and is excited from 3.3 V.
ADS1115_ADDR = 0x4A
ADS1115_BUS = 1
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG = 0x01
ADS1115_MUX_DIFF_2_3 = 0b011
ADS1115_MUX_SINGLE_0 = 0b100
ADS1115_DR_128SPS = 0b100
ADS1115_COMP_QUE_DISABLE = 0b11
ADS1115_GAIN_FSR = {
    0: (0b000, 6.144),
    1: (0b001, 4.096),
    2: (0b010, 2.048),
    4: (0b011, 1.024),
    8: (0b100, 0.512),
    16: (0b101, 0.256),
}
BRIDGE_R_OHM = 10000.0
BRIDGE_EXCITATION_V = 3.3
THERMISTOR_R25_OHM = 10000.0
THERMISTOR_B_K = 3380.0
THERMISTOR_T0_K = 298.15
CURRENT_SENSE_RESISTOR_OHM = 2.2
ADS1115_BRIDGE_GAIN = 2
ADS1115_CURRENT_GAIN = 2

CHANNEL_ALIASES = {
    "vgnd": "VGND",
    "tia": "TIA",
    "tia_lowgain": "TIA_LOWGAIN",
    "tia_low_gain": "TIA_LOWGAIN",
    "acf": "ACF",
    "ivc": "IVC",
    "board_temp": "BOARD_TEMP",
    "board_temperature": "BOARD_TEMP",
    "diode_temp": "DIODE_TEMP",
    "pd_temp": "DIODE_TEMP",
    "diode_temperature": "DIODE_TEMP",
    "bridge_temp": "BRIDGE_TEMP",
    "bridge_temperature": "BRIDGE_TEMP",
    "thermistor": "BRIDGE_TEMP",
    "led_temp": "BRIDGE_TEMP",
    "current": "CURRENT",
    "current_monitor": "CURRENT",
    "led_current": "CURRENT",
}
CHANNEL_MUX = {
    "VGND": Mux.VGND,
    "TIA": Mux.TIA,
    "TIA_LOWGAIN": Mux.TIA,
    "ACF": Mux.ACF,
    "IVC": Mux.IVC,
    "BOARD_TEMP": Mux.BOARD_TMP,
    "DIODE_TEMP": Mux.PD_TMP,
}
CHANNEL_RELAY = {
    "TIA": Relay.TIA.value,
    "TIA_LOWGAIN": Relay.TIA_LOWGAIN.value,
    "ACF": Relay.ACF.value,
    "IVC": Relay.IVC.value,
}
INTEGRATOR_CHANNELS = {"ACF", "IVC"}
ADS1115_CHANNELS = {"BRIDGE_TEMP", "CURRENT"}
PRIORITY_CHANNELS = {"TIA", "TIA_LOWGAIN", "IVC", "ACF"}


@dataclass
class Board:
    name: str
    adc: ads124s08Driver
    dac: dac5311Driver
    applied_bias_voltage_v: float
    current_relay: int | None = None


class Reading(NamedTuple):
    raw_code: int | None
    adc_voltage_v: float | None
    value: float | None
    unit: str


class BridgeReading(NamedTuple):
    raw_code: int | None
    differential_voltage_v: float | None
    resistance_ohm: float | None
    temperature_c: float | None


BRIDGE_READ_ERROR = BridgeReading(None, None, None, None)


class CurrentReading(NamedTuple):
    raw_code: int | None
    voltage_v: float | None
    current_a: float | None


CURRENT_READ_ERROR = CurrentReading(None, None, None)


def bridge_volts_to_resistance(
    differential_voltage_v: float,
    bridge_r_ohm: float = BRIDGE_R_OHM,
    excitation_v: float = BRIDGE_EXCITATION_V,
) -> float:
    """Convert the corrected bridge differential voltage to NTC resistance."""
    half_excitation = excitation_v / 2.0
    denominator = half_excitation - differential_voltage_v
    if denominator == 0:
        return float("inf")
    return bridge_r_ohm * (
        half_excitation + differential_voltage_v
    ) / denominator


def resistance_to_celsius(
    resistance_ohm: float,
    r25_ohm: float = THERMISTOR_R25_OHM,
    beta_k: float = THERMISTOR_B_K,
    t0_k: float = THERMISTOR_T0_K,
) -> float:
    """Convert NTC resistance using its beta-parameter model."""
    if resistance_ohm <= 0:
        return float("nan")
    temperature_k = 1.0 / (
        1.0 / t0_k + math.log(resistance_ohm / r25_ohm) / beta_k
    )
    return temperature_k - 273.15


def _ads1115_to_int16(raw: int) -> int:
    raw &= 0xFFFF
    return raw - 0x10000 if raw & 0x8000 else raw


class Ads1115BridgeMonitor:
    """One-shot ADS1115 reader for the LED bridge and current monitor."""

    def __init__(
        self,
        bus,
        address: int = ADS1115_ADDR,
        gain: int = 2,
        current_gain: int = 2,
    ) -> None:
        for configured_gain in (gain, current_gain):
            if configured_gain not in ADS1115_GAIN_FSR:
                raise ValueError(f"unsupported ADS1115 gain: {configured_gain}")
        self._bus = bus
        self._address = address
        self._gain = gain
        self._current_gain = current_gain

    def _read_code(self, mux: int, gain: int) -> int:
        pga_bits, _ = ADS1115_GAIN_FSR[gain]
        config = 0
        config |= 1 << 15  # OS: start a conversion
        config |= mux << 12
        config |= pga_bits << 9
        config |= 1 << 8  # MODE: single shot
        config |= ADS1115_DR_128SPS << 5
        config |= ADS1115_COMP_QUE_DISABLE
        self._bus.write_i2c_block_data(
            self._address,
            ADS1115_REG_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF],
        )

        time.sleep((1.0 / 128.0) * 1.5)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            status_bytes = self._bus.read_i2c_block_data(
                self._address, ADS1115_REG_CONFIG, 2
            )
            status = (status_bytes[0] << 8) | status_bytes[1]
            if status & 0x8000:
                break
            time.sleep(0.001)
        else:
            raise TimeoutError("ADS1115 conversion did not complete in time")

        conversion = self._bus.read_i2c_block_data(
            self._address, ADS1115_REG_CONVERSION, 2
        )
        return _ads1115_to_int16((conversion[0] << 8) | conversion[1])

    def read(self) -> BridgeReading:
        raw_code = self._read_code(ADS1115_MUX_DIFF_2_3, self._gain)
        _, full_scale_v = ADS1115_GAIN_FSR[self._gain]

        # The installed AIN2/AIN3 wiring is opposite to the polarity used by
        # bridge_volts_to_resistance (verified in test_LED_system.py).
        differential_voltage_v = -(raw_code * full_scale_v / 32768.0)
        resistance_ohm = bridge_volts_to_resistance(differential_voltage_v)
        temperature_c = resistance_to_celsius(resistance_ohm)
        return BridgeReading(
            raw_code, differential_voltage_v, resistance_ohm, temperature_c
        )

    def read_current(self) -> CurrentReading:
        """Read AIN0 and interpret its voltage across the 2.2 ohm shunt."""
        raw_code = self._read_code(ADS1115_MUX_SINGLE_0, self._current_gain)
        _, full_scale_v = ADS1115_GAIN_FSR[self._current_gain]
        voltage_v = raw_code * full_scale_v / 32768.0
        current_a = voltage_v / CURRENT_SENSE_RESISTOR_OHM
        return CurrentReading(raw_code, voltage_v, current_a)

    def close(self) -> None:
        self._bus.close()


def parse_data_rate(value: str) -> DataRate:
    """Accept a DataRate member name, case-insensitively."""
    name = value.strip().upper()
    try:
        return DataRate[name]
    except KeyError as exc:
        choices = ", ".join(DataRate.__members__)
        raise argparse.ArgumentTypeError(
            f"unknown data rate {value!r}; choose one of: {choices}"
        ) from exc


def parse_channel(value: str) -> str:
    """Normalize friendly channel spellings to the CSV channel name."""
    key = value.strip().lower().replace("-", "_").replace(" ", "_")
    try:
        return CHANNEL_ALIASES[key]
    except KeyError as exc:
        raise argparse.ArgumentTypeError(
            "unknown channel {!r}; choose from VGND, TIA, TIA_LOWGAIN, ACF, IVC, "
            "board_temp, diode_temp, bridge_temp, current".format(value)
        ) from exc


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"uvicpdro_calibration_{stamp}.csv")


def close_board(board: Board) -> None:
    """Return one board to a safe state and close both device handles."""
    try:
        board.adc.set_relays(0)
    finally:
        try:
            board.dac.set_voltage(0.0)
        finally:
            try:
                board.adc.close()
            finally:
                board.dac.close()


def open_boards(selection: str, requested_bias_voltage_v: float) -> list[Board]:
    names = ("sgt", "sol") if selection == "both" else (selection,)
    boards: list[Board] = []
    try:
        for key in names:
            name, adc_cs_pin, drdy_pin, start_pin, dac_cs_pin = BOARD_CONFIG[key]
            adc = None
            dac = None
            try:
                adc = ads124s08Driver(
                    SPI_DEVICE, GPIO_CHIP, adc_cs_pin, drdy_pin, start_pin
                )
                dac = dac5311Driver(SPI_DEVICE, GPIO_CHIP, dac_cs_pin)
                applied_bias_voltage_v = dac.set_voltage(requested_bias_voltage_v)
                boards.append(Board(name, adc, dac, applied_bias_voltage_v))
            except Exception:
                try:
                    if dac is not None:
                        try:
                            dac.set_voltage(0.0)
                        finally:
                            dac.close()
                finally:
                    if adc is not None:
                        adc.close()
                raise
    except Exception:
        for board in boards:
            try:
                close_board(board)
            except Exception:
                pass
        raise
    return boards


def read_channel(adc: ads124s08Driver, channel: str) -> Reading:
    """Take one conversion and derive either voltage or temperature."""
    raw = adc._read_single_shot_raw()
    if raw is None:
        return Reading(None, None, None, "C" if channel.endswith("TEMP") else "V")

    volts = float(adc._lib.ads124s08_code_to_volts(raw))
    if channel.endswith("TEMP"):
        resistance = adc._lib.ads124s08_thermistor_volts_to_resistance(volts)
        temperature = adc._lib.ads124s08_resistance_to_celsius(resistance)
        return Reading(raw, volts, float(temperature), "C")
    return Reading(raw, volts, volts, "V")


def configure_boards(boards: list[Board], channel: str, data_rate: DataRate) -> None:
    required_relay = CHANNEL_RELAY.get(channel)
    for board in boards:
        # Temperature channels do not require a relay change. Keeping the last
        # signal path selected also avoids needless switching when returning to it.
        if required_relay is not None and board.current_relay != required_relay:
            board.adc.set_relays(required_relay)
            board.current_relay = required_relay
        if board.adc._configure(CHANNEL_MUX[channel], data_rate) is None:
            raise OSError(f"failed to configure {channel} on {board.name}")


def write_reading(
    writer: csv.writer,
    csv_file,
    started_at: float,
    board_name: str,
    channel: str,
    data_rate: DataRate | None,
    reading: Reading,
    requested_bias_voltage_v: float,
    applied_bias_voltage_v: float | None,
    requested_integration_us: float | None = None,
    measured_integration_us: float | None = None,
    print_terminal: bool = True,
    bridge_reading: BridgeReading = BRIDGE_READ_ERROR,
    current_reading: CurrentReading = CURRENT_READ_ERROR,
) -> None:
    now = datetime.now(timezone.utc)
    elapsed_s = time.perf_counter() - started_at
    writer.writerow(
        [
            now.isoformat(timespec="microseconds"),
            f"{elapsed_s:.6f}",
            board_name,
            channel,
            "" if data_rate is None else data_rate.name,
            f"{requested_bias_voltage_v:.9g}",
            (
                ""
                if applied_bias_voltage_v is None
                else f"{applied_bias_voltage_v:.9g}"
            ),
            "" if reading.raw_code is None else reading.raw_code,
            "" if reading.adc_voltage_v is None else f"{reading.adc_voltage_v:.9g}",
            "" if reading.value is None else f"{reading.value:.9g}",
            reading.unit,
            (
                ""
                if requested_integration_us is None
                else f"{requested_integration_us:.3f}"
            ),
            "" if measured_integration_us is None else f"{measured_integration_us:.3f}",
            "" if bridge_reading.raw_code is None else bridge_reading.raw_code,
            (
                ""
                if bridge_reading.differential_voltage_v is None
                else f"{bridge_reading.differential_voltage_v:.9g}"
            ),
            (
                ""
                if bridge_reading.resistance_ohm is None
                else f"{bridge_reading.resistance_ohm:.9g}"
            ),
            (
                ""
                if bridge_reading.temperature_c is None
                else f"{bridge_reading.temperature_c:.9g}"
            ),
            "" if current_reading.raw_code is None else current_reading.raw_code,
            (
                ""
                if current_reading.voltage_v is None
                else f"{current_reading.voltage_v:.9g}"
            ),
            (
                ""
                if current_reading.current_a is None
                else f"{current_reading.current_a:.9g}"
            ),
        ]
    )
    csv_file.flush()

    if print_terminal:
        value = (
            "READ ERROR"
            if reading.value is None
            else f"{reading.value:.8f} {reading.unit}"
        )
        integration = (
            ""
            if measured_integration_us is None
            else f"  integration={measured_integration_us:.2f} us"
        )
        bias = (
            ""
            if applied_bias_voltage_v is None
            else f"  bias={applied_bias_voltage_v:.5f} V"
        )
        details = ""
        if channel == "BRIDGE_TEMP" and bridge_reading.resistance_ohm is not None:
            details = f"  resistance={bridge_reading.resistance_ohm:.1f} ohm"
        elif channel == "CURRENT" and current_reading.voltage_v is not None:
            details = f"  ain0={current_reading.voltage_v:.6f} V"
        rate = "" if data_rate is None else f"  rate={data_rate.name}"
        print(
            f"{now.isoformat(timespec='milliseconds')}  {board_name:3}  "
            f"{channel:12}  {value}{bias}{integration}{rate}{details}",
            flush=True,
        )


def render_live_dashboard(
    boards: list[Board],
    channels: list[str],
    latest: dict[
        tuple[str, str],
        tuple[Reading, float | None, BridgeReading, CurrentReading],
    ],
    sample_number: int,
    total_samples: int,
    data_rate: DataRate,
    output_path: Path,
    secondary_data_rate: DataRate | None = None,
) -> None:
    target = "infinite" if total_samples == 0 else str(total_samples)
    lines = [
        "UVIC PDRO CALIBRATION — LIVE",
        f"Sample {sample_number} / {target}    Primary rate: {data_rate.name}"
        + (
            ""
            if secondary_data_rate is None
            else f"    Secondary rate: {secondary_data_rate.name}"
        ),
        f"CSV: {output_path.resolve()}",
        "",
        f"{'BOARD':<8} {'CHANNEL':<13} {'RATE':<11} {'VALUE':>17} {'ADC VOLTAGE':>15} "
        f"{'RAW CODE':>11} {'BIAS':>11} {'INTEGRATION':>15}",
        "-" * 110,
    ]

    for board in boards:
        for channel in channels:
            if channel in ADS1115_CHANNELS and board is not boards[0]:
                continue
            source_name = "ads1115" if channel in ADS1115_CHANNELS else board.name
            current = latest.get((source_name, channel))
            if current is None:
                value = adc_voltage = raw_code = integration = "—"
            else:
                (
                    reading,
                    measured_integration_us,
                    bridge_reading,
                    current_reading,
                ) = current
                value = (
                    "READ ERROR"
                    if reading.value is None
                    else f"{reading.value:.8f} {reading.unit}"
                )
                adc_voltage = (
                    "—"
                    if reading.adc_voltage_v is None
                    else f"{reading.adc_voltage_v:.8f} V"
                )
                raw_code = "—" if reading.raw_code is None else str(reading.raw_code)
                integration = (
                    "—"
                    if measured_integration_us is None
                    else f"{measured_integration_us:.2f} us"
                )
            bias = (
                "--"
                if channel in ADS1115_CHANNELS
                else f"{board.applied_bias_voltage_v:.5f} V"
            )
            channel_rate = (
                "128_SPS"
                if channel in ADS1115_CHANNELS
                else (
                    secondary_data_rate.name
                    if secondary_data_rate is not None
                    and channel not in PRIORITY_CHANNELS
                    else data_rate.name
                )
            )
            lines.append(
                f"{source_name:<8} {channel:<13} {channel_rate:<11} "
                f"{value:>17} {adc_voltage:>15} "
                f"{raw_code:>11} {bias:>11} {integration:>15}"
            )

    lines.extend(["", "Ctrl+C to stop safely and reset bias to 0 V."])
    print("\033[H" + "\n".join(lines) + "\033[J", end="", flush=True)


def stream(
    boards: list[Board],
    channels: list[str],
    data_rate: DataRate,
    integration_us: float,
    requested_bias_voltage_v: float,
    samples: int,
    output_path: Path,
    integrator: IntegratorDriver | None,
    live_display: bool = False,
    bridge_monitor: Ads1115BridgeMonitor | None = None,
    secondary_data_rate: DataRate | None = None,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("x", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp_utc",
                "elapsed_s",
                "board",
                "channel",
                "data_rate",
                "requested_bias_voltage_v",
                "applied_bias_voltage_v",
                "raw_adc_code",
                "adc_voltage_v",
                "value",
                "unit",
                "requested_integration_us",
                "measured_integration_us",
                "led_bridge_raw_adc_code",
                "led_bridge_differential_voltage_v",
                "led_bridge_resistance_ohm",
                "led_bridge_temperature_c",
                "ads1115_ain0_raw_adc_code",
                "ads1115_ain0_voltage_v",
                "ads1115_ain0_current_a",
            ]
        )
        csv_file.flush()

        started_at = time.perf_counter()
        latest: dict[
            tuple[str, str],
            tuple[Reading, float | None, BridgeReading, CurrentReading],
        ] = {}
        if live_display:
            print("\033[2J\033[H", end="", flush=True)
        else:
            print(f"Streaming to {output_path.resolve()} (Ctrl+C to stop)")
        completed_samples = 0
        while samples == 0 or completed_samples < samples:
            for channel in channels:
                if channel in ADS1115_CHANNELS:
                    if bridge_monitor is None:
                        raise RuntimeError("ADS1115 monitor is required for this channel")

                    bridge_reading = BRIDGE_READ_ERROR
                    current_reading = CURRENT_READ_ERROR
                    try:
                        if channel == "BRIDGE_TEMP":
                            bridge_reading = bridge_monitor.read()
                            reading = Reading(
                                bridge_reading.raw_code,
                                bridge_reading.differential_voltage_v,
                                bridge_reading.temperature_c,
                                "C",
                            )
                        else:
                            current_reading = bridge_monitor.read_current()
                            reading = Reading(
                                current_reading.raw_code,
                                current_reading.voltage_v,
                                current_reading.current_a,
                                "A",
                            )
                    except (OSError, TimeoutError) as exc:
                        print(
                            f"warning: ADS1115 {channel} read failed: {exc}",
                            file=sys.stderr,
                        )
                        reading = Reading(
                            None,
                            None,
                            None,
                            "C" if channel == "BRIDGE_TEMP" else "A",
                        )

                    write_reading(
                        writer,
                        csv_file,
                        started_at,
                        "ads1115",
                        channel,
                        None,
                        reading,
                        requested_bias_voltage_v,
                        None,
                        print_terminal=not live_display,
                        bridge_reading=bridge_reading,
                        current_reading=current_reading,
                    )
                    if live_display:
                        latest[("ads1115", channel)] = (
                            reading,
                            None,
                            bridge_reading,
                            current_reading,
                        )
                        render_live_dashboard(
                            boards,
                            channels,
                            latest,
                            completed_samples + 1,
                            samples,
                            data_rate,
                            output_path,
                            secondary_data_rate,
                        )
                    continue

                channel_data_rate = (
                    secondary_data_rate
                    if secondary_data_rate is not None
                    and channel not in PRIORITY_CHANNELS
                    else data_rate
                )
                configure_boards(boards, channel, channel_data_rate)

                requested_us = None
                measured_us = None
                if channel in INTEGRATOR_CHANNELS:
                    if integrator is None:  # Guarded in main; keeps this function safe.
                        raise RuntimeError("integrator is required for ACF/IVC")
                    requested_us = integration_us
                    integrator.reset()
                    # Allow the ADC relay and integrator reset switches to settle.
                    time.sleep(0.05)
                    t_start, t_end = integrator.integrate_and_hold(
                        integration_us, print_timing=False
                    )
                    measured_us = (t_end - t_start) * 1_000_000.0

                try:
                    for board in boards:
                        bridge_reading = BRIDGE_READ_ERROR
                        current_reading = CURRENT_READ_ERROR
                        reading = read_channel(board.adc, channel)
                        write_reading(
                            writer,
                            csv_file,
                            started_at,
                            board.name,
                            channel,
                            channel_data_rate,
                            reading,
                            requested_bias_voltage_v,
                            board.applied_bias_voltage_v,
                            requested_us,
                            measured_us,
                            print_terminal=not live_display,
                            bridge_reading=bridge_reading,
                            current_reading=current_reading,
                        )
                        if live_display:
                            latest[(board.name, channel)] = (
                                reading,
                                measured_us,
                                bridge_reading,
                                current_reading,
                            )
                            render_live_dashboard(
                                boards,
                                channels,
                                latest,
                                completed_samples + 1,
                                samples,
                                data_rate,
                                output_path,
                                secondary_data_rate,
                            )
                finally:
                    if channel in INTEGRATOR_CHANNELS and integrator is not None:
                        integrator.reset()
            completed_samples += 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stream UVIC PDRO calibration measurements and save them to CSV"
    )
    parser.add_argument(
        "-b",
        "--boards",
        required=True,
        type=str.lower,
        choices=("sgt", "sol", "both"),
        help="board(s) to sample: sgt, sol, or both",
    )
    parser.add_argument(
        "-r",
        "--data-rate",
        required=True,
        type=parse_data_rate,
        metavar="RATE",
        help="DataRate name, for example SPS_100 or SPS_1000",
    )
    parser.add_argument(
        "-r2",
        "--secondary-data-rate",
        type=parse_data_rate,
        default=None,
        metavar="RATE",
        help=(
            "optional rate for nonpriority UVIC channels; TIA, TIA_LOWGAIN, "
            "IVC, and ACF continue to use -r"
        ),
    )
    parser.add_argument(
        "-c",
        "--channels",
        required=True,
        nargs="+",
        type=parse_channel,
        metavar="CHANNEL",
        help=(
            "one or more of: VGND TIA TIA_LOWGAIN ACF IVC board_temp "
            "diode_temp bridge_temp current"
        ),
    )
    parser.add_argument(
        "-v",
        "--bias-voltage",
        required=True,
        type=float,
        metavar="VOLTS",
        help="photodiode bias voltage to apply with each selected board's DAC (0 to 5.1 V)",
    )
    parser.add_argument(
        "-i",
        "--integration-time-us",
        type=float,
        default=DEFAULT_INTEGRATION_US,
        metavar="MICROSECONDS",
        help=(
            "requested ACF/IVC integration time in microseconds "
            f"(default: {DEFAULT_INTEGRATION_US:g})"
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="CSV output path (default: timestamped file in the current directory)",
    )
    parser.add_argument(
        "-n",
        "--samples",
        type=int,
        default=0,
        metavar="COUNT",
        help="number of complete channel/board sampling passes; 0 means infinite (default: 0)",
    )
    parser.add_argument(
        "--live-display",
        "--live",
        action="store_true",
        help="refresh a readable dashboard in place instead of printing machine-readable rows",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.integration_time_us <= 0:
        print("error: integration time must be greater than zero", file=sys.stderr)
        return 2
    if not 0.0 <= args.bias_voltage <= 5.1:
        print("error: bias voltage must be between 0 and 5.1 V", file=sys.stderr)
        return 2
    if args.samples < 0:
        print("error: samples must be zero or greater", file=sys.stderr)
        return 2
    # Preserve the requested order but avoid sampling a repeated channel twice.
    channels = list(dict.fromkeys(args.channels))
    output_path = args.output or default_output_path()
    live_display = args.live_display and sys.stdout.isatty()
    if args.live_display and not live_display:
        print(
            "warning: --live-display requires a terminal; using line output",
            file=sys.stderr,
        )

    boards: list[Board] = []
    from drivers.integrator_driver import IntegratorDriver
    from drivers.mcp23017 import MCP23017

    io = None
    integrator = None
    bridge_monitor = None
    try:
        # The MCP23017 owns the ADC reset/enable line. IntegratorDriver drives
        # it high, verifies it, and observes the reset-release delay before any
        # ads124s08Driver constructor is allowed to issue SPI commands.
        io = MCP23017()
        integrator = IntegratorDriver(io)
        if ADS1115_CHANNELS.intersection(channels):
            import smbus2

            bridge_monitor = Ads1115BridgeMonitor(
                smbus2.SMBus(ADS1115_BUS),
                address=ADS1115_ADDR,
                gain=ADS1115_BRIDGE_GAIN,
                current_gain=ADS1115_CURRENT_GAIN,
            )
        boards = open_boards(args.boards, args.bias_voltage)
        stream(
            boards,
            channels,
            args.data_rate,
            args.integration_time_us,
            args.bias_voltage,
            args.samples,
            output_path,
            integrator,
            live_display=live_display,
            bridge_monitor=bridge_monitor,
            secondary_data_rate=args.secondary_data_rate,
        )
    except KeyboardInterrupt:
        print("\nStopped; all completed measurements are saved.")
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if bridge_monitor is not None:
            try:
                bridge_monitor.close()
            except Exception as exc:
                print(
                    f"warning: failed to close ADS1115 I2C bus cleanly: {exc}",
                    file=sys.stderr,
                )
        try:
            if integrator is not None:
                integrator.reset()
                integrator.io.close()
            elif io is not None:
                io.close()
        except Exception as exc:
            print(
                f"warning: failed to close integrator cleanly: {exc}", file=sys.stderr
            )
        for board in boards:
            try:
                close_board(board)
            except Exception as exc:
                print(
                    f"warning: failed to return {board.name} to a safe state: {exc}",
                    file=sys.stderr,
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
