"""Stream UVIC PDRO calibration data to the terminal and a CSV file.

Examples (run from ``altairfc``):

    python tests/stream_UVICPDRO_calibration.py -b both -r SPS_100 -c TIA board_temp -v 2.5
    python tests/stream_UVICPDRO_calibration.py -b sgt -r SPS_1000 -c ACF IVC -i 500 -v 3.0

Sampling continues until Ctrl+C.  Every measurement is printed and flushed to
the CSV immediately so that data already collected survives an interrupted run.
"""

from __future__ import annotations

import argparse
import csv
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
    # name, ADC chip select, bias DAC chip select
    "sgt": ("sgt", 13, 12),
    "sol": ("sol", 19, 6),
}
DEFAULT_INTEGRATION_US = 1000.0

CHANNEL_ALIASES = {
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
}
CHANNEL_MUX = {
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
            "unknown channel {!r}; choose from TIA, TIA_LOWGAIN, ACF, IVC, "
            "board_temp, diode_temp".format(value)
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
            name, adc_cs_pin, dac_cs_pin = BOARD_CONFIG[key]
            adc = None
            dac = None
            try:
                adc = ads124s08Driver(SPI_DEVICE, GPIO_CHIP, adc_cs_pin)
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
    data_rate: DataRate,
    reading: Reading,
    requested_bias_voltage_v: float,
    applied_bias_voltage_v: float,
    requested_integration_us: float | None = None,
    measured_integration_us: float | None = None,
) -> None:
    now = datetime.now(timezone.utc)
    elapsed_s = time.perf_counter() - started_at
    writer.writerow(
        [
            now.isoformat(timespec="microseconds"),
            f"{elapsed_s:.6f}",
            board_name,
            channel,
            data_rate.name,
            f"{requested_bias_voltage_v:.9g}",
            f"{applied_bias_voltage_v:.9g}",
            "" if reading.raw_code is None else reading.raw_code,
            "" if reading.adc_voltage_v is None else f"{reading.adc_voltage_v:.9g}",
            "" if reading.value is None else f"{reading.value:.9g}",
            reading.unit,
            "" if requested_integration_us is None else f"{requested_integration_us:.3f}",
            "" if measured_integration_us is None else f"{measured_integration_us:.3f}",
        ]
    )
    csv_file.flush()

    value = "READ ERROR" if reading.value is None else f"{reading.value:.8f} {reading.unit}"
    integration = (
        ""
        if measured_integration_us is None
        else f"  integration={measured_integration_us:.2f} us"
    )
    bias = f"  bias={applied_bias_voltage_v:.5f} V"
    print(
        f"{now.isoformat(timespec='milliseconds')}  {board_name:3}  "
        f"{channel:12}  {value}{bias}{integration}",
        flush=True,
    )


def stream(
    boards: list[Board],
    channels: list[str],
    data_rate: DataRate,
    integration_us: float,
    requested_bias_voltage_v: float,
    samples: int,
    output_path: Path,
    integrator: IntegratorDriver | None,
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
            ]
        )
        csv_file.flush()

        started_at = time.perf_counter()
        print(f"Streaming to {output_path.resolve()} (Ctrl+C to stop)")
        completed_samples = 0
        while samples == 0 or completed_samples < samples:
            for channel in channels:
                configure_boards(boards, channel, data_rate)

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
                        reading = read_channel(board.adc, channel)
                        write_reading(
                            writer,
                            csv_file,
                            started_at,
                            board.name,
                            channel,
                            data_rate,
                            reading,
                            requested_bias_voltage_v,
                            board.applied_bias_voltage_v,
                            requested_us,
                            measured_us,
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
        "-c",
        "--channels",
        required=True,
        nargs="+",
        type=parse_channel,
        metavar="CHANNEL",
        help="one or more of: TIA TIA_LOWGAIN ACF IVC board_temp diode_temp",
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
    needs_integrator = any(channel in INTEGRATOR_CHANNELS for channel in channels)
    output_path = args.output or default_output_path()

    boards: list[Board] = []
    integrator: IntegratorDriver | None = None
    try:
        boards = open_boards(args.boards, args.bias_voltage)
        if needs_integrator:
            # These imports require the Pi's smbus2 package, so keep them out of
            # non-hardware paths such as --help and parser tests.
            from drivers.integrator_driver import IntegratorDriver
            from drivers.mcp23017 import MCP23017

            integrator = IntegratorDriver(MCP23017())
        stream(
            boards,
            channels,
            args.data_rate,
            args.integration_time_us,
            args.bias_voltage,
            args.samples,
            output_path,
            integrator,
        )
    except KeyboardInterrupt:
        print("\nStopped; all completed measurements are saved.")
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if integrator is not None:
                integrator.reset()
                integrator.io.close()
        except Exception as exc:
            print(f"warning: failed to close integrator cleanly: {exc}", file=sys.stderr)
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
