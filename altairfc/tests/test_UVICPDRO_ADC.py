"""Interactive production test for the complete UVIC PDRO board."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.uvic_pdro import (  # noqa: E402
    DataRate,
    Input,
    Readout,
    SignalPath,
    ThermistorReading,
    UVICPDRO,
)


READOUT_NAMES = {
    Readout.SERGEANT: "Sergeant",
    Readout.SOLDIER: "Soldier",
}


def readout_name(readout: Readout) -> str:
    return f"{READOUT_NAMES[readout]} readout"


def check_range(name, val, expected_min, expected_max, unit="V"):
    if val is None:
        actual_str = "None"
        passed = False
    else:
        actual_str = f"{val:.8f} {unit}"
        passed = expected_min <= val <= expected_max

    expected_str = f"[{expected_min:.8f}, {expected_max:.8f}] {unit}"
    status = "PASS" if passed else "FAIL - OUT OF RANGE"
    color = "\033[92m" if passed else "\033[91m\033[1m"
    reset = "\033[0m"
    print(
        f"  {name:25} | Actual: {actual_str:18} | "
        f"Expected: {expected_str:30} | {color}[{status}]{reset}"
    )


def check_thermistor(name, therm_out, expected_min, expected_max):
    unit = "C"
    if therm_out is None:
        actual_str = "None"
        passed = False
    else:
        val = therm_out.temperature_c
        actual_str = f"{val:.2f} {unit}"
        passed = expected_min <= val <= expected_max

    expected_str = f"[{expected_min:.2f}, {expected_max:.2f}] {unit}"
    status = "PASS" if passed else "FAIL - OUT OF RANGE"
    color = "\033[92m" if passed else "\033[91m\033[1m"
    reset = "\033[0m"
    print(
        f"  {name:25} | Actual: {actual_str:18} | "
        f"Expected: {expected_str:30} | {color}[{status}]{reset}"
    )


def run_all_checks(pdro: UVICPDRO):
    print("\n" + "=" * 105)
    print(" " * 42 + "RUNNING BATCH CHECKS")
    print("=" * 105)
    for readout in pdro.readouts:
        print(f"\n--- {readout_name(readout)} ---")
        print("-" * 105)

        expected_config = pdro.configure_input(
            readout, Input.VGND, DataRate.SPS_1000
        )
        read_config = pdro.read_adc_config(readout)
        passed = read_config == expected_config
        status = "PASS" if passed else "FAIL - MISMATCH"
        color = "\033[92m" if passed else "\033[91m\033[1m"
        reset = "\033[0m"
        actual_str = "Match" if passed else "Mismatch"
        print(
            f"  {'Config Read/Write':25} | Actual: {actual_str:18} | "
            f"Expected: {'Match':30} | {color}[{status}]{reset}"
        )

        check_range(
            "VGND Voltage",
            pdro.read_voltage(readout, Input.VGND, select_signal_path=False),
            4.85,
            4.95,
        )
        check_range(
            "TIA Voltage",
            pdro.read_voltage(readout, Input.TIA, select_signal_path=False),
            4.85,
            4.95,
        )
        check_range(
            "IVC Level Shifter",
            pdro.read_voltage(readout, Input.IVC, select_signal_path=False),
            0.0,
            0.1,
        )
        check_range(
            "ACF Level Shifter",
            pdro.read_voltage(readout, Input.ACF, select_signal_path=False),
            0.0,
            0.4,
        )

        therm_out: ThermistorReading | None = pdro.read_board_thermistor(readout)
        check_thermistor("Board Thermistor", therm_out, 20.0, 40.0)
        therm_out = pdro.read_photodiode_thermistor(readout)
        check_thermistor("Photodiode Thermistor", therm_out, 20.0, 40.0)

    print("\n" + "=" * 105)
    print(" " * 44 + "CHECKS COMPLETE")
    print("=" * 105 + "\n")


def select_readout(pdro: UVICPDRO) -> Readout | None:
    if len(pdro.readouts) == 1:
        return pdro.readouts[0]

    print("\nSelect readout:")
    for index, readout in enumerate(pdro.readouts, start=1):
        print(f"{index}: {readout_name(readout)}")
    try:
        index = int(input("Enter choice: ").strip()) - 1
        if not 0 <= index < len(pdro.readouts):
            raise ValueError
        return pdro.readouts[index]
    except ValueError:
        print("Invalid readout choice.")
        return None


def stream_input(pdro: UVICPDRO):
    channels = (
        ("VGND", Input.VGND),
        ("TIA", Input.TIA),
        ("TIA_LOW_GAIN", Input.TIA_LOW_GAIN),
        ("IVC", Input.IVC),
        ("ACF", Input.ACF),
        ("BOARD_TMP", "board_temperature"),
        ("PD_TMP", "photodiode_temperature"),
    )
    print("\nSelect input to stream:")
    for index, (name, _) in enumerate(channels, start=1):
        print(f"{index}: {name}")
    try:
        index = int(input("Enter choice: ").strip()) - 1
        if not 0 <= index < len(channels):
            raise ValueError
    except ValueError:
        print("Invalid choice.")
        return

    readout = select_readout(pdro)
    if readout is None:
        return
    channel_name, channel = channels[index]
    print(
        f"\nStreaming {channel_name} on {readout_name(readout)}. "
        "Press Ctrl+C to stop."
    )
    try:
        while True:
            if channel == "board_temperature":
                reading = pdro.read_board_thermistor(readout)
                value = None if reading is None else reading.temperature_c
                print(
                    f"{channel_name}: {value:.2f} C"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            elif channel == "photodiode_temperature":
                reading = pdro.read_photodiode_thermistor(readout)
                value = None if reading is None else reading.temperature_c
                print(
                    f"{channel_name}: {value:.2f} C"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            else:
                value = pdro.read_voltage(readout, channel)
                print(
                    f"{channel_name}: {value:.8f} V"
                    if value is not None
                    else f"{channel_name}: Read failed"
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped streaming.")


def manually_actuate_relays(pdro: UVICPDRO):
    print("\nSelect relays to actuate (numbers separated by spaces, or 0 for all off):")
    print("1: ACF")
    print("2: IVC")
    print("3: TIA")
    print("4: TIA_LOW_GAIN")
    print("0: All off")
    relay_map = {
        "1": SignalPath.ACF,
        "2": SignalPath.IVC,
        "3": SignalPath.TIA,
        "4": SignalPath.TIA_LOW_GAIN,
    }
    value = input("Enter choices: ").strip()
    if value == "0":
        paths = SignalPath.NONE
    else:
        paths = SignalPath.NONE
        for choice in value.split():
            if choice not in relay_map:
                print(f"Invalid option: {choice}")
                return
            paths |= relay_map[choice]

    for readout in pdro.readouts:
        print(f"Setting relays for {readout_name(readout)} (bitmask {paths.value})")
        pdro.set_signal_paths(readout, paths)


def set_bias_voltage(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        volts = float(input("Enter voltage to set (0 to 5.1): ").strip())
        actual_v = pdro.set_bias_voltage(readout, volts)
        print(f"Set {readout_name(readout)} bias to {actual_v:.8f} V")
    except ValueError as exc:
        print(f"Invalid voltage: {exc}")


def integrator_read(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    print("\nSelect integrator:")
    print("1. IVC")
    print("2. ACF")
    choice = input("Enter choice: ").strip()
    if choice == "1":
        channel = Input.IVC
    elif choice == "2":
        channel = Input.ACF
    else:
        print("Invalid integrator choice.")
        return

    try:
        time_us = float(input("Enter integration time in microseconds: ").strip())
        reading = pdro.integrate_and_read(readout, channel, time_us)
    except ValueError as exc:
        print(f"Invalid integration time: {exc}")
        return

    print(
        f"Integration duration: {reading.actual_time_us:.2f} us "
        f"(requested {reading.requested_time_us:.2f} us)"
    )
    if reading.voltage_v is None:
        print("Read failed")
    else:
        print(f"Read voltage: {reading.voltage_v:.8f} V")


def read_adc_register(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        address = int(input("Enter register address (hex or int): ").strip(), 0)
        if not 0 <= address <= 0x1F:
            raise ValueError("address must be between 0 and 0x1F")
    except ValueError as exc:
        print(f"Invalid register address: {exc}")
        return
    value = pdro.read_adc_register(readout, address)
    if value is None:
        print("Failed to read register.")
    else:
        print(f"Register 0x{address:02X} value: 0x{value:02X} ({value})")


def write_adc_register(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    try:
        address = int(input("Enter register address (hex or int): ").strip(), 0)
        value = int(input("Enter value to write (hex or int): ").strip(), 0)
        if not 0 <= address <= 0x1F:
            raise ValueError("address must be between 0 and 0x1F")
        if not 0 <= value <= 0xFF:
            raise ValueError("value must be between 0 and 0xFF")
    except ValueError as exc:
        print(f"Invalid register write: {exc}")
        return
    if pdro.write_adc_register(readout, address, value):
        print(f"Successfully wrote 0x{value:02X} to register 0x{address:02X}.")
    else:
        print("Failed to write register.")


def reset_adc(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    if pdro.reset_adc(readout):
        print(f"Successfully reset {readout_name(readout)} ADC.")
    else:
        print(f"Failed to reset {readout_name(readout)} ADC.")


def read_all_adc_registers(pdro: UVICPDRO):
    readout = select_readout(pdro)
    if readout is None:
        return
    print(f"\n--- ADC registers for {readout_name(readout)} ---")
    for address in range(0x12):
        value = pdro.read_adc_register(readout, address)
        if value is None:
            print(f"Reg 0x{address:02X}: Failed")
        else:
            print(f"Reg 0x{address:02X}: 0x{value:02X} (0b{value:08b})")


def interactive_menu(pdro: UVICPDRO):
    actions = {
        "1": run_all_checks,
        "2": manually_actuate_relays,
        "3": stream_input,
        "4": set_bias_voltage,
        "5": integrator_read,
        "6": read_adc_register,
        "7": write_adc_register,
        "8": reset_adc,
        "9": read_all_adc_registers,
    }
    while True:
        print("\n--- UVIC PDRO Interactive Test Menu ---")
        print("1. Run all checks")
        print("2. Manually actuate relays")
        print("3. Stream data from an input")
        print("4. Set bias voltage")
        print("5. Integrator read command")
        print("6. Read ADC register")
        print("7. Write ADC register")
        print("8. Send ADC reset")
        print("9. Read all ADC registers")
        print("q. Quit")
        choice = input("Enter choice: ").strip().lower()
        if choice == "q":
            break
        action = actions.get(choice)
        if action is None:
            print("Unknown choice. Please select a valid option.")
        else:
            action(pdro)


def main() -> int:
    parser = argparse.ArgumentParser(description="UVIC PDRO board test script")
    parser.add_argument(
        "-b",
        "--board",
        type=int,
        choices=[1, 2, 3],
        default=3,
        help="Select readout: 1 for Sergeant, 2 for Soldier, 3 for both (default)",
    )
    args = parser.parse_args()
    selected = {
        1: (Readout.SERGEANT,),
        2: (Readout.SOLDIER,),
        3: tuple(Readout),
    }[args.board]

    try:
        with UVICPDRO(readouts=selected) as pdro:
            interactive_menu(pdro)
    except (OSError, RuntimeError) as exc:
        print(f"Failed to initialize or operate UVIC PDRO: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
