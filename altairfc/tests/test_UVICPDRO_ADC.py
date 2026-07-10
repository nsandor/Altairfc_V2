import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.ads124s08_driver import (
    ads124s08Driver,
    Mux,
    DataRate,
    ThermistorReading,
    Relay,
)  # noqa: E402
from drivers.dac5311_driver import dac5311Driver


def open_boards(only, spi_dev, gpiochip, cs1, cs2, dac_cs1, dac_cs2):
    boards = []
    if only in (None, 1):
        adc = ads124s08Driver(spi_dev, gpiochip, cs1)
        dac = dac5311Driver(spi_dev, gpiochip, dac_cs1) if dac_cs1 is not None else None
        boards.append((f"Sergeant ADC (CS=GPIO{cs1})", adc, dac))
    if only in (None, 2):
        adc = ads124s08Driver(spi_dev, gpiochip, cs2)
        dac = dac5311Driver(spi_dev, gpiochip, dac_cs2) if dac_cs2 is not None else None
        boards.append((f"Soldier ADC (CS=GPIO{cs2})", adc, dac))
    return boards


# Check that we can write and read config registers
def test_configset(adc: ads124s08Driver):
    print(f"Testing configuration for {adc}")
    expected_config = adc._configure(Mux.VGND, DataRate.SPS_1000)
    read_config = adc.read_config()
    print(f"Read config: {read_config}")
    assert read_config == expected_config


# Check VGND reading. This should be in the range of 4.85-4.95V
def check_vgnd_read(adc: ads124s08Driver):
    print(f"Testing VGND reading for {adc}")
    adc._configure(Mux.VGND, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"VGND read: {val}")
    assert 4.85 <= val <= 4.95


# Check TIA reading. This should be in the range of 4.85-4.95V
def check_TIA_read(adc: ads124s08Driver):
    print(f"Testing TIA reading for {adc}")
    adc._configure(Mux.TIA, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"TIA read: {val}")
    assert 4.85 <= val <= 4.95


# Check the IVC level shifter output. Should default to near 0V
def check_ivc_read(adc: ads124s08Driver):
    print(f"Testing IVC reading for {adc}")
    adc._configure(Mux.IVC, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"IVC read: {val}")
    assert 0 <= val <= 0.1


# Check the ACF level shifter output. Should default to near 0V
def check_acf_read(adc: ads124s08Driver):
    print(f"Testing ACF reading for {adc}")
    adc._configure(Mux.ACF, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"ACF read: {val}")
    assert 0 <= val <= 0.1


# Check that the board thermistor gives reasonable temperature values
def check_board_thermistor(adc: ads124s08Driver):
    print(f"Testing thermistor reading for {adc}")
    therm_out: ThermistorReading = adc.read_board_thermistor()
    print(f"Thermistor read: {therm_out}")
    assert 20 <= therm_out.temperature_c <= 40


def check_pd_thermistor(adc: ads124s08Driver):
    print(f"Testing photodiode thermistor reading for {adc}")
    therm_out: ThermistorReading = adc.read_pd_thermistor()
    print(f"Thermistor read: {therm_out}")
    # The photodiode temperature sensor should be in the same range as the board thermistor
    assert 20 <= therm_out.temperature_c <= 40


def run_all_checks(boards):
    for board_name, board, dac in boards:
        print(f"\n--- Running checks for {board_name} ---")
        test_configset(board)
        check_vgnd_read(board)
        check_TIA_read(board)
        check_ivc_read(board)
        check_acf_read(board)
        check_board_thermistor(board)
        check_pd_thermistor(board)
        print("Checks passed.")


def interactive_menu(boards):
    while True:
        print("\n--- ADC Interactive Test Menu ---")
        print("1. Run all checks")
        print("2. Manually actuate relays")
        print("3. Stream data from a Mux channel")
        print("4. Set DAC voltage")
        print("q. Quit")

        choice = input("Enter choice: ").strip().lower()

        if choice == "1":
            run_all_checks(boards)
        elif choice == "2":
            print(
                "\nSelect relays to actuate (enter numbers separated by spaces, or 0 for all off):"
            )
            print("1: ACF")
            print("2: IVC")
            print("3: TIA")
            print("4: TIA_LOWGAIN")
            print("0: All off")

            relay_map = {
                "1": Relay.ACF.value,
                "2": Relay.IVC.value,
                "3": Relay.TIA.value,
                "4": Relay.TIA_LOWGAIN.value,
            }

            val = input("Enter choices: ").strip()
            if val == "0":
                relays_val = 0
            else:
                relays_val = 0
                choices = val.split()
                invalid = False
                for c in choices:
                    if c in relay_map:
                        relays_val |= relay_map[c]
                    else:
                        print(f"Invalid option: {c}")
                        invalid = True
                if invalid:
                    continue

            for board_name, board, _ in boards:
                print(f"Setting relays for {board_name} (bitmask {relays_val})")
                board.set_relays(relays_val)

        elif choice == "3":
            print("\nSelect Mux channel to stream:")
            mux_list = list(Mux)
            for i, m in enumerate(mux_list, start=1):
                print(f"{i}: {m.name}")
            mux_choice = input("Enter choice: ").strip()
            try:
                mux_idx = int(mux_choice) - 1
                if not (0 <= mux_idx < len(mux_list)):
                    raise ValueError
                mux = mux_list[mux_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            board_idx = 0
            if len(boards) > 1:
                print("\nSelect board to stream from:")
                for i, (bname, _, _) in enumerate(boards):
                    print(f"{i+1}: {bname}")
                b_choice = input("Enter choice: ").strip()
                try:
                    board_idx = int(b_choice) - 1
                    if not (0 <= board_idx < len(boards)):
                        raise ValueError
                except ValueError:
                    print("Invalid board choice. Defaulting to first board.")
                    board_idx = 0

            board_name, board, _ = boards[board_idx]
            print(f"\nStreaming from {mux.name} on {board_name}. Press Ctrl+C to stop.")

            try:
                while True:
                    if mux == Mux.BOARD_TMP:
                        val = board.read_board_thermistor()
                        if val:
                            print(f"{mux.name}: {val.temperature_c:.2f} C")
                        else:
                            print(f"{mux.name}: Read failed")
                    else:
                        board._configure(mux, DataRate.SPS_100)
                        val = board.read_voltage()
                        if val is not None:
                            print(f"{mux.name}: {val:.4f} V")
                        else:
                            print(f"{mux.name}: Read failed")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nStopped streaming.")
        elif choice == "4":
            print("\nSelect board to set DAC voltage:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, dac))
            
            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, dac = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            if dac is None:
                cs_input = input(f"No DAC initialized for {bname}. Enter GPIO CS pin to initialize (or press Enter to cancel): ").strip()
                if not cs_input:
                    continue
                try:
                    cs_pin = int(cs_input)
                    dac = dac5311Driver("/dev/spidev0.0", "gpiochip0", cs_pin)
                    boards[board_idx] = (boards[board_idx][0], boards[board_idx][1], dac)
                except Exception as e:
                    print(f"Failed to initialize DAC: {e}")
                    continue

            v_input = input("Enter voltage to set (0 to 5.1): ").strip()
            try:
                volts = float(v_input)
                actual_v = dac.set_voltage(volts)
                print(f"Set voltage for {bname} DAC to {actual_v:.4f} V")
            except ValueError:
                print("Invalid voltage.")
        elif choice == "q":
            break
        else:
            print("Unknown choice. Please select a valid option.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="UVIC photodiode readout ADC Test Script"
    )
    parser.add_argument(
        "-b",
        "--board",
        type=int,
        choices=[1, 2, 3],
        default=3,
        help="Select board: 1 for Sergeant, 2 for Soldier, 3 for both (default)",
    )
    parser.add_argument(
        "--dac1-cs",
        type=int,
        default=None,
        help="GPIO CS pin for Sergeant DAC5311 (default: prompt if needed)",
    )
    parser.add_argument(
        "--dac2-cs",
        type=int,
        default=None,
        help="GPIO CS pin for Soldier DAC5311 (default: prompt if needed)",
    )
    args = parser.parse_args()

    board_only = None
    if args.board == 1:
        board_only = 1
    elif args.board == 2:
        board_only = 2

    try:
        boards = open_boards(
            only=board_only,
            spi_dev="/dev/spidev0.0",
            gpiochip="gpiochip0",
            cs1=7,
            cs2=8,
            dac_cs1=args.dac1_cs,
            dac_cs2=args.dac2_cs,
        )
    except OSError as e:
        print(e)
        sys.exit(1)

    if not boards:
        print("No boards opened.")
        sys.exit(1)

    try:
        interactive_menu(boards)
    finally:
        for _, board, dac in boards:
            board.close()
            if dac:
                dac.close()
