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
from drivers.mcp23017 import MCP23017
from drivers.integrator_driver import IntegratorDriver


def open_boards(
    only,
    spi_dev,
    gpiochip,
    cs1,
    cs2,
    drdy1,
    drdy2,
    start1,
    start2,
    dac_cs1,
    dac_cs2,
):
    boards = []
    if only in (None, 1):
        adc = ads124s08Driver(spi_dev, gpiochip, cs1, drdy1, start1)
        dac = dac5311Driver(spi_dev, gpiochip, dac_cs1) if dac_cs1 is not None else None
        boards.append((f"Sergeant ADC (CS=GPIO{cs1})", adc, dac))
    if only in (None, 2):
        adc = ads124s08Driver(spi_dev, gpiochip, cs2, drdy2, start2)
        dac = dac5311Driver(spi_dev, gpiochip, dac_cs2) if dac_cs2 is not None else None
        boards.append((f"Soldier ADC (CS=GPIO{cs2})", adc, dac))
    return boards


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
    print(f"  {name:25} | Actual: {actual_str:18} | Expected: {expected_str:30} | {color}[{status}]{reset}")


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
    print(f"  {name:25} | Actual: {actual_str:18} | Expected: {expected_str:30} | {color}[{status}]{reset}")


def run_all_checks(boards):
    print("\n" + "=" * 105)
    print(" " * 42 + "RUNNING BATCH CHECKS")
    print("=" * 105)
    for board_name, board, dac in boards:
        print(f"\n--- {board_name} ---")
        print("-" * 105)
        
        # 1. Config
        expected_config = board._configure(Mux.VGND, DataRate.SPS_1000)
        read_config = board.read_config()
        passed = (read_config == expected_config)
        status = "PASS" if passed else "FAIL - MISMATCH"
        color = "\033[92m" if passed else "\033[91m\033[1m"
        reset = "\033[0m"
        actual_str = "Match" if passed else "Mismatch"
        expected_str = "Match"
        print(f"  {'Config Read/Write':25} | Actual: {actual_str:18} | Expected: {expected_str:30} | {color}[{status}]{reset}")

        # 2. VGND
        board._configure(Mux.VGND, DataRate.SPS_100)
        check_range("VGND Voltage", board.read_voltage(), 4.85, 4.95, "V")

        # 3. TIA
        board._configure(Mux.TIA, DataRate.SPS_100)
        check_range("TIA Voltage", board.read_voltage(), 4.85, 4.95, "V")

        # 4. IVC
        board._configure(Mux.IVC, DataRate.SPS_100)
        check_range("IVC Level Shifter", board.read_voltage(), 0.0, 0.1, "V")

        # 5. ACF
        board._configure(Mux.ACF, DataRate.SPS_100)
        check_range("ACF Level Shifter", board.read_voltage(), 0.0, 0.4, "V")

        # 6. Board Thermistor
        therm_out: ThermistorReading = board.read_board_thermistor()
        check_thermistor("Board Thermistor", therm_out, 20.0, 40.0)

        # 7. PD Thermistor
        therm_out: ThermistorReading = board.read_pd_thermistor()
        check_thermistor("Photodiode Thermistor", therm_out, 20.0, 40.0)

    print("\n" + "=" * 105)
    print(" " * 44 + "CHECKS COMPLETE")
    print("=" * 105 + "\n")


def interactive_menu(boards, integrator: IntegratorDriver = None):
    while True:
        print("\n--- ADC Interactive Test Menu ---")
        print("1. Run all checks")
        print("2. Manually actuate relays")
        print("3. Stream data from a Mux channel")
        print("4. Set DAC voltage")
        print("5. Integrator read command")
        print("6. Read ADC register")
        print("7. Write ADC register")
        print("8. Send ADC reset")
        print("9. Read all ADC registers")
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
                    elif mux == Mux.PD_TMP:
                        val = board.read_pd_thermistor()
                        if val:
                            print(f"{mux.name}: {val.temperature_c:.2f} C")
                        else:
                            print(f"{mux.name}: Read failed")
                    else:
                        board._configure(mux, DataRate.SPS_100)
                        val = board.read_voltage()
                        if val is not None:
                            print(f"{mux.name}: {val:.8f} V")
                        else:
                            print(f"{mux.name}: Read failed")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nStopped streaming.")
        elif choice == "5":
            if integrator is None:
                print("Integrator driver not initialized.")
                continue

            print("\nSelect board:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, adc))

            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, adc = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            print("\nSelect integrator:")
            print("1. IVC")
            print("2. ACF")
            int_choice = input("Enter choice: ").strip()

            if int_choice == "1":
                mux_channel = Mux.IVC
                relay_channel = Relay.IVC
            elif int_choice == "2":
                mux_channel = Mux.ACF
                relay_channel = Relay.ACF
            else:
                print("Invalid integrator choice.")
                continue

            time_input = input("Enter integration time in microseconds: ").strip()
            try:
                time_us = float(time_input)
            except ValueError:
                print("Invalid time.")
                continue

            # Configure the adc input mux appropriately
            adc._configure(mux_channel, DataRate.SPS_100)

            # Switch the relay to the appropriate channel
            adc.set_relays(relay_channel.value)
            # Wait a moment for relays to settle
            time.sleep(0.05)
            # Trigger an integrate and hold
            integrator.integrate_and_hold(time_us)
            # Make a voltage reading
            val = adc.read_voltage()
            # Reset the integrator
            integrator.reset()
            if val is not None:
                print(f"Read voltage: {val:.8f} V")
            else:
                print("Read failed")

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
                cs_input = input(
                    f"No DAC initialized for {bname}. Enter GPIO CS pin to initialize (or press Enter to cancel): "
                ).strip()
                if not cs_input:
                    continue
                try:
                    cs_pin = int(cs_input)
                    dac = dac5311Driver("/dev/spidev0.0", "gpiochip0", cs_pin)
                    boards[board_idx] = (
                        boards[board_idx][0],
                        boards[board_idx][1],
                        dac,
                    )
                except Exception as e:
                    print(f"Failed to initialize DAC: {e}")
                    continue

            v_input = input("Enter voltage to set (0 to 5.1): ").strip()
            try:
                volts = float(v_input)
                actual_v = dac.set_voltage(volts)
                print(f"Set voltage for {bname} DAC to {actual_v:.8f} V")
            except ValueError:
                print("Invalid voltage.")
        elif choice == "6":
            print("\nSelect board:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, adc))

            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, adc = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            reg_input = input("Enter register address (hex or int): ").strip()
            try:
                addr = int(reg_input, 0)
                if not (0 <= addr <= 0x1F):
                    print("Address out of range (0-0x1F).")
                    continue
            except ValueError:
                print("Invalid register address.")
                continue

            val = adc.read_register(addr)
            if val is not None:
                print(f"Register 0x{addr:02X} value: 0x{val:02X} ({val})")
            else:
                print("Failed to read register.")
        elif choice == "7":
            print("\nSelect board:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, adc))

            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, adc = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            reg_input = input("Enter register address (hex or int): ").strip()
            try:
                addr = int(reg_input, 0)
                if not (0 <= addr <= 0x1F):
                    print("Address out of range (0-0x1F).")
                    continue
            except ValueError:
                print("Invalid register address.")
                continue

            val_input = input("Enter value to write (hex or int): ").strip()
            try:
                val = int(val_input, 0)
                if not (0 <= val <= 0xFF):
                    print("Value out of range (0-0xFF).")
                    continue
            except ValueError:
                print("Invalid value.")
                continue

            success = adc.write_register(addr, val)
            if success:
                print(f"Successfully wrote 0x{val:02X} to register 0x{addr:02X}.")
            else:
                print("Failed to write register.")
        elif choice == "9":
            print("\nSelect board:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, adc))

            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, adc = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            print(f"\n--- Registers for {bname} ---")
            for addr in range(0x12):
                val = adc.read_register(addr)
                if val is not None:
                    print(f"Reg 0x{addr:02X}: 0x{val:02X} (0b{val:08b})")
                else:
                    print(f"Reg 0x{addr:02X}: Failed")
        elif choice == "8":
            print("\nSelect board:")
            valid_boards = []
            for i, (bname, adc, dac) in enumerate(boards):
                print(f"{i+1}: {bname}")
                valid_boards.append((bname, adc))

            b_choice = input("Enter choice: ").strip()
            try:
                board_idx = int(b_choice) - 1
                if not (0 <= board_idx < len(valid_boards)):
                    raise ValueError
                bname, adc = valid_boards[board_idx]
            except ValueError:
                print("Invalid choice.")
                continue

            success = adc.reset()
            if success:
                print(f"Successfully sent reset command to {bname}.")
            else:
                print(f"Failed to send reset command to {bname}.")
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
    args = parser.parse_args()

    board_only = None
    if args.board == 1:
        board_only = 1
    elif args.board == 2:
        board_only = 2

    try:
        mcp = MCP23017()
        integrator = IntegratorDriver(mcp)
    except Exception as e:
        print(f"Failed to initialize IntegratorDriver: {e}")
        sys.exit(1)

    try:
        boards = open_boards(
            only=board_only,
            spi_dev="/dev/spidev0.0",
            gpiochip="gpiochip0",
            cs1=13,
            cs2=19,
            drdy1=22,
            drdy2=24,
            start1=25,
            start2=8,
            dac_cs1=12,
            dac_cs2=6,
        )
    except (OSError, RuntimeError) as e:
        print(e)
        integrator.io.close()
        sys.exit(1)

    if not boards:
        print("No boards opened.")
        integrator.io.close()
        sys.exit(1)

    try:
        interactive_menu(boards, integrator)
    finally:
        if integrator and integrator.io:
            integrator.io.close()
        for _, board, dac in boards:
            board.close()
            if dac:
                dac.close()
