from altairfc.drivers.ads124s08_driver import MUX_VGND
import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.ads124s08_driver import ads124s08_driver,Mux,DataRate,ThermistorReading  # noqa: E402

def open_boards(only, spi_dev, gpiochip, cs1, cs2):
    boards = []
    if only in (None, 1):
        boards.append((f"Sergeant ADC (CS=GPIO{cs1})", ads124s08_driver(spi_dev, gpiochip, cs1)))
    if only in (None, 2):
        boards.append((f"Soldier ADC (CS=GPIO{cs2})", ads124s08_driver(spi_dev, gpiochip, cs2)))
    return boards

# Check that we can write and read config registers
def test_configset(adc: ads124s08_driver):
    print(f"Testing configuration for {adc}")
    expected_config = adc._configure(Mux.VGND, DataRate.SPS_1000)
    read_config = adc.read_config()
    print(f"Read config: {read_config}")
    assert read_config == expected_config

# Check VGND reading. This should be in the range of 4.85-4.95V
def check_vgnd_read(adc: ads124s08_driver):
    print(f"Testing VGND reading for {adc}")
    adc.configure(Mux.VGND, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"VGND read: {val}")
    assert 4.85 <= val <= 4.95

# Check TIA reading. This should be in the range of 4.85-4.95V
def check_TIA_read(adc: ads124s08_driver):
    print(f"Testing TIA reading for {adc}")
    adc.configure(Mux.TIA, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"TIA read: {val}")
    assert 4.85 <= val <= 4.95

# Check the IVC level shifter output. Should default to near 0V
def check_ivc_read(adc: ads124s08_driver):
    print(f"Testing IVC reading for {adc}")
    adc.configure(Mux.IVC, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"IVC read: {val}")
    assert 0 <= val <= 0.1

# Check the ACF level shifter output. Should default to near 0V
def check_acf_read(adc: ads124s08_driver):
    print(f"Testing ACF reading for {adc}")
    adc.configure(Mux.ACF, DataRate.SPS_100)
    val = adc.read_voltage()
    print(f"ACF read: {val}")
    assert 0 <= val <= 0.1

# Check that the board thermistor gives reasonable temperature values
def check_board_thermistor(adc: ads124s08_driver):
    print(f"Testing thermistor reading for {adc}")
    therm_out: ThermistorReading = adc.read_board_thermistor()
    print(f"Thermistor read: {therm_out}")
    assert 20 <= therm_out.temperature_c <= 40

try:
    boards = open_boards(only=None, spi_dev="/dev/spidev0.0", gpiochip="gpiochip0", cs1=7, cs2=8)
except OSError as e:
    print(e)
    sys.exit(1)

for board_name, board in boards:
    test_configset(board)
    check_vgnd_read(board)
    check_TIA_read(board)
    check_ivc_read(board)
    check_acf_read(board)
    check_board_thermistor(board)





