"""Hardware-free tests for the UVIC PDRO board-level driver."""

from __future__ import annotations

import sys
import types
import unittest
from contextlib import contextmanager
from unittest.mock import patch

# The production MCP23017 module imports smbus2 at module load time.  Keep this
# unit test runnable on development machines that do not have Raspberry Pi I2C
# dependencies installed; UVICPDRO's MCP23017 class is replaced below.  Never
# replace a real smbus2 installation when the test runs on the Pi.
try:
    import smbus2  # noqa: F401
except ModuleNotFoundError:
    smbus2_stub = types.ModuleType("smbus2")
    smbus2_stub.SMBus = object
    sys.modules["smbus2"] = smbus2_stub
else:
    smbus2_stub = None

from drivers import uvic_pdro as pdro_module
from drivers.ads124s08_driver import DataRate, Mux
from drivers.uvic_pdro import Input, Readout, SignalPath, UVICPDRO

if smbus2_stub is not None:
    del sys.modules["smbus2"]


EVENTS = []


class FakeIO:
    instances = []

    def __init__(self, *, address, bus):
        self.address = address
        self.bus = bus
        self.closed = False
        self.instances.append(self)
        EVENTS.append("io.open")

    def close(self):
        self.closed = True
        EVENTS.append("io.close")


class FakeIntegrator:
    instances = []

    def __init__(self, io):
        self.io = io
        self.reset_count = 0
        self.instances.append(self)
        EVENTS.append("integrator.open")

    def integrate_and_hold(self, time_us, print_timing=True):
        EVENTS.append(("integrate", time_us, print_timing))
        return 10.0, 10.0 + time_us / 1_000_000.0

    def reset(self):
        self.reset_count += 1
        EVENTS.append("integrator.reset")


class FakeADC:
    instances = []

    def __init__(self, spi_dev, gpiochip, cs, drdy, start):
        self.spi_dev = spi_dev
        self.gpiochip = gpiochip
        self.cs = cs
        self.drdy = drdy
        self.start = start
        self.configurations = []
        self.relays = []
        self.closed = False
        self.instances.append(self)
        EVENTS.append(("adc.open", cs))

    def _configure(self, mux, data_rate):
        self.configurations.append((mux, data_rate))
        return mux, data_rate, 0, 0, 0

    def read_voltage(self):
        return 1.25

    def read_board_thermistor(self):
        return None

    def read_pd_thermistor(self):
        return None

    def set_relays(self, relays):
        self.relays.append(relays)

    def read_config(self):
        return 1, 2, 3, 4, 5

    def read_register(self, address):
        return address

    def write_register(self, address, value):
        return True

    def reset(self):
        return True

    def close(self):
        self.closed = True
        EVENTS.append(("adc.close", self.cs))


class FakeDAC:
    instances = []
    fail_on_cs = None

    def __init__(self, spi_dev, gpiochip, cs):
        if cs == self.fail_on_cs:
            raise OSError("DAC open failed")
        self.cs = cs
        self.voltages = []
        self.closed = False
        self.instances.append(self)
        EVENTS.append(("dac.open", cs))

    def set_voltage(self, volts):
        self.voltages.append(volts)
        return round(volts * 256 / 5.1) * 5.1 / 256

    def close(self):
        self.closed = True
        EVENTS.append(("dac.close", self.cs))


@contextmanager
def fake_hardware():
    EVENTS.clear()
    for fake in (FakeIO, FakeIntegrator, FakeADC, FakeDAC):
        fake.instances.clear()
    FakeDAC.fail_on_cs = None
    with patch.multiple(
        pdro_module,
        MCP23017=FakeIO,
        IntegratorDriver=FakeIntegrator,
        ads124s08Driver=FakeADC,
        dac5311Driver=FakeDAC,
    ):
        yield


class UVICPDROTests(unittest.TestCase):
    def test_owns_hardware_and_initializes_integrator_before_adcs(self):
        with fake_hardware():
            pdro = UVICPDRO()
            self.assertEqual(pdro.readouts, (Readout.SERGEANT, Readout.SOLDIER))
            self.assertEqual(
                EVENTS[:4],
                [
                    "io.open",
                    "integrator.open",
                    ("adc.open", 13),
                    ("dac.open", 12),
                ],
            )
            pdro.close()

    def test_voltage_read_configures_mux_and_signal_path(self):
        with fake_hardware():
            with UVICPDRO(readouts=(Readout.SERGEANT,)) as pdro:
                voltage = pdro.read_voltage(
                    Readout.SERGEANT,
                    Input.TIA_LOW_GAIN,
                    DataRate.SPS_1000,
                )
                adc = FakeADC.instances[0]
                self.assertEqual(voltage, 1.25)
                self.assertEqual(adc.relays[-1], SignalPath.TIA_LOW_GAIN.value)
                self.assertEqual(
                    adc.configurations,
                    [(Mux.TIA, DataRate.SPS_1000)],
                )

    def test_integrate_and_read_resets_integrator_after_measurement(self):
        with fake_hardware(), patch.object(UVICPDRO, "RELAY_SETTLE_S", 0.0):
            with UVICPDRO(readouts=(Readout.SOLDIER,)) as pdro:
                reading = pdro.integrate_and_read(
                    Readout.SOLDIER,
                    Input.IVC,
                    250.0,
                )
                self.assertEqual(reading.voltage_v, 1.25)
                self.assertAlmostEqual(reading.actual_time_us, 250.0)
                self.assertIn(("integrate", 250.0, False), EVENTS)
                self.assertEqual(FakeADC.instances[0].relays[-1], SignalPath.IVC.value)
                # One reset after the measurement; context-manager shutdown adds
                # another after this assertion.
                self.assertEqual(FakeIntegrator.instances[0].reset_count, 1)

    def test_close_makes_every_readout_safe_and_is_idempotent(self):
        with fake_hardware():
            pdro = UVICPDRO()
            pdro.set_bias_voltage(Readout.SERGEANT, 2.5)
            pdro.close()
            pdro.close()

            self.assertTrue(FakeIO.instances[0].closed)
            self.assertTrue(all(adc.closed for adc in FakeADC.instances))
            self.assertTrue(all(dac.closed for dac in FakeDAC.instances))
            self.assertTrue(all(adc.relays[-1] == 0 for adc in FakeADC.instances))
            self.assertTrue(all(dac.voltages[-1] == 0.0 for dac in FakeDAC.instances))
            self.assertEqual(EVENTS.count("io.close"), 1)

    def test_rejects_empty_readout_selection(self):
        with fake_hardware():
            with self.assertRaisesRegex(ValueError, "at least one"):
                UVICPDRO(readouts=())
            self.assertEqual(EVENTS, [])

    def test_partial_initialization_failure_closes_every_open_resource(self):
        with fake_hardware():
            FakeDAC.fail_on_cs = 6
            with self.assertRaisesRegex(OSError, "DAC open failed"):
                UVICPDRO()

            self.assertTrue(FakeIO.instances[0].closed)
            self.assertTrue(all(adc.closed for adc in FakeADC.instances))
            self.assertTrue(all(dac.closed for dac in FakeDAC.instances))


if __name__ == "__main__":
    unittest.main()
