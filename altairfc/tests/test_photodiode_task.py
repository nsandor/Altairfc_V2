from __future__ import annotations

from types import SimpleNamespace

import pytest

from core.datastore import DataStore
from tasks.photodiode_task import PhotodiodeTask


class FakePDRO:
    def __init__(
        self,
        readouts,
        signal_values=(1.0, 2.0),
        board_temperatures=(20.0, 21.0),
        photodiode_temperatures=(22.0, 23.0),
        fail_bias=False,
    ):
        self.readouts = tuple(readouts)
        self.signal_values = iter(signal_values)
        self.board_temperatures = iter(board_temperatures)
        self.photodiode_temperatures = iter(photodiode_temperatures)
        self.fail_bias = fail_bias
        self.bias_calls = []
        self.read_calls = []
        self.closed = False

    def set_bias_voltage(self, readout, voltage):
        self.bias_calls.append((readout, voltage))
        if self.fail_bias and len(self.bias_calls) == 2:
            raise OSError("bias write failed")
        return voltage

    def read_voltage(self, readout, input_channel, data_rate):
        self.read_calls.append(("signal", readout, input_channel, data_rate))
        return next(self.signal_values)

    def read_board_thermistor(self, readout, data_rate):
        self.read_calls.append(("board_temperature", readout, None, data_rate))
        value = next(self.board_temperatures)
        return None if value is None else SimpleNamespace(temperature_c=value)

    def read_photodiode_thermistor(self, readout, data_rate):
        self.read_calls.append(
            ("photodiode_temperature", readout, None, data_rate)
        )
        value = next(self.photodiode_temperatures)
        return None if value is None else SimpleNamespace(temperature_c=value)

    def close(self):
        self.closed = True


class FakePDROFactory:
    def __init__(self, **driver_kwargs):
        self.driver_kwargs = driver_kwargs
        self.instances = []

    def __call__(self, readouts):
        instance = FakePDRO(readouts, **self.driver_kwargs)
        self.instances.append(instance)
        return instance


def make_task(factory, datastore=None, **kwargs):
    return PhotodiodeTask(
        name="photodiode",
        period_s=0.01,
        datastore=datastore or DataStore(),
        pdro_factory=factory,
        **kwargs,
    )


def test_setup_opens_both_readouts_and_applies_bias():
    datastore = DataStore()
    factory = FakePDROFactory()
    task = make_task(factory, datastore, bias_voltage_v=2.5)

    task.setup()

    pdro = factory.instances[0]
    assert [readout.name for readout in pdro.readouts] == ["SERGEANT", "SOLDIER"]
    assert [(readout.name, voltage) for readout, voltage in pdro.bias_calls] == [
        ("SERGEANT", 2.5),
        ("SOLDIER", 2.5),
    ]
    assert datastore.read("system.photodiode_connected") == 1
    task.teardown()


def test_execute_publishes_fixed_six_value_sample_with_category_rates():
    datastore = DataStore()
    factory = FakePDROFactory(
        signal_values=(0.1, 0.2),
        board_temperatures=(24.0, 25.0),
        photodiode_temperatures=(26.0, 27.0),
    )
    task = make_task(
        factory,
        datastore,
        signal_data_rate="SPS_2000",
        temperature_data_rate="SPS_50",
    )
    task.setup()

    task.execute()

    expected = {
        "photodiode.sergeant.tia_low_gain": 0.1,
        "photodiode.soldier.tia_low_gain": 0.2,
        "photodiode.sergeant.board_temperature": 24.0,
        "photodiode.soldier.board_temperature": 25.0,
        "photodiode.sergeant.photodiode_temperature": 26.0,
        "photodiode.soldier.photodiode_temperature": 27.0,
    }
    assert datastore.read_namespace("photodiode.") == expected

    pdro = factory.instances[0]
    assert [call[0] for call in pdro.read_calls] == [
        "signal",
        "signal",
        "board_temperature",
        "board_temperature",
        "photodiode_temperature",
        "photodiode_temperature",
    ]
    assert [call[3].name for call in pdro.read_calls] == [
        "SPS_2000",
        "SPS_2000",
        "SPS_50",
        "SPS_50",
        "SPS_50",
        "SPS_50",
    ]
    assert all(
        datastore.read_with_timestamp(key)[1]
        == datastore.read_with_timestamp(next(iter(expected)))[1]
        for key in expected
    )
    task.teardown()


def test_failed_conversion_does_not_publish_partial_sample_set():
    datastore = DataStore()
    factory = FakePDROFactory(signal_values=(0.1, None))
    task = make_task(factory, datastore)
    task.setup()

    with pytest.raises(OSError, match="soldier TIA low gain"):
        task.execute()

    assert datastore.read_namespace("photodiode.") == {}
    task.teardown()


def test_teardown_closes_board_and_marks_it_disconnected():
    datastore = DataStore()
    factory = FakePDROFactory()
    task = make_task(factory, datastore)
    task.setup()

    task.teardown()
    task.teardown()

    assert factory.instances[0].closed
    assert datastore.read("system.photodiode_connected") == 0


def test_setup_failure_closes_partially_configured_board():
    datastore = DataStore()
    factory = FakePDROFactory(fail_bias=True)
    task = make_task(factory, datastore, bias_voltage_v=2.5)

    with pytest.raises(OSError, match="bias write failed"):
        task.setup()

    assert factory.instances[0].closed
    assert datastore.read("system.photodiode_connected") == 0


@pytest.mark.parametrize("argument", ["signal_data_rate", "temperature_data_rate"])
def test_invalid_data_rate_fails_before_opening_hardware(argument):
    datastore = DataStore()
    factory = FakePDROFactory()
    task = make_task(factory, datastore, **{argument: "not-a-rate"})

    with pytest.raises(ValueError, match="unknown PDRO data rate"):
        task.setup()

    assert factory.instances == []
    assert datastore.read("system.photodiode_connected") == 0
