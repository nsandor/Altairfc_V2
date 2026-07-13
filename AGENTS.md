# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

ALTAIR V2 is a Python flight computer for a high-altitude balloon, running on a Raspberry Pi 4B. It manages attitude control (reaction wheel + momentum management), telemetry downlink via radio, GPS, MAVLink (Pixhawk 6X mini), and automated flight stage sequencing.

The ground station (`ground/receiver.py`) mirrors the binary packet definitions and decodes radio downlink.

## Running the Flight Computer

```bash
cd altairfc
python main.py
```

Via systemd on the Pi:
```bash
sudo systemctl start flight      # start now
sudo systemctl stop flight       # stop now
sudo systemctl status flight     # check state + recent logs
journalctl -u flight -f          # tail logs
sudo systemctl enable flight     # auto-start on boot
sudo systemctl disable flight    # stop auto-starting
```

## Running Tests

```bash
cd altairfc
python -m pytest tests/
python -m pytest tests/test_datastore.py   # run a single test file
```

## Building C Drivers (one-time per deployment)

```bash
cd altairfc/drivers
bash build_gps.sh       # u-blox MAX-M10M GPS
bash build_ina3221.sh   # INA3221 power monitor
```

## Architecture

### Task Scheduler + DataStore Blackboard

The core pattern is a deadline-driven task scheduler where each subsystem runs as a daemon thread. All inter-task communication goes through `DataStore` — a thread-safe blackboard keyed on namespaced strings like `"mavlink.attitude.roll"`.

- `core/datastore.py` — `DataStore.read(key)` / `DataStore.write(key, value)` / `DataStore.read_with_timestamp(key)`
- `core/task_base.py` — `BaseTask` abstract class; subclass and implement `execute()`
- `core/scheduler.py` — `TaskScheduler` registers tasks, starts/stops all, monitors for critical failures
- `core/lifecycle.py` — installs `SIGINT`/`SIGTERM` handlers to trigger graceful shutdown

### Task Registration in main.py

Tasks are instantiated and passed to `scheduler.register()`. The scheduler silently skips tasks whose `[tasks.<name>] enabled = false` in `settings.toml`. Task periods are also configured there.

### Telemetry Packet System

New packets are defined as `@dataclass` classes decorated with `@packet_registry.register(packet_id=N)`. Every field **must** carry `FieldMeta` metadata specifying the `struct` format character, description, and units — the registry compiles a `struct.Struct` from these at import time.

```python
# Pattern for a new packet
from dataclasses import dataclass, field
from telemetry.registry import FieldMeta, packet_registry

@packet_registry.register(packet_id=0x0A)
@dataclass
class MyPacket:
    DATASTORE_KEYS: ClassVar[dict[str, str]] = {"foo": "sensor.foo"}
    foo: float = field(default=0.0, metadata=FieldMeta("f", "Foo value", "m/s").as_metadata())
```

All packet modules must be explicitly imported in `main.py` (before `TelemetryTask` starts) so their `@register` decorators fire. Add the import alongside the existing `import telemetry.packets.*` block.

### Ground Commands

Same decorator pattern via `command_registry` in `telemetry/command_registry.py`. New commands go in `telemetry/commands/` and must be imported in `main.py`.

### Binary Frame Format

```
[SYNC 0xAA][PKT_ID u8][SEQ u8][TIMESTAMP f64][LEN u16][PAYLOAD ...][CRC16]
```

CRC is CRC-16/CCITT (`binascii.crc_hqx`). All multi-byte fields are little-endian. Sequence counter is per packet type.

### Configuration

All runtime parameters live in `altairfc/config/settings.toml`. The `SystemConfig.from_toml()` parser maps TOML sections to typed dataclasses in `config/settings.py`. To add a new config section, add a dataclass in `settings.py` and parse it in `from_toml()`.

Key config sections:
- `[tasks.<name>]` — `enabled` / `period_s` per task
- `[controller.reaction_wheel]` / `[controller.momentum_management]` — PID gains
- `[flight_stage]` — altitude thresholds and timing windows for the state machine
- `[rw_esc]` / `[mm_esc]` — serial ports for VESC motor controllers

### DataStore Key Conventions

| Prefix | Source task | Contents |
|---|---|---|
| `mavlink.*` | MavlinkTask | Pixhawk attitude, rates, baro altitude |
| `gps.*` | GpsTask | Position, velocity, fix quality |
| `vesc.*` | VescTask | Motor RPM, current, temperature |
| `flight.*` | FlightStageTask | `flight.stage` enum value |
| `system.*` | TelemetryTask (heartbeat helpers) | CPU load, uptime, PPS offset |
| `power.*` | PowerTask | Battery voltage/current |
| `photodiode.*` | PhotodiodeTask | Optical sensor readings |

### Hardware Stubs

`FlightStageTask` has placeholder functions for reading the arm switch GPIO and firing the cutdown mechanism — these are `TODO` stubs that return safe defaults. Before flight, implement `_read_arm_switch()` and `_fire_cutdown()` in `tasks/flight_stage_task.py`.
