"""
ALTAIR V2 Flight Computer — Entry Point

Startup sequence:
  1. Load configuration from config/settings.toml
  2. Create the shared DataStore (blackboard)
  3. Import all packet types so the registry is populated before TelemetryTask starts
  4. Instantiate and register all enabled tasks with the TaskScheduler
  5. Install OS signal handlers (SIGINT, SIGTERM)
  6. Start all tasks
  7. Block on the shutdown event
  8. Stop all tasks gracefully
"""

from __future__ import annotations

import logging
import subprocess
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Bootstrap logging before importing project modules so their loggers work
# ---------------------------------------------------------------------------
from core.log_format import setup_logging
setup_logging("INFO")
logger = logging.getLogger("main")

# ---------------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------------
from config.settings import SystemConfig
from core.datastore import DataStore
from core.lifecycle import install_signal_handlers, shutdown_event
from core.scheduler import TaskScheduler
from core.watchdog import WatchdogThread
from core.buzzer_player import BuzzerPlayer
from drivers.buzzer import TUNE_PENDING, TUNE_SUCCESS, TUNE_SUCCESS_REVERSE

# Import all packet modules so their @register decorators fire before
# TelemetryTask.execute() iterates the registry.
import telemetry.packets.heartbeat       # noqa: F401
import telemetry.packets.attitude        # noqa: F401
import telemetry.packets.power           # noqa: F401
import telemetry.packets.vesc            # noqa: F401
import telemetry.packets.photodiode      # noqa: F401
import telemetry.packets.gps             # noqa: F401
import telemetry.packets.environment     # noqa: F401
import telemetry.packets.events          # noqa: F401
import telemetry.packets.ack             # noqa: F401
import telemetry.packets.flight_settings  # noqa: F401
import telemetry.packets.pointing         # noqa: F401

# Import command modules so their @register decorators populate command_registry
import telemetry.commands.arm            # noqa: F401
import telemetry.commands.launch_ok      # noqa: F401
import telemetry.commands.ping           # noqa: F401
import telemetry.commands.update_setting  # noqa: F401
import telemetry.commands.gs_gps         # noqa: F401

from tasks.gps_task import GpsTask
from tasks.mavlink_task import MavlinkTask
from tasks.command_receiver_task import CommandReceiverTask
from tasks.flight_stage_task import FlightStageTask
from tasks.photodiode_task import PhotodiodeTask
from tasks.power_task import PowerTask
from tasks.rw_task import RWTask
from tasks.mm_task import MMTask
from telemetry.telemetry_task import TelemetryTask
from telemetry.transport import SerialTransport
from tasks.pitch_task import PitchTask
from tasks.datalogger_task import DataLoggerTask


def main() -> None:
    buzzer = BuzzerPlayer()
    buzzer.start()
    buzzer.play(TUNE_PENDING)

    build_script = Path(__file__).parent / "drivers" / "build_all.sh"
    logger.info("Building C drivers via %s", build_script)
    subprocess.run(["bash", str(build_script)], check=True)

    config_path = Path(__file__).parent / "config" / "settings.toml"
    logger.info("Loading config from %s", config_path)
    config = SystemConfig.from_toml(config_path)

    # Create the per-session log directory now so both the file logger and
    # DataLoggerTask share the same timestamped folder.
    session_name = time.strftime("%Y-%m-%d_%H-%M-%S")
    datalogger_enabled = config.tasks.get("datalogger", None)
    if datalogger_enabled and datalogger_enabled.enabled:
        session_dir = config.log_root / session_name
        session_dir.mkdir(parents=True, exist_ok=True)
        setup_logging(config.log_level, log_file=session_dir / "flight.log")
        logger.info("Log session: %s", session_dir)
    else:
        session_dir = None
        setup_logging(config.log_level)

    datastore = DataStore()

    # Write all flight settings to DataStore before tasks start.
    # FlightStageTask, RWTask, and MMTask read these keys each cycle so that
    # an UpdateSettingCommand from the GS takes effect without a restart.
    _fs = config.flight_stage
    _rw = config.controller["reaction_wheel"]
    _mm = config.controller["momentum_management"]
    for _key, _val in {
        "settings.termination_altitude_m":       _fs.termination_altitude_m,
        "settings.burst_altitude_m":             _fs.burst_altitude_m,
        "settings.burst_altitude_uncertainty_m": _fs.burst_altitude_uncertainty_m,
        "settings.ascent_detect_window_s":       _fs.ascent_detect_window_s,
        "settings.ascent_detect_gain_m":         _fs.ascent_detect_gain_m,
        "settings.apogee_fraction":              _fs.apogee_fraction,
        "settings.landing_fraction":             _fs.landing_fraction,
        "settings.recovery_stationary_s":        _fs.recovery_stationary_s,
        "settings.termination_confirm_drop_m":   _fs.termination_confirm_drop_m,
        "settings.termination_confirm_window_s": _fs.termination_confirm_window_s,
        "settings.pointing_activate_altitude_m": _fs.pointing_activate_altitude_m,
        "settings.pointing_duration_min":        _fs.pointing_duration_min,
        "settings.rw_kp":          _rw.Kp,
        "settings.rw_kd":          _rw.Kd,
        "settings.rw_max_rpm":     _rw.max,
        "settings.mm_kp":          _mm.Kp,
        "settings.mm_kd":          _mm.Kd,
        "settings.mm_max_current": _mm.max,
    }.items():
        datastore.write(_key, float(_val))
    datastore.write("settings.gs_use_hardcoded", 1.0 if config.ground_station.use_hardcoded else 0.0)
    datastore.write("settings.gs_lat", float(config.ground_station.latitude))
    datastore.write("settings.gs_lon", float(config.ground_station.longitude))
    datastore.write("settings.gs_alt", float(config.ground_station.altitude))
    logger.info("Wrote 18 flight settings to DataStore")

    scheduler = TaskScheduler(datastore, config)

    # ------------------------------------------------------------------
    # Register tasks — scheduler.register() silently skips disabled tasks
    # ------------------------------------------------------------------

    scheduler.register(
        RWTask(
            name="reaction_wheel",
            period_s=config.tasks["rw_control"].period_s,
            datastore=datastore,
            vesc_port=config.rw_esc,
            controller_config=config.controller["reaction_wheel"],
            ground_station=config.ground_station,
        )
    )
    
    scheduler.register(
        MavlinkTask(
            name="mavlink",
            period_s=config.tasks["mavlink"].period_s,
            datastore=datastore,
            port_config=config.mavlink,
        )
    )

    scheduler.register(
        GpsTask(
            name="gps",
            period_s=config.tasks["gps"].period_s,
            datastore=datastore,
        )
    )

#    scheduler.register(
#       MMTask(
#            name="momentum_management",
#            period_s=config.tasks["mm_control"].period_s,
#            datastore=datastore,
#            vesc_port=config.mm_esc,
#            controller_config=config.controller["momentum_management"],
#        )
#    )

    scheduler.register(
        PitchTask(
            name="sphere_pitch",
            period_s=config.tasks["sphere_pitch"].period_s,
            datastore=datastore,
            ground_station=config.ground_station,
        )
    )

    if config.telemetry is not None:
        telemetry_transport = SerialTransport(
            port=config.telemetry.port,
            baud=config.telemetry.baud,
        )
        scheduler.register(
            TelemetryTask(
                name="telemetry",
                period_s=config.tasks["telemetry"].period_s,
                datastore=datastore,
                transport=telemetry_transport,
            )
        )
        scheduler.register(
            CommandReceiverTask(
                name="command_receiver",
                period_s=config.tasks["command_receiver"].period_s,
                datastore=datastore,
                transport=telemetry_transport,
                buzzer=buzzer,
            )
        )
    else:
        logger.info("Telemetry radio not configured — TelemetryTask and CommandReceiverTask skipped")

    scheduler.register(
        FlightStageTask(
            name="flight_stage",
            period_s=config.tasks["flight_stage"].period_s,
            datastore=datastore,
            config=config.flight_stage,
        )
    )

    scheduler.register(
        PhotodiodeTask(
            name="photodiode",
            period_s=config.tasks["photodiode"].period_s,
            datastore=datastore,
        )
    )

    scheduler.register(
        PowerTask(
            name="power",
            period_s=config.tasks["power"].period_s,
            datastore=datastore,
            i2c_dev=config.tasks["power"].extra.get("i2c_dev", "/dev/i2c-1"),
        )
    )

    if session_dir is not None:
        scheduler.register(
            DataLoggerTask(
                name="datalogger",
                period_s=config.tasks["datalogger"].period_s,
                datastore=datastore,
                log_root=session_dir,
            )
        )

    # ------------------------------------------------------------------
    # Signal handlers + startup
    # ------------------------------------------------------------------
    install_signal_handlers(scheduler) # handles CTRL-C and kill signals for graceful shutdown
    logger.info("Starting ALTAIR V2 flight computer")
    scheduler.start_all()
    buzzer.play(TUNE_SUCCESS)

    watchdog = WatchdogThread(scheduler, watchdog_sec=config.watchdog_sec)
    watchdog.start()

    # Block main thread until SIGINT/SIGTERM or a critical task failure
    scheduler.shutdown_event.wait()
    logger.info("Shutdown event received — stopping all tasks")
    buzzer.play(TUNE_SUCCESS_REVERSE)
    watchdog.stop()
    scheduler.stop_all()
    buzzer.stop()
    logger.info("ALTAIR V2 shutdown complete")


if __name__ == "__main__":
    main()
