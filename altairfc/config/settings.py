from __future__ import annotations

import logging
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from drivers.port_detect import find_lr900p_port

logger = logging.getLogger(__name__)


def _resolve_serial_port(cfg: dict[str, Any]) -> "SerialPortConfig | None":
    """
    Build a SerialPortConfig, resolving port="auto" by scanning for a CP210x device.
    Returns None when port="none" or when auto-detect finds no device (logs a warning).
    """
    port = cfg.get("port", "")
    if port.lower() == "none":
        return None
    if port.lower() == "auto":
        detected = find_lr900p_port()
        if detected is None:
            logger.warning(
                "Telemetry port set to 'auto' but no CP210x (LR-900p) device was detected — "
                "telemetry radio disabled. Set port explicitly in config/settings.toml to suppress this."
            )
            return None
        port = detected
    return SerialPortConfig(port=port, baud=cfg["baud"])

@dataclass
class ControllerConfig:
    Kp: float
    Ki: float
    Kd: float
    max: float = 0.0
    min: float = 0.0

ControllerConfigMap = dict[str, ControllerConfig | dict[str, ControllerConfig]]


def _build_controller_config(cfg: dict[str, Any]) -> ControllerConfig:
    max_val = cfg.get("max_rpm", cfg.get("max_current"))
    min_val = cfg.get("min_rpm", cfg.get("min_current"))
    return ControllerConfig(
        Kp=cfg["Kp"],
        Ki=cfg["Ki"],
        Kd=cfg["Kd"],
        max=max_val,
        min=min_val,
    )


@dataclass
class SerialPortConfig:
    port: str
    baud: int


@dataclass
class TaskConfig:
    name: str
    enabled: bool
    period_s: float
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class FlightStageConfig:
    termination_altitude_m:       float = 25000.0
    burst_altitude_m:             float = 30000.0
    burst_altitude_uncertainty_m: float = 2000.0
    ascent_detect_window_s:       float = 30.0
    ascent_detect_gain_m:         float = 50.0
    apogee_fraction:              float = 0.95
    landing_fraction:             float = 0.05
    recovery_stationary_s:        float = 10.0
    termination_confirm_drop_m:   float = 100.0
    termination_confirm_window_s: float = 30.0
    pointing_activate_altitude_m: float = 18000.0
    pointing_duration_min:        float = 120.0
    auto_advance:                  bool  = True
    preflight_debounce_s:          float = 5.0
    bypass_launch_altitude_checks: bool = False


@dataclass
class PointingConfig:
    spinup_rpm: int = 2150
    spinup_s: float = 5.0
    stabilize_yaw_rate: float = 0.1
    stability_threshold: float = 5.0
    saturation_rpm: float = 3500.0
    saturation_s: float = 5.0
    switch_threshold: float = 0.26
    yaw_rate_deadband: float = 0.05
@dataclass
class RadioConfig:
    data_rate:      int   = 1     # 0=Low, 1=Mid, 2=High
    tx_power:       int   = 2     # 0=Low, 1=Mid, 2=High
    channel:        int   = 0     # 0-63
    watchdog_s:     float = 30.0  # seconds without GS contact before rolling back a channel change
    config_enabled: bool  = False # False = transparent passthrough only, no config frames sent
    # Telemetry TX rate scale factor per data_rate index.
    # Applied to every packet's TX_RATE_HZ: effective_rate = TX_RATE_HZ * scale.
    # Index matches data_rate: [Low, Mid, High]
    rate_scale: list = field(default_factory=lambda: [0.1, 0.33, 1.0])


@dataclass
class GroundStationConfig:
    latitude: float
    longitude: float
    altitude: float
    use_hardcoded: bool = True


@dataclass
class SystemConfig:
    mavlink: SerialPortConfig
    telemetry: SerialPortConfig | None
    rw_esc: SerialPortConfig
    controller: ControllerConfigMap
    tasks: dict[str, TaskConfig]
    flight_stage: FlightStageConfig = field(default_factory=FlightStageConfig)
    pointing: PointingConfig = field(default_factory=PointingConfig)
    ground_station: GroundStationConfig = field(
        default_factory=lambda: GroundStationConfig(latitude=0.0, longitude=0.0, altitude=0.0)
    )
    radio_config: RadioConfig = field(default_factory=RadioConfig)
    log_level: str = "INFO"
    monitor_interval_s: float = 5.0
    watchdog_sec: float = 30.0
    log_root: Path = field(default_factory=lambda: Path("logs"))

    @classmethod
    def from_toml(cls, path: Path) -> "SystemConfig":
        with open(path, "rb") as f:
            data = tomllib.load(f)
        mavlink = SerialPortConfig(**data["mavlink"])
        telemetry = _resolve_serial_port(data["telemetry"])
        rw_esc = SerialPortConfig(**data["rw_esc"])
        controller: ControllerConfigMap = {}
        for name, cfg in data.get("controller", {}).items():
            if "Kp" in cfg:
                controller[name] = _build_controller_config(cfg)
            else:
                controller[name] = {
                    mode: _build_controller_config(mode_cfg)
                    for mode, mode_cfg in cfg.items()
                }

        tasks: dict[str, TaskConfig] = {}
        for name, cfg in data.get("tasks", {}).items():
            tasks[name] = TaskConfig(
                name=name,
                enabled=cfg.get("enabled", False),
                period_s=cfg.get("period_s", 1.0),
                extra={k: v for k, v in cfg.items() if k not in ("enabled", "period_s")},
            )

        fs_raw = data.get("flight_stage", {})
        flight_stage = FlightStageConfig(
            termination_altitude_m=fs_raw.get("termination_altitude_m"),
            burst_altitude_m=fs_raw.get("burst_altitude_m"),
            burst_altitude_uncertainty_m=fs_raw.get("burst_altitude_uncertainty_m"),
            ascent_detect_window_s=fs_raw.get("ascent_detect_window_s"),
            ascent_detect_gain_m=fs_raw.get("ascent_detect_gain_m"),
            apogee_fraction=fs_raw.get("apogee_fraction"),
            landing_fraction=fs_raw.get("landing_fraction"),
            recovery_stationary_s=fs_raw.get("recovery_stationary_s"),
            termination_confirm_drop_m=fs_raw.get("termination_confirm_drop_m"),
            termination_confirm_window_s=fs_raw.get("termination_confirm_window_s"),
            pointing_activate_altitude_m=fs_raw.get("pointing_activate_altitude_m"),
            pointing_duration_min=fs_raw.get("pointing_duration_min"),
            auto_advance=fs_raw.get("auto_advance", True),
            preflight_debounce_s=fs_raw.get("preflight_debounce_s", 5.0),
            bypass_launch_altitude_checks=fs_raw.get("bypass_launch_altitude_checks", False),
        )

        pointing_raw =  data.get("pointing", {})
        pointing = PointingConfig(
            spinup_rpm=pointing_raw.get("spinup_rpm", 0.0),
            spinup_s=pointing_raw.get("spinup_s", 0.0),
            stabilize_yaw_rate=pointing_raw.get("stabilize_yaw_rate"),
            stability_threshold=pointing_raw.get("stability_threshold"),
            saturation_rpm=pointing_raw.get("saturation_rpm", 3500.0),
            saturation_s=pointing_raw.get("saturation_s", 5.0),
            switch_threshold=pointing_raw.get("switch_threshold", 0.26),
            yaw_rate_deadband=pointing_raw.get("yaw_rate_deadband", 0.05),
        )

        gs_raw = data.get("ground_station", {})
        ground_station = GroundStationConfig(
            latitude=gs_raw.get("latitude"),
            longitude=gs_raw.get("longitude"),
            altitude=gs_raw.get("altitude"),
            use_hardcoded=gs_raw.get("use_hardcoded", True),
        )

        system = data.get("system", {})
        dl_raw = data.get("datalogger", {})
        log_root_str = dl_raw.get("log_root", "logs")
        log_root = Path(log_root_str)
        if not log_root.is_absolute():
            log_root = Path(__file__).parent.parent / log_root

        rc_raw = data.get("radio_config", {})
        radio_config = RadioConfig(
            data_rate=rc_raw.get("data_rate",           1),
            tx_power=rc_raw.get("tx_power",             2),
            channel=rc_raw.get("channel",               0),
            watchdog_s=rc_raw.get("watchdog_s",         30.0),
            config_enabled=rc_raw.get("config_enabled", False),
            rate_scale=rc_raw.get("rate_scale",         [0.1, 0.33, 1.0]),
        )

        return cls(
            mavlink=mavlink,
            telemetry=telemetry,
            rw_esc=rw_esc,
            controller=controller,
            tasks=tasks,
            flight_stage=flight_stage,
            pointing=pointing,
            ground_station=ground_station,
            radio_config=radio_config,
            log_level=system.get("log_level", "INFO"),
            monitor_interval_s=system.get("monitor_interval_s"),
            watchdog_sec=system.get("watchdog_sec"),
            log_root=log_root,
        )

    def get_task(self, name: str) -> TaskConfig | None:
        return self.tasks.get(name)
