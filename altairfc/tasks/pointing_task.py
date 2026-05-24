from __future__ import annotations

import logging
import time
from enum import Enum

from config.settings import ControllerConfig, GroundStationConfig, PointingConfig, SerialPortConfig
from controls.controller import Controller
from controls.error_computation import compute_error
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.mm_driver import MMDriver
from drivers.rw_driver import RWDriver

logger = logging.getLogger(__name__)

class PointingState(Enum):
    IDLE = "idle" # Momentum management blocks bearing from free spinning
    SPINUP = "spinup" # RW spinup
    STABILIZE = "stabilize" # MM stabilizes and large initial error correction
    POINTING = "pointing" # PID active pointing 

class PointingTask(BaseTask):
    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        rw_port: SerialPortConfig | None,
        mm_port: SerialPortConfig | None,
        rw_controller_config: ControllerConfig,
        mm_controller_config: ControllerConfig,
        ground_station: GroundStationConfig,
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._mm_enabled = pointing_config.mm_enabled
        self._spinup_rpm = pointing_config.spinup_rpm
        self._spinup_s = pointing_config.spinup_s
        self._stabilize_yaw_rate = pointing_config.stabilize_yaw_rate
        self._stability_threshold = pointing_config.stability_threshold
        self._brake_current = pointing_config.brake_current
        self._mm_pulse_length = pointing_config.mm_pulse_length
        self._mm_control_period = pointing_config.mm_control_period
        self.period = period_s
        self.rw = RWDriver(rw_port.port) 
        self.mm = MMDriver(mm_port.port) if self._mm_enabled else None
        self.rw_controller = Controller(rw_controller_config, period_s)
        self.mm_controller = Controller(mm_controller_config, self._mm_control_period)
        self._default_gs_pos = [ground_station.latitude, ground_station.longitude, ground_station.altitude]
        

    def setup(self) -> None:
        self.passed = False
        self._state = PointingState.IDLE
        self._state_started = time.monotonic()
        self._stable_since = None
        self._last_mm_command = 0.0
        self._mm_pulse_until = 0.0

        if self.rw is not None:
            if not self.rw.connect():
                self.datastore.write("system.rw_vesc_connected", 0.0)
                raise ConnectionError("RW ESC enabled but not connected")
            self.datastore.write("system.rw_vesc_connected", 1.0)

        if self.mm is not None:
            if not self.mm.connect():
                self.datastore.write("system.mm_vesc_connected", 0.0)
                raise ConnectionError("MM ESC enabled but not connected")
            self.datastore.write("system.mm_vesc_connected", 1.0)

    def execute(self) -> None:
        self._store()

        active = int(self.datastore.read("event.pointing_active", default=0.0))
        if active == 1:
            if self.passed == False:
                self.passed = True
                if self._spinup_rpm != 0.0:
                    self._set_state(PointingState.SPINUP)
                elif self.mm is not None:
                    self._set_state(PointingState.STABILIZE)
                else:
                    self._set_state(PointingState.POINTING)
        
        if self._state == PointingState.IDLE and self.mm is not None:
            self.mm.set_brake_current(self._brake_current)
        
        if self._state == PointingState.SPINUP:
            self._check()
            self.rw.set_rpm(self._spinup_rpm)
            if time.monotonic() - self._state_started >= self._spinup_s:
                if self.mm is not None:
                    self._set_state(PointingState.STABILIZE)
                else:
                    self._set_state(PointingState.POINTING)

        if self._state == PointingState.STABILIZE and self.mm is not None:
            self._check()
            _, _, _, yaw_rate, _ = self._read()
            # self.rw.set_rpm(self._spinup_rpm)
            self.mm.set_brake_current(self._brake_current)
            if abs(yaw_rate) < self._stabilize_yaw_rate:
                if self._stable_since is None:
                    self._stable_since = time.monotonic()
                elif time.monotonic() - self._stable_since >= self._stability_threshold:
                    self._set_state(PointingState.POINTING)
            else:
                self._stable_since = None
        
        elif self._state == PointingState.POINTING:
            self._check()
            self._point()

    def teardown(self) -> None:
        self.rw.close()
        if self.mm is not None:
            self.mm.close()
    
    def _point(self) -> None:
        quat, pos, gs_pos, yaw_rate, yaw = self._read()
        az_err, _ = compute_error(quat, pos, gs_coords=gs_pos)
        control_signal = self.rw_controller.output(yaw, yaw_rate)
        self.datastore.write("pointing.az_error", az_err)
        self.datastore.write("pointing.control_signal", control_signal)
        self.rw.set_rpm(int(control_signal))

        if self.mm is not None and abs(yaw) > 0.1:
            rpm_err = self.datastore.read("rw.rpm", default = self._spinup_rpm)
            mm_cmd = self.mm_controller.output(rpm_err)
            self.datastore.write("pointing.mm_control_signal", mm_cmd)
            now = time.monotonic()

            if now - self._last_mm_command >= self._mm_control_period:
                self._mm_pulse_until = now + self._mm_pulse_length
                self._last_mm_command = now

            if now < self._mm_pulse_until:
                self.mm.set_current(mm_cmd)
            else:
                self.mm.set_current(0)
            
    
    def _store(self) -> None:
        if self.rw is not None:
            data = self.rw.read()
            self.datastore.write("system.rw_vesc_connected", 1.0 if self.rw.connected else 0.0)
            if data is not None:
                self._write("rw", data)
            elif self.rw.connected == False:
                raise ConnectionError("RW VESC disconnected during pointing task")
        if self.mm is not None:
            data = self.mm.read()
            self.datastore.write("system.mm_vesc_connected", 1.0 if self.mm.connected else 0.0)
            if data is not None:
                self._write("mm", data)
            elif self.mm.connected == False:
                raise ConnectionError("MM VESC disconnected during pointing task")

    def _write(self, prefix: str, data) -> None:
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                    'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                    'tachometer_abs'):
                self.datastore.write(f"{prefix}.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write(f"{prefix}.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))
    
    def _read(self):
        quat = [
            float(self.datastore.read("mavlink.quaternion.x", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.y", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.z", default=0.0)),
            float(self.datastore.read("mavlink.quaternion.w", default=1.0)),
        ]
        pos = [
            float(self.datastore.read("mavlink.gps.lat", default=0.0)),
            float(self.datastore.read("mavlink.gps.lon", default=0.0)),
            float(self.datastore.read("mavlink.gps.alt", default=0.0)),
        ]
        use_hardcoded = float(self.datastore.read("settings.gs_use_hardcoded", default=1.0)) >= 1.0
        if use_hardcoded:
            gs_pos = [
                float(self.datastore.read("settings.gs_lat", default=self._default_gs_pos[0])),
                float(self.datastore.read("settings.gs_lon", default=self._default_gs_pos[1])),
                float(self.datastore.read("settings.gs_alt", default=self._default_gs_pos[2])),
            ]
        else:
            gs_lat = self.datastore.read("command.gs_lat", default=None)
            gs_lon = self.datastore.read("command.gs_lon", default=None)
            gs_alt = self.datastore.read("command.gs_alt", default=None)
            gs_pos = (
                [float(gs_lat), float(gs_lon), float(gs_alt)]
                if all(v is not None for v in (gs_lat, gs_lon, gs_alt)) else self._default_gs_pos
            )
        yaw_rate = float(self.datastore.read("mavlink.attitude.yawspeed", default=0.0))
        yaw = float(self.datastore.read("mavlink.attitude.yaw", default=0.0))
        return quat, pos, gs_pos, yaw_rate, yaw
    
    def _check(self):
        if int(self.datastore.read("event.pointing_active", default=0.0)) != 1:
            logger.info("PointingTask: run duration elapsed — stopping motor")
            self._stop_event.set()
            return
        
    def _set_state(self, state: PointingState) -> None:
        if state != self._state:
            logger.info("PointingTask: state %s -> %s", self._state.value, state.value)
            self._state = state
            self._state_started = time.monotonic()
            self._stable_since = None
