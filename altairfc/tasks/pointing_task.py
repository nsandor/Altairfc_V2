from __future__ import annotations

import logging
import time
from collections import deque
from enum import Enum
import numpy as np

from config.settings import ControllerConfig, GroundStationConfig, PointingConfig, SerialPortConfig
from controls.controller import GainScheduledController
from controls.error_computation import compute_error
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.rw_driver import RWDriver

logger = logging.getLogger(__name__)

class PointingState(Enum):
    IDLE = "idle" # Momentum management blocks bearing from free spinning
    SPINUP = "spinup" # RW spinup
    STABILIZE = "stabilize" # MM stabilizes and large initial error correction
    POINTING = "pointing" # PID active pointing 
    SATURATED = "saturated" # RW speed has exceeded saturation threshold

class PointingTask(BaseTask):
    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        rw_port: SerialPortConfig | None,
        rw_controller_config: ControllerConfig,
        ground_station: GroundStationConfig,
        pointing_config: PointingConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._spinup_rpm = pointing_config.spinup_rpm
        self._spinup_s = pointing_config.spinup_s
        self._stabilize_yaw_rate = pointing_config.stabilize_yaw_rate
        self._max_slew_rate = pointing_config.max_slew_rate
        self._stability_threshold = pointing_config.stability_threshold
        self._saturation_rpm = pointing_config.saturation_rpm
        self._saturation_s = pointing_config.saturation_s
        self._saturation_margin_rpm = 50.0
        self._switch_threshold = pointing_config.switch_threshold
        self._yaw_rate_deadband = pointing_config.yaw_rate_deadband
        self.period = period_s
        self.rw = RWDriver(rw_port.port) 
        self.rw_controller = GainScheduledController(rw_controller_config, period_s)
        self._default_gs_pos = [ground_station.latitude, ground_station.longitude, ground_station.altitude]
        

    def setup(self) -> None:
        self.passed = False
        self._state = PointingState.IDLE
        self._state_started = time.monotonic()
        self._saturated_since = None
        self._unstable_since = None
        self._rate_sum_window = deque(maxlen=max(1, int(5.0 / self.period)))
        self._last_rate_sum = None
        self.err = 0.0
        self._allow_switch = 1
        self._target_offset = 0.0
        self._count = 0
        self._hold_count = 0
        self.j = 0.0

        if self.rw is not None:
            if not self.rw.connect():
                self.datastore.write("system.rw_vesc_connected", 0.0)
                raise ConnectionError("RW ESC enabled but not connected")
            self.datastore.write("system.rw_vesc_connected", 1.0)

    def execute(self) -> None:
        self._store()

        active = int(self.datastore.read("event.pointing_active", default=0.0))
        if active == 1:
            if self.passed == False:
                self.passed = True
                if self._spinup_rpm != 0.0:
                    self._set_state(PointingState.SPINUP)
                else:
                    self._set_state(PointingState.STABILIZE)
        
        if self._state == PointingState.SPINUP:
            self._check()
            self.rw.set_rpm(self._spinup_rpm)
            if time.monotonic() - self._state_started >= self._spinup_s:
                self._set_state(PointingState.STABILIZE)

        if self._state == PointingState.STABILIZE:
            self._check()
            self.rw_controller.set_mode("stabilize")
            _, _, _, yaw_rate, _, rw_rpm = self._read()
            saturated = self._is_saturated(rw_rpm)
            trend = self._acceleration(yaw_rate)
            stability = self._is_stable(yaw_rate)
            if not stability or time.monotonic() - self._state_started < 10.0:
                if np.sign(trend) != np.sign(yaw_rate):
                    delta_rpm = self.rw_controller.output(yaw_rate)
                # elif saturated:
                #     delta_rpm = 0.0
                else:
                    delta_rpm = 0.0
            else:
                self._set_state(PointingState.POINTING)
                delta_rpm = 0.0
            
            self.rw.set_rpm(int(rw_rpm + delta_rpm))

        
        elif self._state == PointingState.POINTING:
            self._check()
            self._point()

        elif self._state == PointingState.SATURATED:
            self._check()
            self._desaturate()

    def teardown(self) -> None:
        self.rw.close()
    
    def _point(self) -> None:
        quat, pos, gs_pos, yaw_rate, yaw, rw_rpm = self._read()
        az_err, _ = compute_error(quat, pos, gs_coords=gs_pos)
        self.datastore.write("pointing.az_error", az_err)
        saturation = self._is_saturated(rw_rpm)
        if saturation:
            self._set_state(PointingState.SATURATED)
            return
        # elif abs(yaw) > 0.5:
        #     self.rw_controller.set_mode("slewing")
        #     err = (np.sign(yaw)*self._max_slew_rate) - yaw_rate
        #     delta_rpm = self.rw_controller.output(err)
        self.rw_controller.set_mode("pointing")
        delta_rpm = self.rw_controller.output(yaw, yaw_rate) - 0.1* rw_rpm

        if abs(rw_rpm) > 1500:
            fraction = (abs(rw_rpm) - 1500) / (self._saturation_rpm - 1500)
            fraction = np.clip(fraction, 0.0, 1.0)

            unload = -np.sign(rw_rpm) * fraction * 300
            delta_rpm += unload

        self.rw.set_rpm(int(rw_rpm + delta_rpm))

            
    def _is_saturated(self, rw_rpm: float) -> bool:
        now = time.monotonic()
        saturated = abs(rw_rpm) >= self._saturation_rpm - self._saturation_margin_rpm
        self.datastore.write("pointing.rw_saturated", 1.0 if saturated else 0.0)

        if not saturated:
            self._saturated_since = None
            self.datastore.write("pointing.saturation_elapsed_s", 0.0)
            return False

        if self._saturated_since is None:
            self._saturated_since = now

        elapsed = now - self._saturated_since
        if elapsed >= self._saturation_s:
            return True
        return False

    def _is_stable(self, yaw_rate: float) -> bool:
        now = time.monotonic()
        unstable = abs(yaw_rate) > self._stabilize_yaw_rate

        if not unstable:
            self._unstable_since = None
            return True

        if self._unstable_since is None:
            self._unstable_since = now

        return (now - self._unstable_since) < self._stability_threshold
    
    # def _desaturate(self) -> None:
    #     self.rw_controller.set_mode("saturated")
    #     _, _, _, yaw_rate, yaw, rw_rpm = self._read()
    #     saturation = self._is_saturated(rw_rpm)

    #     target = yaw + self._target_offset

    #     moving_away = (
    #         abs(yaw) > self._switch_threshold
    #         and np.sign(yaw_rate) != np.sign(yaw)
    #         and abs(yaw_rate) > self._yaw_rate_deadband
    #     )

    #     if saturation and moving_away and self._allow_switch == 1:
    #         self._target_offset += -np.sign(rw_rpm) * 2*np.pi
    #         self._allow_switch = 0
    #         self._count = 0
    #         self._hold_count = 0

    #     target = yaw + self._target_offset

    #     moving_toward = (
    #         np.sign(yaw_rate) == np.sign(yaw)
    #         and abs(yaw_rate) > self._yaw_rate_deadband
    #     )

    #     if np.deg2rad(30) < abs(yaw) < np.deg2rad(170) and moving_toward:
    #         self._allow_switch = 0
    #         self._count += 1

    #     if self._allow_switch == 0:
    #         self._count += 1

    #     if not saturation:
    #         self._hold_count += 1
    #         if self._hold_count >= 80:
    #             self._allow_switch = 1
    #             self._target_offset = 0.0
    #             self._set_state(PointingState.STABILIZE)
    #     else:
    #         self._hold_count = 0


    #     if self._count >= 3600:
    #         self._allow_switch = 1
    #         self._count = 0
        
    #     self.err = target
    #     delta_rpm = self.rw_controller.output(self.err, yaw_rate)

    #     self.rw.set_rpm(int(rw_rpm + delta_rpm))
    def _desaturate(self) -> None:
        self.rw.set_rpm(0)
        if time.monotonic() - self._state_started >= 5.0:
            self._set_state(PointingState.STABILIZE)
    
    def _acceleration(self, yaw_rate: float) -> float:
        self._rate_sum_window.append(yaw_rate)

        if len(self._rate_sum_window) < self._rate_sum_window.maxlen:
            return 0.0

        rate_sum = float(sum(self._rate_sum_window))

        if self._last_rate_sum is None:
            self._last_rate_sum = rate_sum
            return 0.0

        trend = rate_sum - self._last_rate_sum
        self._last_rate_sum = rate_sum
        return trend

    
    def _store(self) -> None:
        if self.rw is not None:
            data = self.rw.read()
            self.datastore.write("system.rw_vesc_connected", 1.0 if self.rw.connected else 0.0)
            if data is not None:
                self._write("rw", data)
            elif self.rw.connected == False:
                raise ConnectionError("RW VESC disconnected during pointing task")

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
        rw_rpm = float(self.datastore.read("rw.rpm", default=0.0))/7
        return quat, pos, gs_pos, yaw_rate, yaw, rw_rpm
    
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
            self._unstable_since = None
            self._saturated_since = None
            self._rate_sum_window.clear()
            self._last_rate_sum = None
