from __future__ import annotations

import logging
import time
from config.settings import GroundStationConfig, SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
from controls.error_computation import compute_error
from controls.controller import Controller

logger = logging.getLogger(__name__)


class RWTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        vesc_port: SerialPortConfig,
        controller_config: list,
        ground_station: GroundStationConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self._default_gs_pos = [
            ground_station.latitude,
            ground_station.longitude,
            ground_station.altitude,
        ]
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        self.motor = None
        self._next_reconnect: float = 0.0
        self._connect_vesc()

        if self.motor is None:
            return

        ## Polling Telemetry During Preflight
        while not self._stop_event.is_set():
            self._store()
            if int(self.datastore.read("event.pointing_active", default=0.0)) == 1:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return


        ## Spinning Up and Stabilizing
        logger.info("RWTask: LAUNCH + altitude reached — spinning up reaction wheel")
        self._hold(self.motor.set_rpm, 1705, duration=5.0)
        while not self._stop_event.is_set():
            logger.info("RWTask: stabilizing payload")
            self._store()
            self.motor.set_rpm(1705)
            yaw_rate = self.datastore.read("mavlink.attitude.yawspeed", default=None)
            if yaw_rate is None:
                logger.warning("mavlink.attitude.yawspeed is missing")
                continue
            if abs(float(yaw_rate)) < 0.1:
                return
            time.sleep(0.05)

    def execute(self) -> None:
        if self.motor is None:
            if time.monotonic() >= self._next_reconnect:
                self._connect_vesc()
            return
        pointing_active = self.datastore.read("event.pointing_active", default=None)

        if pointing_active is None:
            logger.warning("pointing_active is missing")
            return

        if int(pointing_active) != 1:
            logger.info("RWTask: run duration elapsed — stopping motor")
            self._stop_event.set()
            return
        
        self.controller.Kp        = float(self.datastore.read("settings.rw_kp",      default=self.controller.Kp))
        self.controller.Kd        = float(self.datastore.read("settings.rw_kd",      default=self.controller.Kd))
        self.controller.max_value = float(self.datastore.read("settings.rw_max_rpm", default=self.controller.max_value))

        quat, pos, gs_pos, yaw_rate, yaw = self._read()
        az_err, _ = compute_error(quat, pos, gs_coords=gs_pos)
        self._store()
        control_signal = self.controller.output(yaw, yaw_rate) + 2150.0
        logger.info("yaw_error: %f, yaw_rate: %f, control signal: %f", yaw, yaw_rate, control_signal)
        self.motor.set_rpm(int(control_signal))


    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_rpm(0)

    def _connect_vesc(self, retry_interval_s: float = 5.0) -> None:
        """Block until the VESC connects, retrying every retry_interval_s."""
        while not self._stop_event.is_set():
            try:
                self.motor = VESCObject(self._vesc_port)
                self.datastore.write("system.vesc_connected", 1.0)
                logger.info("RWTask: VESC connected on %s", self._vesc_port)
                return
            except Exception as e:
                self.datastore.write("system.vesc_connected", 0.0)
                logger.warning("RWTask: waiting for VESC on %s (%s) — retrying in %.0fs",
                               self._vesc_port, e, retry_interval_s)
                self._stop_event.wait(timeout=retry_interval_s)

    def _store(self):
        if self.motor is None:
            return
        try:
            data = self.motor.get_data(timeout=0.3)
        except Exception as e:
            logger.error("RWTask: VESC disconnected during data read: %s", e)
            self.motor = None
            self.datastore.write("system.vesc_connected", 0.0)
            self._next_reconnect = time.monotonic() + 5.0
            return
        if data:
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                      'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                      'tachometer_abs'):
                self.datastore.write(f"rw.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write("rw.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))

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

    def _hold(self, fn, value, duration, dt = 0.05):
        start_time = time.time()
        while time.time() - start_time < duration:
            self._store()
            fn(value)
            time.sleep(dt)
