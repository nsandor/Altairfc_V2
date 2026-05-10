from __future__ import annotations

import logging
import time
from config.settings import ControllerConfig, SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from drivers.vesc_interface import VESCObject
from controls.controller import Controller

logger = logging.getLogger(__name__)


class MMTask(BaseTask):

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        vesc_port: SerialPortConfig,
        controller_config: ControllerConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._vesc_port = vesc_port.port
        self.controller = Controller(controller_config, period_s)
        

    def setup(self) -> None:
        self.motor = None
        if not self._connect_vesc():
            return


        ## Polling Telemetry During Preflight
        while not self._stop_event.is_set():
            self._store()
            if int(self.datastore.read("event.pointing_active", default=0.0)) == 1:
                break
            self._stop_event.wait(timeout=0.5)

        if self._stop_event.is_set():
            return
        
        ## Stabilizing and Braking Payload
        logger.info("MMTask: LAUNCH + altitude reached — braking payload")
        while not self._stop_event.is_set():
            self._store()
            if self.motor is None:
                return
            yaw_rate = self.datastore.read("mavlink.attitude.yawspeed", default=None)
            if yaw_rate is None:
                logger.warning("mavlink.attitude.yawspeed is missing")
                continue
            motor_rpm = float(self.datastore.read("rw.rpm", default=0.0))
            self.motor.set_brake_current(1650)
            time.sleep(0.05)
            if abs(float(yaw_rate)) < 0.1 and motor_rpm >= 2150:
                break

    def execute(self) -> None:
        self._store()
        if self.motor is None:
            return

        pointing_active = self.datastore.read("event.pointing_active", default=None)

        if pointing_active is None:
            logger.warning("pointing_active is missing")
            return

        if int(pointing_active) != 1:
            logger.info("MMTask: run duration elapsed — stopping motor")
            self._stop_event.set()
            return
        
        motor_speed_err = float(self.datastore.read("rw.rpm", default=0.0)) - 2150
        control_signal = self.controller.output(motor_speed_err)
        self.motor.set_current(int(control_signal))

    def teardown(self) -> None:
        if self.motor is not None:
            self.motor.set_current(0)
        else:
            return

    def _connect_vesc(self) -> bool:
        try:
            motor = VESCObject(self._vesc_port)
            data = motor.get_data(timeout=0.3)
            if data is None:
                motor.port.close()
                raise TimeoutError("no data received from VESC")
            
            self.motor = motor
            self.datastore.write("system.mm_vesc_connected", 1.0)
            logger.info("MMTask: VESC connected on %s", self._vesc_port)
            return True

        except Exception as e:
            self.motor = None
            self.datastore.write("system.mm_vesc_connected", 0.0)
            logger.error("MMTask: VESC not connected on %s: %s", self._vesc_port, e)
            return False

    def _store(self) -> None:
        if self.motor is None:
            return
        try:
            data = self.motor.get_data(timeout=0.3)
            if data is None:
                logger.warning("MMTask: no data received from VESC")
                self.motor = None
                self.datastore.write("system.mm_vesc_connected", 0.0)
                return
            for f in ('rpm', 'duty_now', 'current_motor', 'current_in',
                      'v_in', 'temp_pcb', 'amp_hours', 'tachometer',
                      'tachometer_abs'):
                self.datastore.write(f"mm.{f}", getattr(data, f, 0.0))
            fault = getattr(data, 'mc_fault_code', b'\x00')
            self.datastore.write("mm.mc_fault_code", fault[0] if isinstance(fault, (bytes, bytearray)) else int(fault))

        except Exception as e:
            logger.error("MMTask: VESC disconnected during data read: %s", e)
            self.motor = None
            self.datastore.write("system.mm_vesc_connected", 0.0)
            return
        
    def _hold(self, fn, value, duration, dt = 0.05):
        start_time = time.time()
        while time.time() - start_time < duration:
            self._store()
            if self.motor is None:
                return
            fn(value)
            time.sleep(dt)
