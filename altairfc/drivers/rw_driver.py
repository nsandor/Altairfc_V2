from __future__ import annotations
import numpy as np
import logging
from drivers.vesc_interface import VESCObject

logger = logging.getLogger(__name__)

class RWDriver:
    def __init__(self, port: str) -> None:
        self.port_name = port
        self.motor: VESCObject | None = None
        self.connected = False
        self._last_rpm = 0
        self._desaturation_rate = 100

    def connect(self) -> bool:
        try:
            motor = VESCObject(self.port_name)
            data = motor.get_data(timeout=0.3)
            if data is None:
                motor.port.close()
                raise TimeoutError("no data received from RW VESC")
            
            self.motor = motor
            self.connected = True
            return True

        except Exception as e:
            self.motor = None
            self.connected = False
            logger.error("RWDriver: VESC not connected on %s: %s", self.port_name, e)
            return False
        
    def read(self):
        if self.motor is None:
            self.connected = False
            return None
        try:
            data = self.motor.get_data(timeout=0.3)
            if data is None:
                logger.warning("RW telemetry timeout")
                return None
            self.connected = True
            return data
        except Exception as e:
            logger.error("RW data read failed: %s", e)
            self.connected = False
            return None
        
    def set_rpm(self, rpm: int) -> None:
        limited_rpm = int(np.clip(rpm, 0, 4000))
        self._last_rpm = limited_rpm
        if self.motor is not None:
            self.motor.set_rpm(limited_rpm)

    def decelerate(self, rpm: int) -> None:
        delta = self._desaturation_rate
        if self._last_rpm < rpm:
            new_rpm = min(self._last_rpm + delta, rpm)
        elif self._last_rpm > rpm:
            new_rpm = max(self._last_rpm - delta, rpm)
        else:
            new_rpm = rpm
        new_rpm = int(new_rpm)
        self._last_rpm = new_rpm
        if self.motor is not None:
            self.motor.set_rpm(new_rpm)

    def set_current(self, current: int) -> None:
        if self.motor is not None:
            self.motor.set_current(current)

    def stop(self) -> None:
        if self.motor is not None:
            self.motor.set_rpm(0)

    def close(self) -> None:
        self.stop()
        if self.motor is not None:
            self.motor.port.close()
        self.motor = None
        self.connected = False

