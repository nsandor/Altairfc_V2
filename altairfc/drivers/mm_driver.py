from __future__ import annotations

import logging
from drivers.vesc_interface import VESCObject

logger = logging.getLogger(__name__)

class MMDriver:
    def __init__(self, port: str) -> None:
        self.port_name = port
        self.motor: VESCObject | None = None
        self.connected = False

    def connect(self) -> bool:
        try:
            motor = VESCObject(self.port_name)
            data = motor.get_data(timeout=0.3)
            if data is None:
                motor.port.close()
                raise TimeoutError("no data received from MM VESC")
            
            self.motor = motor
            self.connected = True
            return True

        except Exception as e:
            self.motor = None
            self.connected = False
            logger.error("MMDriver: VESC not connected on %s: %s", self.port_name, e)
            return False
        
    def read(self):
        if self.motor is None:
            self.connected = False
            return None
        try:
            data = self.motor.get_data(timeout=0.3)
            if data is None:
                logger.warning("MM telemetry timeout")
                return None
            self.connected = True
            return data
        except Exception as e:
            logger.error("MM data read failed: %s", e)
            self.connected = False
            return None
        
    def set_current(self, current: int) -> None:
        if self.motor is not None:
            self.motor.set_current(current)

    def set_brake_current(self, current: int) -> None:
        if self.motor is not None:
            self.motor.set_brake_current(current)

    def stop(self) -> None:
        if self.motor is not None:
            self.motor.set_current(0)

    def close(self) -> None:
        self.stop()
        if self.motor is not None:
            self.motor.port.close()
        self.motor = None
        self.connected = False
