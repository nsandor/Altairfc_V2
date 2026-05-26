from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from config.settings import ControllerConfig

logger = logging.getLogger(__name__)

class Controller:
    def __init__(self, config, period):
        self.Kp = config.Kp
        self.Kd = config.Kd
        self.Ki = config.Ki
        self.max_value = config.max
        self.min_value = config.min
        self.dt = period
        self.e_prev = 0.0
        self.e_int = 0.0

    def output(self, error: float, error_derivative: float | None = None):
        P = self.Kp * error
        D = self.Kd * (error - self.e_prev)/self.dt if error_derivative is None else self.Kd * error_derivative

        self.e_int += error * self.dt
        I = self.Ki * self.e_int
        self.e_prev = error

        output = P + D + I
        output = np.clip(output, self.min_value, self.max_value)

        return output

    def reset_integrator(self) -> None:
        self.e_int = 0.0
        self.e_prev = 0.0

class GainScheduledController:
    def __init__(
        self,
        configs: dict[str, ControllerConfig],
        period: float,
        initial_mode: str = "stabilize",
    ):
        self.controllers = {
            mode: Controller(cfg, period)
            for mode, cfg in configs.items()
        }
        self.mode = initial_mode

    def set_mode(self, mode: str) -> None:
        if mode != self.mode:
            logger.info("GainScheduledController: switching %s -> %s", self.mode, mode)
            self.mode = mode
            self.controllers[mode].reset_integrator()

    def output(self, error: float, error_derivative: float | None = None):
        return self.controllers[self.mode].output(error, error_derivative)
