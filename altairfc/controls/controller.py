import numpy as np


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
