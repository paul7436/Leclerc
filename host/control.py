"""Aiming control: PID loops that drive the target onto the aim point."""

from __future__ import annotations

import math

from settings import PidGains


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class PID:
    """PID controller with a bounded integral term and a bounded output.

    The derivative acts on the error and is skipped on the first update after
    a reset, so acquiring a new target does not cause a derivative kick.
    """

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit: float = math.inf,
        output_limit: float = math.inf,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self._integral = 0.0
        self._previous_error: float | None = None

    @classmethod
    def from_gains(cls, gains: PidGains) -> PID:
        return cls(gains.kp, gains.ki, gains.kd, gains.integral_limit, gains.output_limit)

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None

    def update(self, error: float, dt_s: float) -> float:
        """Returns the control output for `error` after `dt_s` seconds."""
        integral_term = 0.0
        derivative = 0.0
        if dt_s > 0:
            integral_term = self._integrate(error, dt_s)
            if self._previous_error is not None:
                derivative = (error - self._previous_error) / dt_s
        elif self.ki > 0:
            integral_term = self.ki * self._integral
        self._previous_error = error

        output = self.kp * error + integral_term + self.kd * derivative
        return clamp(output, -self.output_limit, self.output_limit)

    def _integrate(self, error: float, dt_s: float) -> float:
        if self.ki <= 0:
            return 0.0
        bound = self.integral_limit / self.ki  # anti-windup on the state itself
        self._integral = clamp(self._integral + error * dt_s, -bound, bound)
        return self.ki * self._integral
