"""Aiming control: PID loops that drive the target onto the aim point.

The camera is mounted on the barrel, so it moves with the turret. A target
seen off the aim point therefore calls for a relative correction:

    pixel error -> degrees (calibrated deg/pixel and direction) -> PID
                -> added to the current pan and tilt command.
"""

from __future__ import annotations

import math

from settings import AimSettings, AxisSettings, PidGains, PidSettings


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


class AimController:
    """Owns the pan and tilt command angles and the two tracking PIDs."""

    def __init__(
        self,
        crosshair_px: tuple[float, float],
        pan: AxisSettings,
        tilt: AxisSettings,
        pan_pid: PID,
        tilt_pid: PID,
        deadband_px: float = 0.0,
    ) -> None:
        self.crosshair_px = crosshair_px
        self._pan_axis = pan
        self._tilt_axis = tilt
        self._pan_pid = pan_pid
        self._tilt_pid = tilt_pid
        self._deadband_px = deadband_px
        self._pan_deg = pan.home_deg
        self._tilt_deg = tilt.home_deg

    @classmethod
    def from_settings(cls, aim: AimSettings, pid: PidSettings) -> AimController:
        return cls(
            crosshair_px=aim.crosshair_px,
            pan=aim.pan,
            tilt=aim.tilt,
            pan_pid=PID.from_gains(pid.pan),
            tilt_pid=PID.from_gains(pid.tilt),
            deadband_px=aim.deadband_px,
        )

    @property
    def angles(self) -> tuple[float, float]:
        """Current (pan, tilt) command in degrees."""
        return self._pan_deg, self._tilt_deg

    def set_angles(self, pan_deg: float, tilt_deg: float) -> None:
        """Sets the command, clamped to the configured limits."""
        self._pan_deg = clamp(pan_deg, self._pan_axis.min_deg, self._pan_axis.max_deg)
        self._tilt_deg = clamp(tilt_deg, self._tilt_axis.min_deg, self._tilt_axis.max_deg)

    def home(self) -> None:
        self.set_angles(self._pan_axis.home_deg, self._tilt_axis.home_deg)
        self.reset_tracking()

    def nudge_view(self, right: float, down: float, step_deg: float) -> None:
        """Moves the aim in image directions: right=+1 aims right, down=-1 aims up.

        The calibrated directions make the keys match what the camera shows,
        whatever way the servos are mounted.
        """
        pan_delta = self._pan_axis.direction * right * step_deg
        tilt_delta = self._tilt_axis.direction * down * step_deg
        self.set_angles(self._pan_deg + pan_delta, self._tilt_deg + tilt_delta)

    def reset_tracking(self) -> None:
        """Forgets PID history, for example when the target is lost."""
        self._pan_pid.reset()
        self._tilt_pid.reset()

    @staticmethod
    def error_px(
        target_px: tuple[float, float], aim_point_px: tuple[float, float]
    ) -> tuple[float, float]:
        """Target position minus aim point, in pixels (x right, y down)."""
        return target_px[0] - aim_point_px[0], target_px[1] - aim_point_px[1]

    def track(
        self,
        target_px: tuple[float, float],
        aim_point_px: tuple[float, float],
        dt_s: float,
    ) -> tuple[float, float]:
        """Runs one control step toward the target; returns the new command."""
        error_x, error_y = self.error_px(target_px, aim_point_px)
        pan_step = self._correction(error_x, self._pan_axis, self._pan_pid, dt_s)
        tilt_step = self._correction(error_y, self._tilt_axis, self._tilt_pid, dt_s)
        self.set_angles(self._pan_deg + pan_step, self._tilt_deg + tilt_step)
        return self.angles

    def _correction(self, error_px: float, axis: AxisSettings, pid: PID, dt_s: float) -> float:
        if abs(error_px) <= self._deadband_px:
            error_px = 0.0
        error_deg = error_px * axis.deg_per_px
        return axis.direction * pid.update(error_deg, dt_s)
