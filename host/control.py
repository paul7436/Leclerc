"""Aiming control: PID loops that drive the target onto the aim point.

The camera is mounted on the barrel, so it moves with the turret. A target
seen off the aim point therefore calls for a relative correction:

    pixel error -> degrees (calibrated deg/pixel and direction) -> PID
                -> added to the current pan and tilt command.
"""

from __future__ import annotations

import math
from bisect import bisect_left

from settings import AimSettings, AxisSettings, BallisticsSettings, PidGains, PidSettings


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


# ---------------------------------------------------------------------------
# Ballistics: dart drop compensation
# ---------------------------------------------------------------------------


class HoldoverTable:
    """Extra tilt, in degrees, needed at a given distance to offset dart drop.

    Linear interpolation between calibrated points, held flat beyond the
    first and last points. An empty table means no holdover.
    """

    def __init__(self, rows: list[tuple[float, float]]) -> None:
        ordered = sorted(rows)
        self._distances = [distance for distance, _ in ordered]
        self._holdovers = [holdover for _, holdover in ordered]

    def __len__(self) -> int:
        return len(self._distances)

    def holdover_deg(self, distance_m: float) -> float:
        if not self._distances:
            return 0.0
        if distance_m <= self._distances[0]:
            return self._holdovers[0]
        if distance_m >= self._distances[-1]:
            return self._holdovers[-1]
        upper = bisect_left(self._distances, distance_m)
        d0, d1 = self._distances[upper - 1], self._distances[upper]
        h0, h1 = self._holdovers[upper - 1], self._holdovers[upper]
        return h0 + (h1 - h0) * (distance_m - d0) / (d1 - d0)


def focal_length_px(deg_per_px: float) -> float:
    """Focal length in pixels implied by the calibrated angle of one pixel."""
    return 1.0 / math.tan(math.radians(deg_per_px))


def estimate_distance_m(box_height_px: float, target_height_m: float, focal_px: float) -> float:
    """Pinhole camera estimate of the distance to a target of known height."""
    if box_height_px <= 0:
        raise ValueError("box height must be positive")
    return focal_px * target_height_m / box_height_px


class Ballistics:
    """Moves the aim point to compensate dart drop at the estimated distance.

    The camera tilts with the barrel. To make the barrel point `holdover`
    degrees above the target, the target must sit that many degrees below
    the crosshair in the image, whatever the tilt servo direction.
    """

    def __init__(self, table: HoldoverTable, target_height_m: float, tilt_deg_per_px: float):
        self._table = table
        self._target_height_m = target_height_m
        self._tilt_deg_per_px = tilt_deg_per_px
        self._focal_px = focal_length_px(tilt_deg_per_px)

    @classmethod
    def from_settings(cls, ballistics: BallisticsSettings, aim: AimSettings) -> Ballistics:
        return cls(
            HoldoverTable(ballistics.holdover_table),
            ballistics.target_height_m,
            aim.tilt.deg_per_px,
        )

    @property
    def enabled(self) -> bool:
        return self._target_height_m > 0 and len(self._table) > 0

    def estimate_distance_m(self, box_height_px: float) -> float | None:
        if self._target_height_m <= 0 or box_height_px <= 0:
            return None
        return estimate_distance_m(box_height_px, self._target_height_m, self._focal_px)

    def aim_point(
        self, crosshair_px: tuple[float, float], box_height_px: float
    ) -> tuple[float, float]:
        """Where the target must sit in the image for the dart to hit it."""
        distance_m = self.estimate_distance_m(box_height_px) if self.enabled else None
        if distance_m is None:
            return crosshair_px
        holdover_px = self._table.holdover_deg(distance_m) / self._tilt_deg_per_px
        return crosshair_px[0], crosshair_px[1] + holdover_px
