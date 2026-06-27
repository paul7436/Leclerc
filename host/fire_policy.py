"""Software fire policy: decides whether the host may request a shot.

This is one safety layer among several. A dart only leaves the barrel when
this policy allows the request AND the firmware accepts the F command, which
itself requires the physical arming switch, a live link and its own cooldown.

Every condition is ANDed. None of them can be switched off.
"""

from __future__ import annotations

from dataclasses import dataclass

# Classes that are never targets and whose presence anywhere in the frame
# vetoes firing. Deliberately a constant, not a configuration value.
PROTECTED_CLASSES: frozenset[str] = frozenset({"person", "cat", "dog", "bird"})

# Lowest accepted cooldown between two shot requests, in seconds.
MIN_COOLDOWN_S = 0.5


@dataclass(frozen=True)
class FireInputs:
    """What the policy needs to know about the current frame."""

    target_present: bool
    error_px: tuple[float, float] | None  # target minus aim point, None without target
    hardware_armed: bool  # arming switch, from a fresh firmware status
    link_ok: bool  # firmware status is fresh and reports its link up
    protected_in_view: bool  # a protected or inhibit class is visible


@dataclass(frozen=True)
class FireDecision:
    """Outcome of a policy check; `blockers` explains a refusal."""

    allowed: bool
    blockers: tuple[str, ...]


class FirePolicy:
    """Software arm flag, target lock and cooldown, combined with the inputs.

    The software arm flag starts cleared. AUTO mode additionally requires the
    target to stay within the error threshold for `lock_frames` frames in a
    row; MANUAL mode (the operator aims) skips only that target condition.
    """

    def __init__(self, error_threshold_px: float, lock_frames: int, cooldown_s: float) -> None:
        if error_threshold_px <= 0:
            raise ValueError("error_threshold_px must be positive")
        if lock_frames < 1:
            raise ValueError("lock_frames must be at least 1")
        if cooldown_s < MIN_COOLDOWN_S:
            raise ValueError(f"cooldown_s must be at least {MIN_COOLDOWN_S}")
        self._threshold_px = error_threshold_px
        self._lock_frames_required = lock_frames
        self._cooldown_s = cooldown_s
        self._software_armed = False
        self._locked_frames = 0
        self._last_shot_s: float | None = None

    # -- software arm flag -------------------------------------------------

    @property
    def software_armed(self) -> bool:
        return self._software_armed

    def arm(self) -> None:
        self._software_armed = True

    def disarm(self) -> None:
        self._software_armed = False
        self._locked_frames = 0

    # -- target lock -------------------------------------------------------

    @property
    def locked_frames(self) -> int:
        return self._locked_frames

    def is_on_target(self, error_px: tuple[float, float] | None) -> bool:
        if error_px is None:
            return False
        error_x, error_y = error_px
        return abs(error_x) < self._threshold_px and abs(error_y) < self._threshold_px

    def observe(self, inputs: FireInputs) -> None:
        """Updates the consecutive on-target counter. Call once per frame."""
        if inputs.target_present and self.is_on_target(inputs.error_px):
            self._locked_frames += 1
        else:
            self._locked_frames = 0

    # -- cooldown ----------------------------------------------------------

    def cooldown_remaining(self, now_s: float) -> float:
        if self._last_shot_s is None:
            return 0.0
        return max(0.0, self._cooldown_s - (now_s - self._last_shot_s))

    def record_shot(self, now_s: float) -> None:
        """Starts the cooldown and requires a fresh lock for the next shot."""
        self._last_shot_s = now_s
        self._locked_frames = 0

    # -- decisions ---------------------------------------------------------

    def evaluate_auto(self, inputs: FireInputs, now_s: float) -> FireDecision:
        blockers = self._common_blockers(inputs, now_s)
        if not inputs.target_present:
            blockers.append("no target")
        elif not self.is_on_target(inputs.error_px):
            blockers.append("not on target")
        elif self._locked_frames < self._lock_frames_required:
            blockers.append(f"locking {self._locked_frames}/{self._lock_frames_required}")
        return FireDecision(allowed=not blockers, blockers=tuple(blockers))

    def evaluate_manual(self, inputs: FireInputs, now_s: float) -> FireDecision:
        blockers = self._common_blockers(inputs, now_s)
        return FireDecision(allowed=not blockers, blockers=tuple(blockers))

    def _common_blockers(self, inputs: FireInputs, now_s: float) -> list[str]:
        blockers = []
        if not self._software_armed:
            blockers.append("software disarmed")
        if not inputs.link_ok:
            blockers.append("link down")
        if not inputs.hardware_armed:
            blockers.append("hardware disarmed")
        if inputs.protected_in_view:
            blockers.append("protected class in view")
        remaining = self.cooldown_remaining(now_s)
        if remaining > 0:
            blockers.append(f"cooldown {remaining:.1f}s")
        return blockers
