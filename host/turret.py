"""Turret session: link, aim and fire policy wired together.

This module is the only place where the host sends a fire command, and it
only does so after the fire policy has allowed it. The firmware then applies
its own checks (arming switch, link, trigger busy, cooldown) before moving
the trigger servo.
"""

from __future__ import annotations

import logging
import math

from control import AimController
from fire_policy import FireDecision, FireInputs, FirePolicy
from serial_link import (
    FireReply,
    FirmwareEvent,
    FirmwareStatus,
    Message,
    SerialLink,
    SimulatedLink,
    TurretLink,
)
from settings import Settings

logger = logging.getLogger(__name__)

# Angle changes smaller than this are not worth a new A command.
_RESEND_THRESHOLD_DEG = 0.01


def open_link(settings: Settings, simulate: bool) -> TurretLink:
    """Opens the real serial link, or a simulated firmware for demos."""
    if simulate:
        aim = settings.aim
        return SimulatedLink(
            pan_limits=(aim.pan.min_deg, aim.pan.max_deg),
            tilt_limits=(aim.tilt.min_deg, aim.tilt.max_deg),
            home=(aim.pan.home_deg, aim.tilt.home_deg),
        )
    return SerialLink.open(settings.serial.port, settings.serial.baud)


class Turret:
    """Keeps the firmware link alive and turns policy decisions into commands."""

    def __init__(
        self,
        link: TurretLink,
        aim: AimController,
        policy: FirePolicy,
        status_period_s: float,
        status_stale_s: float,
    ) -> None:
        self.link = link
        self.aim = aim
        self.policy = policy
        self._status_period_s = status_period_s
        self._status_stale_s = status_stale_s
        self._last_status_request_s = -math.inf
        self._sent_angles: tuple[float, float] | None = None
        self._was_link_ok = False
        self._was_hardware_armed = False
        self.last_fire_reply: FireReply | None = None

    @classmethod
    def from_settings(cls, settings: Settings, link: TurretLink) -> Turret:
        policy_settings = settings.fire_policy
        return cls(
            link=link,
            aim=AimController.from_settings(settings.aim, settings.pid),
            policy=FirePolicy(
                policy_settings.error_threshold_px,
                policy_settings.lock_frames,
                policy_settings.cooldown_s,
            ),
            status_period_s=settings.serial.status_period_s,
            status_stale_s=settings.serial.status_stale_s,
        )

    # -- link health ---------------------------------------------------------

    def link_ok(self, now_s: float) -> bool:
        """True while a recent status says the firmware link is up."""
        status = self.link.status
        return (
            status is not None
            and status.link_ok
            and self.link.status_age_s(now_s) <= self._status_stale_s
        )

    def hardware_armed(self, now_s: float) -> bool:
        """Arming switch state; unknown (stale) counts as disarmed."""
        status = self.link.status
        return self.link_ok(now_s) and status is not None and status.armed

    def service(self, now_s: float) -> list[Message]:
        """Call once per frame: reads replies, polls status, syncs arm state."""
        messages = self.link.poll(now_s)
        for message in messages:
            self._log_message(message)
        if now_s - self._last_status_request_s >= self._status_period_s:
            self.link.request_status()  # also the firmware watchdog keepalive
            self._last_status_request_s = now_s
        self._sync_link_state(now_s)
        return messages

    def _sync_link_state(self, now_s: float) -> None:
        link_ok = self.link_ok(now_s)
        hardware_armed = self.hardware_armed(now_s)

        lost = (self._was_link_ok and not link_ok) or (
            self._was_hardware_armed and not hardware_armed
        )
        if lost and self.policy.software_armed:
            self.policy.disarm()
            logger.warning("software arm cleared: hardware disarmed or link lost")

        if link_ok and not self._was_link_ok and self.link.status is not None:
            self._adopt_firmware_angles(self.link.status)

        self._was_link_ok = link_ok
        self._was_hardware_armed = hardware_armed

    def _adopt_firmware_angles(self, status: FirmwareStatus) -> None:
        """Starts from where the firmware is, so a reconnection never jumps."""
        self.aim.set_angles(status.pan_deg, status.tilt_deg)
        self.aim.reset_tracking()
        self._sent_angles = self.aim.angles

    def _log_message(self, message: Message) -> None:
        if isinstance(message, FireReply):
            self.last_fire_reply = message
            if message.accepted:
                logger.info("firmware accepted the shot")
            else:
                logger.info("firmware refused the shot: %s", message.reason)
        elif isinstance(message, FirmwareEvent):
            if message.name == "ERR":
                logger.warning("firmware rejected a command: %s", message.value)
            elif message.name == "READY":
                logger.info("firmware ready: %s", message.value)
            else:
                logger.info("firmware event %s %s", message.name, message.value)

    # -- aiming --------------------------------------------------------------

    def push_aim(self) -> None:
        """Sends the commanded angles when they changed since the last send."""
        angles = self.aim.angles
        if self._sent_angles is not None and all(
            abs(new - old) < _RESEND_THRESHOLD_DEG
            for new, old in zip(angles, self._sent_angles, strict=True)
        ):
            return
        self.link.set_angles(*angles)
        self._sent_angles = angles

    # -- firing --------------------------------------------------------------

    def fire_inputs(
        self, error_px: tuple[float, float] | None, veto: bool, now_s: float
    ) -> FireInputs:
        return FireInputs(
            target_present=error_px is not None,
            error_px=error_px,
            hardware_armed=self.hardware_armed(now_s),
            link_ok=self.link_ok(now_s),
            protected_in_view=veto,
        )

    def try_fire_auto(self, inputs: FireInputs, now_s: float) -> FireDecision:
        """AUTO mode, once per frame: updates the lock, fires if allowed."""
        self.policy.observe(inputs)
        return self._fire_if_allowed(self.policy.evaluate_auto(inputs, now_s), now_s)

    def try_fire_manual(self, inputs: FireInputs, now_s: float) -> FireDecision:
        """MANUAL mode, on the fire key: fires if allowed."""
        return self._fire_if_allowed(self.policy.evaluate_manual(inputs, now_s), now_s)

    def _fire_if_allowed(self, decision: FireDecision, now_s: float) -> FireDecision:
        if decision.allowed:
            self.link.fire()
            self.policy.record_shot(now_s)
            logger.info("shot requested")
        return decision

    # -- shutdown ------------------------------------------------------------

    def shutdown(self) -> None:
        """Disarms in software and sends the turret home."""
        self.policy.disarm()
        self.aim.home()
        self.push_aim()
