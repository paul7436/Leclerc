"""Turret host: MANUAL (keyboard aim, manual fire) and AUTO (track and fire) modes.

    python main.py                   real turret on the configured serial port
    python main.py --simulate        simulated firmware, no ESP32 needed
    python main.py --source clip.mp4 recorded video instead of the webcam

Always starts in MANUAL mode with the software arm flag cleared.

    m          toggle MANUAL / AUTO (clears the software arm flag)
    x          toggle the software arm flag
    w a s d    nudge the aim in MANUAL mode, Shift for coarse steps
    space      fire one dart in MANUAL mode (checked by the fire policy)
    h          return to the home position
    q / Esc    quit (disarms and returns home)
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from enum import Enum
from typing import Any

import cv2
import serial

from control import AimController, Ballistics
from detection import Camera, CameraError, Detection, YoloDetector
from fire_policy import FireDecision, FireInputs
from overlay import HudState, draw_hud
from settings import DEFAULT_CONFIG_PATH, ConfigError, Settings, load_settings
from turret import Turret, open_link

logger = logging.getLogger("turret")

WINDOW_TITLE = "Leclerc turret"
KEY_ESCAPE = 27
KEY_HELP = "m mode  x arm  wasd aim (shift coarse)  space fire  h home  q quit"
MAX_DT_S = 0.2  # a stalled frame must not turn into a huge PID step

# Manual aim keys as (right, down) image directions.
NUDGE_KEYS = {"a": (-1, 0), "d": (1, 0), "w": (0, -1), "s": (0, 1)}


class Mode(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"


class TurretApp:
    """Processes one camera frame at a time: detect, aim, decide, report."""

    def __init__(
        self, settings: Settings, turret: Turret, detector: Any, ballistics: Ballistics
    ) -> None:
        self.settings = settings
        self.turret = turret
        self.detector = detector
        self.ballistics = ballistics
        self.mode = Mode.MANUAL
        self._fire_requested = False
        self._previous_frame_s: float | None = None
        self._fps = 0.0

    # -- per frame -------------------------------------------------------------

    def step(self, frame: Any, now_s: float) -> HudState:
        dt_s = self._frame_interval(now_s)
        self.turret.service(now_s)

        aim = self.turret.aim
        target, detections = self.detector.best_target(frame, aim.crosshair_px)
        aim_point = aim.crosshair_px
        error_px = None
        if target is not None:
            aim_point = self.ballistics.aim_point(aim.crosshair_px, target.height)
            error_px = AimController.error_px(target.center, aim_point)
        inputs = self.turret.fire_inputs(error_px, detections.veto, now_s)

        if self.mode is Mode.AUTO:
            decision = self._auto_step(target, aim_point, inputs, dt_s, now_s)
        else:
            decision = self._manual_step(inputs, now_s)
        self.turret.push_aim()

        return HudState(
            mode=self.mode.value,
            software_armed=self.turret.policy.software_armed,
            hardware_armed=self.turret.hardware_armed(now_s),
            link_ok=self.turret.link_ok(now_s),
            angles=aim.angles,
            crosshair_px=aim.crosshair_px,
            aim_point_px=aim_point,
            fire_allowed=decision.allowed,
            blockers=decision.blockers,
            target=target,
            detections=detections,
            distance_m=self.ballistics.estimate_distance_m(target.height) if target else None,
            fps=self._fps,
            footer=KEY_HELP,
        )

    def _auto_step(
        self,
        target: Detection | None,
        aim_point: tuple[float, float],
        inputs: FireInputs,
        dt_s: float,
        now_s: float,
    ) -> FireDecision:
        if target is None:
            self.turret.aim.reset_tracking()
        else:
            self.turret.aim.track(target.center, aim_point, dt_s)
        return self.turret.try_fire_auto(inputs, now_s)

    def _manual_step(self, inputs: FireInputs, now_s: float) -> FireDecision:
        if not self._fire_requested:
            return self.turret.policy.evaluate_manual(inputs, now_s)  # display only
        self._fire_requested = False
        decision = self.turret.try_fire_manual(inputs, now_s)
        if not decision.allowed:
            logger.info("fire refused: %s", ", ".join(decision.blockers))
        return decision

    def _frame_interval(self, now_s: float) -> float:
        previous, self._previous_frame_s = self._previous_frame_s, now_s
        if previous is None:
            return 0.0
        dt_s = now_s - previous
        if dt_s > 0:
            self._fps = 0.9 * self._fps + 0.1 / dt_s if self._fps else 1.0 / dt_s
        return min(max(dt_s, 0.0), MAX_DT_S)

    # -- keyboard --------------------------------------------------------------

    def handle_key(self, key: int) -> bool:
        """Applies one key press (ASCII code). Returns False to quit."""
        if key in (ord("q"), KEY_ESCAPE):
            return False
        if key == ord("m"):
            self._toggle_mode()
        elif key == ord("x"):
            self._toggle_software_arm()
        elif key == ord("h"):
            self.turret.aim.home()
        elif key == ord(" ") and self.mode is Mode.MANUAL:
            self._fire_requested = True
        elif self.mode is Mode.MANUAL and 0 <= key < 128:
            self._nudge(chr(key))
        return True

    def _toggle_mode(self) -> None:
        self.mode = Mode.AUTO if self.mode is Mode.MANUAL else Mode.MANUAL
        self.turret.policy.disarm()
        self.turret.aim.reset_tracking()
        self._fire_requested = False
        logger.info("mode %s, software DISARMED", self.mode.value)

    def _toggle_software_arm(self) -> None:
        policy = self.turret.policy
        if policy.software_armed:
            policy.disarm()
            logger.info("software DISARMED")
        else:
            policy.arm()
            logger.warning("software ARMED (the hardware switch must also be on to fire)")

    def _nudge(self, char: str) -> None:
        direction = NUDGE_KEYS.get(char.lower())
        if direction is None:
            return
        aim_settings = self.settings.aim
        coarse = char.isupper()
        step = aim_settings.manual_coarse_step_deg if coarse else aim_settings.manual_step_deg
        self.turret.aim.nudge_view(*direction, step_deg=step)


# ---------------------------------------------------------------------------
# Program
# ---------------------------------------------------------------------------


def run(app: TurretApp, camera: Camera) -> None:
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    while True:
        frame = camera.read()
        hud = app.step(frame, time.monotonic())
        draw_hud(frame, hud)
        cv2.imshow(WINDOW_TITLE, frame)
        key = cv2.waitKey(1)
        if key != -1 and not app.handle_key(key & 0xFF):
            return
        if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            return  # window closed


def shutdown_safely(turret: Turret) -> None:
    try:
        turret.shutdown()
    except serial.SerialException as exc:
        logger.warning("could not send the home command (%s); the firmware holds still", exc)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision-guided foam-dart turret host.")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="configuration file")
    parser.add_argument(
        "--simulate", action="store_true", help="simulate the firmware instead of the ESP32"
    )
    parser.add_argument("--source", help="camera index or video file, overrides the config")
    parser.add_argument("--verbose", action="store_true", help="debug logging")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        settings = load_settings(args.config)
        detector = YoloDetector(settings.detection, settings.safety)
    except (ConfigError, ValueError) as exc:
        logger.error("configuration error: %s", exc)
        return 2
    if args.simulate:
        logger.warning("SIMULATION: the firmware is simulated, no hardware is driven")

    try:
        with (
            open_link(settings, args.simulate) as link,
            Camera(settings.camera, args.source) as camera,
        ):
            turret = Turret.from_settings(settings, link)
            ballistics = Ballistics.from_settings(settings.ballistics, settings.aim)
            app = TurretApp(settings, turret, detector, ballistics)
            logger.info("starting in MANUAL mode, software DISARMED")
            try:
                run(app, camera)
            finally:
                shutdown_safely(turret)
    except (CameraError, serial.SerialException) as exc:
        logger.error("%s", exc)
        return 1
    except KeyboardInterrupt:
        logger.info("interrupted")
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
