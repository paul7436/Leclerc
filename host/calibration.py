"""Interactive calibration. Results are written back into config.yaml.

    python calibration.py crosshair   pick the pixel the barrel points at
    python calibration.py degpx       measure servo degrees per image pixel
    python calibration.py holdover    build the distance -> tilt holdover table

Keep the arming switch OFF for crosshair and degpx: they never move the
trigger. The holdover step fires test shots, and every one of them goes
through the same fire policy and firmware checks as main.py.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np
import serial

from control import Ballistics
from detection import Camera, CameraError, YoloDetector
from main import shutdown_safely
from overlay import GREEN, RED, WHITE, YELLOW, HudState, draw_crosshair, draw_hud, draw_text
from settings import DEFAULT_CONFIG_PATH, ConfigError, Settings, load_settings, save_calibration
from turret import Turret, open_link

logger = logging.getLogger("calibration")

WINDOW_TITLE = "Leclerc calibration"
KEY_ENTER = (10, 13)
KEY_ESCAPE = 27

# w a s d as (right, down) directions; Shift selects the coarse step.
DIRECTION_KEYS = {"a": (-1, 0), "d": (1, 0), "w": (0, -1), "s": (0, 1)}

TextLines = list[tuple[str, tuple]]

# deg/pixel measurement
NUDGE_DEG = 4.0  # servo move used for each measurement
SETTLE_S = 0.8  # time for the servo, the mount and the camera to settle
MIN_SHIFT_PX = 8.0  # smaller image shifts are too imprecise
MIN_RESPONSE = 0.08  # weaker phase correlation peaks are not trusted


# holdover table
DISTANCE_STEP_M = 0.25
MIN_DISTANCE_M = 0.25
HOLDOVER_HELP = "[ ] distance  wasd aim  x arm  space fire  click impact  u undo  Enter save  Esc"


class CalibrationError(RuntimeError):
    """A measurement could not be trusted."""


class ClickTracker:
    """Remembers the last left click in the calibration window."""

    def __init__(self) -> None:
        self._point: tuple[float, float] | None = None

    def on_mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self._point = (float(x), float(y))

    def take(self) -> tuple[float, float] | None:
        """Returns the pending click once, or None."""
        point, self._point = self._point, None
        return point


def key_direction(key: int) -> tuple[tuple[int, int], bool] | None:
    """Maps w a s d to a (right, down) direction; True means Shift (coarse)."""
    if not 0 <= key < 128:
        return None
    char = chr(key)
    direction = DIRECTION_KEYS.get(char.lower())
    if direction is None:
        return None
    return direction, char.isupper()


def show(frame: np.ndarray, lines: TextLines, origin: tuple[int, int] = (10, 24)) -> int:
    """Draws text lines, displays the frame and returns the key pressed (-1 if none)."""
    x, y = origin
    for text, color in lines:
        draw_text(frame, text, (x, y), color)
        y += 24
    cv2.imshow(WINDOW_TITLE, frame)
    key = cv2.waitKey(1)
    return -1 if key == -1 else key & 0xFF


# ---------------------------------------------------------------------------
# Step 1: crosshair pixel
# ---------------------------------------------------------------------------


def move_point(
    point: tuple[float, float], direction: tuple[int, int], step_px: float, size: tuple[int, int]
) -> tuple[float, float]:
    """Moves a pixel position, keeping it inside a (width, height) image."""
    width, height = size
    x = min(max(point[0] + direction[0] * step_px, 0.0), width - 1.0)
    y = min(max(point[1] + direction[1] * step_px, 0.0), height - 1.0)
    return x, y


def calibrate_crosshair(settings: Settings, camera: Camera, config_path: Path) -> bool:
    """Lets the user click the pixel the barrel actually points at."""
    point = settings.aim.crosshair_px
    clicks = ClickTracker()
    cv2.setMouseCallback(WINDOW_TITLE, clicks.on_mouse)
    while True:
        frame = camera.read()
        point = clicks.take() or point
        draw_crosshair(frame, point, GREEN)
        key = show(
            frame,
            [
                ("CROSSHAIR: click the mark the barrel points at", WHITE),
                (f"crosshair {point[0]:.0f}, {point[1]:.0f}", GREEN),
                ("w a s d nudge 1 px (Shift 10 px)   Enter save   Esc cancel", YELLOW),
            ],
        )
        if key == KEY_ESCAPE:
            return False
        if key in KEY_ENTER:
            save_calibration(config_path, crosshair_px=point)
            logger.info("crosshair saved: %.0f, %.0f", *point)
            return True
        moved = key_direction(key)
        if moved is not None:
            direction, coarse = moved
            size = (frame.shape[1], frame.shape[0])
            point = move_point(point, direction, 10.0 if coarse else 1.0, size)


# ---------------------------------------------------------------------------
# Step 2: degrees per pixel
# ---------------------------------------------------------------------------


def measure_shift(before: np.ndarray, after: np.ndarray) -> tuple[float, float, float]:
    """Image translation from `before` to `after` by phase correlation.

    Returns (dx, dy, response): positive dx and dy mean the content moved
    right and down. The response (0 to 1) measures how clear the peak is.
    """
    first = _to_gray_float(before)
    second = _to_gray_float(after)
    window = cv2.createHanningWindow((first.shape[1], first.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(first, second, window)
    return float(dx), float(dy), float(response)


def _to_gray_float(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image.astype(np.float32)


def axis_from_samples(px_per_deg: list[float]) -> tuple[float, int]:
    """Turns image shifts per servo degree into (deg_per_px, direction).

    The camera moves with the barrel. If a positive servo move shifts the
    image by s pixels, a target e pixels off the aim point needs a move of
    -e / s degrees, hence direction = -sign(s).
    """
    if not px_per_deg:
        raise CalibrationError("no measurement")
    if len({math.copysign(1.0, sample) for sample in px_per_deg}) != 1:
        raise CalibrationError(f"inconsistent shift directions: {px_per_deg}")
    mean = sum(px_per_deg) / len(px_per_deg)
    return 1.0 / abs(mean), -int(math.copysign(1.0, mean))


def settle(turret: Turret, camera: Camera, angles: tuple[float, float]) -> np.ndarray:
    """Moves, keeps the firmware link alive while waiting, returns a fresh frame."""
    turret.aim.set_angles(*angles)
    turret.push_aim()
    deadline = time.monotonic() + SETTLE_S
    while True:
        frame = camera.read()  # reading continuously also drains stale frames
        turret.service(time.monotonic())
        if time.monotonic() >= deadline:
            return frame
        show(frame.copy(), [("measuring, keep the scene still", YELLOW)])


def measure_axis(turret: Turret, camera: Camera, axis: int, name: str) -> list[float]:
    """Nudges one axis both ways and returns image pixels moved per degree."""
    base = turret.aim.angles
    samples = []
    for sign in (1.0, -1.0):
        before = settle(turret, camera, base)
        moved = list(base)
        moved[axis] += sign * NUDGE_DEG
        after = settle(turret, camera, (moved[0], moved[1]))
        delta_deg = turret.aim.angles[axis] - base[axis]  # after clamping
        dx, dy, response = measure_shift(before, after)
        shift_px = dx if axis == 0 else dy
        logger.info(
            "%s %+.1f deg: shift %.1f px, response %.2f", name, delta_deg, shift_px, response
        )
        if abs(delta_deg) < 0.5 * NUDGE_DEG:
            raise CalibrationError(f"{name}: too close to a limit, move away from it first")
        if response < MIN_RESPONSE or abs(shift_px) < MIN_SHIFT_PX:
            raise CalibrationError(
                f"{name}: unreliable shift ({shift_px:.1f} px, response {response:.2f}); "
                "aim at a static, textured scene"
            )
        samples.append(shift_px / delta_deg)
    settle(turret, camera, base)
    return samples


def wait_for_start(turret: Turret, camera: Camera, title: str) -> bool:
    """Live view with manual aiming until Enter (True) or Esc (False)."""
    while True:
        frame = camera.read()
        turret.service(time.monotonic())
        turret.push_aim()
        link_ok = turret.link_ok(time.monotonic())
        link_line = ("link OK", GREEN) if link_ok else ("waiting for the firmware link", RED)
        key = show(
            frame,
            [
                (title, WHITE),
                link_line,
                ("w a s d aim (Shift coarse)   Enter start   Esc cancel", YELLOW),
            ],
        )
        if key == KEY_ESCAPE:
            return False
        if key in KEY_ENTER and link_ok:
            return True
        moved = key_direction(key)
        if moved is not None:
            # Directions are not calibrated yet: keys drive the servo angles directly.
            (right, down), coarse = moved
            aim = turret.aim
            step = 5.0 if coarse else 1.0
            aim.set_angles(aim.angles[0] + right * step, aim.angles[1] + down * step)


def calibrate_deg_per_px(settings: Settings, camera: Camera, config_path: Path) -> bool:
    """Measures deg/pixel and direction for pan and tilt."""
    with open_link(settings, simulate=False) as link:
        turret = Turret.from_settings(settings, link)
        try:
            title = "DEG/PX: aim at a static, textured scene (arming switch OFF)"
            if not wait_for_start(turret, camera, title):
                return False
            pan = axis_from_samples(measure_axis(turret, camera, 0, "pan"))
            tilt = axis_from_samples(measure_axis(turret, camera, 1, "tilt"))
        except CalibrationError as exc:
            logger.error("%s", exc)
            return False
        finally:
            shutdown_safely(turret)
    save_calibration(config_path, pan=pan, tilt=tilt)
    logger.info("pan %.5f deg/px direction %+d", *pan)
    logger.info("tilt %.5f deg/px direction %+d", *tilt)
    return True


# ---------------------------------------------------------------------------
# Step 3: holdover table
# ---------------------------------------------------------------------------


def holdover_from_impact(impact_y: float, crosshair_y: float, tilt_deg_per_px: float) -> float:
    """Extra elevation, in degrees, that moves the impact onto the crosshair.

    The shot was aimed with the crosshair on the target. A dart that landed
    below it (larger y) needs the barrel raised by that many degrees.
    """
    return (impact_y - crosshair_y) * tilt_deg_per_px


def upsert_point(
    table: list[tuple[float, float]], distance_m: float, holdover_deg: float
) -> list[tuple[float, float]]:
    """Adds or replaces the point at `distance_m`, keeping the table sorted."""
    return sorted(remove_point(table, distance_m) + [(distance_m, holdover_deg)])


def remove_point(table: list[tuple[float, float]], distance_m: float) -> list[tuple[float, float]]:
    return [(d, h) for d, h in table if not math.isclose(d, distance_m)]


class HoldoverSession:
    """State of the holdover step: test distance, table and pending fire."""

    def __init__(self, settings: Settings, turret: Turret) -> None:
        self.settings = settings
        self.turret = turret
        self.table = list(settings.ballistics.holdover_table)
        self.distance_m = 1.0
        self.fire_requested = False

    def record_impact(self, impact_px: tuple[float, float]) -> None:
        crosshair_y = self.turret.aim.crosshair_px[1]
        tilt_deg_per_px = self.settings.aim.tilt.deg_per_px
        holdover = holdover_from_impact(impact_px[1], crosshair_y, tilt_deg_per_px)
        self.table = upsert_point(self.table, self.distance_m, holdover)
        logger.info("%.2f m: holdover %+.2f deg", self.distance_m, holdover)

    def handle_key(self, key: int) -> None:
        if key == ord("["):
            self.distance_m = max(MIN_DISTANCE_M, self.distance_m - DISTANCE_STEP_M)
        elif key == ord("]"):
            self.distance_m += DISTANCE_STEP_M
        elif key == ord("u"):
            self.table = remove_point(self.table, self.distance_m)
        elif key == ord("x"):
            policy = self.turret.policy
            if policy.software_armed:
                policy.disarm()
            else:
                policy.arm()
        elif key == ord(" "):
            self.fire_requested = True
        elif (moved := key_direction(key)) is not None:
            direction, coarse = moved
            aim = self.settings.aim
            step = aim.manual_coarse_step_deg if coarse else aim.manual_step_deg
            self.turret.aim.nudge_view(*direction, step_deg=step)

    def table_lines(self) -> TextLines:
        lines = [(f"test distance {self.distance_m:.2f} m", WHITE)]
        for distance, holdover in self.table:
            color = GREEN if math.isclose(distance, self.distance_m) else YELLOW
            lines.append((f"{distance:5.2f} m  {holdover:+.2f} deg", color))
        return lines


def calibrate_holdover(settings: Settings, camera: Camera, config_path: Path) -> bool:
    """Fires test shots at known distances and records where they land."""
    detector = YoloDetector(settings.detection, settings.safety)  # protected-class veto
    detector.warm_up(settings.camera.width, settings.camera.height)
    ballistics = Ballistics.from_settings(settings.ballistics, settings.aim)
    clicks = ClickTracker()
    cv2.setMouseCallback(WINDOW_TITLE, clicks.on_mouse)

    with open_link(settings, simulate=False) as link:
        turret = Turret.from_settings(settings, link)
        session = HoldoverSession(settings, turret)
        try:
            while True:
                frame = camera.read()
                now_s = time.monotonic()
                turret.service(now_s)
                target, detections = detector.best_target(frame, turret.aim.crosshair_px)
                inputs = turret.fire_inputs(None, detections.veto, now_s)
                if session.fire_requested:
                    session.fire_requested = False
                    decision = turret.try_fire_manual(inputs, now_s)
                else:
                    decision = turret.policy.evaluate_manual(inputs, now_s)
                turret.push_aim()

                impact = clicks.take()
                if impact is not None:
                    session.record_impact(impact)

                crosshair = turret.aim.crosshair_px
                hud = HudState(
                    mode="HOLDOVER",
                    software_armed=turret.policy.software_armed,
                    hardware_armed=turret.hardware_armed(now_s),
                    link_ok=turret.link_ok(now_s),
                    angles=turret.aim.angles,
                    crosshair_px=crosshair,
                    aim_point_px=crosshair,
                    fire_allowed=decision.allowed,
                    blockers=decision.blockers,
                    target=target,
                    detections=detections,
                    distance_m=ballistics.estimate_distance_m(target.height) if target else None,
                    footer=HOLDOVER_HELP,
                )
                draw_hud(frame, hud)
                key = show(frame, session.table_lines(), origin=(frame.shape[1] - 260, 24))
                if key == KEY_ESCAPE:
                    return False
                if key in KEY_ENTER:
                    break
                session.handle_key(key)
        finally:
            shutdown_safely(turret)

    save_calibration(config_path, holdover_table=session.table)
    logger.info("holdover table saved with %d points", len(session.table))
    return True


# ---------------------------------------------------------------------------
# Program
# ---------------------------------------------------------------------------

Step = Callable[[Settings, Camera, Path], bool]
STEPS: dict[str, Step] = {
    "crosshair": calibrate_crosshair,
    "degpx": calibrate_deg_per_px,
    "holdover": calibrate_holdover,
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Turret calibration tool.")
    parser.add_argument("step", choices=sorted(STEPS), help="calibration step to run")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--source", help="camera index or video file, overrides the config")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        settings = load_settings(args.config)
    except ConfigError as exc:
        logger.error("configuration error: %s", exc)
        return 2

    try:
        with Camera(settings.camera, args.source) as camera:
            cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
            saved = STEPS[args.step](settings, camera, args.config)
    except (CameraError, ConfigError, ValueError, serial.SerialException) as exc:
        logger.error("%s", exc)
        return 1
    finally:
        cv2.destroyAllWindows()

    logger.info("%s: %s", args.step, "saved to config" if saved else "cancelled, nothing saved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
