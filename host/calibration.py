"""Interactive calibration. Results are written back into config.yaml.

    python calibration.py crosshair   pick the pixel the barrel points at

Keep the arming switch OFF for this step: it never moves the trigger.
"""

from __future__ import annotations

import argparse
import logging
import sys
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np
import serial

from detection import Camera, CameraError
from overlay import GREEN, WHITE, YELLOW, draw_crosshair, draw_text
from settings import DEFAULT_CONFIG_PATH, ConfigError, Settings, load_settings, save_calibration

logger = logging.getLogger("calibration")

WINDOW_TITLE = "Leclerc calibration"
KEY_ENTER = (10, 13)
KEY_ESCAPE = 27

# w a s d as (right, down) directions; Shift selects the coarse step.
DIRECTION_KEYS = {"a": (-1, 0), "d": (1, 0), "w": (0, -1), "s": (0, 1)}

TextLines = list[tuple[str, tuple]]


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
# Program
# ---------------------------------------------------------------------------

Step = Callable[[Settings, Camera, Path], bool]
STEPS: dict[str, Step] = {
    "crosshair": calibrate_crosshair,
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
