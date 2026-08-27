"""Heads-up display drawn over the camera image (OpenCV, BGR colors)."""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from detection import Detection, FrameDetections

GREEN = (60, 200, 60)
YELLOW = (0, 210, 255)
RED = (40, 40, 230)
WHITE = (240, 240, 240)
GRAY = (150, 150, 150)
BLACK = (0, 0, 0)

FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class HudState:
    """Everything the HUD shows for one frame."""

    mode: str
    software_armed: bool
    hardware_armed: bool
    link_ok: bool
    angles: tuple[float, float]
    crosshair_px: tuple[float, float]
    aim_point_px: tuple[float, float]
    fire_allowed: bool = False
    blockers: tuple[str, ...] = ()
    target: Detection | None = None
    detections: FrameDetections = field(default_factory=FrameDetections)
    distance_m: float | None = None
    fps: float = 0.0
    footer: str = ""


def draw_hud(frame: np.ndarray, hud: HudState) -> None:
    """Draws the full HUD in place."""
    _draw_detections(frame, hud.detections, hud.target)
    draw_crosshair(frame, hud.crosshair_px, GREEN)
    if hud.aim_point_px != hud.crosshair_px:
        draw_point(frame, hud.aim_point_px, YELLOW)
    _draw_status_panel(frame, hud)
    if hud.detections.veto:
        _draw_banner(frame, "PROTECTED CLASS IN VIEW - FIRE INHIBITED", RED)
    if hud.footer:
        _draw_footer(frame, hud.footer)


def _draw_detections(
    frame: np.ndarray, detections: FrameDetections, target: Detection | None
) -> None:
    for detection in detections.targets:
        color = GREEN if detection is target else YELLOW
        _draw_box(frame, detection, color, f"{detection.class_name} {detection.confidence:.2f}")
    for detection in detections.vetoes:
        _draw_box(frame, detection, RED, f"NO FIRE: {detection.class_name}")


def _draw_box(frame: np.ndarray, detection: Detection, color: tuple, label: str) -> None:
    x1, y1, x2, y2 = (int(round(v)) for v in detection.box)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    draw_text(frame, label, (x1, max(16, y1 - 6)), color, scale=0.5)


def draw_crosshair(frame: np.ndarray, center: tuple[float, float], color: tuple) -> None:
    x, y = int(round(center[0])), int(round(center[1]))
    gap, arm = 5, 18
    cv2.line(frame, (x - arm, y), (x - gap, y), color, 2)
    cv2.line(frame, (x + gap, y), (x + arm, y), color, 2)
    cv2.line(frame, (x, y - arm), (x, y - gap), color, 2)
    cv2.line(frame, (x, y + gap), (x, y + arm), color, 2)


def draw_point(frame: np.ndarray, point: tuple[float, float], color: tuple) -> None:
    cv2.circle(frame, (int(round(point[0])), int(round(point[1]))), 6, color, 2)


def _draw_status_panel(frame: np.ndarray, hud: HudState) -> None:
    pan, tilt = hud.angles
    lines = [
        (f"MODE {hud.mode}", WHITE),
        _flag_line("SOFTWARE", hud.software_armed),
        _flag_line("HARDWARE", hud.hardware_armed),
        ("LINK OK" if hud.link_ok else "LINK DOWN", GREEN if hud.link_ok else RED),
        (f"pan {pan:6.1f}  tilt {tilt:6.1f}  {hud.fps:4.1f} fps", GRAY),
    ]
    if hud.distance_m is not None:
        lines.append((f"distance {hud.distance_m:.2f} m", GRAY))
    if hud.fire_allowed:
        lines.append(("FIRE ALLOWED", RED))
    elif hud.blockers:
        lines.append(("hold: " + ", ".join(hud.blockers), YELLOW))

    y = 24
    for text, color in lines:
        draw_text(frame, text, (10, y), color)
        y += 24


def _flag_line(name: str, armed: bool) -> tuple[str, tuple]:
    return (f"{name} ARMED", RED) if armed else (f"{name} DISARMED", GREEN)


def _draw_banner(frame: np.ndarray, text: str, color: tuple) -> None:
    """Wide warning strip near the bottom, clear of the crosshair."""
    height, width = frame.shape[:2]
    (text_width, _), _ = cv2.getTextSize(text, FONT, 0.8, 2)
    x = max(10, (width - text_width) // 2)
    draw_text(frame, text, (x, height - 48), color, scale=0.8, thickness=2)


def _draw_footer(frame: np.ndarray, text: str) -> None:
    height = frame.shape[0]
    draw_text(frame, text, (10, height - 12), GRAY, scale=0.5)


def draw_text(
    frame: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple,
    scale: float = 0.6,
    thickness: int = 1,
) -> None:
    """Text on a dark box so it stays readable on any background."""
    (width, height), baseline = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = origin
    cv2.rectangle(frame, (x - 3, y - height - 4), (x + width + 3, y + baseline + 1), BLACK, -1)
    cv2.putText(frame, text, origin, FONT, scale, color, thickness, cv2.LINE_AA)
