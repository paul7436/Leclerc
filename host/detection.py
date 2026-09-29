"""Camera capture and YOLO target detection.

YoloDetector.detect() splits each frame's detections into targets and vetoes
(protected or inhibit classes anywhere in view). select_target() then picks
the target to engage, and best_target() does both and hands back its box.

OpenCV and Ultralytics are imported where they are used, so the selection
and veto logic can be unit tested without either of them installed.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from fire_policy import PROTECTED_CLASSES
from settings import CameraSettings, DetectionSettings, SafetySettings


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    box: tuple[float, float, float, float]  # x1, y1, x2, y2 in pixels

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    @property
    def height(self) -> float:
        return self.box[3] - self.box[1]


@dataclass(frozen=True)
class FrameDetections:
    targets: tuple[Detection, ...] = ()
    vetoes: tuple[Detection, ...] = ()  # protected or inhibit classes in view

    @property
    def veto(self) -> bool:
        return bool(self.vetoes)


def classify(
    detections: Iterable[Detection],
    target_classes: Sequence[str],
    target_confidence: float,
    veto_classes: frozenset[str],
    veto_confidence: float,
) -> FrameDetections:
    """Splits raw detections into targets and vetoes.

    A veto class is never a target, whatever `target_classes` says. An empty
    `target_classes` accepts every class that is not a veto class.
    """
    targets = []
    vetoes = []
    for detection in detections:
        if detection.class_name in veto_classes:
            if detection.confidence >= veto_confidence:
                vetoes.append(detection)
            continue
        wanted = not target_classes or detection.class_name in target_classes
        if wanted and detection.confidence >= target_confidence:
            targets.append(detection)
    return FrameDetections(targets=tuple(targets), vetoes=tuple(vetoes))


def select_target(
    targets: Sequence[Detection], reference_px: tuple[float, float], strategy: str
) -> Detection | None:
    """Picks the target nearest to `reference_px`, or the most confident one."""
    if not targets:
        return None
    if strategy == "confidence":
        return max(targets, key=lambda d: d.confidence)
    return min(targets, key=lambda d: math.dist(d.center, reference_px))


# ---------------------------------------------------------------------------
# YOLO inference
# ---------------------------------------------------------------------------


def _load_yolo(path: str) -> Any:
    from ultralytics import YOLO

    return YOLO(path)


class YoloDetector:
    """Runs the target model and, when needed, the protected-class guard model.

    If the target model already knows every protected class (a COCO model),
    its own detections drive the veto. Otherwise (a custom model) a second,
    COCO-trained guard model runs on every frame. The guard cannot be turned
    off: a detector that cannot see protected classes refuses to start.
    """

    def __init__(
        self,
        detection: DetectionSettings,
        safety: SafetySettings,
        load_model: Callable[[str], Any] = _load_yolo,
    ) -> None:
        self._settings = detection
        self._guard_confidence = safety.guard_confidence
        self._veto_classes = PROTECTED_CLASSES | frozenset(safety.extra_inhibit_classes)

        self._target_model = load_model(detection.model_path)
        target_names = set(self._target_model.names.values())
        missing = sorted(set(detection.target_classes) - target_names)
        if missing:
            raise ValueError(f"{detection.model_path} has no class named: {', '.join(missing)}")

        self._guard_model = None
        known_names = target_names
        if not target_names >= PROTECTED_CLASSES:
            self._guard_model = load_model(safety.guard_model_path)
            known_names = target_names | set(self._guard_model.names.values())
        undetectable = sorted(self._veto_classes - known_names)
        if undetectable:
            raise ValueError(f"no loaded model can detect: {', '.join(undetectable)}")

    @property
    def uses_guard_model(self) -> bool:
        return self._guard_model is not None

    def warm_up(self, width: int, height: int) -> None:
        """Runs one inference on a blank frame.

        The first inference is several times slower than the next ones. Doing
        it before the serial link opens keeps it from stalling the main loop
        past the firmware link timeout.
        """
        self.detect(np.zeros((height, width, 3), dtype=np.uint8))

    def detect(self, frame: Any) -> FrameDetections:
        # One pass at the lower threshold serves both targets and vetoes.
        threshold = min(self._settings.confidence, self._guard_confidence)
        detections = self._run(self._target_model, frame, threshold)
        if self._guard_model is not None:
            guard_hits = self._run(self._guard_model, frame, self._guard_confidence)
            detections += [d for d in guard_hits if d.class_name in self._veto_classes]
        return classify(
            detections,
            self._settings.target_classes,
            self._settings.confidence,
            self._veto_classes,
            self._guard_confidence,
        )

    def best_target(
        self, frame: Any, reference_px: tuple[float, float]
    ) -> tuple[Detection | None, FrameDetections]:
        """Detects, then returns the target to engage (its .center) and all detections."""
        frame_detections = self.detect(frame)
        target = select_target(frame_detections.targets, reference_px, self._settings.selection)
        return target, frame_detections

    def _run(self, model: Any, frame: Any, confidence: float) -> list[Detection]:
        results = model.predict(
            frame,
            conf=confidence,
            imgsz=self._settings.image_size,
            device=self._settings.device or None,
            verbose=False,
        )
        detections = []
        for result in results:
            boxes = result.boxes
            for box, score, class_id in zip(
                boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist(), strict=True
            ):
                name = result.names[int(class_id)]
                detections.append(Detection(name, float(score), tuple(float(v) for v in box)))
        return detections


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


class CameraError(RuntimeError):
    """The camera could not be opened or stopped delivering frames."""


class Camera:
    """OpenCV capture from a webcam index or a video file."""

    def __init__(self, settings: CameraSettings, source: int | str | None = None) -> None:
        import cv2

        if source is None:
            source = settings.source
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        self._capture = cv2.VideoCapture(source)
        if not self._capture.isOpened():
            raise CameraError(f"cannot open camera source {source!r}")
        if isinstance(source, int):
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.height)
            self._capture.set(cv2.CAP_PROP_FPS, settings.fps)
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # newest frame, lowest latency

    def read(self) -> Any:
        ok, frame = self._capture.read()
        if not ok:
            raise CameraError("the camera returned no frame (disconnected or end of video)")
        return frame

    def release(self) -> None:
        self._capture.release()

    def __enter__(self) -> Camera:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.release()
