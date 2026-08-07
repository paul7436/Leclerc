import pytest

from detection import Detection, YoloDetector, classify, select_target
from fire_policy import PROTECTED_CLASSES
from settings import DetectionSettings, SafetySettings

COCO_NAMES = {0: "person", 14: "bird", 15: "cat", 16: "dog", 32: "sports ball", 39: "bottle"}
CUSTOM_NAMES = {0: "balloon", 1: "cardboard_target", 2: "robot"}


def det(name: str, confidence: float = 0.9, box=(0.0, 0.0, 10.0, 10.0)) -> Detection:
    return Detection(name, confidence, box)


class FakeTensor:
    def __init__(self, values):
        self._values = values

    def tolist(self):
        return list(self._values)


class FakeBoxes:
    def __init__(self, detections):
        self.xyxy = FakeTensor([d[2] for d in detections])
        self.conf = FakeTensor([d[1] for d in detections])
        self.cls = FakeTensor([d[0] for d in detections])


class FakeResult:
    def __init__(self, names, detections):
        self.names = names
        self.boxes = FakeBoxes(detections)


class FakeModel:
    """Mimics ultralytics.YOLO: names plus predict() returning fixed results."""

    def __init__(self, names, detections=()):
        self.names = names
        self.detections = list(detections)  # (class_id, confidence, box)
        self.calls = []

    def predict(self, frame, conf, imgsz, device, verbose):
        self.calls.append(conf)
        kept = [d for d in self.detections if d[1] >= conf]
        return [FakeResult(self.names, kept)]


def make_detector(models: dict, target_classes=(), extra_inhibit=()) -> YoloDetector:
    detection = DetectionSettings(
        model_path="target.pt", confidence=0.5, target_classes=list(target_classes)
    )
    safety = SafetySettings(
        guard_model_path="guard.pt",
        guard_confidence=0.3,
        extra_inhibit_classes=list(extra_inhibit),
    )
    return YoloDetector(detection, safety, load_model=models.__getitem__)


# -- pure selection logic ------------------------------------------------------


def test_detection_geometry():
    detection = det("cup", box=(10.0, 20.0, 30.0, 60.0))
    assert detection.center == (20.0, 40.0)
    assert detection.height == 40.0


def test_classify_splits_targets_and_vetoes():
    frame = classify(
        [det("bottle", 0.8), det("person", 0.35), det("cup", 0.4), det("chair", 0.9)],
        target_classes=["bottle", "cup"],
        target_confidence=0.5,
        veto_classes=PROTECTED_CLASSES,
        veto_confidence=0.3,
    )
    assert [d.class_name for d in frame.targets] == ["bottle"]
    assert [d.class_name for d in frame.vetoes] == ["person"]
    assert frame.veto


def test_protected_class_is_never_a_target_even_if_listed():
    frame = classify(
        [det("person", 0.95)],
        target_classes=["person"],
        target_confidence=0.5,
        veto_classes=PROTECTED_CLASSES,
        veto_confidence=0.3,
    )
    assert frame.targets == ()
    assert frame.veto


def test_empty_target_list_accepts_every_non_veto_class():
    frame = classify(
        [det("chair"), det("dog")],
        target_classes=[],
        target_confidence=0.5,
        veto_classes=PROTECTED_CLASSES,
        veto_confidence=0.3,
    )
    assert [d.class_name for d in frame.targets] == ["chair"]
    assert [d.class_name for d in frame.vetoes] == ["dog"]


def test_select_target_strategies():
    near = det("cup", 0.6, box=(600.0, 340.0, 640.0, 380.0))
    sure = det("cup", 0.95, box=(0.0, 0.0, 40.0, 40.0))
    assert select_target([sure, near], (640.0, 360.0), "nearest") is near
    assert select_target([near, sure], (640.0, 360.0), "confidence") is sure
    assert select_target([], (640.0, 360.0), "nearest") is None


# -- YoloDetector wiring --------------------------------------------------------


def test_coco_model_vetoes_by_itself():
    target = FakeModel(
        COCO_NAMES,
        [(39, 0.8, (0, 0, 10, 10)), (0, 0.35, (50, 50, 90, 200))],
    )
    detector = make_detector({"target.pt": target}, target_classes=["bottle"])
    assert not detector.uses_guard_model

    frame = detector.detect(frame=None)
    assert target.calls == [0.3]  # one pass at the lower (guard) threshold
    assert [d.class_name for d in frame.targets] == ["bottle"]
    assert [d.class_name for d in frame.vetoes] == ["person"]


def test_custom_model_gets_a_guard_model():
    target = FakeModel(CUSTOM_NAMES, [(0, 0.9, (0, 0, 10, 10))])
    guard = FakeModel(COCO_NAMES, [(16, 0.4, (5, 5, 50, 50)), (39, 0.9, (0, 0, 5, 5))])
    detector = make_detector({"target.pt": target, "guard.pt": guard}, ["balloon"])
    assert detector.uses_guard_model

    frame = detector.detect(frame=None)
    assert [d.class_name for d in frame.targets] == ["balloon"]
    assert [d.class_name for d in frame.vetoes] == ["dog"]  # guard bottles are ignored


def test_best_target_returns_selected_detection():
    target = FakeModel(CUSTOM_NAMES, [(2, 0.7, (600, 340, 680, 380)), (1, 0.9, (0, 0, 9, 9))])
    detector = make_detector({"target.pt": target, "guard.pt": FakeModel(COCO_NAMES)})
    best, frame = detector.best_target(frame=None, reference_px=(640.0, 360.0))
    assert best.class_name == "robot"
    assert best.center == (640.0, 360.0)
    assert len(frame.targets) == 2


def test_unknown_target_class_is_rejected():
    with pytest.raises(ValueError, match="has no class named: balloon"):
        make_detector({"target.pt": FakeModel(COCO_NAMES)}, target_classes=["balloon"])


def test_guard_model_must_know_protected_classes():
    blind_guard = FakeModel({0: "bottle"})
    with pytest.raises(ValueError, match="no loaded model can detect"):
        make_detector({"target.pt": FakeModel(CUSTOM_NAMES), "guard.pt": blind_guard})


def test_extra_inhibit_class_must_be_detectable():
    with pytest.raises(ValueError, match="no loaded model can detect: unicorn"):
        make_detector({"target.pt": FakeModel(COCO_NAMES)}, extra_inhibit=["unicorn"])
