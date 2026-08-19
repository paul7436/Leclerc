import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")

from detection import Detection, FrameDetections  # noqa: E402
from overlay import HudState, draw_hud  # noqa: E402


def test_draw_hud_draws_every_element_without_error():
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    target = Detection("balloon", 0.9, (300.0, 150.0, 340.0, 200.0))
    person = Detection("person", 0.4, (10.0, 10.0, 100.0, 300.0))
    hud = HudState(
        mode="AUTO",
        software_armed=True,
        hardware_armed=False,
        link_ok=True,
        angles=(90.0, 85.0),
        crosshair_px=(320.0, 180.0),
        aim_point_px=(320.0, 190.0),
        blockers=("hardware disarmed", "protected class in view"),
        target=target,
        detections=FrameDetections(targets=(target,), vetoes=(person,)),
        distance_m=2.4,
        fps=29.7,
        footer="x arm  m mode  q quit",
    )
    draw_hud(frame, hud)
    assert frame.any()
