import pytest

cv2 = pytest.importorskip("cv2")

from calibration import ClickTracker, key_direction, move_point  # noqa: E402


def test_key_direction_maps_wasd_and_shift():
    assert key_direction(ord("d")) == ((1, 0), False)
    assert key_direction(ord("W")) == ((0, -1), True)
    assert key_direction(ord("x")) is None
    assert key_direction(-1) is None
    assert key_direction(300) is None


def test_move_point_stays_inside_the_image():
    assert move_point((10.0, 10.0), (1, 0), 5.0, (640, 480)) == (15.0, 10.0)
    assert move_point((1.0, 478.0), (-1, 1), 10.0, (640, 480)) == (0.0, 479.0)


def test_click_tracker_reports_each_click_once():
    clicks = ClickTracker()
    clicks.on_mouse(cv2.EVENT_MOUSEMOVE, 5, 5, 0, None)
    assert clicks.take() is None
    clicks.on_mouse(cv2.EVENT_LBUTTONDOWN, 120, 80, 0, None)
    assert clicks.take() == (120.0, 80.0)
    assert clicks.take() is None
