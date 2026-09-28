import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from calibration import (  # noqa: E402
    CalibrationError,
    ClickTracker,
    axis_from_samples,
    key_direction,
    measure_shift,
    move_point,
)


def textured_image(seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    noise = rng.random((240, 320)).astype(np.float32)
    return (cv2.GaussianBlur(noise, (0, 0), 2.0) * 255).astype(np.uint8)


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


@pytest.mark.parametrize(("dx", "dy"), [(12, 0), (-9, 0), (0, 15), (0, -11)])
def test_measure_shift_recovers_known_translation(dx, dy):
    image = textured_image()
    moved = np.roll(image, shift=(dy, dx), axis=(0, 1))
    measured_dx, measured_dy, response = measure_shift(image, moved)
    assert measured_dx == pytest.approx(dx, abs=0.5)
    assert measured_dy == pytest.approx(dy, abs=0.5)
    assert response > 0.3


def test_measure_shift_accepts_color_frames():
    image = cv2.cvtColor(textured_image(), cv2.COLOR_GRAY2BGR)
    measured_dx, _, _ = measure_shift(image, np.roll(image, 10, axis=1))
    assert measured_dx == pytest.approx(10, abs=0.5)


def test_axis_from_samples_gives_magnitude_and_direction():
    # +1 degree moves the image 20 px right: the servo must turn the other way.
    deg_per_px, direction = axis_from_samples([20.0, 22.0])
    assert deg_per_px == pytest.approx(1 / 21)
    assert direction == -1
    assert axis_from_samples([-25.0, -25.0]) == (pytest.approx(0.04), 1)


@pytest.mark.parametrize("samples", [[], [20.0, -20.0]])
def test_axis_from_samples_rejects_bad_measurements(samples):
    with pytest.raises(CalibrationError):
        axis_from_samples(samples)
