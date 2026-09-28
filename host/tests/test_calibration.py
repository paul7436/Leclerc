import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from calibration import (  # noqa: E402
    MIN_DISTANCE_M,
    CalibrationError,
    ClickTracker,
    HoldoverSession,
    axis_from_samples,
    holdover_from_impact,
    key_direction,
    measure_shift,
    move_point,
    remove_point,
    upsert_point,
)
from serial_link import SimulatedLink  # noqa: E402
from settings import load_settings  # noqa: E402
from turret import Turret  # noqa: E402


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


def test_holdover_from_impact():
    # The dart landed 40 px below the crosshair at 0.05 deg/px: raise 2 degrees.
    assert holdover_from_impact(400.0, 360.0, 0.05) == pytest.approx(2.0)
    assert holdover_from_impact(350.0, 360.0, 0.05) == pytest.approx(-0.5)


def test_upsert_and_remove_points():
    table = upsert_point([(3.0, 1.5)], 1.0, 0.4)
    assert table == [(1.0, 0.4), (3.0, 1.5)]
    assert upsert_point(table, 1.0, 0.6) == [(1.0, 0.6), (3.0, 1.5)]
    assert remove_point(table, 3.0) == [(1.0, 0.4)]


def make_session() -> HoldoverSession:
    settings = load_settings()
    turret = Turret.from_settings(settings, SimulatedLink())
    return HoldoverSession(settings, turret)


def test_session_distance_keys_and_undo():
    session = make_session()
    session.handle_key(ord("]"))
    assert session.distance_m == pytest.approx(1.25)
    for _ in range(10):
        session.handle_key(ord("["))
    assert session.distance_m == pytest.approx(MIN_DISTANCE_M)

    session.record_impact((640.0, 400.0))
    assert len(session.table) == 1
    session.handle_key(ord("u"))
    assert session.table == []


def test_session_records_impact_at_current_distance():
    session = make_session()
    crosshair_y = session.turret.aim.crosshair_px[1]
    deg_per_px = session.settings.aim.tilt.deg_per_px
    session.record_impact((600.0, crosshair_y + 20.0))
    assert session.table == [(1.0, pytest.approx(20.0 * deg_per_px))]


def test_session_arm_toggle_fire_request_and_aim():
    session = make_session()
    assert not session.turret.policy.software_armed
    session.handle_key(ord("x"))
    assert session.turret.policy.software_armed
    session.handle_key(ord("x"))
    assert not session.turret.policy.software_armed

    session.handle_key(ord(" "))
    assert session.fire_requested

    pan_before = session.turret.aim.angles[0]
    session.handle_key(ord("D"))
    assert session.turret.aim.angles[0] != pan_before
