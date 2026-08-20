import pytest

pytest.importorskip("cv2")

from control import Ballistics, HoldoverTable  # noqa: E402
from detection import Detection, FrameDetections  # noqa: E402
from main import Mode, TurretApp, parse_args  # noqa: E402
from serial_link import SimulatedLink  # noqa: E402
from settings import load_settings  # noqa: E402
from turret import Turret  # noqa: E402

FRAME_DT = 1 / 30


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now


class FakeDetector:
    """Returns whatever the test puts in `target` and `vetoes`."""

    def __init__(self) -> None:
        self.target: Detection | None = None
        self.vetoes: tuple[Detection, ...] = ()

    def best_target(self, frame, reference_px):
        targets = (self.target,) if self.target else ()
        return self.target, FrameDetections(targets=targets, vetoes=self.vetoes)


def box_at(center, size=40.0) -> tuple[float, float, float, float]:
    x, y = center
    return (x - size / 2, y - size / 2, x + size / 2, y + size / 2)


@pytest.fixture
def app_parts():
    settings = load_settings()
    clock = FakeClock()
    link = SimulatedLink(hardware_armed=True, clock=clock)
    turret = Turret.from_settings(settings, link)
    detector = FakeDetector()
    ballistics = Ballistics(HoldoverTable([]), 0.0, settings.aim.tilt.deg_per_px)
    app = TurretApp(settings, turret, detector, ballistics)
    return app, link, detector, clock


def run_frames(app: TurretApp, clock: FakeClock, count: int):
    hud = None
    for _ in range(count):
        hud = app.step(frame=None, now_s=clock.now)
        clock.now += FRAME_DT
    return hud


def test_starts_manual_and_disarmed(app_parts):
    app, link, _, clock = app_parts
    hud = run_frames(app, clock, 3)
    assert app.mode is Mode.MANUAL
    assert not hud.software_armed
    assert hud.hardware_armed
    assert "software disarmed" in hud.blockers


def test_mode_switch_always_disarms(app_parts):
    app, _, _, clock = app_parts
    run_frames(app, clock, 2)
    app.handle_key(ord("x"))
    assert app.turret.policy.software_armed
    app.handle_key(ord("m"))
    assert app.mode is Mode.AUTO
    assert not app.turret.policy.software_armed


def test_auto_tracks_locks_and_fires_once(app_parts):
    app, link, detector, clock = app_parts
    detector.target = Detection("balloon", 0.9, box_at(app.turret.aim.crosshair_px))
    run_frames(app, clock, 2)
    app.handle_key(ord("m"))
    app.handle_key(ord("x"))
    run_frames(app, clock, 10)
    assert link.shots_fired == 1


def test_auto_never_fires_with_protected_class_in_view(app_parts):
    app, link, detector, clock = app_parts
    detector.target = Detection("balloon", 0.9, box_at(app.turret.aim.crosshair_px))
    detector.vetoes = (Detection("person", 0.5, (0.0, 0.0, 100.0, 300.0)),)
    run_frames(app, clock, 2)
    app.handle_key(ord("m"))
    app.handle_key(ord("x"))
    hud = run_frames(app, clock, 30)
    assert link.shots_fired == 0
    assert "protected class in view" in hud.blockers


def test_auto_tracking_moves_toward_target(app_parts):
    app, _, detector, clock = app_parts
    crosshair_x, crosshair_y = app.turret.aim.crosshair_px
    detector.target = Detection("balloon", 0.9, box_at((crosshair_x + 200, crosshair_y)))
    run_frames(app, clock, 2)
    app.handle_key(ord("m"))
    pan_before, _ = app.turret.aim.angles
    run_frames(app, clock, 3)
    pan_after, _ = app.turret.aim.angles
    pan_direction = app.settings.aim.pan.direction
    assert (pan_after - pan_before) * pan_direction > 0


def test_manual_fire_key_fires_once_when_armed(app_parts):
    app, link, _, clock = app_parts
    run_frames(app, clock, 2)
    app.handle_key(ord(" "))
    run_frames(app, clock, 2)
    assert link.shots_fired == 0  # still software disarmed

    app.handle_key(ord("x"))
    app.handle_key(ord(" "))
    run_frames(app, clock, 3)
    assert link.shots_fired == 1


def test_fire_key_is_ignored_in_auto(app_parts):
    app, link, _, clock = app_parts
    run_frames(app, clock, 2)
    app.handle_key(ord("m"))
    app.handle_key(ord("x"))
    app.handle_key(ord(" "))
    run_frames(app, clock, 3)
    assert link.shots_fired == 0  # no target, and space does nothing in AUTO


def test_manual_nudge_and_coarse_nudge(app_parts):
    app, _, _, clock = app_parts
    run_frames(app, clock, 2)
    pan, tilt = app.turret.aim.angles
    aim = app.settings.aim
    app.handle_key(ord("d"))
    assert app.turret.aim.angles[0] == pytest.approx(pan + aim.pan.direction * aim.manual_step_deg)
    app.handle_key(ord("W"))
    expected_tilt = tilt - aim.tilt.direction * aim.manual_coarse_step_deg
    assert app.turret.aim.angles[1] == pytest.approx(expected_tilt)


def test_parse_args_simulate_flag():
    assert parse_args(["--simulate"]).simulate


@pytest.mark.parametrize("key", [ord("q"), 27])
def test_quit_keys_stop_the_loop(app_parts, key):
    app, *_ = app_parts
    assert app.handle_key(key) is False
    assert app.handle_key(ord("h")) is True
