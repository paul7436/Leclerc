import math

import pytest

from control import (
    PID,
    AimController,
    Ballistics,
    HoldoverTable,
    estimate_distance_m,
    focal_length_px,
)
from settings import AxisSettings, PidGains


def make_axis(direction: int = 1, deg_per_px: float = 0.05) -> AxisSettings:
    return AxisSettings(
        deg_per_px=deg_per_px, direction=direction, min_deg=20.0, max_deg=160.0, home_deg=90.0
    )


def make_controller(pan_direction: int = -1, tilt_direction: int = 1, kp: float = 0.6):
    return AimController(
        crosshair_px=(640.0, 360.0),
        pan=make_axis(pan_direction),
        tilt=make_axis(tilt_direction),
        pan_pid=PID(kp=kp, output_limit=6.0),
        tilt_pid=PID(kp=kp, output_limit=6.0),
        deadband_px=2.0,
    )


def observed_pixel(command_deg, target_deg, axis: AxisSettings, center_px):
    """Camera-on-barrel model: where a target at `target_deg` appears.

    `direction` is calibrated as minus the sign of the image shift caused by
    increasing the servo angle, so the shift sign is -direction.
    """
    return center_px - axis.direction * (command_deg - target_deg) / axis.deg_per_px


def test_proportional_only():
    pid = PID(kp=0.5)
    assert pid.update(4.0, dt_s=0.1) == pytest.approx(2.0)
    assert pid.update(-2.0, dt_s=0.1) == pytest.approx(-1.0)


def test_integral_accumulates_and_is_bounded():
    pid = PID(kp=0.0, ki=1.0, integral_limit=0.5)
    assert pid.update(1.0, dt_s=0.1) == pytest.approx(0.1)
    assert pid.update(1.0, dt_s=0.1) == pytest.approx(0.2)
    for _ in range(50):
        output = pid.update(1.0, dt_s=0.1)
    assert output == pytest.approx(0.5)
    # No windup: the integral unwinds as soon as the error changes sign.
    assert pid.update(-1.0, dt_s=0.1) == pytest.approx(0.4)


def test_derivative_skips_first_update_after_reset():
    pid = PID(kp=0.0, kd=1.0)
    assert pid.update(5.0, dt_s=0.1) == 0.0
    assert pid.update(6.0, dt_s=0.1) == pytest.approx(10.0)
    pid.reset()
    assert pid.update(-3.0, dt_s=0.1) == 0.0


def test_output_is_bounded():
    pid = PID(kp=10.0, output_limit=6.0)
    assert pid.update(5.0, dt_s=0.1) == pytest.approx(6.0)
    assert pid.update(-5.0, dt_s=0.1) == pytest.approx(-6.0)


def test_zero_time_step_uses_proportional_and_held_integral():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.update(1.0, dt_s=0.5)
    assert pid.update(2.0, dt_s=0.0) == pytest.approx(2.0 + 0.5)


def test_from_gains():
    pid = PID.from_gains(PidGains(kp=0.4, ki=0.1, kd=0.02, integral_limit=3.0, output_limit=5.0))
    assert (pid.kp, pid.ki, pid.kd) == (0.4, 0.1, 0.02)
    assert (pid.integral_limit, pid.output_limit) == (3.0, 5.0)


def test_controller_starts_home_and_clamps():
    controller = make_controller()
    assert controller.angles == (90.0, 90.0)
    controller.set_angles(500.0, -10.0)
    assert controller.angles == (160.0, 20.0)


@pytest.mark.parametrize(("pan_direction", "tilt_direction"), [(1, 1), (-1, 1), (1, -1), (-1, -1)])
def test_tracking_converges_whatever_the_servo_directions(pan_direction, tilt_direction):
    controller = make_controller(pan_direction, tilt_direction)
    pan_axis, tilt_axis = make_axis(pan_direction), make_axis(tilt_direction)
    target_pan, target_tilt = 112.0, 74.0
    for _ in range(40):
        pan, tilt = controller.angles
        target_px = (
            observed_pixel(pan, target_pan, pan_axis, 640.0),
            observed_pixel(tilt, target_tilt, tilt_axis, 360.0),
        )
        controller.track(target_px, controller.crosshair_px, dt_s=1 / 30)
    pan, tilt = controller.angles
    assert pan == pytest.approx(target_pan, abs=0.15)
    assert tilt == pytest.approx(target_tilt, abs=0.15)


def test_tracking_step_is_bounded_by_pid_output_limit():
    controller = make_controller(kp=10.0)
    pan_before, _ = controller.angles
    controller.track((1200.0, 360.0), controller.crosshair_px, dt_s=1 / 30)
    pan_after, _ = controller.angles
    assert abs(pan_after - pan_before) == pytest.approx(6.0)


def test_deadband_ignores_small_errors():
    controller = make_controller()
    controller.track((641.5, 358.5), controller.crosshair_px, dt_s=1 / 30)
    assert controller.angles == (90.0, 90.0)


def test_error_is_measured_from_the_aim_point():
    assert AimController.error_px((700.0, 300.0), (640.0, 360.0)) == (60.0, -60.0)


def test_nudge_view_follows_calibrated_directions():
    controller = make_controller(pan_direction=-1, tilt_direction=1)
    controller.nudge_view(right=1, down=0, step_deg=2.0)
    assert controller.angles == (88.0, 90.0)
    controller.nudge_view(right=0, down=-1, step_deg=2.0)  # aim up
    assert controller.angles == (88.0, 88.0)


def test_home_restores_home_angles():
    controller = make_controller()
    controller.set_angles(30.0, 100.0)
    controller.home()
    assert controller.angles == (90.0, 90.0)


def test_holdover_table_interpolates_and_holds_ends():
    table = HoldoverTable([(3.0, 2.0), (1.0, 0.5)])
    assert table.holdover_deg(0.5) == pytest.approx(0.5)
    assert table.holdover_deg(1.0) == pytest.approx(0.5)
    assert table.holdover_deg(2.0) == pytest.approx(1.25)
    assert table.holdover_deg(10.0) == pytest.approx(2.0)


def test_empty_holdover_table_is_zero():
    assert HoldoverTable([]).holdover_deg(2.0) == 0.0


def test_focal_length_and_distance_estimate():
    focal = focal_length_px(0.05)
    assert focal == pytest.approx(1 / math.tan(math.radians(0.05)))
    # A 0.2 m target that appears 0.2 * focal / 2 pixels tall is 2 m away.
    assert estimate_distance_m(0.2 * focal / 2.0, 0.2, focal) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        estimate_distance_m(0.0, 0.2, focal)


def test_ballistics_lowers_aim_point_by_holdover():
    focal = focal_length_px(0.05)
    ballistics = Ballistics(HoldoverTable([(1.0, 0.5), (3.0, 1.5)]), 0.2, tilt_deg_per_px=0.05)
    box_height_at_2m = 0.2 * focal / 2.0
    x, y = ballistics.aim_point((640.0, 360.0), box_height_at_2m)
    assert x == 640.0
    assert y == pytest.approx(360.0 + 1.0 / 0.05)  # 1 degree of holdover, 20 px lower


def test_ballistics_disabled_without_table_or_target_height():
    no_table = Ballistics(HoldoverTable([]), 0.2, tilt_deg_per_px=0.05)
    no_height = Ballistics(HoldoverTable([(1.0, 0.5)]), 0.0, tilt_deg_per_px=0.05)
    for ballistics in (no_table, no_height):
        assert not ballistics.enabled
        assert ballistics.aim_point((640.0, 360.0), 100.0) == (640.0, 360.0)
