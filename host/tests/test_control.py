import pytest

from control import PID
from settings import PidGains


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
