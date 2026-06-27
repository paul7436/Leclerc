import pytest

from fire_policy import PROTECTED_CLASSES, FireInputs, FirePolicy


def make_policy(lock_frames: int = 3) -> FirePolicy:
    return FirePolicy(error_threshold_px=10, lock_frames=lock_frames, cooldown_s=2.0)


def good_inputs(**overrides) -> FireInputs:
    values = dict(
        target_present=True,
        error_px=(2.0, -3.0),
        hardware_armed=True,
        link_ok=True,
        protected_in_view=False,
    )
    values.update(overrides)
    return FireInputs(**values)


def locked_armed_policy() -> FirePolicy:
    policy = make_policy(lock_frames=3)
    policy.arm()
    for _ in range(3):
        policy.observe(good_inputs())
    return policy


def test_starts_software_disarmed():
    policy = make_policy()
    assert not policy.software_armed
    decision = policy.evaluate_manual(good_inputs(), now_s=0.0)
    assert not decision.allowed
    assert "software disarmed" in decision.blockers


def test_auto_fires_only_when_every_condition_holds():
    policy = locked_armed_policy()
    decision = policy.evaluate_auto(good_inputs(), now_s=10.0)
    assert decision.allowed
    assert decision.blockers == ()


@pytest.mark.parametrize(
    ("overrides", "blocker"),
    [
        ({"hardware_armed": False}, "hardware disarmed"),
        ({"link_ok": False}, "link down"),
        ({"protected_in_view": True}, "protected class in view"),
        ({"target_present": False, "error_px": None}, "no target"),
        ({"error_px": (10.0, 0.0)}, "not on target"),
        ({"error_px": (0.0, -12.0)}, "not on target"),
    ],
)
def test_auto_refuses_when_any_condition_fails(overrides, blocker):
    policy = locked_armed_policy()
    decision = policy.evaluate_auto(good_inputs(**overrides), now_s=10.0)
    assert not decision.allowed
    assert blocker in decision.blockers


def test_auto_requires_consecutive_lock_frames():
    policy = make_policy(lock_frames=3)
    policy.arm()
    policy.observe(good_inputs())
    policy.observe(good_inputs())
    decision = policy.evaluate_auto(good_inputs(), now_s=10.0)
    assert not decision.allowed
    assert "locking 2/3" in decision.blockers

    policy.observe(good_inputs(error_px=(50.0, 0.0)))  # drifts off target
    assert policy.locked_frames == 0


def test_cooldown_blocks_until_elapsed_and_resets_lock():
    policy = locked_armed_policy()
    policy.record_shot(now_s=10.0)
    assert policy.locked_frames == 0
    assert policy.cooldown_remaining(11.5) == pytest.approx(0.5)

    decision = policy.evaluate_manual(good_inputs(), now_s=11.5)
    assert not decision.allowed
    assert "cooldown 0.5s" in decision.blockers
    assert policy.evaluate_manual(good_inputs(), now_s=12.0).allowed


def test_manual_ignores_target_but_keeps_every_other_check():
    policy = make_policy()
    policy.arm()
    no_target = good_inputs(target_present=False, error_px=None)
    assert policy.evaluate_manual(no_target, now_s=0.0).allowed

    vetoed = good_inputs(target_present=False, error_px=None, protected_in_view=True)
    assert not policy.evaluate_manual(vetoed, now_s=0.0).allowed
    assert not policy.evaluate_manual(good_inputs(hardware_armed=False), now_s=0.0).allowed


def test_disarm_clears_flag_and_lock():
    policy = locked_armed_policy()
    policy.disarm()
    assert not policy.software_armed
    assert policy.locked_frames == 0


def test_reports_every_blocker_at_once():
    policy = make_policy()
    inputs = FireInputs(
        target_present=False,
        error_px=None,
        hardware_armed=False,
        link_ok=False,
        protected_in_view=True,
    )
    blockers = policy.evaluate_auto(inputs, now_s=0.0).blockers
    assert set(blockers) == {
        "software disarmed",
        "link down",
        "hardware disarmed",
        "protected class in view",
        "no target",
    }


@pytest.mark.parametrize(
    "kwargs",
    [
        {"error_threshold_px": 0, "lock_frames": 3, "cooldown_s": 2.0},
        {"error_threshold_px": 10, "lock_frames": 0, "cooldown_s": 2.0},
        {"error_threshold_px": 10, "lock_frames": 3, "cooldown_s": 0.1},
    ],
)
def test_rejects_unsafe_parameters(kwargs):
    with pytest.raises(ValueError):
        FirePolicy(**kwargs)


def test_people_and_pets_are_protected():
    assert {"person", "cat", "dog"} <= PROTECTED_CLASSES
