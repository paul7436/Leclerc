import pytest

from serial_link import SimulatedLink
from settings import load_settings
from turret import Turret, open_link


class FakeClock:
    def __init__(self) -> None:
        self.now = 50.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


def make_turret(clock: FakeClock, hardware_armed: bool = True) -> tuple[Turret, SimulatedLink]:
    settings = load_settings()
    link = SimulatedLink(hardware_armed=hardware_armed, clock=clock)
    turret = Turret.from_settings(settings, link)
    return turret, link


def run_frames(turret: Turret, clock: FakeClock, frames: int = 1, dt: float = 1 / 30) -> None:
    for _ in range(frames):
        turret.service(clock.now)
        clock.advance(dt)
    turret.service(clock.now)  # collect the replies to the last request


def ready_turret(clock: FakeClock) -> tuple[Turret, SimulatedLink]:
    turret, link = make_turret(clock)
    run_frames(turret, clock)
    turret.policy.arm()
    return turret, link


def on_target(turret: Turret, clock: FakeClock, veto: bool = False):
    return turret.fire_inputs((1.0, -1.0), veto=veto, now_s=clock.now)


def test_service_polls_status_and_brings_link_up(clock):
    turret, _ = make_turret(clock)
    assert not turret.link_ok(clock.now)
    run_frames(turret, clock)
    assert turret.link_ok(clock.now)
    assert turret.hardware_armed(clock.now)


def test_stale_status_counts_as_link_down_and_disarmed(clock):
    turret, _ = make_turret(clock)
    run_frames(turret, clock)
    clock.advance(1.0)  # no service call: nothing refreshes the status
    assert not turret.link_ok(clock.now)
    assert not turret.hardware_armed(clock.now)


def test_manual_fire_needs_software_arm(clock):
    turret, link = make_turret(clock)
    run_frames(turret, clock)
    decision = turret.try_fire_manual(on_target(turret, clock), clock.now)
    assert not decision.allowed
    assert link.shots_fired == 0

    turret.policy.arm()
    assert turret.try_fire_manual(on_target(turret, clock), clock.now).allowed
    run_frames(turret, clock)
    assert link.shots_fired == 1
    assert turret.last_fire_reply.accepted


def test_hardware_disarmed_firmware_refuses_even_if_host_would_fire(clock):
    turret, link = make_turret(clock, hardware_armed=False)
    run_frames(turret, clock)
    turret.policy.arm()
    decision = turret.try_fire_manual(on_target(turret, clock), clock.now)
    assert not decision.allowed
    assert "hardware disarmed" in decision.blockers
    assert link.shots_fired == 0


def test_auto_fires_only_after_lock_and_respects_cooldown(clock):
    turret, link = ready_turret(clock)
    lock_frames = load_settings().fire_policy.lock_frames
    for _ in range(lock_frames - 1):
        assert not turret.try_fire_auto(on_target(turret, clock), clock.now).allowed
        run_frames(turret, clock)
    assert turret.try_fire_auto(on_target(turret, clock), clock.now).allowed
    run_frames(turret, clock)
    assert link.shots_fired == 1

    for _ in range(lock_frames + 1):
        decision = turret.try_fire_auto(on_target(turret, clock), clock.now)
        run_frames(turret, clock)
    assert not decision.allowed
    assert any(b.startswith("cooldown") for b in decision.blockers)
    assert link.shots_fired == 1


def test_veto_blocks_every_mode(clock):
    turret, link = ready_turret(clock)
    for _ in range(10):
        turret.try_fire_auto(on_target(turret, clock, veto=True), clock.now)
        run_frames(turret, clock)
    assert not turret.try_fire_manual(on_target(turret, clock, veto=True), clock.now).allowed
    assert link.shots_fired == 0


def test_hardware_disarm_clears_software_arm(clock):
    turret, link = ready_turret(clock)
    link.set_hardware_armed(False)
    run_frames(turret, clock)
    assert not turret.policy.software_armed

    link.set_hardware_armed(True)
    run_frames(turret, clock)
    assert not turret.policy.software_armed  # re-arming is always explicit


def test_link_loss_clears_software_arm(clock):
    turret, _ = ready_turret(clock)
    clock.advance(1.0)
    turret.service(clock.now)
    assert not turret.policy.software_armed


def test_push_aim_sends_only_changes(clock):
    turret, link = make_turret(clock)
    run_frames(turret, clock)
    turret.aim.set_angles(100.0, 80.0)
    turret.push_aim()
    assert link.angles == (100.0, 80.0)

    link.set_angles(95.0, 95.0)  # someone else moved it
    turret.push_aim()  # unchanged command: nothing sent
    assert link.angles == (95.0, 95.0)


def test_link_restore_adopts_firmware_angles(clock):
    turret, link = make_turret(clock)
    link.set_angles(120.0, 70.0)
    run_frames(turret, clock)
    assert turret.aim.angles == (120.0, 70.0)


def test_shutdown_disarms_and_homes(clock):
    turret, link = ready_turret(clock)
    turret.aim.set_angles(30.0, 100.0)
    turret.push_aim()
    turret.shutdown()
    assert not turret.policy.software_armed
    assert link.angles == (90.0, 90.0)


def test_open_link_simulated_uses_config_limits():
    settings = load_settings()
    link = open_link(settings, simulate=True)
    link.set_angles(0.0, 0.0)
    assert link.angles == (settings.aim.pan.min_deg, settings.aim.tilt.min_deg)
