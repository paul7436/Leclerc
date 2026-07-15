import math

import pytest

from serial_link import (
    FireReply,
    FirmwareEvent,
    FirmwareStatus,
    SerialLink,
    SimulatedLink,
    format_aim,
    parse_line,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakePort:
    """Minimal pyserial stand-in: records writes, serves queued input."""

    def __init__(self) -> None:
        self.written = b""
        self.incoming = b""
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self.incoming)

    def read(self, size: int) -> bytes:
        data, self.incoming = self.incoming[:size], self.incoming[size:]
        return data

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def close(self) -> None:
        self.is_open = False


# -- protocol helpers --------------------------------------------------------


def test_format_aim():
    assert format_aim(90, 75.456) == "A90.00 75.46\n"
    assert format_aim(-5.5, 0) == "A-5.50 0.00\n"


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_format_aim_rejects_non_finite(bad):
    with pytest.raises(ValueError):
        format_aim(bad, 90)


def test_parse_status_line():
    message = parse_line(
        "STATUS armed=1 link=0 pan=101.50 tilt=80.25 trigger=HOLD cooldown_ms=1200\r"
    )
    assert message == FirmwareStatus(
        armed=True,
        link_ok=False,
        pan_deg=101.5,
        tilt_deg=80.25,
        trigger="HOLD",
        cooldown_ms=1200,
    )


def test_parse_status_ignores_unknown_keys():
    message = parse_line("STATUS armed=0 link=1 pan=90 tilt=90 trigger=IDLE cooldown_ms=0 v=2")
    assert isinstance(message, FirmwareStatus)


@pytest.mark.parametrize(
    "line",
    [
        "STATUS armed=1 link=1 pan=90 tilt=90 trigger=IDLE",
        "STATUS armed=yes link=1 pan=90 tilt=90 trigger=IDLE cooldown_ms=0",
        "STATUS armed=1 link=1 pan=abc tilt=90 trigger=IDLE cooldown_ms=0",
        "FIRE MAYBE",
        "FIRE DENIED",
        "EVT ARM 2",
        "EVT TEMP 1",
        "hello",
        "",
    ],
)
def test_parse_rejects_malformed_lines(line):
    assert parse_line(line) is None


def test_parse_fire_events_and_errors():
    assert parse_line("FIRE OK") == FireReply(accepted=True, reason="")
    assert parse_line("FIRE DENIED COOLDOWN") == FireReply(accepted=False, reason="COOLDOWN")
    assert parse_line("EVT ARM 1") == FirmwareEvent("ARM", "1")
    assert parse_line("EVT LINK 0") == FirmwareEvent("LINK", "0")
    assert parse_line("READY 1.0.0") == FirmwareEvent("READY", "1.0.0")
    assert parse_line("ERR SYNTAX") == FirmwareEvent("ERR", "SYNTAX")


# -- SerialLink over a fake port ---------------------------------------------


def test_serial_link_writes_commands():
    port = FakePort()
    link = SerialLink(port)
    link.set_angles(100, 80)
    link.request_status()
    link.fire()
    assert port.written == b"A100.00 80.00\nS\nF\n"


def test_serial_link_assembles_split_lines_and_tracks_status():
    port = FakePort()
    link = SerialLink(port)
    port.incoming = b"READY 1.0.0\r\nSTATUS armed=0 link=1 pan=90.00 "
    assert link.poll(now_s=1.0) == [FirmwareEvent("READY", "1.0.0")]
    assert link.status is None

    port.incoming = b"tilt=90.00 trigger=IDLE cooldown_ms=0\nnoise\nEVT ARM 1\n"
    messages = link.poll(now_s=2.0)
    assert len(messages) == 2
    assert link.status.armed  # EVT ARM updated the cached status
    assert link.status_age_s(now_s=2.5) == pytest.approx(0.5)


def test_serial_link_drops_runaway_garbage():
    port = FakePort()
    link = SerialLink(port)
    port.incoming = b"x" * 5000
    assert link.poll(now_s=0.0) == []
    port.incoming = b"FIRE OK\n"
    assert link.poll(now_s=0.0) == [FireReply(True, "")]


def test_status_age_is_infinite_before_first_status():
    assert SerialLink(FakePort()).status_age_s(now_s=10.0) == math.inf


def test_close_releases_port():
    port = FakePort()
    with SerialLink(port):
        pass
    assert not port.is_open


# -- SimulatedLink follows the firmware rules ---------------------------------


def make_sim(**kwargs) -> tuple[SimulatedLink, FakeClock]:
    clock = FakeClock()
    sim = SimulatedLink(clock=clock, **kwargs)
    sim.poll(clock.now)  # consume the READY banner
    return sim, clock


def fire_reply(sim: SimulatedLink, clock: FakeClock) -> FireReply:
    sim.fire()
    replies = [m for m in sim.poll(clock.now) if isinstance(m, FireReply)]
    assert len(replies) == 1
    return replies[0]


def test_sim_refuses_fire_when_disarmed_by_default():
    sim, clock = make_sim()
    sim.request_status()
    sim.poll(clock.now)
    assert fire_reply(sim, clock) == FireReply(False, "DISARMED")
    assert sim.shots_fired == 0


def test_sim_link_must_be_established_by_status_or_aim():
    sim, clock = make_sim(hardware_armed=True)
    assert fire_reply(sim, clock) == FireReply(False, "LINK")
    sim.request_status()
    messages = sim.poll(clock.now)
    assert FirmwareEvent("LINK", "1") in messages
    assert sim.status.link_ok
    assert fire_reply(sim, clock) == FireReply(True, "")
    assert sim.shots_fired == 1


def test_sim_busy_then_cooldown_then_ready():
    sim, clock = make_sim(hardware_armed=True, sweep_s=0.5, cooldown_s=1.5)
    sim.request_status()
    assert fire_reply(sim, clock).accepted
    clock.advance(0.2)
    assert fire_reply(sim, clock) == FireReply(False, "BUSY")
    clock.advance(0.4)
    assert fire_reply(sim, clock) == FireReply(False, "COOLDOWN")
    clock.advance(1.4)
    sim.request_status()  # keepalive: F alone never restores a dropped link
    assert fire_reply(sim, clock).accepted


def test_sim_watchdog_drops_link_after_silence():
    sim, clock = make_sim(hardware_armed=True, link_timeout_s=0.5)
    sim.set_angles(90, 90)
    sim.request_status()
    sim.poll(clock.now)
    clock.advance(0.6)
    assert FirmwareEvent("LINK", "0") in sim.poll(clock.now)
    assert not sim.status.link_ok
    assert fire_reply(sim, clock) == FireReply(False, "LINK")


def test_sim_clamps_angles_and_rejects_bad_commands():
    sim, clock = make_sim(pan_limits=(20, 160), tilt_limits=(60, 115))
    sim.set_angles(500, -40)
    assert sim.angles == (160, 60)
    sim._write("A90\nX\n")
    errors = [m for m in sim.poll(clock.now) if isinstance(m, FirmwareEvent) and m.name == "ERR"]
    assert [e.value for e in errors] == ["SYNTAX", "UNKNOWN"]
    assert sim.angles == (160, 60)


def test_sim_arming_switch_event_updates_status():
    sim, clock = make_sim(hardware_armed=True)
    sim.request_status()
    sim.poll(clock.now)
    sim.set_hardware_armed(False)
    assert FirmwareEvent("ARM", "0") in sim.poll(clock.now)
    assert not sim.status.armed
