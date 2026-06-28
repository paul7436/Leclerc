import math

import pytest

from serial_link import (
    FireReply,
    FirmwareEvent,
    FirmwareStatus,
    SerialLink,
    format_aim,
    parse_line,
)


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
