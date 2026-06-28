"""Serial link to the turret firmware. Protocol: docs/protocol.md.

SerialLink talks to the ESP32 through pyserial.

Nothing here waits for a reply: commands are written immediately, and poll()
collects whatever the firmware has sent since the previous call. The main
loop calls poll() once per frame.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, replace

import serial

logger = logging.getLogger(__name__)

FIRE_COMMAND = "F\n"
STATUS_COMMAND = "S\n"

# Unterminated input beyond this many bytes is garbage and is dropped.
_MAX_PENDING_BYTES = 1024


# ---------------------------------------------------------------------------
# Messages and protocol helpers
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FirmwareStatus:
    """Parsed STATUS line."""

    armed: bool
    link_ok: bool
    pan_deg: float
    tilt_deg: float
    trigger: str
    cooldown_ms: int


@dataclass(frozen=True)
class FireReply:
    """Parsed FIRE line; `reason` is empty when the shot was accepted."""

    accepted: bool
    reason: str


@dataclass(frozen=True)
class FirmwareEvent:
    """READY banner, EVT ARM / EVT LINK change, or ERR reply."""

    name: str  # "READY", "ARM", "LINK" or "ERR"
    value: str


Message = FirmwareStatus | FireReply | FirmwareEvent


def format_aim(pan_deg: float, tilt_deg: float) -> str:
    """Builds an A command. The firmware clamps the angles to its limits."""
    if not (math.isfinite(pan_deg) and math.isfinite(tilt_deg)):
        raise ValueError("aim angles must be finite numbers")
    return f"A{pan_deg:.2f} {tilt_deg:.2f}\n"


def parse_line(line: str) -> Message | None:
    """Parses one firmware line. Unknown or malformed lines return None."""
    head, _, rest = line.strip().partition(" ")
    rest = rest.strip()
    if head == "STATUS":
        return _parse_status(rest)
    if head == "FIRE":
        return _parse_fire(rest)
    if head == "EVT":
        name, _, value = rest.partition(" ")
        if name in ("ARM", "LINK") and value in ("0", "1"):
            return FirmwareEvent(name, value)
        return None
    if head in ("READY", "ERR"):
        return FirmwareEvent(head, rest)
    return None


def _parse_status(rest: str) -> FirmwareStatus | None:
    fields = dict(item.split("=", 1) for item in rest.split() if "=" in item)
    try:
        return FirmwareStatus(
            armed=_parse_flag(fields["armed"]),
            link_ok=_parse_flag(fields["link"]),
            pan_deg=float(fields["pan"]),
            tilt_deg=float(fields["tilt"]),
            trigger=fields["trigger"],
            cooldown_ms=int(fields["cooldown_ms"]),
        )
    except (KeyError, ValueError):
        return None


def _parse_flag(text: str) -> bool:
    if text not in ("0", "1"):
        raise ValueError(f"not a flag: {text!r}")
    return text == "1"


def _parse_fire(rest: str) -> FireReply | None:
    if rest == "OK":
        return FireReply(accepted=True, reason="")
    verdict, _, reason = rest.partition(" ")
    if verdict == "DENIED" and reason:
        return FireReply(accepted=False, reason=reason.strip())
    return None


# ---------------------------------------------------------------------------
# Common link behaviour
# ---------------------------------------------------------------------------


class TurretLink:
    """Commands, reply collection and the latest firmware status.

    Subclasses provide the transport through _write() and _read_lines().
    """

    def __init__(self) -> None:
        self._status: FirmwareStatus | None = None
        self._status_time_s: float | None = None

    @property
    def status(self) -> FirmwareStatus | None:
        """Latest status, updated by STATUS lines and EVT ARM/LINK events."""
        return self._status

    def status_age_s(self, now_s: float) -> float:
        """Seconds since the last STATUS line (infinite before the first)."""
        if self._status_time_s is None:
            return math.inf
        return now_s - self._status_time_s

    def set_angles(self, pan_deg: float, tilt_deg: float) -> None:
        self._write(format_aim(pan_deg, tilt_deg))

    def request_status(self) -> None:
        self._write(STATUS_COMMAND)

    def fire(self) -> None:
        """Sends F. Only turret.Turret calls this, after the fire policy."""
        self._write(FIRE_COMMAND)

    def poll(self, now_s: float) -> list[Message]:
        """Reads and applies every complete line received so far."""
        messages = []
        for line in self._read_lines():
            message = parse_line(line)
            if message is None:
                logger.debug("ignored firmware line %r", line)
                continue
            self._apply(message, now_s)
            messages.append(message)
        return messages

    def close(self) -> None:
        """Releases the transport."""

    def __enter__(self) -> TurretLink:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def _apply(self, message: Message, now_s: float) -> None:
        if isinstance(message, FirmwareStatus):
            self._status = message
            self._status_time_s = now_s
        elif isinstance(message, FirmwareEvent) and self._status is not None:
            if message.name == "ARM":
                self._status = replace(self._status, armed=message.value == "1")
            elif message.name == "LINK":
                self._status = replace(self._status, link_ok=message.value == "1")

    def _write(self, text: str) -> None:
        raise NotImplementedError

    def _read_lines(self) -> list[str]:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Real hardware
# ---------------------------------------------------------------------------


class SerialLink(TurretLink):
    """USB serial connection to the ESP32.

    Use SerialLink.open(port, baud); the constructor takes an already open
    pyserial-like port so tests can pass a fake one.
    """

    def __init__(self, port: serial.SerialBase) -> None:
        super().__init__()
        self._serial = port
        self._pending = bytearray()

    @classmethod
    def open(cls, port: str, baud: int = 115200) -> SerialLink:
        handle = serial.Serial()
        handle.port = port
        handle.baudrate = baud
        handle.timeout = 0  # reads never block
        handle.write_timeout = 0.2
        # Keep DTR and RTS released so opening the port does not hold the
        # ESP32 in reset through the board's auto-reset circuit.
        handle.dtr = False
        handle.rts = False
        handle.open()
        return cls(handle)

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def _write(self, text: str) -> None:
        self._serial.write(text.encode("ascii"))

    def _read_lines(self) -> list[str]:
        waiting = self._serial.in_waiting
        if waiting:
            self._pending.extend(self._serial.read(waiting))
        lines = []
        while (end := self._pending.find(b"\n")) >= 0:
            raw = bytes(self._pending[:end])
            del self._pending[: end + 1]
            lines.append(raw.decode("ascii", errors="replace").strip())
        if len(self._pending) > _MAX_PENDING_BYTES:
            self._pending.clear()
        return lines
