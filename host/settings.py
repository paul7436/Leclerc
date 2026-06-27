"""Load, validate and update the host configuration file (config.yaml).

Loading turns the YAML document into typed dataclasses and rejects missing,
unknown or unsafe values with a ConfigError that names the offending key.
Saving only touches calibration values and keeps the file's comments.
"""

from __future__ import annotations

import dataclasses
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq

from fire_policy import MIN_COOLDOWN_S, PROTECTED_CLASSES

DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.yaml")
SELECTION_STRATEGIES = ("nearest", "confidence")

# The firmware link watchdog fires after 500 ms without a command; the status
# poll doubles as the keepalive, so it must run comfortably faster.
MAX_STATUS_PERIOD_S = 0.25


class ConfigError(ValueError):
    """config.yaml is missing a value or holds an invalid one."""


@dataclass
class SerialSettings:
    port: str
    baud: int = 115200
    status_period_s: float = 0.2
    status_stale_s: float = 0.6


@dataclass
class CameraSettings:
    source: int | str = 0
    width: int = 1280
    height: int = 720
    fps: int = 30


@dataclass
class DetectionSettings:
    model_path: str = "yolo11n.pt"
    confidence: float = 0.45
    image_size: int = 640
    device: str = ""
    target_classes: list[str] = field(default_factory=list)
    selection: str = "nearest"


@dataclass
class SafetySettings:
    guard_model_path: str = "yolo11n.pt"
    guard_confidence: float = 0.30
    extra_inhibit_classes: list[str] = field(default_factory=list)


@dataclass
class AxisSettings:
    deg_per_px: float
    direction: int
    min_deg: float
    max_deg: float
    home_deg: float


@dataclass
class AimSettings:
    crosshair_px: tuple[float, float]
    pan: AxisSettings
    tilt: AxisSettings
    deadband_px: float = 3.0
    manual_step_deg: float = 1.0
    manual_coarse_step_deg: float = 5.0


@dataclass
class PidGains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 5.0
    output_limit: float = 6.0


@dataclass
class PidSettings:
    pan: PidGains
    tilt: PidGains


@dataclass
class FirePolicySettings:
    error_threshold_px: float = 12.0
    lock_frames: int = 5
    cooldown_s: float = 2.0


@dataclass
class BallisticsSettings:
    target_height_m: float = 0.0
    holdover_table: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class Settings:
    serial: SerialSettings
    camera: CameraSettings
    detection: DetectionSettings
    safety: SafetySettings
    aim: AimSettings
    pid: PidSettings
    fire_policy: FirePolicySettings
    ballistics: BallisticsSettings


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def load_settings(path: str | Path = DEFAULT_CONFIG_PATH) -> Settings:
    """Reads and validates a configuration file."""
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc
    return parse_settings(_yaml().load(text))


def parse_settings(raw: Any) -> Settings:
    """Builds validated settings from an already parsed YAML document."""
    root = _mapping(raw, "config")
    _reject_unknown(root, {f.name for f in dataclasses.fields(Settings)}, "config")

    settings = Settings(
        serial=_build(SerialSettings, root.get("serial"), "serial"),
        camera=_build(CameraSettings, root.get("camera", {}), "camera"),
        detection=_build(DetectionSettings, root.get("detection", {}), "detection"),
        safety=_build(SafetySettings, root.get("safety", {}), "safety"),
        aim=_parse_aim(root.get("aim")),
        pid=_parse_pid(root.get("pid")),
        fire_policy=_build(FirePolicySettings, root.get("fire_policy", {}), "fire_policy"),
        ballistics=_parse_ballistics(root.get("ballistics", {})),
    )
    _validate(settings)
    return settings


def _parse_aim(raw: Any) -> AimSettings:
    section = dict(_mapping(raw, "aim"))
    pan = _build(AxisSettings, section.pop("pan", None), "aim.pan")
    tilt = _build(AxisSettings, section.pop("tilt", None), "aim.tilt")
    crosshair = _number_pair(section.pop("crosshair_px", None), "aim.crosshair_px")
    return _build(AimSettings, section, "aim", crosshair_px=crosshair, pan=pan, tilt=tilt)


def _parse_pid(raw: Any) -> PidSettings:
    section = _mapping(raw, "pid")
    _reject_unknown(section, {"pan", "tilt"}, "pid")
    return PidSettings(
        pan=_build(PidGains, section.get("pan"), "pid.pan"),
        tilt=_build(PidGains, section.get("tilt"), "pid.tilt"),
    )


def _parse_ballistics(raw: Any) -> BallisticsSettings:
    section = dict(_mapping(raw, "ballistics"))
    rows = section.pop("holdover_table", None) or []
    if not isinstance(rows, list):
        raise ConfigError("'ballistics.holdover_table' must be a list of pairs")
    table = [_number_pair(row, f"ballistics.holdover_table[{i}]") for i, row in enumerate(rows)]
    return _build(BallisticsSettings, section, "ballistics", holdover_table=sorted(table))


def _build(cls: type, raw: Any, name: str, **extra: Any) -> Any:
    section = _mapping(raw, name)
    fields = dataclasses.fields(cls)
    _reject_unknown(section, {f.name for f in fields} - set(extra), name)
    missing = [
        f.name
        for f in fields
        if f.name not in section
        and f.name not in extra
        and f.default is dataclasses.MISSING
        and f.default_factory is dataclasses.MISSING
    ]
    if missing:
        raise ConfigError(f"'{name}' is missing: {', '.join(missing)}")
    instance = cls(**section, **extra)
    _check_scalar_types(instance, name)
    return instance


def _check_scalar_types(instance: Any, name: str) -> None:
    """Rejects a value whose type does not match a float, int or str field."""
    for f in dataclasses.fields(instance):
        value = getattr(instance, f.name)
        if f.type == "float" and not _is_number(value):
            raise ConfigError(f"'{name}.{f.name}' must be a number")
        if f.type == "int" and not (isinstance(value, int) and not isinstance(value, bool)):
            raise ConfigError(f"'{name}.{f.name}' must be an integer")
        if f.type == "str" and not isinstance(value, str):
            raise ConfigError(f"'{name}.{f.name}' must be a string")
        if f.type == "list[str]" and not (
            isinstance(value, list) and all(isinstance(item, str) for item in value)
        ):
            raise ConfigError(f"'{name}.{f.name}' must be a list of names")


def _mapping(raw: Any, name: str) -> dict:
    if raw is None:
        raise ConfigError(f"missing section '{name}'")
    if not isinstance(raw, dict):
        raise ConfigError(f"'{name}' must be a mapping")
    return raw


def _reject_unknown(section: dict, allowed: set[str], name: str) -> None:
    unknown = sorted(set(section) - allowed)
    if unknown:
        raise ConfigError(f"unknown key(s) in '{name}': {', '.join(map(str, unknown))}")


def _number_pair(raw: Any, name: str) -> tuple[float, float]:
    if (
        not isinstance(raw, list | tuple)
        or len(raw) != 2
        or not all(_is_number(value) for value in raw)
    ):
        raise ConfigError(f"'{name}' must be a list of two numbers")
    return float(raw[0]), float(raw[1])


def _is_number(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ConfigError(message)


def _validate(settings: Settings) -> None:
    _validate_serial(settings.serial)
    _validate_detection(settings.detection, settings.safety)
    _validate_aim(settings.aim)
    for axis_name, gains in (("pan", settings.pid.pan), ("tilt", settings.pid.tilt)):
        _validate_pid(gains, f"pid.{axis_name}")
    _validate_fire_policy(settings.fire_policy)
    _validate_ballistics(settings.ballistics)


def _validate_serial(serial: SerialSettings) -> None:
    _require(bool(serial.port), "'serial.port' must not be empty")
    _require(
        0 < serial.status_period_s <= MAX_STATUS_PERIOD_S,
        f"'serial.status_period_s' must be in (0, {MAX_STATUS_PERIOD_S}] to keep the link alive",
    )
    _require(
        serial.status_stale_s > serial.status_period_s,
        "'serial.status_stale_s' must be longer than 'serial.status_period_s'",
    )


def _validate_detection(detection: DetectionSettings, safety: SafetySettings) -> None:
    _require(0 < detection.confidence < 1, "'detection.confidence' must be in (0, 1)")
    _require(0 < safety.guard_confidence < 1, "'safety.guard_confidence' must be in (0, 1)")
    _require(
        detection.selection in SELECTION_STRATEGIES,
        f"'detection.selection' must be one of {', '.join(SELECTION_STRATEGIES)}",
    )
    forbidden = sorted(PROTECTED_CLASSES.intersection(detection.target_classes))
    _require(
        not forbidden,
        f"'detection.target_classes' must not contain protected classes: {', '.join(forbidden)}",
    )
    overlap = sorted(set(safety.extra_inhibit_classes).intersection(detection.target_classes))
    _require(
        not overlap,
        f"classes cannot be both targets and inhibit classes: {', '.join(overlap)}",
    )


def _validate_aim(aim: AimSettings) -> None:
    _require(aim.deadband_px >= 0, "'aim.deadband_px' must not be negative")
    _require(aim.manual_step_deg > 0, "'aim.manual_step_deg' must be positive")
    _require(aim.manual_coarse_step_deg > 0, "'aim.manual_coarse_step_deg' must be positive")
    for name, axis in (("aim.pan", aim.pan), ("aim.tilt", aim.tilt)):
        _require(axis.deg_per_px > 0, f"'{name}.deg_per_px' must be positive")
        _require(axis.direction in (-1, 1), f"'{name}.direction' must be 1 or -1")
        _require(
            0 <= axis.min_deg < axis.max_deg <= 180,
            f"'{name}' limits must satisfy 0 <= min_deg < max_deg <= 180",
        )
        _require(
            axis.min_deg <= axis.home_deg <= axis.max_deg,
            f"'{name}.home_deg' must lie within its limits",
        )


def _validate_pid(gains: PidGains, name: str) -> None:
    _require(min(gains.kp, gains.ki, gains.kd) >= 0, f"'{name}' gains must not be negative")
    _require(gains.integral_limit > 0, f"'{name}.integral_limit' must be positive")
    _require(gains.output_limit > 0, f"'{name}.output_limit' must be positive")


def _validate_fire_policy(policy: FirePolicySettings) -> None:
    _require(policy.error_threshold_px > 0, "'fire_policy.error_threshold_px' must be positive")
    _require(policy.lock_frames >= 1, "'fire_policy.lock_frames' must be at least 1")
    _require(
        policy.cooldown_s >= MIN_COOLDOWN_S,
        f"'fire_policy.cooldown_s' must be at least {MIN_COOLDOWN_S}",
    )


def _validate_ballistics(ballistics: BallisticsSettings) -> None:
    _require(ballistics.target_height_m >= 0, "'ballistics.target_height_m' must not be negative")
    distances = [distance for distance, _ in ballistics.holdover_table]
    _require(all(d > 0 for d in distances), "holdover distances must be positive")
    _require(len(set(distances)) == len(distances), "holdover distances must be unique")


# ---------------------------------------------------------------------------
# Saving calibration results
# ---------------------------------------------------------------------------


def save_calibration(
    path: str | Path = DEFAULT_CONFIG_PATH,
    *,
    crosshair_px: tuple[float, float] | None = None,
    pan: tuple[float, int] | None = None,
    tilt: tuple[float, int] | None = None,
    holdover_table: list[tuple[float, float]] | None = None,
) -> None:
    """Writes calibration results into the config file, keeping comments.

    `pan` and `tilt` are (deg_per_px, direction) pairs. Values left as None
    are not touched. The result is validated before the file is replaced.
    """
    path = Path(path)
    yaml = _yaml()
    document = yaml.load(path.read_text(encoding="utf-8"))
    aim = document["aim"]

    if crosshair_px is not None:
        aim["crosshair_px"] = _replace_items(
            aim["crosshair_px"], [round(float(v)) for v in crosshair_px]
        )
    for name, values in (("pan", pan), ("tilt", tilt)):
        if values is not None:
            deg_per_px, direction = values
            aim[name]["deg_per_px"] = round(float(deg_per_px), 5)
            aim[name]["direction"] = int(direction)
    if holdover_table is not None:
        rows = [_flow_pair(round(d, 3), round(h, 3)) for d, h in sorted(holdover_table)]
        table = _replace_items(document["ballistics"]["holdover_table"], rows)
        if rows:
            table.fa.set_block_style()
        else:
            table.fa.set_flow_style()

    parse_settings(document)  # never write a file that would not load
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.dump(document, stream)
    os.replace(temporary, path)


def _replace_items(sequence: Any, items: list) -> CommentedSeq:
    """Replaces the items in place so the sequence keeps its comments."""
    if not isinstance(sequence, CommentedSeq):
        sequence = CommentedSeq()
    sequence[:] = items
    return sequence


def _flow_pair(first: float, second: float) -> CommentedSeq:
    pair = CommentedSeq([first, second])
    pair.fa.set_flow_style()
    return pair


def _yaml() -> YAML:
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    return yaml
