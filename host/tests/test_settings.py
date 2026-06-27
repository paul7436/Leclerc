import copy
import shutil
from pathlib import Path

import pytest

from settings import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    load_settings,
    parse_settings,
    save_calibration,
)


@pytest.fixture
def raw_config() -> dict:
    """The shipped config.yaml as plain Python data."""
    from ruamel.yaml import YAML

    return YAML(typ="safe").load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))


@pytest.fixture
def config_copy(tmp_path: Path) -> Path:
    target = tmp_path / "config.yaml"
    shutil.copy(DEFAULT_CONFIG_PATH, target)
    return target


_DELETE = object()  # marker: remove the key instead of setting it


def with_change(raw: dict, dotted_key: str, value) -> dict:
    changed = copy.deepcopy(raw)
    *parents, last = dotted_key.split(".")
    node = changed
    for key in parents:
        node = node[key]
    if value is _DELETE:
        del node[last]
    else:
        node[last] = value
    return changed


def test_shipped_config_is_valid():
    settings = load_settings()
    assert settings.serial.baud == 115200
    assert settings.aim.crosshair_px == (640.0, 360.0)
    assert settings.aim.pan.direction in (-1, 1)
    assert settings.fire_policy.lock_frames >= 1


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("serial", _DELETE, "missing section 'serial'"),
        ("aim.pan.deg_per_px", _DELETE, "'aim.pan' is missing: deg_per_px"),
        ("aim.pan.typo", 1, "unknown key(s) in 'aim.pan': typo"),
        ("bogus", {}, "unknown key(s) in 'config': bogus"),
        ("serial.status_period_s", 0.5, "keep the link alive"),
        ("serial.status_stale_s", 0.1, "status_stale_s"),
        ("detection.confidence", 1.5, "detection.confidence"),
        ("detection.selection", "random", "detection.selection"),
        ("detection.target_classes", ["balloon", "person"], "protected classes: person"),
        ("safety.extra_inhibit_classes", ["cup"], "both targets and inhibit classes: cup"),
        ("aim.crosshair_px", [640], "aim.crosshair_px"),
        ("aim.pan.direction", 0, "aim.pan.direction"),
        ("aim.tilt.min_deg", 150.0, "aim.tilt"),
        ("aim.tilt.home_deg", 10.0, "aim.tilt.home_deg"),
        ("pid.pan.kp", "fast", "'pid.pan.kp' must be a number"),
        ("pid.tilt.ki", -0.1, "must not be negative"),
        ("fire_policy.lock_frames", 2.5, "'fire_policy.lock_frames' must be an integer"),
        ("fire_policy.cooldown_s", 0.1, "fire_policy.cooldown_s"),
        ("ballistics.holdover_table", [[1.0, 0.5], [1.0, 0.7]], "unique"),
        ("ballistics.holdover_table", [[1.0]], "holdover_table[0]"),
    ],
)
def test_invalid_values_are_rejected(raw_config, key, value, message):
    with pytest.raises(ConfigError) as excinfo:
        parse_settings(with_change(raw_config, key, value))
    assert message in str(excinfo.value)


def test_holdover_table_is_sorted_by_distance(raw_config):
    raw = with_change(raw_config, "ballistics.holdover_table", [[3.0, 2.0], [1.0, 0.5]])
    assert parse_settings(raw).ballistics.holdover_table == [(1.0, 0.5), (3.0, 2.0)]


def test_missing_file_is_a_config_error(tmp_path):
    with pytest.raises(ConfigError):
        load_settings(tmp_path / "absent.yaml")


def test_save_calibration_updates_values_and_keeps_comments(config_copy):
    save_calibration(
        config_copy,
        crosshair_px=(612.4, 377.6),
        pan=(0.0512345, 1),
        tilt=(0.048, -1),
        holdover_table=[(2.5, 1.75), (1.0, 0.4)],
    )
    settings = load_settings(config_copy)
    assert settings.aim.crosshair_px == (612.0, 378.0)
    assert settings.aim.pan.deg_per_px == pytest.approx(0.05123)
    assert settings.aim.pan.direction == 1
    assert settings.aim.tilt.direction == -1
    assert settings.ballistics.holdover_table == [(1.0, 0.4), (2.5, 1.75)]

    text = config_copy.read_text(encoding="utf-8")
    assert "# [calibrated] pixel the barrel points at" in text
    assert "    - [1.0, 0.4]" in text
    assert 'device: ""' in text


def test_save_calibration_can_clear_the_holdover_table(config_copy):
    save_calibration(config_copy, holdover_table=[(1.0, 0.4)])
    save_calibration(config_copy, holdover_table=[])
    assert load_settings(config_copy).ballistics.holdover_table == []
    assert "holdover_table: []" in config_copy.read_text(encoding="utf-8")


def test_save_calibration_refuses_invalid_results(config_copy):
    before = config_copy.read_text(encoding="utf-8")
    with pytest.raises(ConfigError):
        save_calibration(config_copy, pan=(0.05, 0))
    assert config_copy.read_text(encoding="utf-8") == before
