import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location(
    "capture_localization",
    Path(__file__).with_name("capture_competition_localization_parameters.py"),
)
capture_localization = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_localization)


def _runner(command, **kwargs):
    expected = capture_localization.EXPECTED_PARAMETERS[command[3]][command[4]]
    return SimpleNamespace(
        returncode=0,
        stdout=f"value is: {str(expected).lower() if isinstance(expected, bool) else expected}",
        stderr="",
    )


def test_parse_parameter_value_preserves_expected_type():
    assert capture_localization.parse_parameter_value("Boolean value is: True", True) is True
    assert capture_localization.parse_parameter_value("Double value is: 50.0", 50.0) == 50.0
    assert capture_localization.parse_parameter_value('String value is: "map"', "map") == "map"
    with pytest.raises(ValueError, match="boolean"):
        capture_localization.parse_parameter_value("Integer value is: 1", False)


def test_capture_requires_every_expected_parameter():
    report = capture_localization.capture(_runner, timeout_sec=1.0)
    assert report["schema_version"] == 1
    assert report["all_expected"] is True
    assert set(report["nodes"]) == set(capture_localization.EXPECTED_PARAMETERS)
    assert report["nodes"]["/global_ekf"]["world_frame"]["actual"] == "map"
    assert report["nodes"]["/amcl"]["tf_broadcast"]["actual"] is False


def test_capture_fails_closed_on_one_mismatch():
    def wrong_runner(command, **kwargs):
        if command[:3] == ["ros2", "param", "get"] and command[4] == "tf_broadcast":
            return SimpleNamespace(returncode=0, stdout="value is: True", stderr="")
        return _runner(command, **kwargs)

    report = capture_localization.capture(wrong_runner, timeout_sec=1.0)
    assert report["all_expected"] is False
    assert report["nodes"]["/amcl"]["tf_broadcast"]["matched"] is False


def test_capture_retains_command_failure(tmp_path):
    def failed_runner(command, **kwargs):
        return SimpleNamespace(returncode=1, stdout="", stderr="service unavailable")

    report = capture_localization.capture(failed_runner, timeout_sec=1.0)
    assert report["all_expected"] is False
    value = report["nodes"]["/global_ekf"]["world_frame"]
    assert value["actual"] is None
    assert value["error"] == "service unavailable"

    output = tmp_path / "effective-parameters.json"
    output.write_text(json.dumps(report), encoding="utf-8")
    assert json.loads(output.read_text(encoding="utf-8"))["all_expected"] is False
