import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location(
    "capture_localization",
    Path(__file__).with_name("capture_competition_localization_parameters.py"),
)
capture_localization = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_localization)


def _runner(command, **kwargs):
    expected = capture_localization.EXPECTED_PARAMETERS[command[-2]][command[-1]]
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
        if command[:3] == ["ros2", "param", "get"] and command[-1] == "tf_broadcast":
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
    assert value["attempt_count"] == 3

    output = tmp_path / "effective-parameters.json"
    output.write_text(json.dumps(report), encoding="utf-8")
    assert json.loads(output.read_text(encoding="utf-8"))["all_expected"] is False


def test_capture_retries_transient_parameter_timeout():
    calls = {}

    def transient_runner(command, **kwargs):
        key = (command[-2], command[-1])
        calls[key] = calls.get(key, 0) + 1
        if key == ("/amcl", "global_frame_id") and calls[key] == 1:
            raise subprocess.TimeoutExpired(command, kwargs["timeout"])
        return _runner(command, **kwargs)

    report = capture_localization.capture(
        transient_runner,
        timeout_sec=1.0,
        attempts=3,
        sleeper=lambda _: None,
    )
    value = report["nodes"]["/amcl"]["global_frame_id"]
    assert report["all_expected"] is True
    assert value["actual"] == "map"
    assert value["attempt_count"] == 2
    assert value["attempts"][0]["error"].startswith("TimeoutExpired:")


def test_capture_uses_bounded_discovery_and_timeout_options():
    commands = []

    def observing_runner(command, **kwargs):
        commands.append(command)
        return _runner(command, **kwargs)

    report = capture_localization.capture(
        observing_runner,
        timeout_sec=1.0,
        attempts=1,
        spin_time_sec=3.0,
        discovery_timeout_sec=7,
        sleeper=lambda _: None,
    )
    assert report["all_expected"] is True
    assert commands
    assert commands[0][:3] == ["ros2", "param", "get"]
    assert commands[0][3:5] == ["--spin-time", "3.0"]
    assert commands[0][5:7] == ["--timeout", "7"]


def test_stabilizer_owner_adds_exact_runtime_parameter_contract():
    expected_parameters = capture_localization.expected_parameters_for_owner(
        "/map_odom_stabilizer"
    )
    assert expected_parameters["/map_odom_stabilizer"] == {
        "tau_sec": 1.5,
        "max_filter_dt_sec": 0.1,
        "max_gap_sec": 0.5,
        "input_tf_topic": "/localization/raw_map_odom",
    }
    def stabilizer_runner(command, **kwargs):
        expected = expected_parameters[command[-2]][command[-1]]
        return SimpleNamespace(
            returncode=0,
            stdout=(
                f"value is: {str(expected).lower() if isinstance(expected, bool) else expected}"
            ),
            stderr="",
        )

    report = capture_localization.capture(
        stabilizer_runner,
        expected_parameters=expected_parameters,
        timeout_sec=1.0,
    )
    assert report["all_expected"] is True
    assert set(report["nodes"]) == set(expected_parameters)
