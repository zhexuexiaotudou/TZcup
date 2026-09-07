from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_r065_w1_dynamic_footprint_live.sh"
OPERATOR = ROOT / "scripts" / "formal_w1_operator_safety.sh"
STATUS = ROOT / "scripts" / "formal_w1_operator_safety_status.py"


def _status_module():
    spec = importlib.util.spec_from_file_location("formal_w1_operator_safety_status", STATUS)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _envelope(tmp_path: Path, payload: object) -> Path:
    path = tmp_path / "status.yaml"
    encoded = json.dumps(payload, indent=2)
    wrapped = "\n".join(f"  {line}" for line in encoded.splitlines())
    path.write_text(
        f"data: |-\n{wrapped}\n---\n",
        encoding="utf-8",
    )
    return path


def test_status_parser_accepts_wrapped_string_and_rejects_malformed(tmp_path: Path) -> None:
    module = _status_module()
    path = _envelope(
        tmp_path,
        {
            "state": "BASE_COMMAND_STOPPED",
            "active_reasons": "manipulator_base_inhibit",
            "status_publish_count": 12,
        },
    )
    value = module.load_status(path)
    assert module.count(value) == 12

    path.write_text("data: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="not text"):
        module.load_status(path)
    path.write_text("data: '[1, 2]'\n", encoding="utf-8")
    with pytest.raises(ValueError, match="not an object"):
        module.load_status(path)
    path.write_text("data: '{}'\n---\ndata: '{}'\n", encoding="utf-8")
    with pytest.raises(ValueError, match="exactly one populated"):
        module.load_status(path)


def test_operator_sequence_is_fail_closed_and_has_no_control_or_truth_writer() -> None:
    operator = OPERATOR.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    for topic in (
        "/formal_vehicle/simulation/command/emergency_stop",
        "/formal_vehicle/simulation/command/emergency_stop_reset",
        "/formal_vehicle/simulation/command/main_power",
    ):
        assert topic in operator
    assert 'ros2 topic pub --rate "$FORMAL_W1_OPERATOR_RATE_HZ"' in operator
    assert "FORMAL_W1_OPERATOR_RATE_HZ=10" in operator
    assert "formal_w1_operator_require_sole_physical_publishers" in operator
    assert "R065_W1_OPERATOR_PUBLISHER_DISCOVERY_TIMEOUT_SECONDS" in operator
    assert 'operator-$(basename "$topic").topic-info.$attempt.txt' in operator
    assert 'kill -0 "$pid"' in operator
    assert 'timeout --signal=TERM --kill-after=1s "${remaining}s" ros2 topic info' in operator
    assert (
        "formal_w1_operator_wait_status post-release BASE_COMMAND_STOPPED "
        "manipulator_base_inhibit" in operator
    )
    assert "formal_w1_operator_wait_status teardown-estop INHIBITED manual_estop" in operator
    assert 'ros2 topic echo --once --full-length "$FORMAL_W1_OPERATOR_SAFETY"' in operator
    assert 'kill -TERM "$pid"' in operator
    assert 'kill -KILL "$pid"' in operator
    assert 'kill -KILL -- "-$pid"' not in operator
    assert operator.index("formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_ESTOP_FALSE_PID") < operator.index(
        'formal_w1_operator_start_publisher "$FORMAL_W1_OPERATOR_ESTOP" true'
    ) < operator.index("formal_w1_operator_stop_publisher FORMAL_W1_OPERATOR_POWER_PID")
    assert runner.index("formal_w1_operator_start_and_release") < runner.index(
        "formal-dynamic-footprint-runtime-gate"
    )
    assert "formal_w1_operator_teardown || operator_cleanup_status=$?" in runner
    for forbidden in ("/cmd_vel", "/joint_states", "/ground_truth", "/evaluation/"):
        assert forbidden not in operator


def test_runner_documents_only_the_allowed_operator_inputs() -> None:
    runner = RUNNER.read_text(encoding="utf-8")
    assert "The installed runtime gate is the only test participant that publishes" not in runner
    assert 'source "${repo_root}/scripts/formal_w1_operator_safety.sh"' in runner
    assert "no actuator, action, cmd_vel, joint-state, or" in runner
    assert "evaluator-truth writer" in runner
