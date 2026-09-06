import argparse
import json
import subprocess
from pathlib import Path

import collect_formal_safety_speed_readback as collector


def _binding(path: Path) -> Path:
    path.write_text(json.dumps({
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"},
    }), encoding="utf-8")
    return path


def _args(tmp_path: Path) -> argparse.Namespace:
    return argparse.Namespace(
        output=tmp_path / "readback.json",
        runtime_binding=_binding(tmp_path / "runtime_binding.json"),
        expected_cap=1.0,
        expected_profile="dry_cleaning_competition_candidate",
        expected_state="isolated_same_map_dry_coverage",
        timeout_sec=5.0,
    )


def _producer_stdout() -> str:
    return """Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
Endpoint type: PUBLISHER
Node name: whole_vehicle_safety_manager
Node namespace: /
"""


def test_collector_owns_successful_ros_capture(monkeypatch, tmp_path: Path) -> None:
    seen = []

    def fake_run(command, **kwargs):
        seen.append((command, kwargs))
        stdout = (
            json.dumps({
                "effective_max_linear_velocity_mps": 1.0,
                "operation_speed_profile": "dry_cleaning_competition_candidate",
                "speed_qualification_state": "isolated_same_map_dry_coverage",
            })
            if command[2] == "echo" else _producer_stdout()
        )
        return subprocess.CompletedProcess(command, 0, stdout=stdout, stderr="")

    monkeypatch.setattr(collector.subprocess, "run", fake_run)
    args = _args(tmp_path)
    receipt, passed = collector.collect(args)

    assert passed
    assert receipt["producer_identity"]["node_name"] == "whole_vehicle_safety_manager"
    assert receipt["status_capture"]["command"] == [
        "ros2", "topic", "echo", "--once", "--field", "data", "/safety/status_json"
    ]
    assert len(seen) == 2
    assert json.loads(args.output.read_text())["capture_status"] == "PASSED"


def test_collector_rejects_injected_unscoped_status(monkeypatch, tmp_path: Path) -> None:
    def fake_run(command, **kwargs):
        stdout = (
            json.dumps({
                "effective_max_linear_velocity_mps": 1.0,
                "operation_speed_profile": "dry_cleaning_competition_candidate",
                "speed_qualification_state": "none",
            })
            if command[2] == "echo" else _producer_stdout()
        )
        return subprocess.CompletedProcess(command, 0, stdout=stdout, stderr="")

    monkeypatch.setattr(collector.subprocess, "run", fake_run)
    receipt, passed = collector.collect(_args(tmp_path))

    assert not passed
    assert "qualification state" in receipt["error"]
    assert receipt["status_capture"]["returncode"] == 0
    assert "--raw" not in collector.__doc__


def test_collector_keeps_failed_ros_capture_receipt(monkeypatch, tmp_path: Path) -> None:
    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(command, 42, stdout="", stderr="graph unavailable")

    monkeypatch.setattr(collector.subprocess, "run", fake_run)
    args = _args(tmp_path)
    receipt, passed = collector.collect(args)

    assert not passed
    assert receipt["status_capture"]["returncode"] == 42
    assert receipt["status_capture"]["stderr"] == "graph unavailable"
    assert json.loads(args.output.read_text())["capture_status"] == "FAILED"
