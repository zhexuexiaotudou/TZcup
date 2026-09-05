from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from validate_formal_service_door_runtime import DOORS, FAILED_STATUS, PASSED_STATUS, evaluate


ROOT = Path(__file__).resolve().parents[1]


def _targets(open_hinge: bool, latch: float):
    return {
        door: {"hinge": spec[4] if open_hinge else 0.0, "latch": latch}
        for door, spec in DOORS.items()
    }


def _positions(hinge_scale: float, latch: float):
    return {
        joint: value
        for _, (hinge, latch_joint, _, _, target) in DOORS.items()
        for joint, value in ((hinge, target * hinge_scale), (latch_joint, latch))
    }


def _phase(command, positions, timestamp_offset):
    return {
        "commanded_targets_rad": command,
        "joint_state_samples": [
            {
                "received_monotonic_ns": timestamp_offset + index + 1,
                "positions_rad": dict(positions),
            }
            for index in range(6)
        ],
    }


def passing_evidence():
    return {
        "source_binding": {"expanded_urdf_sha256": "a" * 64},
        "evidence_authority": "GAZEBO_SENSOR_MSGS_JOINT_STATE",
        "phases": {
            "initial_locked": _phase(_targets(False, 0.0), _positions(0.0, 0.0), 0),
            "locked_open_rejected": _phase(_targets(True, 0.0), _positions(0.0, 0.0), 10),
            "unlocked": _phase(_targets(False, 0.6), _positions(0.0, 0.6), 20),
            "open": _phase(_targets(True, 0.6), _positions(1.0, 0.6), 30),
            "closed_unlocked": _phase(_targets(False, 0.6), _positions(0.0, 0.6), 40),
            "transport_locked": _phase(_targets(False, 0.0), _positions(0.0, 0.0), 50),
            "relock_open_rejected": _phase(_targets(True, 0.0), _positions(0.0, 0.0), 60),
        },
    }


def test_complete_physical_sequence_passes() -> None:
    report = evaluate(passing_evidence())
    assert report["passed"] is True
    assert report["status"] == PASSED_STATUS
    assert all(report["checks"].values())


def test_opening_while_locked_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["phases"]["locked_open_rejected"]["joint_state_samples"] = [
        {"received_monotonic_ns": index, "positions_rad": _positions(0.6, 0.0)}
        for index in range(6)
    ]
    report = evaluate(evidence)
    assert report["status"] == FAILED_STATUS
    assert not report["checks"]["locked_hinges_reject_open_command"]


def test_relabeling_commands_cannot_bypass_unlock_order() -> None:
    evidence = passing_evidence()
    evidence["phases"]["open"]["commanded_targets_rad"]["power"]["latch"] = 0.0
    report = evaluate(evidence)
    assert not report["checks"]["command_sequence_unlocks_before_opening"]


def test_missing_live_joint_samples_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["phases"]["open"]["joint_state_samples"] = []
    report = evaluate(evidence)
    assert not report["checks"]["all_phases_have_fresh_complete_samples"]


def test_reused_joint_samples_across_phases_fail_freshness_gate() -> None:
    evidence = passing_evidence()
    evidence["phases"]["open"]["joint_state_samples"][0][
        "received_monotonic_ns"
    ] = 21
    report = evaluate(evidence)
    assert not report["checks"]["joint_samples_are_strictly_ordered_across_phases"]


def test_joint_limit_violation_fails_closed() -> None:
    evidence = passing_evidence()
    bad = deepcopy(evidence["phases"]["open"]["joint_state_samples"][0])
    bad["positions_rad"]["bodywork_power_service_door_hinge_joint"] = 1.9
    evidence["phases"]["open"]["joint_state_samples"][0] = bad
    report = evaluate(evidence)
    assert not report["checks"]["all_samples_remain_inside_urdf_limits"]


def test_runner_collector_and_force_plugin_use_physical_joint_state() -> None:
    runner = (ROOT / "scripts/run_formal_service_door_runtime.sh").read_text(encoding="utf-8")
    collector = (ROOT / "scripts/collect_formal_service_door_runtime.py").read_text(encoding="utf-8")
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    plugin = (
        ROOT / "starter_ws/src/sanitation_gazebo_auxiliary/src/ServiceDoorSystem.cc"
    ).read_text(encoding="utf-8")
    assert "formal_vehicle_sim.launch.py" in runner
    assert "collect_formal_service_door_runtime.py" in runner
    assert "service_door_evaluation_interfaces:=true" in runner
    assert '"service_door_evaluation_interfaces"' in launch
    assert '"service_door_evaluation_interfaces",\n                default_value="false"' in launch
    assert "condition=IfCondition(service_door_evaluation_interfaces)" in launch
    assert "sensor_msgs.msg import JointState" in collector
    assert '"/joint_states"' in collector
    assert "components::JointPosition" in plugin
    assert "components::JointForceCmd" in plugin
    assert "measuredUnlocked" in plugin
    assert "JointPositionReset" not in plugin
    assert "--check --output \"${snapshot}\"" in runner
    assert "--session \"${session}\"" in runner
    assert "def _bound_runtime_evidence(" in collector
    assert "session_manifest_sha256" in collector
