from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from collect_formal_service_door_runtime import _parse_plugin_diagnostics
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
        "ros_publisher_subscription_counts": {
            f"{door}_{kind}": 1 for door in DOORS for kind in ("hinge", "latch")
        },
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
        "plugin_diagnostics": {
            "lifecycle": [
                {
                    "event": "subscription",
                    "door": door,
                    "hinge_subscribed": 1.0,
                    "latch_subscribed": 1.0,
                }
                for door in DOORS
            ]
            + [{"event": "configured", "configured": 1.0, "doors": 4.0}],
            "records": [
                {
                    "door": door,
                    "sim_time_sec": float(sample + 1),
                    "received_hinge_messages": float(sample + 1),
                    "received_latch_messages": float(sample + 1),
                    "received_hinge_target_rad": spec[4],
                    "received_latch_target_rad": 0.6,
                    "requested_hinge_rad": spec[4],
                    "requested_latch_rad": 0.6,
                    "hinge_force_writes": float(sample + 1),
                    "latch_force_writes": float(sample + 1),
                    "effective_latch_rad": 0.6,
                    "effective_hinge_rad": spec[4],
                    "hinge_position_rad": 0.0,
                    "latch_position_rad": 0.0,
                    "hinge_force_nm": 0.0,
                    "latch_force_nm": 0.0,
                    "postupdate_hinge_force_present": 1.0,
                    "postupdate_latch_force_present": 1.0,
                    "postupdate_hinge_force_nm": 0.0,
                    "postupdate_latch_force_nm": 0.0,
                }
                for door, spec in DOORS.items()
                for sample in range(3)
            ]
        },
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


def test_missing_evaluator_bridge_subscriber_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["phases"]["open"]["ros_publisher_subscription_counts"]["power_hinge"] = 0
    report = evaluate(evidence)
    assert not report["checks"]["all_target_publishers_have_ros_bridge_subscribers"]


def test_missing_plugin_telemetry_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["plugin_diagnostics"] = {"records": []}
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_missing_plugin_lifecycle_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["plugin_diagnostics"]["lifecycle"] = []
    report = evaluate(evidence)
    assert not report["checks"]["plugin_lifecycle_configuration_observed"]


def test_malformed_plugin_lifecycle_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["plugin_diagnostics"]["lifecycle"][0]["hinge_subscribed"] = "not-a-number"
    report = evaluate(evidence)
    assert not report["checks"]["plugin_lifecycle_configuration_observed"]


def test_nonfinite_plugin_numeric_field_is_rejected(tmp_path: Path) -> None:
    log = tmp_path / "launch.log"
    log.write_text(
        "[gazebo-1] SERVICE_DOOR_DIAGNOSTIC door=power "
        "hinge_force_nm=nan\n",
        encoding="utf-8",
    )
    assert "invalid_numeric_field" in _parse_plugin_diagnostics(log)["parse_error"]


def test_nonfinite_injected_plugin_numeric_fails_closed() -> None:
    evidence = passing_evidence()
    evidence["plugin_diagnostics"]["records"][0]["hinge_force_nm"] = float("nan")
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_boolean_lifecycle_value_is_rejected(tmp_path: Path) -> None:
    log = tmp_path / "launch.log"
    log.write_text(
        "[gazebo-1] SERVICE_DOOR_LIFECYCLE event=subscription door=power "
        "hinge_subscribed=true latch_subscribed=true\n",
        encoding="utf-8",
    )
    assert "invalid_lifecycle_numeric_field" in _parse_plugin_diagnostics(log)["parse_error"]


def test_duplicate_plugin_field_is_rejected(tmp_path: Path) -> None:
    log = tmp_path / "launch.log"
    log.write_text(
        "SERVICE_DOOR_DIAGNOSTIC door=power sim_time_sec=1 sim_time_sec=2\n",
        encoding="utf-8",
    )
    assert "invalid_field:duplicate_field" == _parse_plugin_diagnostics(log)["parse_error"]


def test_plugin_counter_rollback_or_duplicate_timestamp_fails_closed() -> None:
    evidence = passing_evidence()
    records = evidence["plugin_diagnostics"]["records"]
    records[1]["hinge_force_writes"] = 0.0
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]
    evidence = passing_evidence()
    records = evidence["plugin_diagnostics"]["records"]
    records[1]["sim_time_sec"] = records[0]["sim_time_sec"]
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_incomplete_plugin_postupdate_telemetry_fails_closed() -> None:
    evidence = passing_evidence()
    del evidence["plugin_diagnostics"]["records"][0]["postupdate_hinge_force_nm"]
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_nonfinite_or_absent_postupdate_readback_fails_closed() -> None:
    evidence = passing_evidence()
    for record in evidence["plugin_diagnostics"]["records"]:
        record["postupdate_hinge_force_present"] = 0.0
        record["postupdate_latch_force_nm"] = float("nan")
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_plugin_telemetry_without_unlock_and_open_coverage_fails_closed() -> None:
    evidence = passing_evidence()
    for record in evidence["plugin_diagnostics"]["records"]:
        record["effective_latch_rad"] = 0.0
        record["effective_hinge_rad"] = 0.0
    report = evaluate(evidence)
    assert not report["checks"]["plugin_reports_received_targets_and_force_writes"]


def test_plugin_diagnostic_parser_retains_postupdate_observation(tmp_path: Path) -> None:
    log = tmp_path / "launch.log"
    log.write_text(
        "[gazebo-1] [INFO] SERVICE_DOOR_LIFECYCLE event=subscription door=power "
        "hinge_subscribed=1 latch_subscribed=1\n"
        "[Msg] SERVICE_DOOR_DIAGNOSTIC door=power sim_time_sec=1 "
        "received_hinge_messages=2 received_latch_messages=2 "
        "hinge_force_writes=3 latch_force_writes=3 "
        "postupdate_hinge_force_present=1 postupdate_hinge_force_nm=4\n",
        encoding="utf-8",
    )
    parsed = _parse_plugin_diagnostics(log)
    assert parsed["lifecycle"] == [
        {
            "event": "subscription",
            "door": "power",
            "hinge_subscribed": 1.0,
            "latch_subscribed": 1.0,
        }
    ]
    assert parsed["records"] == [
        {
            "door": "power",
            "sim_time_sec": 1.0,
            "received_hinge_messages": 2.0,
            "received_latch_messages": 2.0,
            "hinge_force_writes": 3.0,
            "latch_force_writes": 3.0,
            "postupdate_hinge_force_present": 1.0,
            "postupdate_hinge_force_nm": 4.0,
        }
    ]


def test_plugin_diagnostic_parser_rejects_malformed_numeric_field(tmp_path: Path) -> None:
    log = tmp_path / "launch.log"
    log.write_text(
        "SERVICE_DOOR_DIAGNOSTIC door=power hinge_force_nm=not-a-number\n",
        encoding="utf-8",
    )
    assert "invalid_numeric_field" in _parse_plugin_diagnostics(log)["parse_error"]


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
    assert "--plugin-diagnostic-log \"${log}\"" in runner
    assert "def _bound_runtime_evidence(" in collector
    assert "session_manifest_sha256" in collector
    assert "self.target_publishers" in collector
    assert "target_subscription_counts" in collector
    assert "received_hinge_messages" in plugin
    assert "effective_hinge_rad" in plugin
    assert "hinge_force_nm" in plugin
    assert "PostUpdate" in plugin
    assert "postupdate_hinge_force_nm" in plugin
    assert "JointForceCmd" in plugin
    assert "SERVICE_DOOR_LIFECYCLE" in plugin
    assert "std::cerr" in plugin
    assert "self.publishers =" not in collector
