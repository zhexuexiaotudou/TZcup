from __future__ import annotations

import ast
from copy import deepcopy
import json
from pathlib import Path

from collect_formal_service_door_runtime import (
    _load_joint_velocity_limits,
    _parse_plugin_diagnostics,
    _plugin_target_echo_status,
    _phase_duration_from_targets,
)
from collect_formal_service_door_gz_sidecar import (
    _joint_names_from_model_text,
    _publisher_count_from_topic_info,
)
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
    snapshot = json.loads(
        (ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    evidence = {
        "source_binding": {
            "expanded_urdf_sha256": snapshot["outputs"][
                "reports/engineering/formal_competition_vehicle.urdf"
            ]["sha256"]
        },
        "evidence_authority": "GAZEBO_MODEL_JOINT_STATE_BRIDGE",
        "physical_joint_state_topic": "/formal/service_door_joint_states",
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
    expected_joints = {joint for spec in DOORS.values() for joint in spec[:2]}
    limits = {joint: 0.5 if "hinge" in joint else 1.0 for joint in expected_joints}
    evidence["urdf_velocity_limits_rad_per_s"] = limits
    evidence["phase_timing_contract"] = {
        "minimum_duration_s": 2.5,
        "settling_margin_s": 1.0,
        "minimum_fresh_samples": 5,
    }
    evidence["gazebo_partition"] = "tzcup_test_partition"
    evidence["gazebo_joint_state_sidecar"] = {
        "status": "PASSED",
        "gz_partition": "tzcup_test_partition",
        "gazebo_transport_topic": "/formal_vehicle/evaluation/bodywork_service/joint_states",
        "discovered_topic": "/formal_vehicle/evaluation/bodywork_service/joint_states",
        "topic_candidates": ["/formal_vehicle/evaluation/bodywork_service/joint_states"],
        "message_type": "gz.msgs.Model",
        "publisher_count": 1,
        "observed_joint_names": sorted(expected_joints),
        "topic_list_output": "/formal_vehicle/evaluation/bodywork_service/joint_states\n",
        "topic_info_output": "Message Type: gz.msgs.Model\nPublishers:\n  tcp://producer\n",
        "topic_sample_output": "".join(
            f'joint {{\n  name: "{joint}"\n}}\n' for joint in sorted(expected_joints)
        ),
        "launcher_pid": 123,
        "launcher_liveness_checks": [True, True],
    }
    evidence["target_transport"] = {
        "fresh_complete_samples_after_ready": [
            {"sim_clock_ns": index, "positions_rad": _positions(0.0, 0.0)}
            for index in range(1, 7)
        ]
    }
    for phase_index, phase_name in enumerate(evidence["phases"], start=1):
        phase = evidence["phases"][phase_name]
        phase["simulated_duration_requested_s"] = 4.0
        phase["sim_clock_start_ns"] = phase_index * 10_000_000_000
        phase["sim_clock_end_ns"] = phase["sim_clock_start_ns"] + 4_000_000_000
        phase["plugin_target_echo"] = {
            door: {
                kind: {
                    "received_messages": float(phase_index),
                    "previous_received_messages": float(phase_index - 1),
                    "received_target_rad": target,
                    "delivered_after_previous_generation": True,
                }
                for kind, target in row.items()
            }
            for door, row in phase["commanded_targets_rad"].items()
        }
        phase["fresh_complete_samples_after_plugin_echo"] = [
            {"sim_clock_ns": phase["sim_clock_start_ns"] + index, "positions_rad": _positions(0.0, 0.0)}
            for index in range(1, 7)
        ]
    return evidence


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


def test_delivery_timing_and_independent_gazebo_sidecar_fail_closed() -> None:
    def delivery_fails(mutate) -> None:
        evidence = passing_evidence()
        mutate(evidence)
        assert not evaluate(evidence)["checks"]["startup_and_phase_delivery_use_fresh_advancing_simulated_time"]

    delivery_fails(lambda e: e["urdf_velocity_limits_rad_per_s"].pop(
        "bodywork_power_service_door_hinge_joint"))
    delivery_fails(lambda e: e["urdf_velocity_limits_rad_per_s"].update(
        bodywork_power_service_door_hinge_joint=1.0e9))
    delivery_fails(lambda e: e["phases"]["open"].update(
        sim_clock_end_ns=e["phases"]["open"]["sim_clock_start_ns"] + 1))
    delivery_fails(lambda e: e["phases"]["open"]["plugin_target_echo"]["power"]["hinge"].update(
        delivered_after_previous_generation=False))
    delivery_fails(lambda e: e["phases"]["open"]["fresh_complete_samples_after_plugin_echo"].__setitem__(
        2, dict(e["phases"]["open"]["fresh_complete_samples_after_plugin_echo"][1])))
    delivery_fails(lambda e: e["target_transport"]["fresh_complete_samples_after_ready"].__setitem__(
        2, dict(e["target_transport"]["fresh_complete_samples_after_ready"][1])))

    for key, value in (
        ("publisher_count", 0), ("publisher_count", 2),
        ("message_type", "gz.msgs.String"), ("observed_joint_names", []),
        ("gz_partition", "wrong_partition"),
    ):
        evidence = passing_evidence()
        evidence["gazebo_joint_state_sidecar"][key] = value
        assert not evaluate(evidence)["checks"]["independent_gazebo_joint_state_sidecar_is_complete"]
    evidence = passing_evidence()
    evidence["gazebo_joint_state_sidecar"]["observed_joint_names"][-1] = (
        evidence["gazebo_joint_state_sidecar"]["observed_joint_names"][0]
    )
    assert not evaluate(evidence)["checks"]["independent_gazebo_joint_state_sidecar_is_complete"]
    evidence = passing_evidence()
    evidence["gazebo_joint_state_sidecar"]["topic_sample_output"] += 'joint {\n  name: "extra_joint"\n}\n'
    assert not evaluate(evidence)["checks"]["independent_gazebo_joint_state_sidecar_is_complete"]


def test_gazebo_model_sidecar_parses_all_joint_names_and_retains_duplicates() -> None:
    model = '''name: "formal_vehicle"
joint {
  name: "hinge_a"
}
joint {
  name: "hinge_a"
}
joint {
  name: "unexpected_joint"
}
'''
    assert _joint_names_from_model_text(model) == ["hinge_a", "hinge_a", "unexpected_joint"]


def test_gazebo_topic_info_publisher_section_requires_exactly_one_endpoint() -> None:
    fixture = """Topic: /formal_vehicle/evaluation/bodywork_service/joint_states
Message Type: gz.msgs.Model
Publishers:
  tcp://172.17.0.2:42513
Subscribers:
  tcp://172.17.0.2:33877
"""
    assert _publisher_count_from_topic_info(fixture) == 1
    assert _publisher_count_from_topic_info(fixture.replace("Subscribers:", "  tcp://172.17.0.3:42514\nSubscribers:")) == 2


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


def test_phase_duration_covers_urdf_limited_target_travel() -> None:
    zero = _targets(False, 0.0)
    open_unlocked = _targets(True, 0.6)
    velocity_limits = {
        joint: 0.5 if "hinge" in joint else 1.0
        for spec in DOORS.values()
        for joint in spec[:2]
    }
    # The 1.2-rad hinge command at 0.5 rad/s needs 2.4 simulated seconds;
    # the dwell cannot shrink to the former wall-clock default.
    assert _phase_duration_from_targets(
        zero, open_unlocked, velocity_limits, 2.5, 1.0
    ) == 3.4


def test_phase_duration_rejects_missing_velocity_limit() -> None:
    zero = _targets(False, 0.0)
    limits = {
        joint: 0.5
        for spec in DOORS.values()
        for joint in spec[:2]
        if joint != "bodywork_power_service_door_hinge_joint"
    }
    try:
        _phase_duration_from_targets(zero, _targets(True, 0.6), limits, 2.5, 1.0)
    except ValueError as exc:
        assert "missing usable velocity limit" in str(exc)
    else:
        raise AssertionError("missing URDF velocity limit must fail closed")


def test_phase_requires_a_new_plugin_target_echo_for_all_eight_commands() -> None:
    targets = _targets(True, 0.6)
    baseline = {door: {"hinge": 10.0, "latch": 11.0} for door in DOORS}
    records = [
        {
            "door": door,
            "received_hinge_messages": 11.0,
            "received_latch_messages": 12.0,
            "received_hinge_target_rad": target["hinge"],
            "received_latch_target_rad": target["latch"],
        }
        for door, target in targets.items()
    ]
    ready, detail = _plugin_target_echo_status(records, targets, baseline)
    assert ready is True
    assert all(
        item[axis]["delivered_after_previous_generation"]
        for item in detail.values()
        for axis in ("hinge", "latch")
    )
    records[-1]["received_hinge_messages"] = 10.0
    ready, detail = _plugin_target_echo_status(records, targets, baseline)
    assert ready is False
    assert detail["rear_dry"]["hinge"]["delivered_after_previous_generation"] is False


def test_runtime_path_passes_echo_and_freshness_to_every_phase() -> None:
    tree = ast.parse(
        (ROOT / "scripts/collect_formal_service_door_runtime.py").read_text(encoding="utf-8")
    )
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "node"
        and node.func.attr == "phase"
    ]
    assert len(calls) == 1
    call = calls[0]
    assert len(call.args) == 5
    assert isinstance(call.args[3], ast.Name) and call.args[3].id == "minimum_fresh_samples"
    assert isinstance(call.args[4], ast.Name) and call.args[4].id == "plugin_diagnostic_log"


def test_velocity_limits_are_loaded_from_bound_expanded_urdf() -> None:
    limits = _load_joint_velocity_limits(
        ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    )
    assert set(limits) == {joint for spec in DOORS.values() for joint in spec[:2]}
    assert all(limit > 0.0 for limit in limits.values())


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
    assert 'physical_joint_states_topic="/formal/service_door_joint_states"' in runner
    assert 'grep -Fxq /joint_states' not in runner
    assert '"service_door_evaluation_interfaces"' in launch
    assert '"service_door_evaluation_interfaces",\n                default_value="false"' in launch
    assert "condition=IfCondition(service_door_evaluation_interfaces)" in launch
    assert "sensor_msgs.msg import JointState" in collector
    assert 'PHYSICAL_JOINT_STATES_TOPIC = "/formal/service_door_joint_states"' in collector
    assert 'PHYSICAL_JOINT_STATE_AUTHORITY = "GAZEBO_MODEL_JOINT_STATE_BRIDGE"' in collector
    assert "JointState, PHYSICAL_JOINT_STATES_TOPIC" in collector
    assert 'JointState, "/joint_states"' not in collector
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
    assert "use_sim_time" in collector
    assert "wait_for_fresh_complete_samples" in collector
    assert "_phase_duration_from_targets" in collector
    assert "_load_joint_velocity_limits" in collector
    assert "simulated_duration_requested_s" in collector
    assert "plugin_target_echo" in collector
    assert "_plugin_target_echo_status" in collector
    assert "received_hinge_messages" in plugin
    assert "effective_hinge_rad" in plugin
    assert "hinge_force_nm" in plugin
    assert "PostUpdate" in plugin
    assert "postupdate_hinge_force_nm" in plugin
    assert "JointForceCmd" in plugin
    assert "SERVICE_DOOR_LIFECYCLE" in plugin
    assert "std::cerr" in plugin
    assert "self.publishers =" not in collector
    assert 'name="formal_service_door_physical_state_bridge"' in launch
    assert '"/formal_vehicle/evaluation/bodywork_service/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"' in launch
    assert '"/formal_vehicle/evaluation/bodywork_service/joint_states",' in launch
    assert '"/formal/service_door_joint_states",' in launch
