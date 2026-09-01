#!/usr/bin/env python3
"""Fail-closed validator for physical cleaning-motor runtime evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding


MOTOR_NAMES = (
    "left_side_brush",
    "right_side_brush",
    "central_roller",
    "cleaning_lift",
    "recovery_pump",
)
RATED_CURRENT_A = (0.75, 0.75, 0.75, 0.50, 6.0)
STALL_CURRENT_A = (3.0, 3.0, 3.0, 1.0, 10.0)
LIFT_INDEX = 3
VECTOR_TOPIC_SUFFIXES = (
    "/motor_current_a",
    "/motor_temperature_c",
    "/estimated_output_load",
)
VECTOR_TOPIC_ROOT = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors"
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    """Recompute the immutable vehicle identity used by the live collector."""

    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _active_frozen_overlay(runtime_install_root: str) -> dict[str, str]:
    """Require this validator to resolve the vehicle package from the sidecar overlay."""

    expected = Path(runtime_install_root).resolve()
    if not (expected / "setup.bash").is_file():
        raise ValueError("runtime binding install root has no setup.bash")
    prefixes: list[Path] = []
    for raw in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if raw:
            prefixes.append(Path(raw).resolve())
    if expected not in prefixes:
        raise ValueError("active AMENT_PREFIX_PATH omits the frozen runtime install root")
    package_marker = Path("share/ament_index/resource_index/packages/sanitation_vehicle_description")
    selected = next((prefix for prefix in prefixes if (prefix / package_marker).is_file()), None)
    if selected != expected:
        raise ValueError(
            "sanitation_vehicle_description does not resolve from the frozen runtime overlay"
        )
    return {
        "runtime_install_root": str(expected),
        "selected_vehicle_description_prefix": str(selected),
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object], dict[str, str]]:
    """Cross-check snapshot, active session, closure sidecar, and live overlay."""

    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding["acceptance_session_binding"]
    closure = binding["runtime_closure_binding"]
    if not isinstance(bound_session, dict) or not isinstance(closure, dict):
        raise ValueError("runtime binding is incomplete")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from cleaning-motor source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
        or Path(str(bound_session.get("session_manifest", ""))).resolve()
        != session_path.resolve()
    ):
        raise ValueError("runtime binding session differs from cleaning-motor session")
    if bound_session.get("snapshot_current_source_verified") is not True:
        raise ValueError("runtime binding did not verify current snapshot sources")
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise ValueError("runtime binding closure is not verified")
    install_root = closure.get("runtime_install_root")
    if not isinstance(install_root, str) or not install_root:
        raise ValueError("runtime binding has no frozen runtime install root")
    return source_binding, bound_session, binding, _active_frozen_overlay(install_root)


def _phase(samples: list[dict[str, Any]], name: str) -> list[dict[str, Any]]:
    return [sample for sample in samples if sample.get("phase") == name]


def _motor_rows(sample: dict[str, Any]) -> list[dict[str, Any]]:
    rows = sample.get("status", {}).get("motors", [])
    return rows if isinstance(rows, list) else []


def _valid_vector_sample(sample: dict[str, Any]) -> bool:
    currents = sample.get("current_a", [])
    temperatures = sample.get("temperature_c", [])
    loads = sample.get("output_load", [])
    rows = _motor_rows(sample)
    return (
        isinstance(currents, list)
        and isinstance(temperatures, list)
        and len(currents) == len(temperatures) == len(loads) == len(rows) == 5
        and [row.get("name") for row in rows] == list(MOTOR_NAMES)
        and all(
            math.isfinite(float(value))
            for value in currents + temperatures + loads
        )
    )


def _single_writer_bridge_graph(artifact: dict[str, Any]) -> bool:
    graph = artifact.get("cleaning_vector_bridge_graph", {})
    if not isinstance(graph, dict):
        return False
    for suffix in VECTOR_TOPIC_SUFFIXES:
        endpoint = graph.get(VECTOR_TOPIC_ROOT + suffix, {})
        publishers = endpoint.get("publishers", [])
        if (
            endpoint.get("publisher_count") != 1
            or endpoint.get("ros_subscription_count", 0) < 1
            or not isinstance(publishers, list)
            or len(publishers) != 1
            or publishers[0].get("node_name") != "cleaning_actuator_motor_bridge"
            or publishers[0].get("topic_type")
            != "std_msgs/msg/Float64MultiArray"
        ):
            return False
    return True


def _normal_sample(sample: dict[str, Any]) -> bool:
    if not _valid_vector_sample(sample):
        return False
    currents = [float(value) for value in sample["current_a"]]
    return (
        sample.get("safety_enabled") is True
        and sample.get("fault") is False
        and all(value > 1.0e-4 for value in currents)
        and all(value <= rated + 1.0e-6 for value, rated in zip(currents, RATED_CURRENT_A))
        and all(row.get("fault") == "none" for row in _motor_rows(sample))
    )


def _physical_stall_sample(sample: dict[str, Any]) -> bool:
    if not _valid_vector_sample(sample):
        return False
    reference = float(sample.get("lift_reference_m", math.nan))
    position = float(sample.get("lift_position_m", math.nan))
    velocity = float(sample.get("lift_velocity_m_s", math.nan))
    lift = _motor_rows(sample)[LIFT_INDEX]
    return (
        reference >= 0.120
        and position <= 0.1005
        and reference - position >= 0.015
        and abs(velocity) <= 0.0003
        and float(sample["current_a"][LIFT_INDEX]) >= 0.95 * STALL_CURRENT_A[LIFT_INDEX]
        and lift.get("current_above_rating") is True
        and lift.get("fault") == "stall"
        and sample.get("fault") is True
    )


def _fault_globally_inhibited(sample: dict[str, Any]) -> bool:
    values = sample.get("whole_vehicle_safety_values", {})
    return (
        sample.get("fault") is True
        and sample.get("safety_enabled") is False
        and sample.get("whole_vehicle_safety_state") == "INHIBITED"
        and values.get("cleaning_motor_fault_active") == "true"
        and values.get("actuators_enabled") == "false"
    )


def _idle_cooling_observed(samples: list[dict[str, Any]]) -> bool:
    idle = [
        sample
        for sample in samples
        if _valid_vector_sample(sample)
        and all(abs(float(value)) <= 1.0e-6 for value in sample["current_a"])
        and sample.get("fault") is True
    ]
    if len(idle) < 2:
        return False
    first = float(idle[0]["temperature_c"][LIFT_INDEX])
    last = float(idle[-1]["temperature_c"][LIFT_INDEX])
    return first > last and last > 20.0


def _reset_recovered(sample: dict[str, Any]) -> bool:
    return (
        _valid_vector_sample(sample)
        and sample.get("fault") is False
        and sample.get("safety_enabled") is True
        and all(abs(float(value)) <= 1.0e-6 for value in sample["current_a"])
        and all(row.get("fault") == "none" for row in _motor_rows(sample))
    )


def validate(
    path: Path,
    *,
    snapshot_path: Path,
    session_path: Path,
    runtime_binding_path: Path,
) -> dict[str, Any]:
    artifact = json.loads(path.read_text(encoding="utf-8"))
    (
        source_binding,
        acceptance_session_binding,
        runtime_gate_binding,
        active_frozen_overlay,
    ) = _bound_runtime_evidence(snapshot_path, session_path, runtime_binding_path)
    if artifact.get("source_binding") != source_binding:
        raise ValueError("cleaning-motor capture source binding differs from active snapshot")
    samples = artifact.get("samples", [])
    if not isinstance(samples, list):
        samples = []
    vector_samples = [sample for sample in samples if _valid_vector_sample(sample)]
    normal = _phase(samples, "normal_load")
    stalled = _phase(samples, "physical_travel_stop_stall")
    cooling = _phase(samples, "idle_cooling")
    recovered = _phase(samples, "explicit_reset") + _phase(samples, "recovered_idle")

    bounded = bool(vector_samples) and all(
        all(
            0.0 <= float(value) <= STALL_CURRENT_A[index] + 1.0e-6
            for index, value in enumerate(sample["current_a"])
        )
        for sample in vector_samples
    )
    checks = {
        "capture_schema_v2": artifact.get("schema_version") == 2,
        "live_product_command_evidence": (
            artifact.get("evidence_authority")
            == "GAZEBO_PHYSICAL_JOINT_AND_POST_SAFETY_CONTROLLER_OBSERVATION"
            and artifact.get("joint_state_mutation_used") is False
            and artifact.get("production_motor_parameters_modified") is False
            and artifact.get("lift_trajectory_published") is True
        ),
        "scenario_completed_without_error": "scenario_error" not in artifact,
        "double_v_bridge_has_exact_type_direction_and_single_ros_writer": (
            _single_writer_bridge_graph(artifact)
        ),
        "five_named_motor_vectors_finite": bool(vector_samples),
        "all_currents_bounded_by_stall_or_fuse": bounded,
        "normal_load_all_five_nonzero_and_at_or_below_rating": any(
            _normal_sample(sample) for sample in normal
        ),
        "physical_lift_travel_stop_reached_stall_boundary": any(
            _physical_stall_sample(sample) for sample in stalled
        ),
        "stall_fault_caused_whole_vehicle_global_inhibit": any(
            _fault_globally_inhibited(sample) for sample in stalled + cooling
        ),
        "latched_fault_idle_cooling_observed": _idle_cooling_observed(cooling),
        "explicit_reset_was_published": int(artifact.get("reset_publish_count", 0)) > 0,
        "explicit_reset_restored_healthy_idle_and_global_permit": any(
            _reset_recovered(sample) for sample in recovered
        ),
        "live_overtemperature_not_claimed": (
            artifact.get("live_overtemperature_claimed") is False
            and artifact.get("thermal_protection_evidence", {}).get("kind")
            == "separate_core_unit_test"
        ),
    }
    failed = [name for name, passed in checks.items() if not passed]
    passed = not failed
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_cleaning_actuator_motor_runtime_acceptance_v1",
        "status": (
            "FORMAL_CLEANING_ACTUATOR_MOTOR_RUNTIME_PASSED"
            if passed
            else "FORMAL_CLEANING_ACTUATOR_MOTOR_RUNTIME_FAILED"
        ),
        "passed": passed,
        "checks": checks,
        "failed_checks": failed,
        "sample_count": len(samples),
        "source_binding": source_binding,
        "acceptance_session_binding": acceptance_session_binding,
        "runtime_gate_binding": runtime_gate_binding,
        "active_frozen_overlay": active_frozen_overlay,
        "thermal_evidence_boundary": {
            "live_overtemperature_tested": False,
            "core_overtemperature_test_required": True,
            "reason": (
                "The production motor thermal constants are not shortened. "
                "This live gate proves normal load, mechanical travel-stop stall, "
                "global inhibit, idle cooling and explicit reset only."
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    args = parser.parse_args()
    report = validate(
        args.artifact,
        snapshot_path=args.snapshot,
        session_path=args.session,
        runtime_binding_path=args.runtime_binding,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
