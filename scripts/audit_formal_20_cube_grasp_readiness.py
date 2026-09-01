#!/usr/bin/env python3
"""Static, Windows-safe preflight for the formal 20-cube grasp gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import runpy
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "reports/engineering/formal_20_cube_grasp_static_readiness.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(root: Path = ROOT) -> dict[str, Any]:
    prepare_path = root / "scripts/prepare_formal_20_cube_grasp_acceptance.py"
    runner_path = root / "scripts/run_formal_20_cube_grasp_acceptance.sh"
    validator_path = root / "scripts/validate_formal_20_cube_grasp_runtime.py"
    executor_path = (
        root
        / "starter_ws/src/sanitation_manipulation/sanitation_manipulation/formal_grasp_executor.py"
    )
    scene_path = (
        root
        / "starter_ws/src/sanitation_manipulation/sanitation_manipulation/formal_20_cube_scene.py"
    )
    cube_path = root / "starter_ws/src/sanitation_manipulation/urdf/material_cube.urdf.xacro"
    monitor_path = root / "starter_ws/src/sanitation_gazebo_control/src/DryBinMonitorSystem.cc"
    payload_path = root / "starter_ws/src/sanitation_gazebo_control/src/DynamicPayloadSystem.cc"
    contact_gate_path = root / "starter_ws/src/sanitation_gazebo_control/src/GripperContactGateSystem.cc"
    closure_path = root / "scripts/formal_final_runtime_closure.py"
    orchestrator_path = root / "scripts/run_formal_final_acceptance.py"
    layout_path = root / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
    vehicle_path = root / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    snapshot_tool_path = root / "scripts/generate_formal_vehicle_snapshot.py"
    launch_path = (
        root
        / "starter_ws/src/sanitation_manipulation/launch/formal_20_cube_pick_place.launch.py"
    )

    module = runpy.run_path(str(prepare_path))
    manifest = module["build_manifest"](6020)
    snapshot_module = runpy.run_path(str(snapshot_tool_path))
    snapshot_error = None
    try:
        snapshot_module["verify_snapshot"](root)
        snapshot_current = True
    except Exception as exc:  # Audit must report drift without aborting other checks.
        snapshot_current = False
        snapshot_error = str(exc)
    package_root = root / "starter_ws/src/sanitation_manipulation"
    sys.path.insert(0, str(package_root))
    try:
        from sanitation_manipulation.formal_20_cube_scene import load_scene_manifest

        temporary = root / ".work/formal_20_cube_static_manifest.json"
        temporary.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        _, specs = load_scene_manifest(temporary)
    finally:
        sys.path.pop(0)

    runner = runner_path.read_text(encoding="utf-8")
    validator = validator_path.read_text(encoding="utf-8")
    executor = executor_path.read_text(encoding="utf-8")
    scene = scene_path.read_text(encoding="utf-8")
    cube = cube_path.read_text(encoding="utf-8")
    monitor = monitor_path.read_text(encoding="utf-8")
    payload = payload_path.read_text(encoding="utf-8")
    contact_gate = contact_gate_path.read_text(encoding="utf-8")
    closure = closure_path.read_text(encoding="utf-8")
    orchestrator = orchestrator_path.read_text(encoding="utf-8")
    layout = layout_path.read_text(encoding="utf-8")
    vehicle = vehicle_path.read_text(encoding="utf-8")
    launch = launch_path.read_text(encoding="utf-8")

    positions = [(spec.x_m, spec.y_m, spec.z_m) for spec in specs]
    minimum_clearance = min(
        math.hypot(a[0] - b[0], a[1] - b[1]) - math.sqrt(2.0) * 0.03
        for index, a in enumerate(positions)
        for b in positions[index + 1 :]
    )
    material_counts = {
        material: sum(spec.material == material for spec in specs)
        for material in ("paperboard", "PP", "PET", "aluminum")
    }
    checks = {
        "committed_vehicle_snapshot_current": snapshot_current,
        "twenty_unique_physical_targets": len(specs) == 20
        and len({spec.target_id for spec in specs}) == 20
        and len({spec.model_name for spec in specs}) == 20,
        "single_layer_5_by_4_with_clearance": len(set(positions)) == 20
        and {z for _, _, z in positions} == {0.015}
        and minimum_clearance >= 0.005 - 1.0e-12,
        "four_real_material_masses_five_each": material_counts
        == {"paperboard": 5, "PP": 5, "PET": 5, "aluminum": 5}
        and abs(sum(spec.mass_kg for spec in specs) - 0.7668) <= 1.0e-9
        and all(token in cube for token in ("700.0", "900.0", "1380.0", "2700.0")),
        "maximum_two_attempts_per_target": manifest["runtime_requirements"].get(
            "maximum_attempts_per_target"
        )
        == 2
        and "for attempt_number in range(1, maximum_attempts + 1)" in validator,
        "retry_requires_safe_recovery_and_unchanged_payload": all(
            token in validator
            for token in (
                "retryable_without_operator",
                "failure_recovery",
                "retry_payload_unchanged",
                "_evaluator_payload_matches",
            )
        )
        and "not physical_grasp_phase_entered and not release_commanded" in executor,
        "target_conditioned_wrist_ik_moveit_contact_lift_release_chain": all(
            token in executor
            for token in (
                "_wait_wrist_recheck(request_base)",
                "_validate_ik(",
                "MoveGroup",
                "GetCartesianPath",
                "_common_live_contact()",
                "LINEAR_COLLISION_CHECKED_LIFT",
                "DETACH_OVER_DRY_BIN",
                "_wait_bin_increment(baseline)",
            )
        ),
        "dual_finger_common_body_contact_gates_detachable_joint": all(
            token in contact_gate
            for token in (
                "CommonContact(",
                "leftFresh && rightFresh",
                "contactedCollision = common.value()",
                "gz::sim::components::DetachableJoint(info)",
                "PublishAttachmentState(true)",
                "RequestRemoveEntity(this->jointEntity)",
            )
        )
        and '"/manipulation/gripper/dual_contact' in runner,
        "physical_bodies_retained_and_mass_stepped": all(
            token in validator
            for token in (
                "physical_rigid_body_payload_retained",
                "evaluator_physical_mass_matches",
                "cumulative_mass_matches",
                "physical_resident_mass_chain_passed",
            )
        )
        and "Discrete litter is never deleted" in monitor
        and "inertial->Data().MassMatrix().Mass()" in monitor,
        "aggregate_double_count_prevented": "payload/dry_mass_kg" not in monitor
        and "A physically retained cube must not also be" in payload,
        "final_session_and_current_snapshot_required": all(
            token in runner
            for token in (
                "FORMAL_ACCEPTANCE_SESSION",
                "generate_formal_vehicle_snapshot.py",
                "--check --output",
                "--session",
                "--snapshot",
            )
        )
        and "formal acceptance session must be RUNNING" in validator
        and "formal acceptance session snapshot mismatch" in validator,
        "unified_non_symlink_runtime_closure_required": all(
            token in runner
            for token in (
                ".work/final_frozen_runtime/install",
                "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST",
                "formal_runtime_gate_binding.py",
                '--runtime-binding "${runtime_binding}"',
            )
        )
        and 'runtime_binding = load_binding(args.runtime_binding)' in validator
        and '"runtime_gate_binding": runtime_binding' in validator
        and "selected runtime install root must not be a symbolic link" in closure
        and "selected runtime install root does not match the frozen closure" in closure
        and 'environment["FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST"]' in orchestrator,
        "current_vehicle_8_30kg_wastewater_source_bound": (
            "final_usable_capacity_l: 8.300" in layout
            and "<wastewater_capacity_kg>8.30</wastewater_capacity_kg>" in vehicle
            and "<tank_capacity_kg>8.30</tank_capacity_kg>" in vehicle
            and "double waterCapacityKg{8.30}" in payload
        ),
        "preexisting_canonical_evidence_is_preserved_away": ".superseded." in runner
        and 'for retained in "${output}" "${runtime_binding}"' in runner
        and 'mv -- "${retained}"' in runner,
        "scene_spawn_uses_evaluator_only_material_and_color": "load_scene_manifest" in launch
        and "spec.material" in launch
        and "spec.color_rgb" in launch
        and "request.get(\"truth_used\") is not False" in scene,
    }
    source_paths = (
        prepare_path,
        runner_path,
        validator_path,
        executor_path,
        scene_path,
        cube_path,
        monitor_path,
        payload_path,
        contact_gate_path,
        closure_path,
        orchestrator_path,
        layout_path,
        vehicle_path,
        snapshot_tool_path,
        launch_path,
    )
    source_contract_passed = all(
        value
        for name, value in checks.items()
        if name != "committed_vehicle_snapshot_current"
    )
    passed = source_contract_passed and snapshot_current
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_20_cube_grasp_static_readiness_v1",
        "passed": passed,
        "status": (
            "FORMAL_20_CUBE_STATIC_READINESS_PASSED"
            if passed
            else (
                "FORMAL_20_CUBE_STATIC_SOURCE_PASSED_SNAPSHOT_REFRESH_REQUIRED"
                if source_contract_passed and not snapshot_current
                else "FORMAL_20_CUBE_STATIC_READINESS_FAILED"
            )
        ),
        "source_contract_passed": source_contract_passed,
        "checks": checks,
        "measurements": {
            "target_count": len(specs),
            "grid_rows": manifest["dry_bin_capacity_contract"]["grid_rows"],
            "grid_columns": manifest["dry_bin_capacity_contract"]["grid_columns"],
            "minimum_rotated_envelope_clearance_m": minimum_clearance,
            "material_counts": material_counts,
            "expected_final_physical_mass_kg": sum(spec.mass_kg for spec in specs),
            "maximum_attempts_per_target": manifest["runtime_requirements"][
                "maximum_attempts_per_target"
            ],
        },
        "provenance_boundary": {
            "target_request_source": "seeded_manifest_injected_as_truth_free_v2_schema",
            "live_dosod_edgesam_perception_proven": False,
            "wrist_recheck_transport_contract_present": True,
            "required_separate_gates": [
                "formal_random_scene_perception",
                "formal_single_episode_cleaning_mission",
            ],
        },
        "runtime_state": {
            "gazebo_started_by_this_audit": False,
            "wsl_build_executed_by_this_audit": False,
            "twenty_cube_live_result_claimed": False,
            "next_gate": "fresh WSL overlay build, then session-bound live 20-cube runtime",
            "snapshot_error": snapshot_error,
        },
        "source_sha256": {
            str(path.relative_to(root)).replace("\\", "/"): _sha256(path)
            for path in source_paths
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    report = audit()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
