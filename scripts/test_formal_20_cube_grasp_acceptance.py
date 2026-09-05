from pathlib import Path
import json
import math
import runpy
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE = runpy.run_path(str(ROOT / "scripts" / "prepare_formal_20_cube_grasp_acceptance.py"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_manipulation"))
from sanitation_manipulation.formal_20_cube_scene import load_scene_manifest  # noqa: E402


def test_manifest_has_20_randomized_four_material_rich_requests_and_mass_chain():
    manifest = MODULE["build_manifest"](6020)
    assert manifest["task_count"] == 20
    assert len(manifest["requests"]) == 20
    assert {row["acceptance"]["actual_material_evaluator_only"] for row in manifest["requests"]} == {
        "paperboard",
        "PP",
        "PET",
        "aluminum",
    }
    assert {row["material"] for row in manifest["requests"]} == {"unknown"}
    assert all(row["schema_version"] == 2 for row in manifest["requests"])
    assert all(set(row["pose"]) == {"x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"} for row in manifest["requests"])
    assert all(row["size_m"] == [0.03, 0.03, 0.03] for row in manifest["requests"])
    expected = sum(row["acceptance"]["expected_increment_kg"] for row in manifest["requests"])
    assert manifest["expected_final_physical_resident_mass_kg"] == expected
    assert manifest["expected_final_aggregate_dry_mass_kg"] == 0.0
    assert manifest["dry_payload_accounting"] == {
        "mode": "physical_resident",
        "aggregate_dry_mass_must_remain_kg": 0.0,
        "nonzero_aggregate_input_rejected": True,
        "load_transfer": "independent_rigid_bodies_contact",
    }
    assert manifest["dry_bin_capacity_contract"]["maximum_count"] == 20
    assert manifest["dry_bin_capacity_contract"]["single_layer"] is True
    assert manifest["scene_contract"]["physical_rigid_bodies_retained_after_deposit"] is True
    assert len({row["acceptance"]["scene_model_name"] for row in manifest["requests"]}) == 20
    colors = [tuple(row["acceptance"]["random_color_rgb_evaluator_only"]) for row in manifest["requests"]]
    assert len(set(colors)) == 20
    assert all(
        len(
            {
                colors[index]
                for index, row in enumerate(manifest["requests"])
                if row["acceptance"]["actual_material_evaluator_only"] == material
            }
        )
        == 5
        for material in manifest["evaluator_materials"]
    )


def test_manifest_slots_are_unique_single_layer_non_overlapping_and_arm_reachable():
    manifest = MODULE["build_manifest"](6020)
    requests = manifest["requests"]
    positions = [
        (row["pose"]["x_m"], row["pose"]["y_m"], row["pose"]["z_m"])
        for row in requests
    ]
    assert len(set(positions)) == 20
    assert {position[2] for position in positions} == {0.015}
    minimum_spacing = manifest["dry_bin_capacity_contract"]["minimum_inter_cube_spacing_m"]
    cube_diagonal = math.sqrt(2.0) * 0.03
    for index, first in enumerate(positions):
        for second in positions[index + 1 :]:
            center_distance = math.hypot(first[0] - second[0], first[1] - second[1])
            assert center_distance - cube_diagonal >= minimum_spacing
    reach = manifest["arm_reach_contract"]
    arm_x, arm_y = reach["arm_base_xy_m"]
    distances = [math.hypot(x - arm_x, y - arm_y) for x, y, _ in positions]
    assert max(distances) <= reach["maximum_planar_reach_m"]
    assert max(distances) == pytest.approx(reach["maximum_generated_planar_reach_m"])


def test_material_and_slot_assignments_are_seeded_and_randomized():
    first = MODULE["build_manifest"](6020)
    repeated = MODULE["build_manifest"](6020)
    other = MODULE["build_manifest"](6021)
    first_assignments = [
        (
            row["acceptance"]["actual_material_evaluator_only"],
            row["acceptance"]["single_layer_slot"],
        )
        for row in first["requests"]
    ]
    assert first_assignments == [
        (
            row["acceptance"]["actual_material_evaluator_only"],
            row["acceptance"]["single_layer_slot"],
        )
        for row in repeated["requests"]
    ]
    assert first_assignments != [
        (
            row["acceptance"]["actual_material_evaluator_only"],
            row["acceptance"]["single_layer_slot"],
        )
        for row in other["requests"]
    ]


def test_runtime_entry_requires_wrist_recheck_mass_and_base_inhibit():
    runtime = (ROOT / "scripts" / "validate_formal_20_cube_grasp_runtime.py").read_text(
        encoding="utf-8"
    )
    runner = (ROOT / "scripts" / "run_formal_20_cube_grasp_acceptance.sh").read_text(
        encoding="utf-8"
    )
    assert "/perception/wrist/grasp_recheck" in runtime
    assert "/manipulation/base_motion_inhibited" in runtime
    assert "mass_matches" in runtime
    assert "material_matches" in runtime
    assert "count_matches" in runtime
    assert "cumulative_mass_matches" in runtime
    assert "physical_resident_mass_chain_passed" in runtime
    assert "source_binding" in runtime
    assert "formal_20_cube_pick_place.launch.py" in runner
    assert "formal_physical_grasp.launch.py" in runner
    assert "formal_acceptance/evaluator/dry_bin/status_json" in runner
    assert "FORMAL_ACCEPTANCE_SESSION" in runner
    assert "ROS_DOMAIN_ID" in runner
    assert "GZ_PARTITION" in runner
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}"' in runner
    assert "formal_runtime_install_traps cleanup" in runner
    assert "prepare_formal_20_cube_grasp_acceptance.py" in runner
    assert "acceptance_session_binding" in runtime
    assert "candidate_model_count" in runtime
    assert "physical_rigid_body_payload_retained" in runtime
    assert "physical_final_mass_matches_resident_chain" in runtime
    assert "final_evaluator_physical_mass_kg" in runtime
    assert "whole-vehicle safety permit unavailable" in runtime
    assert "maximum_attempts_per_target" in runtime
    assert "for attempt_number in range(1, maximum_attempts + 1)" in runtime
    assert "retry_payload_unchanged" in runtime
    assert "retryable_without_operator" in runtime
    assert "generate_formal_vehicle_snapshot.py" in runner
    assert "--check --output" in runner
    assert ".superseded." in runner
    rotation_loop = (
        'for retained in "${output}" "${runtime_binding}" "${manifest}" "${launch_log}"; do'
    )
    assert rotation_loop in runner
    assert '[[ -e "${retained}" || -L "${retained}" ]]' in runner
    assert runner.index(rotation_loop) < runner.index(
        "source /opt/ros/jazzy/setup.bash"
    )
    assert runner.index(rotation_loop) < runner.index(
        'if [[ ! -f "${runtime_ws}/setup.bash" ]]; then'
    )
    assert runner.index("formal_runtime_install_traps cleanup") < runner.index(
        'ros2 launch sanitation_manipulation formal_20_cube_pick_place.launch.py'
    )
    assert ".work/final_frozen_runtime/install" in runner
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in runner
    assert "formal_runtime_gate_binding.py" in runner
    assert '--runtime-binding "${runtime_binding}"' in runner
    assert 'runtime_binding = load_binding(args.runtime_binding)' in runtime
    assert '"runtime_gate_binding": runtime_binding' in runtime
    assert "_active_overlay_identity" in runtime
    assert "active_overlay_matches_runtime_binding" in runtime
    assert "formal_source_bound_preflight.sh" in runner
    assert 'formal_source_bound_verify_overlay "${runtime_ws}"' in runner


def test_retry_contract_is_bounded_fail_closed_and_precontact_only():
    manifest = MODULE["build_manifest"](6020)
    runtime = manifest["runtime_requirements"]
    assert runtime["maximum_attempts_per_target"] == 2
    assert runtime["retry_requires_safe_transport_restored"] is True
    assert runtime["retry_requires_unchanged_evaluator_payload"] is True
    assert runtime["duplicate_payload_accounting_forbidden"] is True
    executor = (
        ROOT
        / "starter_ws/src/sanitation_manipulation/sanitation_manipulation/formal_grasp_executor.py"
    ).read_text(encoding="utf-8")
    assert "physical_grasp_phase_entered = True" in executor
    assert "not physical_grasp_phase_entered and not release_commanded" in executor
    assert 'evidence["retryable_without_operator"] = False' in executor
    physical_phase = executor.index(
        "physical_grasp_phase_entered = True",
        executor.index("LINEAR_CONTACT_APPROACH") - 500,
    )
    contact_approach = executor.index(
        'self._cartesian_move(waypoints.pick, "LINEAR_CONTACT_APPROACH")'
    )
    assert physical_phase < contact_approach


def test_windows_static_readiness_audit_is_explicit_about_perception_boundary():
    audit_module = runpy.run_path(
        str(ROOT / "scripts/audit_formal_20_cube_grasp_readiness.py")
    )
    report = audit_module["audit"](ROOT)
    assert report["source_contract_passed"] is True
    snapshot_current = report["checks"]["committed_vehicle_snapshot_current"]
    if snapshot_current:
        assert report["passed"] is True
        assert report["status"] == "FORMAL_20_CUBE_STATIC_READINESS_PASSED"
        assert report["runtime_state"]["snapshot_error"] is None
    else:
        assert report["passed"] is False
        assert report["status"] == (
            "FORMAL_20_CUBE_STATIC_SOURCE_PASSED_SNAPSHOT_REFRESH_REQUIRED"
        )
    assert report["runtime_state"]["gazebo_started_by_this_audit"] is False
    assert report["runtime_state"]["twenty_cube_live_result_claimed"] is False
    assert (
        report["provenance_boundary"]["live_dosod_edgesam_perception_proven"]
        is False
    )
    assert report["checks"]["unified_non_symlink_runtime_closure_required"] is True
    assert report["checks"]["current_vehicle_8_30kg_wastewater_source_bound"] is True
    assert report["checks"]["dual_finger_common_body_contact_gates_detachable_joint"] is True


def test_scene_loader_rejects_truth_leak_overlap_material_drift_and_count(tmp_path):
    manifest = MODULE["build_manifest"](6020)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    loaded, specs = load_scene_manifest(path)
    assert loaded["manifest_id"] == manifest["manifest_id"]
    assert len(specs) == 20
    assert sum(spec.mass_kg for spec in specs) == pytest.approx(0.7668)
    assert {spec.model_name for spec in specs} == {
        f"object_{index:02d}" for index in range(1, 21)
    }

    broken = json.loads(json.dumps(manifest))
    broken["requests"][0]["material"] = "PET"
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="leaks evaluator truth"):
        load_scene_manifest(path)

    broken = json.loads(json.dumps(manifest))
    broken["requests"][1]["pose"] = broken["requests"][0]["pose"]
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="overlap|unique"):
        load_scene_manifest(path)

    broken = json.loads(json.dumps(manifest))
    broken["requests"][0]["acceptance"]["expected_increment_kg"] = 0.5
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="material mass"):
        load_scene_manifest(path)

    broken = json.loads(json.dumps(manifest))
    broken["requests"].pop()
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="exactly 20"):
        load_scene_manifest(path)

    broken = json.loads(json.dumps(manifest))
    broken["requests"][1]["acceptance"]["single_layer_slot"] = broken["requests"][0][
        "acceptance"
    ]["single_layer_slot"]
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate 5x4 slot"):
        load_scene_manifest(path)

    broken = json.loads(json.dumps(manifest))
    broken["dry_bin_capacity_contract"]["grid_columns"] = 4
    path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="5x4 single-layer grid"):
        load_scene_manifest(path)


def test_20_cube_launch_disables_single_cube_and_spawns_manifest_models():
    base_launch = (
        ROOT / "starter_ws/src/sanitation_manipulation/launch/formal_cube_pick_place.launch.py"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT
        / "starter_ws/src/sanitation_manipulation/launch/formal_20_cube_pick_place.launch.py"
    ).read_text(encoding="utf-8")
    cube = (
        ROOT / "starter_ws/src/sanitation_manipulation/urdf/material_cube.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("spawn_single_cube", default_value="true")' in base_launch
    assert "condition=IfCondition(spawn_single_cube)" in base_launch
    assert '"dry_accounting_mode", default_value="physical_resident"' in base_launch
    assert '"spawn_single_cube": "false"' in launch
    assert '"physics_engine": "gz-physics-dartsim-plugin"' in launch
    assert '"dry_accounting_mode": "physical_resident"' in launch
    assert "load_scene_manifest" in launch
    assert '"-name", spec.model_name' in launch
    assert "color_r" in cube and "color_g" in cube and "color_b" in cube
