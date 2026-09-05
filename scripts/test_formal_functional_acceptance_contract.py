from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from validate_formal_functional_acceptance_contract import (
    DEFAULT_CONTRACT,
    DEFAULT_REGISTER,
    FunctionalAcceptanceError,
    _strict_json_equal,
    audit,
)


def test_sensor_runtime_gate_declares_current_report_schema() -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    sensor = contract["evidence_gates"]["sensor_runtime"]
    assert sensor["report_id"] == "tzcup_formal_vehicle_headless_runtime_v5"
    assert sensor["success_statuses"] == [
        "FORMAL_GAZEBO_CONTROL_AND_SENSOR_RUNTIME_PASSED_EXTERNAL_FIDELITY_GATES_PENDING"
    ]


def test_required_value_comparison_rejects_bool_integer_coercion() -> None:
    assert _strict_json_equal(True, True)
    assert _strict_json_equal([True, {"passed": False}], [True, {"passed": False}])
    assert not _strict_json_equal(1, True)
    assert not _strict_json_equal(0, False)
    assert not _strict_json_equal([1], [True])


def test_contract_covers_all_38_registered_positions_and_is_fail_closed(
    tmp_path: Path,
) -> None:
    result = audit(root=tmp_path)
    assert result["registered_position_count"] == 38
    assert result["status"] == "FORMAL_FUNCTIONAL_ACCEPTANCE_PENDING"
    assert result["complete"] is False
    assert result["acceptance_session_complete"] is False
    assert result["passed_position_count"] < 38
    assert result["acceptance_session"]["state"] == "missing"
    assert "s100_live_runtime" in result["unresolved_mission_gates"]
    assert "end_to_end_cleaning_mission" in result["unresolved_mission_gates"]
    assert "sensor_fov_and_occlusion" in result["unresolved_mission_gates"]
    assert "inertia_cog_and_swept_volume" in result["unresolved_mission_gates"]
    assert "product_visual_acceptance" in result["unresolved_mission_gates"]
    assert "service_visual_acceptance" in result["unresolved_mission_gates"]
    assert result["gate_results"]["s100_live_runtime"]["state"] == "missing"
    # A stale standalone drivetrain report may be removed before the fresh
    # matrix.  Whether absent or retained-but-unbound, it must never pass.
    assert result["gate_results"]["a300_drivetrain_runtime"]["state"] in {
        "missing",
        "unbound",
    }
    assert result["gate_results"]["service_interface_acceptance"]["state"] == "missing"
    assert "a300_drivetrain_runtime" in result["positions"]["mobility"]["unresolved_gates"]
    assert "service_interface_acceptance" in result["positions"]["charge_interface"]["unresolved_gates"]
    assert "service_interface_acceptance" in result["positions"]["wastewater_drain"]["unresolved_gates"]
    assert result["positions"]["bodywork_service_access"]["required_gates"] == [
        "component_register",
        "inertia_cog_and_swept_volume",
        "product_visual_acceptance",
        "service_visual_acceptance",
        "service_door_runtime",
    ]
    assert result["gate_results"]["service_door_runtime"]["state"] == "missing"
    assert result["gate_results"]["cleaning_motor_runtime"]["state"] == "missing"
    assert result["gate_results"]["rl_cross_map_policy"]["state"] in {
        "missing",
        "stale",
        "unbound",
    }


@pytest.mark.parametrize(
    "session_status",
    [
        "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
    ],
)
def test_current_observable_session_without_evidence_stays_pending(
    tmp_path: Path,
    session_status: str,
) -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    expanded_rel = "reports/engineering/formal_competition_vehicle.urdf"
    expanded = tmp_path / expanded_rel
    expanded.parent.mkdir(parents=True, exist_ok=True)
    expanded.write_bytes(b'<robot name="running-session"/>\n')
    expanded_hash = hashlib.sha256(expanded.read_bytes()).hexdigest()

    snapshot_path = tmp_path / contract["acceptance_session"]["snapshot_manifest"]
    snapshot_path.parent.mkdir(parents=True, exist_ok=True)
    snapshot = {
        "source_inventory_sha256": "running-source-hash",
        "source_inventory": {},
        "outputs": {
            expanded_rel: {
                "sha256": expanded_hash,
                "size_bytes": expanded.stat().st_size,
            }
        },
    }
    snapshot_path.write_text(json.dumps(snapshot), encoding="utf-8")
    session_path = tmp_path / contract["acceptance_session"]["path"]
    session_path.parent.mkdir(parents=True, exist_ok=True)
    session_path.write_text(
        json.dumps(
            {
                "status": session_status,
                "snapshot": {
                    "snapshot_manifest_sha256": hashlib.sha256(
                        snapshot_path.read_bytes()
                    ).hexdigest(),
                    "source_inventory_sha256": "running-source-hash",
                    "expanded_urdf_sha256": expanded_hash,
                },
                "evidence": {},
            }
        ),
        encoding="utf-8",
    )

    result = audit(root=tmp_path)
    assert result["acceptance_session"]["state"] == "valid"
    assert result["acceptance_session"]["status"] == session_status
    assert result["complete"] is False
    assert result["acceptance_session_complete"] is False
    assert result["passed_position_count"] == 0
    assert len(result["unresolved_mission_gates"]) == len(
        contract["mission_level_gates"]
    )


def test_every_runtime_effect_class_is_required_by_a_function_position() -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    used = {
        gate
        for gates in contract["functional_positions"].values()
        for gate in gates
    }
    expected = {
        "auxiliary_power_lighting",
        "physical_grasp_and_bin",
        "formal_20_cube_grasp_and_dynamic_mass",
        "ground_dirt_cleaning",
        "water_recovery",
        "first_map_then_clean",
        "random_scene_perception",
        "dynamic_obstacle_avoidance",
        "a300_drivetrain_runtime",
        "service_interface_acceptance",
        "service_door_runtime",
        "cleaning_motor_runtime",
        "s100_live_runtime",
        "product_visual_acceptance",
        "service_visual_acceptance",
    }
    assert expected <= used
    assert contract["evidence_gates"]["component_register"]["session_bound"] is True
    assert contract["evidence_gates"]["rl_cross_map_policy"]["session_bound"] is True
    for gate in ("a300_drivetrain_runtime", "water_recovery"):
        assert contract["evidence_gates"][gate]["snapshot_urdf_hash_field"] == (
            "source_binding.expanded_urdf_sha256"
        )
        assert contract["evidence_gates"][gate]["snapshot_source_hash_field"] == (
            "source_binding.source_inventory_sha256"
        )
    runtime_bound_gates = {
        "sensor_runtime",
        "product_visual_acceptance",
        "service_visual_acceptance",
        "integrated_basic_physics",
        "a300_drivetrain_runtime",
        "whole_vehicle_interlock",
        "auxiliary_power_lighting",
        "cleaning_actuators",
        "cleaning_motor_runtime",
        "ground_dirt_cleaning",
        "first_map_then_clean",
        "random_scene_perception",
        "dynamic_obstacle_avoidance",
        "end_to_end_cleaning_mission",
        "multi_site_product_generalization",
        "water_recovery",
        "service_door_runtime",
        "service_interface_acceptance",
        "manipulator_trajectory",
        "physical_grasp_and_bin",
        "formal_20_cube_grasp_and_dynamic_mass",
    }
    contracted = {
        gate_id
        for gate_id, row in contract["evidence_gates"].items()
        if isinstance(row, dict) and row.get("runtime_binding") is not None
    }
    assert contracted == runtime_bound_gates
    for gate_id in runtime_bound_gates:
        gate = contract["evidence_gates"][gate_id]
        assert gate["runtime_binding"] == {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        }
        assert gate["required_values"]["runtime_gate_binding.status"] == (
            "FORMAL_RUNTIME_GATE_BOUND"
        )
        assert gate["required_values"][
            "runtime_gate_binding.runtime_closure_binding.status"
        ] == "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
    assert contract["evidence_gates"]["ground_dirt_cleaning"]["report_id"] == (
        "tzcup_formal_ground_dirt_physical_cleaning_v1"
    )
    assert contract["evidence_gates"]["water_recovery"]["report_id"] == (
        "tzcup_formal_water_recovery_acceptance_v1"
    )
    s100 = contract["evidence_gates"]["s100_live_runtime"]
    assert s100["session_bound"] is True
    assert s100["snapshot_source_hash_field"] == (
        "source_binding.source_inventory_sha256"
    )
    assert s100["evidence_origin"] == "external_rdk_s100_live_hardware_only"
    for position in (
        "grasp_observation",
        "manipulation",
        "grasping",
        "dry_deposition",
        "dry_storage",
        "dry_fill_monitor",
    ):
        assert "formal_20_cube_grasp_and_dynamic_mass" in contract[
            "functional_positions"
        ][position]
    assert "end_to_end_cleaning_mission" in contract["mission_level_gates"]
    assert "sensor_fov_and_occlusion" in contract["mission_level_gates"]
    assert "inertia_cog_and_swept_volume" in contract["mission_level_gates"]
    assert "product_visual_acceptance" in contract["mission_level_gates"]
    assert "service_visual_acceptance" in contract["mission_level_gates"]
    for gate, profile in (
        ("product_visual_acceptance", "product"),
        ("service_visual_acceptance", "service"),
    ):
        visual = contract["evidence_gates"][gate]
        assert visual["session_bound"] is True
        assert visual["snapshot_source_hash_field"] == (
            "source_binding.source_inventory_sha256"
        )
        assert visual["required_values"] == {
            "passed": True,
            "bodywork_profile": profile,
            "camera_count": 19,
            "ground_truth_anti_drift.passed": True,
            "ground_truth_anti_drift.world_name": "formal_vehicle_visual_acceptance",
            "ground_truth_anti_drift.model_name": "tzcup_formal_sanitation_vehicle",
            "ground_truth_anti_drift.violations": [],
            "bodywork_profile_verified_from_robot_description": True,
            "renderer_diagnostics.passed": True,
            "engineering_visual_crosswalk.passed": True,
            "engineering_visual_crosswalk.method": (
                "registered_id_crosswalk_plus_commanded_pose_urdf_link_projection"
            ),
            "engineering_visual_crosswalk.bodywork_profile": profile,
            "engineering_visual_crosswalk.camera_count": 19,
            "engineering_visual_crosswalk.functional_position_count": 38,
            "engineering_visual_crosswalk.sensor_installation_count": 9,
            "engineering_visual_crosswalk.mechanical_subassembly_count": 18,
            "engineering_visual_crosswalk.required_registered_physical_link_count": (
                110 if profile == "product" else 96
            ),
            "engineering_visual_crosswalk.inspected_physical_link_count": (
                156 if profile == "product" else 142
            ),
            "runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND",
            "runtime_gate_binding.acceptance_session_binding.session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "runtime_gate_binding.acceptance_session_binding.snapshot_current_source_verified": True,
            "runtime_gate_binding.runtime_closure_binding.status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        }
        assert visual["runtime_binding"] == {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        }
        assert set(visual["required_mapping_keys"]) == {
            "frames",
            "profile_evidence_scope",
            "view_target_contract",
        }
        assert len(visual["required_mapping_keys"]["frames"]) == 19
        assert visual["bound_file_mapping"] == "frames"
    for position in (
        "side_sweeping",
        "main_sweeping",
        "cleaning_head_lift",
        "water_pumping",
    ):
        assert "cleaning_motor_runtime" in contract["functional_positions"][position]
    unrelated_positions = set(contract["functional_positions"]) - {
        "side_sweeping",
        "main_sweeping",
        "cleaning_head_lift",
        "water_pumping",
    }
    assert all(
        "cleaning_motor_runtime" not in contract["functional_positions"][position]
        for position in unrelated_positions
    )


def test_new_formal_runtime_gates_match_emitted_status_and_snapshot_binding() -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    gates = contract["evidence_gates"]
    assert gates["cleaning_actuators"]["success_statuses"] == [
        "FORMAL_CLEANING_STORAGE_SERVICE_AND_RECOVERY_ACTUATORS_PASSED"
    ]
    assert gates["cleaning_actuators"]["snapshot_source_hash_field"] == (
        "source_binding.source_inventory_sha256"
    )
    assert gates["cleaning_actuators"]["runtime_binding"] == {
        "report_field": "runtime_gate_binding",
        "sidecar_suffix": ".runtime_binding.json",
    }
    for gate in (
        "manipulator_trajectory",
        "cleaning_actuators",
        "integrated_basic_physics",
    ):
        assert gates[gate]["session_bound"] is True
        assert (
            gates[gate]["snapshot_urdf_hash_field"]
            == "source_binding.expanded_urdf_sha256"
        )


def test_map_and_dynamic_runtime_gates_lock_their_semantic_success_fields() -> None:
    gates = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))["evidence_gates"]
    assert gates["first_map_then_clean"]["required_values"] == {
        "schema_version": 1,
        "passed": True,
        "truth_used_for_control": False,
        "checks.quality_gated_map_manifest": True,
        "checks.mapping_runtime_passed": True,
        "checks.saved_map_cleaning_runtime_passed": True,
        "runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND",
        "runtime_gate_binding.acceptance_session_binding.session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "runtime_gate_binding.acceptance_session_binding.snapshot_current_source_verified": True,
        "runtime_gate_binding.runtime_closure_binding.status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
    }
    assert gates["dynamic_obstacle_avoidance"]["report_id"] == (
        "tzcup_formal_dynamic_obstacle_avoidance_acceptance_v1"
    )
    dynamic_values = gates["dynamic_obstacle_avoidance"]["required_values"]
    assert dynamic_values["passed"] is True
    assert dynamic_values["checks.saved_map_lifecycle_artifact_valid"] is True
    assert dynamic_values["checks.exactly_eight_random_pedestrians_active"] is True
    assert dynamic_values["checks.product_control_truth_free"] is True
    assert dynamic_values["checks.no_pedestrian_velocity_estimation"] is True
    assert dynamic_values["metrics.active_pedestrian_count"] == 8
    assert dynamic_values["metrics.expected_pedestrian_count"] == 8
    assert dynamic_values["metrics.pedestrian_velocity_estimation_used"] is False
    assert dynamic_values["metrics.control_truth_topics_subscribed"] == []


def test_perception_rl_and_mission_gates_lock_formal_sample_scale_and_boundaries() -> None:
    gates = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))["evidence_gates"]
    perception = gates["random_scene_perception"]["required_values"]
    assert "episode_count" not in perception
    assert perception["minimum_episode_count"] == 30
    assert perception["gates.minimum_disjoint_episode_count"] is True
    assert perception["gates.all_required_validation_maps_covered"] is True
    assert perception["gates.minimum_episodes_per_validation_map"] is True
    assert perception["statistical_scope.required_validation_map_indices"] == list(range(8))
    assert perception["statistical_scope.smoke_eligible_for_final_product_evidence"] is False
    assert perception["truth_isolation.truth_used_by_product_control"] is False
    assert perception["claim_boundary.no_fine_tuning_performed"] is True
    assert perception["claim_boundary.real_world_accuracy_claimed"] is False

    rl = gates["rl_cross_map_policy"]
    assert rl["required_values"]["policy_output"] == "global_reference_trajectory"
    assert rl["required_values"]["return_distance_included"] is False
    assert rl["required_values"]["full_map_generalization_contract.required_map_counts.train"] == 32
    assert rl["required_values"]["full_map_generalization_contract.required_map_counts.validation"] == 8
    assert rl["required_values"]["full_map_generalization_contract.required_map_counts.hidden"] == 12
    assert rl["required_values"]["full_map_generalization_contract.full_map_coverage"] is True
    assert rl["required_values"]["full_map_generalization_contract.smoke_subset_accepted_as_generalization"] is False
    assert rl["required_values"]["stage_a_fixed_map_budget_contract.task_counts.train"] == 10000
    assert rl["required_values"]["stage_a_fixed_map_budget_contract.task_counts.validation"] == 500
    assert rl["required_values"]["stage_a_fixed_map_budget_contract.task_counts.hidden"] == 1000
    assert rl["required_list_item_values"]["stage_a_fixed_map_budget_contract.policy_runs"] == {
        "train_episode_count": 10000,
        "validation_episode_count": 500,
        "hidden_episode_count": 1000,
    }
    assert "fixed_selection" in rl["required_mapping_keys"]["full_map_generalization_contract"]
    assert rl["required_list_item_values"]["q_with_systematic_coverage_backstop"][
        "formal_success"
    ] is True
    assert rl["required_list_item_minimums"]["q_with_systematic_coverage_backstop"] == {
        "observed_ratio": 0.95,
        "ground_clear_ratio": 0.95,
        "discrete_clear_ratio": 0.95,
    }
    assert rl["required_list_item_maximums"]["q_with_systematic_coverage_backstop"] == {
        "path_ratio_to_full_coverage": 1.0
    }

    mission = gates["end_to_end_cleaning_mission"]["required_values"]
    assert mission["passed"] is True
    assert mission["errors"] == []
    assert mission["validated_closed_loop.actual_brushed_area_at_least_95_percent"] is True
    assert mission["validated_closed_loop.all_20_discrete_targets_physically_deposited"] is True
    assert mission["validated_closed_loop.water_recovery_at_least_95_percent"] is True
    assert mission["validated_closed_loop.task_distance_not_above_full_coverage_baseline"] is True
    assert mission["validated_closed_loop.same_map_full_coverage_efficiency_at_least_3500"] is True
    assert mission["validated_closed_loop.return_started_only_after_task_complete"] is True
    assert "same_map_full_coverage_efficiency_at_least_3500" in (
        gates["end_to_end_cleaning_mission"]["required_mapping_keys"]["validated_closed_loop"]
    )


def test_missing_position_crosswalk_fails_closed(tmp_path: Path) -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    contract["functional_positions"].pop("front_contact_safety")
    mutated = tmp_path / "contract.yaml"
    mutated.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    with pytest.raises(FunctionalAcceptanceError, match="crosswalk mismatch"):
        audit(contract_path=mutated)


def test_matching_evidence_status_closes_a_gate(tmp_path: Path) -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    register = yaml.safe_load(DEFAULT_REGISTER.read_text(encoding="utf-8"))
    expanded_urdf_bytes = b'<robot name="test"/>\n'
    urdf_hash = hashlib.sha256(expanded_urdf_bytes).hexdigest()
    evidence_rows = {}
    for gate, row in contract["evidence_gates"].items():
        row["path"] = f"evidence/{gate}.json"
        path = tmp_path / row["path"]
        path.parent.mkdir(parents=True, exist_ok=True)
        value = {"status": row["success_statuses"][0]}
        if row.get("report_id") is not None:
            value["report_id"] = row["report_id"]
        for dotted, expected in row.get("required_values", {}).items():
            target = value
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target.setdefault(key, {})
            target[keys[-1]] = expected
        for dotted, expected_keys in row.get("required_mapping_keys", {}).items():
            target = value
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target.setdefault(key, {})
            mapping = target.setdefault(keys[-1], {})
            assert isinstance(mapping, dict)
            for key in expected_keys:
                mapping.setdefault(str(key), {})
        for dotted, requirements in row.get("required_mapping_item_values", {}).items():
            target = value
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target.setdefault(key, {})
            mapping = target.setdefault(keys[-1], {})
            for item in mapping.values():
                for required_field, expected in requirements.items():
                    nested = item
                    field_keys = required_field.split(".")
                    for field_key in field_keys[:-1]:
                        nested = nested.setdefault(field_key, {})
                    nested[field_keys[-1]] = expected
        list_fields = (
            set(row.get("required_list_item_values", {}))
            | set(row.get("required_list_item_minimums", {}))
            | set(row.get("required_list_item_maximums", {}))
        )
        for dotted in list_fields:
            target = value
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target.setdefault(key, {})
            item = {}
            target[keys[-1]] = [item]
            for requirements in (
                row.get("required_list_item_values", {}).get(dotted, {}),
                row.get("required_list_item_minimums", {}).get(dotted, {}),
                row.get("required_list_item_maximums", {}).get(dotted, {}),
            ):
                for required_field, expected in requirements.items():
                    nested = item
                    field_keys = required_field.split(".")
                    for field_key in field_keys[:-1]:
                        nested = nested.setdefault(field_key, {})
                    nested[field_keys[-1]] = expected
        path.write_text(json.dumps(value), encoding="utf-8")
        if row.get("snapshot_urdf_hash_field") == "urdf_sha256":
            value["urdf_sha256"] = urdf_hash
            path.write_text(json.dumps(value), encoding="utf-8")
        elif row.get("snapshot_urdf_hash_field") == "inputs.expanded_urdf_sha256":
            value.setdefault("inputs", {})["expanded_urdf_sha256"] = urdf_hash
            path.write_text(json.dumps(value), encoding="utf-8")
        elif row.get("snapshot_urdf_hash_field") == "source_binding.expanded_urdf_sha256":
            value.setdefault("source_binding", {})[
                "expanded_urdf_sha256"
            ] = urdf_hash
            path.write_text(json.dumps(value), encoding="utf-8")
        elif row.get("snapshot_urdf_hash_field") == (
            "acceptance_session_binding.snapshot.expanded_urdf_sha256"
        ):
            value.setdefault("acceptance_session_binding", {}).setdefault(
                "snapshot", {}
            )["expanded_urdf_sha256"] = urdf_hash
            path.write_text(json.dumps(value), encoding="utf-8")
        if row.get("snapshot_source_hash_field") == "source_binding.source_inventory_sha256":
            value = json.loads(path.read_text(encoding="utf-8"))
            value.setdefault("source_binding", {})["source_inventory_sha256"] = (
                "source-hash"
            )
            path.write_text(json.dumps(value), encoding="utf-8")
        elif row.get("snapshot_source_hash_field") == (
            "acceptance_session_binding.snapshot.source_inventory_sha256"
        ):
            value = json.loads(path.read_text(encoding="utf-8"))
            value.setdefault("acceptance_session_binding", {}).setdefault(
                "snapshot", {}
            )["source_inventory_sha256"] = "source-hash"
            path.write_text(json.dumps(value), encoding="utf-8")
        for dotted, relative_source in row.get("current_file_hashes", {}).items():
            source = tmp_path / relative_source
            source.parent.mkdir(parents=True, exist_ok=True)
            if not source.exists():
                source.write_bytes(f"current:{relative_source}\n".encode("utf-8"))
            value = json.loads(path.read_text(encoding="utf-8"))
            target = value
            keys = dotted.split(".")
            for key in keys[:-1]:
                target = target.setdefault(key, {})
            target[keys[-1]] = hashlib.sha256(source.read_bytes()).hexdigest()
            path.write_text(json.dumps(value), encoding="utf-8")
        if row.get("session_bound") is True:
            evidence_rows[gate] = {
                "path": row["path"],
                "status": row["success_statuses"][0],
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
    snapshot_path = tmp_path / contract["acceptance_session"]["snapshot_manifest"]
    snapshot_path.parent.mkdir(parents=True, exist_ok=True)
    expanded_urdf = tmp_path / "reports/engineering/formal_competition_vehicle.urdf"
    expanded_urdf.write_bytes(expanded_urdf_bytes)
    snapshot = {
        "source_inventory_sha256": "source-hash",
        "outputs": {
            "reports/engineering/formal_competition_vehicle.urdf": {
                "sha256": urdf_hash,
                "size_bytes": expanded_urdf.stat().st_size,
            }
        },
    }
    snapshot_path.write_text(json.dumps(snapshot), encoding="utf-8")
    session_path = tmp_path / contract["acceptance_session"]["path"]
    session_path.parent.mkdir(parents=True, exist_ok=True)
    session_snapshot = {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": "source-hash",
        "expanded_urdf_sha256": urdf_hash,
    }
    closure_binding = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest_sha256": "a" * 64,
        "closure_sha256": "b" * 64,
        "runtime_install_root": "/frozen/runtime/install",
        "symbolic_link_count": 0,
    }
    session_path.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": 123,
                "snapshot": session_snapshot,
                "runtime_closure_binding": closure_binding,
                "evidence": evidence_rows,
            }
        ),
        encoding="utf-8",
    )
    runtime_binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "verified_epoch_ns": 124,
        "acceptance_session_binding": {
            "session_manifest_sha256": hashlib.sha256(session_path.read_bytes()).hexdigest(),
            "session_started_epoch_ns": 123,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "snapshot": session_snapshot,
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": closure_binding,
    }
    for gate, row in contract["evidence_gates"].items():
        if row.get("runtime_binding") is None:
            continue
        path = tmp_path / row["path"]
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["runtime_gate_binding"] = runtime_binding
        path.write_text(json.dumps(payload), encoding="utf-8")
        path.with_name(path.name + ".runtime_binding.json").write_text(
            json.dumps(runtime_binding), encoding="utf-8"
        )
        evidence_rows[gate]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["evidence"] = evidence_rows
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    contract_path = tmp_path / "contract.yaml"
    register_path = tmp_path / "register.yaml"
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    register_path.write_text(yaml.safe_dump(register, sort_keys=False), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["status"] == "FORMAL_FUNCTIONAL_ACCEPTANCE_PENDING"
    assert result["complete"] is False
    assert result["acceptance_session_complete"] is False
    assert result["passed_position_count"] == 38
    assert result["pending_position_count"] == 0
    assert result["unresolved_mission_gates"] == []

    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["status"] = "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is True
    assert result["acceptance_session_complete"] is True
    assert result["passed_position_count"] == 38
    assert result["unresolved_mission_gates"] == []

    runtime_path = tmp_path / contract["evidence_gates"]["sensor_runtime"]["path"]
    runtime_sidecar = runtime_path.with_name(runtime_path.name + ".runtime_binding.json")
    drifted_binding = json.loads(runtime_sidecar.read_text(encoding="utf-8"))
    drifted_binding["runtime_closure_binding"]["closure_sha256"] = "c" * 64
    runtime_sidecar.write_text(json.dumps(drifted_binding), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["sensor_runtime"]["state"] == "unbound"
    assert "differs from its sidecar" in result["gate_results"]["sensor_runtime"]["error"]
    runtime_sidecar.write_text(json.dumps(runtime_binding), encoding="utf-8")

    scanner = tmp_path / contract["evidence_gates"]["inertia_cog_and_swept_volume"][
        "current_file_hashes"
    ]["inputs.scanner_sha256"]
    scanner.write_bytes(scanner.read_bytes() + b"drift\n")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["inertia_cog_and_swept_volume"]["state"] == "stale"
    assert "inputs.scanner_sha256" in result["gate_results"][
        "inertia_cog_and_swept_volume"
    ]["error"]
    scanner.write_bytes(scanner.read_bytes().removesuffix(b"drift\n"))

    dynamic_path = tmp_path / contract["evidence_gates"][
        "dynamic_obstacle_avoidance"
    ]["path"]
    dynamic_payload = json.loads(dynamic_path.read_text(encoding="utf-8"))
    dynamic_payload["metrics"]["pedestrian_velocity_estimation_used"] = True
    dynamic_path.write_text(json.dumps(dynamic_payload), encoding="utf-8")
    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["evidence"]["dynamic_obstacle_avoidance"]["sha256"] = (
        hashlib.sha256(dynamic_path.read_bytes()).hexdigest()
    )
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["dynamic_obstacle_avoidance"]["state"] == "failed"
    assert "metrics.pedestrian_velocity_estimation_used" in result["gate_results"][
        "dynamic_obstacle_avoidance"
    ]["error"]

    dynamic_payload["metrics"]["pedestrian_velocity_estimation_used"] = False
    dynamic_path.write_text(json.dumps(dynamic_payload), encoding="utf-8")
    session_payload["evidence"]["dynamic_obstacle_avoidance"]["sha256"] = (
        hashlib.sha256(dynamic_path.read_bytes()).hexdigest()
    )
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")

    map_path = tmp_path / contract["evidence_gates"]["first_map_then_clean"]["path"]
    map_payload = json.loads(map_path.read_text(encoding="utf-8"))
    map_payload["truth_used_for_control"] = True
    map_path.write_text(json.dumps(map_payload), encoding="utf-8")
    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["evidence"]["first_map_then_clean"]["sha256"] = (
        hashlib.sha256(map_path.read_bytes()).hexdigest()
    )
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["first_map_then_clean"]["state"] == "failed"
    assert "truth_used_for_control" in result["gate_results"][
        "first_map_then_clean"
    ]["error"]

    map_payload["truth_used_for_control"] = False
    map_path.write_text(json.dumps(map_payload), encoding="utf-8")
    session_payload["evidence"]["first_map_then_clean"]["sha256"] = (
        hashlib.sha256(map_path.read_bytes()).hexdigest()
    )
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")

    rl_path = tmp_path / contract["evidence_gates"]["rl_cross_map_policy"]["path"]
    rl_payload = json.loads(rl_path.read_text(encoding="utf-8"))
    rl_payload["q_with_systematic_coverage_backstop"][0]["formal_success"] = 1
    rl_path.write_text(json.dumps(rl_payload), encoding="utf-8")
    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["evidence"]["rl_cross_map_policy"]["sha256"] = hashlib.sha256(
        rl_path.read_bytes()
    ).hexdigest()
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["rl_cross_map_policy"]["state"] == "failed"
    assert "q_with_systematic_coverage_backstop[0]" in result["gate_results"][
        "rl_cross_map_policy"
    ]["error"]

    rl_payload["q_with_systematic_coverage_backstop"][0]["formal_success"] = True
    rl_path.write_text(json.dumps(rl_payload), encoding="utf-8")
    session_payload["evidence"]["rl_cross_map_policy"]["sha256"] = hashlib.sha256(
        rl_path.read_bytes()
    ).hexdigest()
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")

    interlock_path = tmp_path / contract["evidence_gates"]["whole_vehicle_interlock"]["path"]
    interlock_payload = json.loads(interlock_path.read_text(encoding="utf-8"))
    interlock_payload["checks"][
        "managed_command_topics_have_single_gateway_writer"
    ] = 1
    interlock_path.write_text(json.dumps(interlock_payload), encoding="utf-8")
    session_payload = json.loads(session_path.read_text(encoding="utf-8"))
    session_payload["evidence"]["whole_vehicle_interlock"]["sha256"] = hashlib.sha256(
        interlock_path.read_bytes()
    ).hexdigest()
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["whole_vehicle_interlock"]["state"] == "failed"
    assert "checks.managed_command_topics_have_single_gateway_writer" in result[
        "gate_results"
    ]["whole_vehicle_interlock"]["error"]

    interlock_payload["checks"][
        "managed_command_topics_have_single_gateway_writer"
    ] = True
    interlock_payload["hard_interlock_evidence"]["front_bumper_contact"][
        "position_goal_result_statuses"
    ]["/arm_controller/follow_joint_trajectory"] = 4
    interlock_path.write_text(json.dumps(interlock_payload), encoding="utf-8")
    session_payload["evidence"]["whole_vehicle_interlock"]["sha256"] = hashlib.sha256(
        interlock_path.read_bytes()
    ).hexdigest()
    session_path.write_text(json.dumps(session_payload), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["complete"] is False
    assert result["gate_results"]["whole_vehicle_interlock"]["state"] == "failed"
    assert "hard_interlock_evidence.front_bumper_contact" in result[
        "gate_results"
    ]["whole_vehicle_interlock"]["error"]


def test_passing_historical_evidence_is_unbound_without_session(tmp_path: Path) -> None:
    contract = yaml.safe_load(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    register = yaml.safe_load(DEFAULT_REGISTER.read_text(encoding="utf-8"))
    gate_id = "sensor_fov_and_occlusion"
    gate = contract["evidence_gates"][gate_id]
    gate.pop("current_file_hashes", None)
    gate["path"] = "evidence/sensor.json"
    path = tmp_path / gate["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"status": gate["success_statuses"][0]}), encoding="utf-8")
    contract_path = tmp_path / "contract.yaml"
    register_path = tmp_path / "register.yaml"
    contract_path.write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    register_path.write_text(yaml.safe_dump(register, sort_keys=False), encoding="utf-8")
    result = audit(contract_path, register_path, root=tmp_path)
    assert result["gate_results"][gate_id]["state"] == "unbound"
