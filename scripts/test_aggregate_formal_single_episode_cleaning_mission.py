import argparse
import json
import time
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from aggregate_formal_single_episode_cleaning_mission import AggregateError, aggregate, canonical_session_id
from collect_formal_single_episode_cleaning_mission import (
    CONTROL_PROHIBITED_TRUTH_TOPICS, REQUIRED_RUNTIME_NODES,
    build_input_binding, sha256_file,
)
from generate_formal_same_map_baseline import build_report

MATERIALS = ("paperboard", "PP", "PET", "aluminum")
MASSES = {"paperboard": 0.0189, "PP": 0.0243, "PET": 0.03726, "aluminum": 0.0729}


def _write(path: Path, value: dict | str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value if isinstance(value, str) else json.dumps(value, sort_keys=True), encoding="utf-8")
    return path


def _grasp(target_id: str, material: str, mass: float, count: int, stamp: int) -> dict:
    pose = {"x_m": .3, "y_m": -.95, "z_m": .1, "qx": 0., "qy": 0., "qz": 0., "qw": 1.}
    planned = [{"step": step, "collision_checked": True, "ik_validated": True,
                "target_pose": pose, "trajectory_points": 2} for step in (
        "TARGET_CONDITIONED_PREGRASP", "WRIST_REFINED_PREGRASP",
        "LINEAR_CONTACT_APPROACH", "LINEAR_COLLISION_CHECKED_LIFT",
        "COLLISION_CHECKED_DEPOSIT", "COLLISION_CHECKED_BIN_RETREAT")]
    return {"schema_version": 2, "target_id": target_id, "verified_in_bin": True,
            "reason": "physical_cube_verified_in_bin", "collector_received_epoch_ns": stamp,
            "evidence": {"truth_used_for_control": False,
                "simulator_entity_identity_in_request": False,
                "planning_backend": "MoveGroup_action_GetPositionIK_GetCartesianPath",
                "perceived_target": {"target_id": target_id, "material": "unknown"},
                "wrist_near_field_recheck": {"accepted": True}, "sequence": planned,
                "physical_hold_after_lift": {"attachment_state_ack": True,
                                              "persistent_dual_finger_contact": False},
                "dry_bin_verification": {"physical_monitor_confirmed": True,
                    "dynamic_payload_increment_confirmed": True,
                    "measured_increment_kg": mass, "pre_grasp_material": "unknown",
                    "post_deposit_material_from_load_increment": material,
                    "contained_object_count": count}}}


def build_raw(tmp_path: Path) -> Path:
    episode_id, map_id, seed = "ep-001", "map-001", 701
    world = _write(tmp_path / "episode/public/world.sdf", "<sdf version='1.11'/>\n")
    public = {"schema_version": 1, "episode_id": episode_id, "map_id": map_id,
        "profile": "formal", "field": {"width_m": 200., "height_m": 100.,
            "area_m2": 20000., "geofence_polygon_m": [[-100,-50],[100,-50],[100,50],[-100,50]]},
        "counts": {"discrete_cubes": 20, "pedestrians": 3},
        "cube_contract": {"edge_m": .03},
        "dynamic_pedestrians_present": True,
        "vehicle_start_pose_map": {"x_m": -98., "y_m": 0., "yaw_rad": 0.}}
    episode = _write(tmp_path / "episode/public/episode_manifest.json", public)
    seeds = {"layout": 700, "dirt": seed, "cubes": 702, "pedestrians": 703, "sensor": 704}
    evaluator_manifest = _write(tmp_path / "episode/evaluator/episode_manifest.json", {
        "episode_id": episode_id, "map_id": map_id, "seeds": seeds,
        "world_sha256": sha256_file(world), "truth_boundary": {"control_use_prohibited": True},
        "runtime_environment": {"pedestrian_schedule": "environment/pedestrian_schedule.json"}})
    cubes = []
    for index in range(20):
        material = MATERIALS[index % 4]
        cubes.append({"object_id": f"cube-{index:02d}", "edge_m": .03,
                      "material": material, "mass_kg": MASSES[material]})
    truth = _write(tmp_path / "episode/evaluator/ground_truth.json", {
        "episode_id": episode_id, "map_id": map_id, "control_use_prohibited": True,
        "dirt_union_area_m2": 1., "discrete_cubes": cubes,
        "pedestrians": [{"object_id": f"ped-{i}"} for i in range(3)]})
    schedule = _write(tmp_path / "episode/environment/pedestrian_schedule.json", {
        "pedestrians": [{"object_id": f"ped-{i}"} for i in range(3)]})
    snapshot = _write(tmp_path / "snapshot.json", {
        "source_inventory_sha256": "b" * 64,
        "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {
            "sha256": "c" * 64,
        }},
    })
    # Keep the synthetic session unambiguously older than subsequently written
    # fixture evidence on Windows filesystems and virtualized test volumes.
    # Ten milliseconds was smaller than the observed wall-clock/mtime skew and
    # made the freshness test nondeterministic without exercising production
    # behavior.
    started = time.time_ns() - 1_000_000_000
    session_payload = {"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": started, "snapshot": {
            "snapshot_manifest_sha256": sha256_file(snapshot),
            "source_inventory_sha256": "b" * 64,
            "expanded_urdf_sha256": "c" * 64,
        }}
    session = _write(tmp_path / "session.json", session_payload)
    saved_map = tmp_path / "saved_map"
    saved_map.mkdir()
    _write(saved_map / "occupancy.yaml", "image: occupancy.pgm\n")
    mission = {
        "schema_version": 1, "mission_id": f"formal-lifecycle-{episode_id}",
        "vehicle_start_pose_map": {"x_m": 0., "y_m": 0., "yaw_rad": 0.},
        "source_fixed_start_pose": [-98., 0., 0.],
        "truth_boundary": {"world_geometry_used_for_product_map": False,
            "evaluator_truth_used": False, "dirt_truth_used": False},
    }
    (saved_map / "mission_geometry.yaml").write_text(
        yaml.safe_dump(mission, sort_keys=False), encoding="utf-8"
    )
    manifest_hashes = {name: sha256_file(saved_map / name)
                       for name in ("occupancy.yaml", "mission_geometry.yaml")}
    _write(saved_map / "map_lifecycle_manifest.json", {
        "status": "ready_for_localization_cleaning", "episode_id": episode_id,
        "map_id": map_id, "observed_fraction": .96, "fixed_start_verified": True,
        "mapping_ignored_dirt": True, "world_truth_used_for_control": False,
        "sha256": manifest_hashes,
    })
    mapping_runtime = _write(tmp_path / "mapping_runtime.json", {
        "passed": True, "truth_used_for_control": False,
        "robot_description_sha256": "c" * 64,
    })
    cleaning_runtime = _write(tmp_path / "cleaning_runtime.json", {
        "passed": True, "truth_used_for_control": False,
        "localization_backend": "amcl", "robot_description_sha256": "c" * 64,
        "saved_map_sha256_verified": True, "hard_restart_verified": True,
        "cleaning_stack_ready": True, "coverage_server_ready": True,
        "world_derived_map_fallback": False,
        "hard_restart_record": {"mapping_stopped_before_cleaning": True,
            "mapping_process_count_before_cleaning": 0,
            "restart_type": "separate_process_hard_restart"},
    })
    lifecycle = _write(tmp_path / "lifecycle.json", {
        "status": "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED",
        "passed": True, "truth_used_for_control": False,
        "checks": {"map": True, "mapping": True, "cleaning": True},
    })
    coverage = _write(tmp_path / "coverage.json", {
        "schema_version": 2, "mission_id": mission["mission_id"],
        "planner": "OpenNav Coverage + Fields2Cover", "success": True,
        "planning_success": True, "full_execution_success": True,
        "coverage_quality_success": True, "safety_success": True,
        "localization_success": True,
        "competition_efficiency_pass": True,
        "evaluation_injection": {"ground_truth_used_for_control": False},
        "planned_metrics": {"path_length_m": 980.},
        "empirical_metrics": {
            "actual_path_length_m": 1000.,
            "covered_area_m2": 20000.,
            "actual_duration_sec": 20000.,
            "net_efficiency_m2_h": 3600.,
        },
    })
    baseline = _write(tmp_path / "baseline.json", build_report(
        episode_manifest=episode, map_root=saved_map,
        mapping_runtime=mapping_runtime, cleaning_runtime=cleaning_runtime,
        lifecycle_acceptance=lifecycle, coverage_runtime=coverage,
        session_path=session, snapshot_path=snapshot,
    ))
    policy = _write(tmp_path / "q_policy.json", {"policy": "q_learning",
        "truth_access_used": False, "q_table": {"state": {"frontier": 1.}}})
    runtime_binding = _write(tmp_path / "runtime_binding.json", {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "session_started_epoch_ns": started,
            "snapshot": session_payload["snapshot"],
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "manifest_sha256": "d" * 64,
            "runtime_install_root": str(tmp_path.resolve()),
        },
    })
    perception = tmp_path / "perception"
    _write(perception / "manifest.json", {"ready": True})
    binding = build_input_binding(argparse.Namespace(
        episode_manifest=episode, evaluator_episode_manifest=evaluator_manifest,
        evaluator_ground_truth=truth, world=world, pedestrian_schedule=schedule,
        session_status=session, same_map_baseline=baseline, policy_checkpoint=policy,
        runtime_binding=runtime_binding,
        saved_map=saved_map, perception_artifacts=perception))
    binding_path = _write(tmp_path / "input_binding.json", binding)
    identity = {"session_id": canonical_session_id(session_payload), "episode_id": episode_id,
        "episode_seed": seed, "runtime_id": "runtime-1", "gazebo_process_id": 99,
        "session_start_epoch_ns": started, "ros_domain_id": 251,
        "gz_partition": "single-episode-runtime-1"}
    topic_classes = {"planner":"product", "mission_complete":"product", "trajectory":"product",
        "grasp_result":"product", "odometry":"product", "ground_dirt":"evaluator_truth",
        "water":"evaluator_truth", "dry_bin":"evaluator_truth", "pedestrians":"evaluator_truth",
        "collision":"evaluator_truth", "front_bumper":"evaluator_truth", "rear_bumper":"evaluator_truth"}
    sources = [{**identity, "metric": name, "topic": f"/{name}",
                "source_class": source, "sample_count": 2} for name, source in topic_classes.items()]
    dry_mass = sum(cube["mass_kg"] for cube in cubes)
    grasps = [_grasp(cube["object_id"], cube["material"], cube["mass_kg"], index + 1,
                     started + 1_000_000 + index) for index, cube in enumerate(cubes)]
    initial = {"ground_dirt": {"initial_area_m2": 1., "cleaned_area_m2": 0.},
        "water": {"ground_volume_l": 2., "recovered_volume_l": 0., "tank_mass_kg": 1.},
        "dry_bin": {"contained_object_count": 0, "physical_contained_mass_kg": 0.},
        "pedestrians": {"state": "WAITING_FOR_SET_POSE"}}
    terminal = {"ground_dirt": {"initial_area_m2": 1., "cleaned_area_m2": .96,
            "left_ready": True, "right_ready": True, "roller_ready": True,
            "rigid_litter_entities_modified": 0},
        "water": {"ground_volume_l": .08, "recovered_volume_l": 1.92,
            "tank_mass_kg": 2.92, "brush_ready": True, "squeegee_ready": True,
            "nozzle_ready": True, "pump_ready": True},
        "dry_bin": {"contained_object_count": 20, "physical_contained_mass_kg": dry_mass},
        "pedestrians": {"state": "ACTIVE", "pedestrian_count": 3}}
    subscribers = {topic: ["/formal_single_episode_cleaning_collector"]
                   for topic in CONTROL_PROHIBITED_TRUTH_TOPICS}
    planner = {"diagnostic_name": "formal_active_cleaning_policy_planner",
        "hardware_id": "frozen_truth_free_q_policy", "level": 0, "state": "COMPLETE",
        "reason": "task_complete_and_fixed_start_pose_reached", "truth_used_for_control": "false",
        "product_inputs_fresh": "true", "observed_ratio": ".96",
        "task_distance_m_excluding_return": "900", "return_distance_m": "30",
        "returning_home": "true"}
    planner_initial = {**planner, "state": "RUNNING", "reason": "cleaning",
        "observed_ratio": ".10", "returning_home": "false"}
    task_trajectory = [[0., 0.], [300., 0.], [600., 0.], [900., 0.]]
    return_trajectory = [[900., 0.], [910., 0.], [920., 0.], [930., 0.]]
    trajectory_evidence = [
        {"collector_received_epoch_ns": started + index + 1,
         "frame_id": "map", "pose_count": 2,
         "trajectory_xy_m": [[float(index), 0.], [float(index + 1), 0.]]}
        for index in range(4)
    ]
    raw = {"schema_version": 2, "artifact_kind": "single_live_episode_raw_collection",
        "created_epoch_ns": time.time_ns(), "run_identity": identity,
        "input_binding": {"path": str(binding_path.resolve()), "sha256": sha256_file(binding_path),
                          "artifacts": binding["artifacts"]}, "metric_sources": sources,
        "runtime_graph": {"nodes": sorted(REQUIRED_RUNTIME_NODES | {"/formal_single_episode_cleaning_collector"}),
            "required_nodes": sorted(REQUIRED_RUNTIME_NODES), "required_nodes_present": True,
            "control_prohibited_truth_topic_subscribers": subscribers},
        "runtime_parameters": {"/formal_active_cleaning_policy_planner": {
                "policy_checkpoint": str(policy.resolve()), "episode_seed": seed,
                "maximum_task_distance_m": 1000.},
            "/pc_open_vocab_product_adapter": {"artifact_root": str(perception.resolve())},
            "/formal_map_lifecycle_manager": {"mode": "cleaning",
                "episode_manifest": str(episode.resolve()), "artifact_directory": str(saved_map.resolve())}},
        "product": {"planner_status": planner, "mission_complete": True,
            "trajectory_publish_count": 4, "grasp_results": grasps,
            "trajectory_evidence": trajectory_evidence,
            "planner_status_samples": [planner_initial, planner],
            "successful_grasp_target_ids": sorted(cube["object_id"] for cube in cubes),
            "odom_sample_count": 100, "return_started_seen": True,
            "operator_start_received": True,
            "task_odom_trajectory_xy_m": task_trajectory,
            "return_odom_trajectory_xy_m": return_trajectory,
            "return_start_state": {"planner_status": planner,
                "evaluator": {"ground_dirt": terminal["ground_dirt"], "water": terminal["water"],
                              "dry_bin": terminal["dry_bin"]},
                "successful_grasp_target_ids": sorted(cube["object_id"] for cube in cubes)}},
        "evaluator": {"initial": initial, "terminal": terminal, "collision_count": 0,
                      "collision_monitor_intervention_count": 2},
        "collector_ready_before_operator_start": True, "timed_out": False}
    return _write(tmp_path / "raw.json", raw)


def test_aggregate_one_live_run_has_manifest_derived_and_delta_evidence(tmp_path: Path) -> None:
    result = aggregate(build_raw(tmp_path))
    assert result["field"]["source"] == "episode.public.field"
    assert result["planning"]["planner"] == "q_learning"
    assert result["planning"]["same_map_full_coverage_efficiency"] == {
        "threshold_m2_h": 3500.0,
        "covered_area_m2": 20000.0,
        "actual_duration_sec": 20000.0,
        "measured_net_efficiency_m2_h": 3600.0,
        "recomputed_net_efficiency_m2_h": 3600.0,
        "return_distance_included": False,
        "passed": True,
    }
    assert result["discrete_litter"]["episode_target_ids"] == result["discrete_litter"]["successful_target_ids"]
    assert result["water_recovery"]["tank_mass_increment_kg"] == pytest.approx(1.92)


def test_rejects_changed_directory_after_prelaunch_freeze(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    _write(tmp_path / "perception/new-file.bin", "changed")
    with pytest.raises(AggregateError, match="input hash mismatch"):
        aggregate(path)


def test_rejects_truth_subscription_by_product_node(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    topic = next(iter(row["runtime_graph"]["control_prohibited_truth_topic_subscribers"]))
    row["runtime_graph"]["control_prohibited_truth_topic_subscribers"][topic].append("/planner")
    _write(path, row)
    with pytest.raises(AggregateError, match="non-collector subscriber"):
        aggregate(path)


def test_rejects_grasp_id_or_ik_inference(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    row["product"]["grasp_results"][0]["evidence"]["sequence"][0]["ik_validated"] = False
    _write(path, row)
    with pytest.raises(AggregateError, match="IK/collision proof missing"):
        aggregate(path)


def test_rejects_positive_but_nonincrementing_bin_mass(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    row["evaluator"]["initial"]["dry_bin"]["physical_contained_mass_kg"] = row["evaluator"]["terminal"]["dry_bin"]["physical_contained_mass_kg"]
    _write(path, row)
    with pytest.raises(AggregateError, match="not empty before operator start"):
        aggregate(path)


def test_rejects_planner_mileage_not_supported_by_recorded_odom(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    row["product"]["task_odom_trajectory_xy_m"][-1][0] = 700.0
    _write(path, row)
    with pytest.raises(AggregateError, match="task mileage differs"):
        aggregate(path)


def test_rejects_terminal_only_coverage_claim(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    row["product"]["planner_status_samples"] = [row["product"]["planner_status"]]
    _write(path, row)
    with pytest.raises(AggregateError, match="observed-area progress"):
        aggregate(path)


def test_rejects_unverified_runtime_closure_binding(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    raw = json.loads(path.read_text())
    binding_path = Path(raw["input_binding"]["artifacts"]["runtime_binding"]["path"])
    binding = json.loads(binding_path.read_text())
    binding["runtime_closure_binding"]["status"] = "BLOCKED"
    binding_path.write_text(json.dumps(binding), encoding="utf-8")
    with pytest.raises(AggregateError, match="input hash mismatch"):
        aggregate(path)


def test_rejects_positive_but_nonincrementing_wet_tank_mass(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    row = json.loads(path.read_text())
    row["evaluator"]["terminal"]["water"]["tank_mass_kg"] = row["evaluator"]["initial"]["water"]["tank_mass_kg"]
    _write(path, row)
    with pytest.raises(AggregateError, match="wet-tank mass increment"):
        aggregate(path)


def test_rejects_tampered_same_map_competition_efficiency(tmp_path: Path) -> None:
    path = build_raw(tmp_path)
    raw = json.loads(path.read_text())
    baseline_path = Path(raw["input_binding"]["artifacts"]["same_map_baseline"]["path"])
    baseline = json.loads(baseline_path.read_text())
    baseline["competition_efficiency"]["measured_net_efficiency_m2_h"] = 3499.0
    _write(baseline_path, baseline)
    with pytest.raises(AggregateError, match="input hash mismatch"):
        aggregate(path)
