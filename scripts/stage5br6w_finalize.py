#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--review", type=Path, required=True)
    args = parser.parse_args()
    root, runtime, review = args.root.resolve(), args.runtime.resolve(), args.review.resolve()
    review.mkdir(parents=True, exist_ok=True)
    coverage = read_json(runtime / "coverage_report.json")
    footprint_audit = read_json(runtime / "runtime_footprint_audit.json")
    policy = root / "starter_ws/src/sanitation_learning/config/perception_evaluability_policy_v2_engineering.yaml"
    footprint = root / "starter_ws/src/sanitation_navigation/config/stage5br6w_v4_candidate_footprint.yaml"
    waiver = root / "config/engineering_waiver_stage5br6w.json"
    local_bag = runtime / "static_coverage_bag/metadata.yaml"
    replay_observed = (runtime / "replay_coverage_state.txt").is_file() and (runtime / "replay_coverage_state.txt").stat().st_size > 0

    first_failure = "stage5br6w_phase4_candidate_footprint_static_seed0_no_reachable_clean_route"
    regression = {
        "schema_version": 1,
        "stage": "Stage5BR6W-Phase4",
        "profile": "stage5br6w_v4",
        "required_static_trials": 5,
        "completed_static_trials": 1,
        "successful_static_trials": 0,
        "static_matrix_pass": False,
        "seed0": {
            "planning_success": coverage["planning_success"],
            "transit_to_start_success": coverage["transit_to_start_success"],
            "full_execution_success": coverage["full_execution_success"],
            "component_count": coverage["component_count"],
            "empirical_coverage_rate": coverage["empirical_metrics"]["coverage_rate"],
            "collision_count": coverage["collision_count"],
            "keepout_violation_sample_count": coverage["keepout_violation_sample_count"],
            "brush_disabled_on_exit": coverage["brush_disabled_on_exit"],
            "hybrid_localization_rmse_m": coverage["localization_regression_during_coverage"]["rmse_m"],
            "hybrid_localization_formal_gate_pass": coverage["localization_regression_during_coverage"]["pass_rmse_at_most_0_05m"],
            "failure": coverage["transit_to_start"]["error"],
            "swath_exclusion_intersection_count": coverage["swath_exclusion_intersection_count"],
            "cleanable_area_m2": coverage["mission_geometry"]["cleanable_area_m2"],
            "footprint_radius_m": coverage["mission_geometry"]["footprint_radius_m"],
        },
        "dynamic_interactions_required": 20,
        "dynamic_interactions_executed": 0,
        "estop_trials_required": 30,
        "estop_trials_executed": 0,
        "rosbag_recorded": local_bag.is_file(),
        "rosbag_replay": replay_observed,
        "runtime_footprint_audit_pass": footprint_audit["all_runtime_consumers_same_candidate_footprint"],
        "first_blocking_layer": first_failure,
        "phase4_pass": False,
        "stop_before_oracle": True,
    }
    (review / "stage4w_candidate_footprint_regression.json").write_text(json.dumps(regression, indent=2) + "\n", encoding="utf-8")

    planner = {
        "schema_version": 1,
        "stage": "Stage5BR6W",
        "full_camera_se3": True,
        "v4_lateral_offset": True,
        "actual_camera_info_required_in_engineering_mode": True,
        "full_candidate_footprint_polygon": True,
        "polygon_boundary_keepout_intersection": True,
        "costmap_footprint_cost_required": True,
        "pose_dependent_target_self_overlap": True,
        "predicted_roi_and_short_side": True,
        "path_length_turning_clearance_scoring": True,
        "missing_engineering_input_fail_closed": True,
        "no_feasible_pose_terminal": "UNREACHABLE",
        "targeted_pytest_passed": True,
        "runtime_projection_calibration_executed": False,
        "runtime_projection_calibration_blocker": first_failure,
    }
    (review / "planner_hardening_report.json").write_text(json.dumps(planner, indent=2) + "\n", encoding="utf-8")

    status = {
        "schema_version": 1,
        "stage": "Stage5BR6W",
        "engineering_waiver": read_json(waiver),
        "historical_stage5br6a_preserved": True,
        "reviewer_packages_modified": False,
        "sealed_truth_modified": False,
        "engineering_verification_camera": "V4",
        "engineering_camera_candidate_frozen": True,
        "competition_camera_selected": False,
        "human_validated": False,
        "engineering_policy": {"path": str(policy.relative_to(root)).replace("\\", "/"), "sha256": sha256(policy), "competition_metric_eligible": False},
        "candidate_footprint": {"path": str(footprint.relative_to(root)).replace("\\", "/"), "sha256": sha256(footprint), "runtime_audit_pass": footprint_audit["all_runtime_consumers_same_candidate_footprint"]},
        "phase4_footprint_regression": regression,
        "phase5_oracle_active_observation": {
            "executed": False,
            "world_count": 0,
            "scene_count": 0,
            "valid_target_trial_count": 0,
            "unreachable_keepout_case_count": 0,
            "false_no_target_case_count": 0,
            "blocker": first_failure,
        },
        "detector_or_area_model_training_executed": False,
        "j6_gate_executed": False,
        "first_blocking_layer": first_failure,
        "AWAITING_HUMAN_REVIEW": True,
        "HUMAN_REVIEW_COMPLETED": False,
        "MANUAL_AUDIT_PASS": False,
        "READY_FOR_STAGE5BR6_ORACLE": False,
        "READY_FOR_GPT_REVIEW_STAGE5BR6": False,
        "READY_FOR_STAGE5BR7": False,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
        "READY_FOR_STAGE5BR6W_ORACLE_ENGINEERING": False,
        "READY_FOR_STAGE5BR7_ENGINEERING": False,
        "competition_perception_pass": False,
        "real_domain_evaluation_executed": False,
        "j6_runtime_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": 1053,
        "target_efficiency_m2_h": 3500,
        "REVIEW_PACKET_COMPLETE": True,
    }
    (review / "stage5br6w_status.json").write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")

    for source, name in (
        (waiver, "engineering_waiver_stage5br6w.json"),
        (policy, "perception_evaluability_policy_v2_engineering.yaml"),
        (footprint, "stage5br6w_v4_candidate_footprint.yaml"),
        (runtime / "runtime_footprint_audit.json", "runtime_footprint_audit.json"),
        (runtime / "coverage_report.json", "seed0_coverage_report.json"),
        (runtime / "rosbag_info.txt", "seed0_rosbag_info.txt"),
    ):
        shutil.copy2(source, review / name)

    manifest = []
    for path in sorted(review.iterdir()):
        if path.is_file() and path.name != "artifact_manifest.json":
            manifest.append({"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)})
    (review / "artifact_manifest.json").write_text(json.dumps({"schema_version": 1, "stage": "Stage5BR6W", "files": manifest}, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
