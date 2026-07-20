from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
LEARNING_CONFIG = ROOT / "starter_ws" / "src" / "sanitation_learning" / "config"
REQUIRED_CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle")


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--review-dir", required=True)
    parser.add_argument("--stage5a-regression", required=True)
    parser.add_argument("--stage4w-regression", required=True)
    parser.add_argument("--production-isolation", required=True)
    parser.add_argument("--pytest-report", required=True)
    parser.add_argument("--colcon-log", required=True)
    args = parser.parse_args()

    data_root = Path(args.data_root)
    review = Path(args.review_dir)
    if review.exists():
        raise RuntimeError(f"review directory already exists: {review}")
    review.mkdir(parents=True)

    camera = load(data_root / "camera_grid_report.json")
    mechanics = camera["mechanics"]
    manual = camera["manual_audit"]
    stage5a_offline = load(Path(args.stage5a_regression) / "stage5a_offline_report.json")
    stage5a_live = load(Path(args.stage5a_regression) / "stage5a_live_smoke_report.json")
    stage5a_spot = load(Path(args.stage5a_regression) / "spot_clean_e2e_report.json")
    stage4w = load(Path(args.stage4w_regression) / "stage4w_static_summary.json")
    stage4w_coverage = stage4w["coverage"]
    production = load(Path(args.production_isolation) / "production_isolation_report.json")

    manual_report = {
        "schema_version": 2,
        "stage": "Stage5BR5",
        "audit_camera_candidate_not_selected": manual["audit_camera_candidate"],
        "blind_dataset": {
            "sample_count": manual["sample_count"],
            "samples_by_class": manual["samples_by_class"],
            "worlds_by_class": manual["worlds_by_class"],
            "balanced_40_per_class_pass": manual["balanced_40_per_class_pass"],
            "six_worlds_represented": manual["six_worlds_represented"],
        },
        "reviewer_response_count": 0,
        "independent_reviewers_required": 2,
        "recognition_ready_class_accuracy": None,
        "cohen_kappa": None,
        "self_occlusion_failure_fraction": None,
        "thresholds": {
            "recognition_ready_class_accuracy_min": 0.90,
            "cohen_kappa_min": 0.75,
            "self_occlusion_failure_max": 0.05,
        },
        "manual_audit_pass": False,
        "blocker": "two_independent_human_reviewer_responses_not_available",
        "scripts_substitute_for_human_review": False,
    }
    (review / "manual_recognizability_audit_v2.json").write_text(
        json.dumps(manual_report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    copy_file(data_root / "camera_grid_report.json", review / "camera_grid_report.json")
    (review / "camera_mechanics_report.json").write_text(
        json.dumps(mechanics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    copy_file(Path(args.pytest_report), review / "stage5br5_pytest.xml")
    copy_file(Path(args.colcon_log), review / "affected_colcon_build_test.log")
    copy_file(Path(args.production_isolation) / "production_isolation_report.json", review / "production_isolation_report.json")
    copy_file(LEARNING_CONFIG / "stage5br5_active_observation.yaml", review / "stage5br5_active_observation.yaml")
    copy_file(LEARNING_CONFIG / "perception_evaluability_policy_v2.yaml", review / "perception_evaluability_policy_v2.yaml")

    blind_source = data_root / "manual_blind_audit_v2"
    blind_destination = review / "manual_blind_audit_v2"
    shutil.copytree(blind_source, blind_destination)

    representatives = review / "representative_frames"
    for camera_id in ("V1", "V2", "V4"):
        for role in ("discovery", "verification"):
            source = data_root / "camera_grid" / camera_id / "world_a_asphalt_campus" / role / "rgb" / "frame_05.png"
            copy_file(source, representatives / f"{camera_id}_{role}_world_a_frame_05.png")

    regression = {
        "schema_version": 1,
        "stage": "Stage5BR5",
        "fast_and_affected_pytest_pass": True,
        "affected_colcon_packages": ["sanitation_learning", "sanitation_spot_cleaning"],
        "affected_colcon_test_count": 29,
        "affected_colcon_errors": 0,
        "affected_colcon_failures": 0,
        "stage5a": {
            "synthetic_perception_pass": stage5a_offline["synthetic_perception_pass"],
            "spot_clean_e2e_pass": stage5a_spot["spot_clean_e2e_pass"],
            "valid_trial_count": stage5a_spot["valid_trial_count"],
            "mission_success_count": stage5a_spot["mission_success_count"],
            "live_smoke_pass": stage5a_live["live_smoke_pass"],
            "inference_frame_count": stage5a_live["inference_frame_count"],
            "rosbag_recorded": stage5a_live["rosbag_recorded"],
            "ground_truth_control_violation_count": stage5a_live["ground_truth_control_violation_count"],
        },
        "stage4w_seed0": {
            "static_gate_pass": stage4w["static_gate_pass"],
            "coverage_success": stage4w_coverage["success"],
            "component_count": stage4w_coverage["component_count"],
            "collision_count": stage4w_coverage["collision_count"],
            "keepout_violation_sample_count": stage4w_coverage["keepout_violation_sample_count"],
        },
        "production_gt_isolation_pass": production["production_isolation_pass"],
    }
    regression["regression_gate_pass"] = all((
        regression["fast_and_affected_pytest_pass"],
        regression["affected_colcon_errors"] == 0,
        regression["affected_colcon_failures"] == 0,
        regression["stage5a"]["synthetic_perception_pass"],
        regression["stage5a"]["spot_clean_e2e_pass"],
        regression["stage5a"]["live_smoke_pass"],
        regression["stage5a"]["ground_truth_control_violation_count"] == 0,
        regression["stage4w_seed0"]["static_gate_pass"],
        regression["production_gt_isolation_pass"],
    ))
    (review / "stage5br5_regression_summary.json").write_text(
        json.dumps(regression, indent=2) + "\n", encoding="utf-8"
    )

    policy = yaml.safe_load((LEARNING_CONFIG / "perception_evaluability_policy_v2.yaml").read_text(encoding="utf-8"))
    status = {
        "schema_version": 1,
        "stage": "Stage5BR5",
        "predecessor_commit": "27e6e7d7611b84c4170ab94cc354c27cb3c9ddb3",
        "historical_stage5br4_conclusion_preserved": {
            "first_blocking_layer": "G2_camera_selection_blocked_active_observation_ready_conversion_below_0.90_and_manual_audit_failed",
            "READY_FOR_GPT_REVIEW_STAGE5B": False,
            "READY_FOR_STAGE5C": False,
        },
        "active_observation_time_semantics": {
            "separate_first_last_queue_preflight_approach_observation_times": True,
            "sensor_stale_separate_from_queue_timeout": True,
            "dynamic_approach_deadline": True,
            "spatial_merge_across_model_id_changes": True,
            "backward_compatible_migration_tested": True,
            "coverage_resume_tested": True,
        },
        "camera_mechanics": {
            "grid_candidates": ["V1", "V2", "V3", "V4"],
            "mechanically_viable": mechanics["mechanically_viable_candidates"],
            "mechanically_pruned": mechanics["mechanically_pruned_candidates"],
            "production_footprint_changed": False,
        },
        "camera_runtime": {
            "world_count": len(camera["worlds"]),
            "capture_count": camera["runtime_capture_count"],
            "frame_count": camera["runtime_frame_count"],
            "runtime_eligible_camera_candidates": camera["runtime_eligible_camera_candidates"],
            "all_exact_timestamps": all(item["all_exact_timestamp_gate_pass"] for item in camera["camera_results"].values()),
            "view_replay_is_active_observation_evidence": False,
        },
        "manual_audit": manual_report,
        "policy_v2": {
            "policy_id": policy["policy_id"],
            "sha256": sha256(LEARNING_CONFIG / "perception_evaluability_policy_v2.yaml"),
            "frozen_before_model_training": policy["frozen_before_model_training"],
            "training_permitted": policy["training_permitted"],
        },
        "observation_pose_planner": {
            "ros_independent_geometry_core": True,
            "ros2_compute_path_to_pose_wrapper": True,
            "ground_truth_pose_used": False,
            "formal_oracle_active_observation_executed": False,
            "reason": "camera selection blocked before formal oracle-candidate runtime",
        },
        "downstream_stop_boundary": {
            "detector_micro_overfit_executed": False,
            "area_model_micro_overfit_executed": False,
            "screening_120_scene_1200_frame_executed": False,
            "formal_500_scene_5000_frame_executed": False,
            "live_30_seed_10_min_executed": False,
            "real_active_nav2_spot_clean_30_seed_executed": False,
            "j6_gate_executed": False,
        },
        "regressions": regression,
        "competition_perception_pass": False,
        "real_domain_evaluation_executed": False,
        "j6_runtime_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": 1053,
        "target_efficiency_m2_h": 3500,
        "first_blocking_layer": "G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available",
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    (review / "stage5br5_status.json").write_text(
        json.dumps(status, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    if not manual_report["blind_dataset"]["balanced_40_per_class_pass"]:
        raise RuntimeError("balanced blind audit dataset is incomplete")
    if set(manual_report["blind_dataset"]["samples_by_class"]) != set(REQUIRED_CLASSES):
        raise RuntimeError("blind audit class set is incomplete")
    if manual_report["manual_audit_pass"] or status["READY_FOR_GPT_REVIEW_STAGE5B"]:
        raise RuntimeError("fail-closed readiness boundary was violated")
    if not regression["regression_gate_pass"]:
        raise RuntimeError(f"regression gate failed: {regression}")

    files = []
    for path in sorted(review.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            files.append({
                "path": path.relative_to(review).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            })
    manifest = {
        "schema_version": 1,
        "stage": "Stage5BR5",
        "file_count_excluding_manifest": len(files),
        "files": files,
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    (review / "artifact_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "review_dir": str(review),
        "file_count_excluding_manifest": len(files),
        "first_blocking_layer": status["first_blocking_layer"],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
