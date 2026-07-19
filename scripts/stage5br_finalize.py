#!/usr/bin/env python3
"""Build a compact, fail-closed Stage5BR review artifact."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PHASE_A = ROOT / "artifacts" / "stage5br_phase_a_run3"
G1 = ROOT / "artifacts" / "stage5br_g1_smoke50_scaled"
MODEL_RUNS = [ROOT / "artifacts" / f"stage5br_g1_model_screening{i}" for i in (1, 2, 3)]
STAGE5A = ROOT / "artifacts" / "stage5br_stage5a_regression"
STAGE4W = ROOT / "artifacts" / "stage5br_stage4w_regression_seed0"
ROS = ROOT / "artifacts" / "stage5br_ros_validation"
REVIEW = ROOT / "artifacts" / "stage5br_20260719_review"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def copy(source: Path, name: str | None = None) -> None:
    shutil.copy2(source, REVIEW / (name or source.name))


def model_metrics(report: dict) -> dict:
    return {
        "in_domain_macro_f1": report["in_domain"]["macro_f1"],
        "cross_asset_world_macro_f1": report["cross_asset_world"]["macro_f1"],
        "cross_asset_world_leaf_puddle_miou": report["cross_asset_world"]["leaf_puddle_miou"],
        "color_stress_macro_f1": report["color_stress"]["aggregate_macro_f1"],
        "held_out_test_macro_f1": report["held_out_test_after_selection"]["macro_f1"],
        "screening_pass": report["G1_model_screening_pass"],
    }


def build_visuals() -> None:
    visuals = REVIEW / "visuals"
    visuals.mkdir(parents=True, exist_ok=True)
    for index in range(2):
        copy(PHASE_A / f"micro_overlay_{index:02d}.png", f"visuals/micro_overlay_{index:02d}.png")
    scene = G1 / "scenes" / "scene_0007"
    rgb = cv2.imread(str(scene / "images" / "scene_0007_frame_05.png"))
    labels = np.load(scene / "semantic" / "scene_0007_frame_05.npy", allow_pickle=False)
    palette_rgb = np.asarray([
        (30, 30, 30), (64, 180, 255), (220, 100, 70),
        (245, 220, 80), (80, 190, 90), (60, 120, 230),
    ], dtype=np.uint8)
    overlay = cv2.cvtColor(palette_rgb[labels], cv2.COLOR_RGB2BGR)
    composed = np.concatenate((rgb, overlay, cv2.addWeighted(rgb, 0.55, overlay, 0.45, 0)), axis=1)
    cv2.putText(composed, "G1 actual Gazebo RGB | semantic GT | overlay", (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(visuals / "g1_gazebo_sample.png"), composed)
    for run_index in (1, 2, 3):
        source = MODEL_RUNS[run_index - 1] / "g1_cross_asset_overlay_00.png"
        copy(source, f"visuals/model_attempt_{run_index}_cross_overlay.png")
    report = load(MODEL_RUNS[2] / "g1_model_screening_report.json")
    canvas = np.full((520, 900, 3), 248, dtype=np.uint8)
    cv2.putText(canvas, "Stage5BR final model attempt: validation metrics", (45, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (25, 25, 25), 2, cv2.LINE_AA)
    curves = report["curves"]
    x0, y0, width, height = 70, 470, 760, 370
    cv2.rectangle(canvas, (x0, y0 - height), (x0 + width, y0), (60, 60, 60), 1)
    series = [
        ("in-domain F1", "in_domain_macro_f1", (40, 150, 40)),
        ("cross-asset/world F1", "cross_asset_world_macro_f1", (200, 80, 40)),
        ("cross leaf/puddle mIoU", "cross_asset_world_leaf_puddle_miou", (40, 80, 210)),
    ]
    max_epoch = max(item["epoch"] for item in curves)
    for legend_index, (label, key, color) in enumerate(series):
        points = []
        for item in curves:
            x = x0 + int(width * item["epoch"] / max_epoch)
            y = y0 - int(height * float(item[key]))
            points.append((x, y))
        cv2.polylines(canvas, [np.asarray(points, np.int32)], False, color, 2)
        cv2.putText(canvas, label, (520, 75 + legend_index * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
    for value in (0.25, 0.5, 0.75, 1.0):
        y = y0 - int(height * value)
        cv2.line(canvas, (x0, y), (x0 + width, y), (215, 215, 215), 1)
        cv2.putText(canvas, f"{value:.2f}", (20, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    cv2.imwrite(str(visuals / "model_attempt_3_curves.png"), canvas)


def main() -> None:
    if REVIEW.exists():
        shutil.rmtree(REVIEW)
    REVIEW.mkdir(parents=True)
    micro = load(PHASE_A / "micro_overfit_report.json")
    parity = load(PHASE_A / "pipeline_parity_report.json")
    ladder = load(PHASE_A / "split_ladder_report.json")
    g1_summary = load(G1 / "g1_smoke_summary.json")
    g1_qa = load(G1 / "g1_annotation_qa.json")
    model_reports = [load(path / "g1_model_screening_report.json") for path in MODEL_RUNS]
    stage5a_offline = load(STAGE5A / "stage5a_offline_report.json")
    stage5a_live = load(STAGE5A / "stage5a_live_smoke_report.json")
    spot = load(STAGE5A / "spot_clean_e2e_report.json")
    coverage = load(STAGE4W / "coverage_report.json")
    attempts = [model_metrics(report) for report in model_reports]
    status = {
        "schema_version": 1,
        "stage": "Stage5BR Gazebo-camera data recovery and model screening boundary",
        "phase_a": {
            "micro_overfit_pass": micro["micro_overfit_pass"],
            "micro_macro_f1": micro["final_metrics"]["macro_f1"],
            "micro_foreground_miou": micro["final_metrics"]["foreground_miou"],
            "pipeline_parity_pass": parity["pipeline_parity_pass"],
            "pytorch_onnx_max_logit_error": parity["pytorch_onnx_max_logit_error"],
            "pytorch_onnx_argmax_agreement": parity["pytorch_onnx_argmax_agreement"],
            "P1_first_collapse_layer": ladder["first_collapse_layer_below_macro_f1_0_70"],
        },
        "G1_smoke": {
            "scene_count": g1_qa["actual_scene_count"], "frame_count": g1_qa["actual_frame_count"],
            "annotation_completeness": g1_qa["annotation_completeness"],
            "sampled_label_error_rate": g1_qa["sampled_label_error_rate"],
            "exact_sensor_timestamp_match": g1_qa["rgb_depth_semantic_instance_exact_timestamp_match"],
            "asset_leakage": g1_qa["asset_leakage"],
            "cross_split_exact_duplicate_count": g1_qa["cross_split_exact_duplicate_count"],
            "cross_split_perceptual_duplicate_count": g1_qa["cross_split_perceptual_duplicate_count"],
            "G1_smoke_pass": g1_summary["G1_smoke_pass"],
        },
        "model_attempts": attempts,
        "best_screening_attempt_by_cross_asset_world_macro_f1": 1 + int(np.argmax([item["cross_asset_world_macro_f1"] for item in attempts])),
        "G1_model_screening_pass": any(item["screening_pass"] for item in attempts),
        "first_blocking_layer": "G1_model_recovery_in_domain_cross_asset_world_and_color_stress",
        "formal_G1_500_scene_5000_frame_executed": False,
        "formal_G1_not_executed_reason": "all three 50-scene screening model attempts failed required validation gates",
        "formal_live_30_seed_10_min_executed": False,
        "formal_live_not_executed_reason": "offline G1 model screening failed",
        "real_nav2_spot_clean_executed": False,
        "real_nav2_spot_clean_not_executed_reason": "offline and live formal gates did not pass",
        "stage5a_regression": {
            "offline_pass": stage5a_offline["synthetic_perception_pass"] and stage5a_offline["spot_clean_e2e_pass"],
            "live_pass": stage5a_live["live_smoke_pass"],
            "inference_frame_count": stage5a_live["inference_frame_count"],
            "rosbag_recorded": stage5a_live["rosbag_recorded"],
            "ground_truth_control_violation_count": stage5a_live["ground_truth_control_violation_count"],
            "synthetic_spot_clean_trials": spot["valid_trial_count"],
            "synthetic_spot_clean_success_rate": spot["mission_success_rate"],
        },
        "stage4w_seed0_regression": {
            "success": coverage["success"], "component_count": coverage["component_count"],
            "successful_component_count": sum(bool(item["success"]) for item in coverage["component_results"]),
            "empirical_coverage_rate": coverage["empirical_metrics"]["coverage_rate"],
            "collision_count": coverage["collision_count"],
            "keepout_violation_sample_count": coverage["keepout_violation_sample_count"],
            "brush_state_violation_sample_count": coverage["brush_state_violation_sample_count"],
            "brush_disabled_on_exit": coverage["brush_disabled_on_exit"],
            "coverage_localization_rmse_m": coverage["localization_regression_during_coverage"]["rmse_m"],
        },
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
        "competition_perception_pass": False,
        "real_domain_evaluation_executed": False,
        "j6_runtime_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": 1053.0,
        "target_efficiency_m2_h": 3500.0,
    }
    write(REVIEW / "stage5br_status.json", status)
    for source in (
        PHASE_A / "micro_overfit_report.json", PHASE_A / "pipeline_parity_report.json",
        PHASE_A / "split_ladder_report.json", PHASE_A / "collapse_audit_report.json",
        G1 / "g1_smoke_summary.json", G1 / "g1_annotation_qa.json",
        G1 / "stage5br_g1_world.manifest.json",
    ):
        copy(source)
    for seed in (0, 7, 8):
        copy(G1 / "scene_manifests" / f"scene_{seed:04d}.json", f"scene_manifest_{seed:04d}.json")
    for index, path in enumerate(MODEL_RUNS, 1):
        copy(path / "g1_model_screening_report.json", f"g1_model_screening_attempt_{index}.json")
    copy(MODEL_RUNS[2] / "stage5br_g1_baseline.onnx")
    copy(STAGE5A / "stage5a_offline_report.json")
    copy(STAGE5A / "stage5a_live_smoke_report.json")
    write(REVIEW / "stage5a_spot_clean_summary.json", {
        key: spot[key] for key in ("valid_trial_count", "mission_success_count", "mission_success_rate", "coverage_resume_success_count", "coverage_resume_success_rate", "spot_clean_e2e_pass")
    })
    write(REVIEW / "stage4w_seed0_regression_summary.json", status["stage4w_seed0_regression"])
    copy(ROS / "colcon_test_results.txt")
    build_visuals()
    files = []
    for path in sorted(REVIEW.rglob("*")):
        if path.is_file():
            files.append({
                "path": str(path.relative_to(REVIEW)).replace("\\", "/"),
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            })
    write(REVIEW / "MANIFEST.json", {
        "schema_version": 1, "artifact": "stage5br_20260719_review",
        "file_count": len(files), "files": files,
    })
    print(json.dumps(status, indent=2))


if __name__ == "__main__":
    main()
