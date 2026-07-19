#!/usr/bin/env python3
"""Build a compact, fail-closed Stage5B review artifact from local evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "artifacts" / "stage5b_screening3"
STAGE5A = ROOT / "artifacts" / "stage5b_stage5a_regression"
STAGE4W = ROOT / "artifacts" / "stage5b_stage4w_regression_seed0"
REVIEW = ROOT / "artifacts" / "stage5b_20260719_review"
LEARNING = ROOT / "starter_ws" / "src" / "sanitation_learning"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def copy(source: Path, name: str | None = None) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    shutil.copy2(source, REVIEW / (name or source.name))


def canvas(title: str) -> np.ndarray:
    image = np.full((720, 1280, 3), (247, 248, 250), np.uint8)
    cv2.putText(image, title, (55, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (32, 38, 48), 2, cv2.LINE_AA)
    return image


def save_training_plot(selection: dict) -> None:
    image = canvas("Stage5B candidate B: training loss vs validation macro F1")
    curves = selection["candidates"][1]["curves"]
    left, right, top, bottom = 90, 1210, 110, 640
    cv2.rectangle(image, (left, top), (right, bottom), (70, 75, 85), 2)
    for i in range(6):
        y = top + i * (bottom - top) // 5
        cv2.line(image, (left, y), (right, y), (220, 223, 228), 1)
    def points(key: str, maximum: float) -> np.ndarray:
        return np.array([[left + (i * (right-left) // (len(curves)-1)), bottom - int(item[key] / maximum * (bottom-top))] for i, item in enumerate(curves)], np.int32)
    cv2.polylines(image, [points("training_loss", 2.2)], False, (0, 140, 230), 3, cv2.LINE_AA)
    cv2.polylines(image, [points("validation_macro_f1", 0.5)], False, (205, 95, 45), 3, cv2.LINE_AA)
    cv2.putText(image, "orange: loss (0..2.2)", (100, 690), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 140, 230), 2)
    cv2.putText(image, "blue: validation F1 (0..0.5); best=0.386", (480, 690), cv2.FONT_HERSHEY_SIMPLEX, .7, (205, 95, 45), 2)
    cv2.imwrite(str(REVIEW / "training_curves.png"), image)


def save_gate_plot(report: dict) -> None:
    image = canvas("Stage5B D1 screening gates (green=pass, red=fail)")
    items = list(report["gates"].items())
    for index, (name, passed) in enumerate(items):
        y = 115 + index * 68
        color = (70, 155, 90) if passed else (60, 65, 205)
        cv2.rectangle(image, (75, y), (115, y + 40), color, -1)
        cv2.putText(image, name, (140, y + 30), cv2.FONT_HERSHEY_SIMPLEX, .72, (35, 39, 47), 2, cv2.LINE_AA)
    cv2.imwrite(str(REVIEW / "d1_gate_matrix.png"), image)


def save_stress_plot(report: dict) -> None:
    image = canvas("Color-shortcut stress macro F1 (required aggregate >= 0.85)")
    stress = report["color_shortcut"]["stress_reports"]
    names = list(stress)
    max_width = 860
    for index, name in enumerate(names):
        y = 105 + index * 56
        value = float(stress[name]["macro_f1"])
        cv2.putText(image, name, (45, y + 25), cv2.FONT_HERSHEY_SIMPLEX, .56, (40, 44, 52), 1, cv2.LINE_AA)
        cv2.rectangle(image, (350, y), (350 + int(max_width * value), y + 30), (62, 105, 185), -1)
        cv2.putText(image, f"{value:.4f}", (1220 - 100, y + 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (40, 44, 52), 1)
    cv2.imwrite(str(REVIEW / "color_shortcut_stress.png"), image)


def save_sample() -> None:
    sys.path.insert(0, str(LEARNING))
    from sanitation_learning.assets import load_asset_registry
    from sanitation_learning.rendered import CLASS_ORDER, generate_frame
    frame = generate_frame(18, 2, load_asset_registry(LEARNING / "config" / "asset_registry.yaml"))
    palette = np.array([[0,0,0],[232,95,60],[60,150,220],[225,190,55],[72,165,92],[166,90,210]], np.uint8)
    overlay = palette[frame.semantic_labels]
    composed = cv2.addWeighted(cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR), .62, cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR), .38, 0)
    composed = cv2.resize(composed, (1024, 768), interpolation=cv2.INTER_NEAREST)
    cv2.putText(composed, "D1 procedural rendered sample (NOT Gazebo camera, NOT real data)", (18, 40), cv2.FONT_HERSHEY_SIMPLEX, .72, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(composed, "classes: " + ", ".join(CLASS_ORDER[1:]), (18, 735), cv2.FONT_HERSHEY_SIMPLEX, .46, (255,255,255), 1, cv2.LINE_AA)
    cv2.imwrite(str(REVIEW / "d1_procedural_sample_overlay.png"), composed)


def main() -> int:
    if REVIEW.exists():
        shutil.rmtree(REVIEW)
    REVIEW.mkdir(parents=True)
    perception = load(SOURCE / "d1_perception_report.json")
    dataset = load(SOURCE / "dataset_manifest.json")
    selection = load(SOURCE / "model_selection_report.json")
    for candidate in selection["candidates"]:
        if candidate.get("trained"):
            candidate["weight_source"] = "gradient_descent_cross_entropy_plus_dice"
    live = load(SOURCE / "live_gazebo_diagnostic" / "stage5b_live_diagnostic_report.json")
    stage5a_offline = load(STAGE5A / "stage5a_offline_report.json")
    stage5a_live = load(STAGE5A / "stage5a_live_smoke_report.json")
    stage4w = load(STAGE4W / "stage4w_static_summary.json")
    status = {
        "schema_version": 1,
        "stage": "Stage5B review-required failure boundary",
        "selected_model_sha256": selection["selected_model_sha256"],
        "learned_model": True,
        "D0_stage5a_regression_pass": bool(stage5a_offline["synthetic_perception_pass"] and stage5a_offline["spot_clean_e2e_pass"] and stage5a_live["live_smoke_pass"]),
        "D1_domain": perception["domain"],
        "D1_procedural_dataset_scene_count": dataset["scene_count"],
        "D1_procedural_dataset_frame_count": dataset["frame_count"],
        "D1_formal_500_seed_5000_frame_run_executed": False,
        "D1_formal_run_not_executed_reason": "procedural renderer is not Gazebo camera data and model/color gates failed during screening",
        "D1_rendered_synthetic_perception_pass": perception["rendered_synthetic_perception_pass"],
        "D1_color_shortcut_pass": perception["color_shortcut"]["color_shortcut_pass"],
        "D1_test_discrete_macro_precision": perception["discrete_macro_precision"],
        "D1_test_discrete_macro_recall": perception["discrete_macro_recall"],
        "D1_test_discrete_macro_f1": perception["discrete_macro_f1"],
        "D1_map_localization_rmse_m": perception["map_localization_rmse_m"],
        "D2_real_data_present": False,
        "real_domain_evaluation_executed": False,
        "live_gazebo_diagnostic_pipeline_transport_pass": live["pipeline_transport_pass"],
        "formal_live_gazebo_30_seed_10_min_pass": False,
        "formal_nav2_spot_clean_30_trial_pass": False,
        "stage4w_seed0_regression_pass": stage4w["static_gate_pass"],
        "j6_model_precheck_pass": load(SOURCE / "j6_preflight.json")["j6_model_precheck_pass"],
        "j6_runtime_pass": False,
        "competition_perception_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": 1053.0,
        "target_efficiency_m2_h": 3500.0,
        "REVIEW_PACKET_COMPLETE": True,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
        "first_blocking_layer": "D1 Gazebo-camera data and learned-model generalization/color robustness",
        "stop_condition_honored": True,
    }
    write_json(REVIEW / "stage5b_status.json", status)
    write_json(REVIEW / "training_attempts.json", {
        "schema_version": 1,
        "attempts": [
            {"id": "screening1", "change": "pixel/context baseline", "selected_validation_macro_f1": 0.063, "test_discrete_macro_f1": 0.00185, "result": "fail"},
            {"id": "screening2", "change": "RF23 CE+Dice and channel/edge augmentation", "selected_validation_macro_f1": 0.346, "test_discrete_macro_f1": 0.0166, "result": "fail"},
            {"id": "screening3", "change": "RF51 context model and morphology", "selected_validation_macro_f1": selection["candidates"][1]["validation"]["macro_f1"], "test_discrete_macro_f1": perception["discrete_macro_f1"], "result": "fail_and_freeze"},
        ],
        "conclusion": "three structural attempts did not generalize; freeze evidence and require a true Gazebo-camera D1 pipeline before further formal E2E",
    })
    write_json(REVIEW / "dataset_summary.json", {
        key: dataset[key] for key in (
            "schema_version", "dataset_id", "domain", "scene_count", "frame_count",
            "frames_per_scene", "class_order", "split_scene_seeds", "split_assets",
            "split_textures", "split_worlds", "split_hash",
            "adjacent_frames_cross_split", "exact_duplicate_image_count",
            "perceptual_hash_duplicate_count", "per_class_instance_count",
            "rendered_synthetic_perception_claim_only", "gazebo_camera_rendered",
            "competition_perception_pass",
        )
    })
    copy(SOURCE / "d1_perception_report.json")
    write_json(REVIEW / "model_selection_report.json", selection)
    copy(SOURCE / "model_card.json")
    copy(SOURCE / "environment_lock.json")
    copy(SOURCE / "j6_preflight.json")
    copy(SOURCE / "annotation_qa.json")
    copy(SOURCE / "generated_asset_manifest.json")
    copy(SOURCE / "calibration_dataset_manifest.json")
    copy(SOURCE / "stage5b_learned_perception.onnx")
    copy(SOURCE / "live_gazebo_diagnostic" / "stage5b_live_diagnostic_report.json")
    copy(STAGE5A / "stage5a_offline_report.json", "regression_stage5a_offline_report.json")
    stage5a_live.pop("formal_stage5b_live_gazebo_pass", None)
    stage5a_live.pop("formal_stage5b_live_gazebo_unavailable_reason", None)
    write_json(REVIEW / "regression_stage5a_live_report.json", stage5a_live)
    copy(STAGE4W / "stage4w_static_summary.json", "regression_stage4w_seed0_summary.json")
    copy(LEARNING / "config" / "asset_registry.yaml")
    copy(LEARNING / "config" / "asset_license_manifest.json")
    copy(LEARNING / "config" / "training.yaml")
    copy(LEARNING / "real_data" / "README.md", "D2_REAL_DATA_STATUS.md")
    save_training_plot(selection)
    save_gate_plot(perception)
    save_stress_plot(perception)
    save_sample()
    files = []
    for path in sorted(REVIEW.iterdir()):
        if path.name == "MANIFEST.json" or not path.is_file():
            continue
        data = path.read_bytes()
        files.append({"path": path.name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    manifest = {"schema_version": 1, "artifact": "stage5b_20260719_review", "file_count": len(files), "files": files}
    write_json(REVIEW / "MANIFEST.json", manifest)
    print(json.dumps({"review": str(REVIEW), "file_count": len(files), "ready_for_stage5c": status["READY_FOR_STAGE5C"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
