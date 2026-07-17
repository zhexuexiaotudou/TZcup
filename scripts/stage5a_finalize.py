#!/usr/bin/env python3
"""Assemble compact, fail-closed Stage5A review evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def render_visuals(formal: Path, synthetic: dict, spot: dict, output: Path) -> None:
    """Render compact review visuals from the same machine-readable evidence."""
    import cv2
    import numpy as np

    visuals = output / "visuals"
    visuals.mkdir(parents=True, exist_ok=True)
    scale = 4
    image = cv2.imread(str(formal / "dataset_smoke" / "images" / "scene_0016.png"))
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    annotation = load(formal / "dataset_smoke" / "annotations" / "scene_0016.json")
    colors = [(50, 50, 230), (50, 220, 50), (230, 50, 50), (30, 190, 230), (230, 190, 30)]
    overlay = image.copy()
    for index, obj in enumerate(annotation["objects"]):
        points = np.asarray(obj["segmentation"][0], dtype=np.int32).reshape(-1, 2) * scale
        cv2.fillPoly(overlay, [points], colors[index])
    overlay = cv2.addWeighted(image, 0.55, overlay, 0.45, 0.0)
    for index, obj in enumerate(annotation["objects"]):
        x, y, width, height = (int(value) * scale for value in obj["bbox_xywh"])
        cv2.rectangle(overlay, (x, y), (x + width, y + height), colors[index], 1)
        cv2.putText(overlay, obj["class_id"], (x, max(y - 5, 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    cv2.imwrite(str(visuals / "segmentation_overlay_scene_0016.png"), overlay)

    matrix = np.asarray(synthetic["confusion_matrix"], dtype=np.float64)
    normalized = matrix / np.maximum(matrix.sum(axis=1, keepdims=True), 1.0)
    cell = 72
    canvas = np.full((cell * 6 + 70, cell * 6 + 70, 3), 255, dtype=np.uint8)
    for row in range(6):
        for column in range(6):
            value = normalized[row, column]
            shade = int(255 * (1.0 - value))
            cv2.rectangle(canvas, (60 + column * cell, 10 + row * cell), (60 + (column + 1) * cell, 10 + (row + 1) * cell), (shade, shade, 255), -1)
            cv2.putText(canvas, f"{value:.2f}", (67 + column * cell, 50 + row * cell), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.imwrite(str(visuals / "confusion_matrix.png"), canvas)

    plot = np.full((520, 520, 3), 255, dtype=np.uint8)
    cv2.line(plot, (60, 460), (480, 460), (0, 0, 0), 2)
    cv2.line(plot, (60, 460), (60, 40), (0, 0, 0), 2)
    recall = float(synthetic["discrete_macro_recall"])
    precision = float(synthetic["discrete_macro_precision"])
    cv2.circle(plot, (60 + int(recall * 420), 460 - int(precision * 420)), 7, (20, 20, 220), -1)
    cv2.putText(plot, "recall", (420, 495), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(plot, "precision", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(plot, "held-out operating point", (140, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
    cv2.imwrite(str(visuals / "precision_recall_operating_point.png"), plot)

    map_plot = np.full((520, 520, 3), 255, dtype=np.uint8)
    cv2.line(map_plot, (40, 260), (480, 260), (180, 180, 180), 1)
    cv2.line(map_plot, (260, 40), (260, 480), (180, 180, 180), 1)
    for index, obj in enumerate(annotation["objects"]):
        x_m, y_m, _ = obj["map_pose"]
        point = (260 + int(x_m * 220), 260 - int(y_m * 220))
        cv2.circle(map_plot, point, 7, colors[index], -1)
        cv2.putText(map_plot, obj["class_id"], (point[0] + 8, point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    cv2.imwrite(str(visuals / "map_targets_scene_0016.png"), map_plot)
    (output / "e2e_timeline_seed_100.json").write_text(
        json.dumps(spot["trials"][0], ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal", type=Path, required=True)
    parser.add_argument("--live", type=Path)
    parser.add_argument("--regression", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--refresh-existing", action="store_true")
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()) and not args.refresh_existing:
        raise SystemExit(f"refusing non-empty output directory: {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)

    live_dir = args.live or args.formal
    offline = load(args.formal / "stage5a_offline_report.json")
    synthetic = load(args.formal / "synthetic_perception_report.json")
    spot = load(args.formal / "spot_clean_e2e_report.json")
    live = load(live_dir / "stage5a_live_smoke_report.json")
    j6 = load(args.formal / "j6_fail_closed.json")
    regression_path = args.regression / "stage4w_static_summary.json"
    regression = load(regression_path)
    coverage = regression.get("coverage", {})
    localization = coverage.get("localization_regression_during_coverage", {})
    colcon_results = (args.formal / "colcon_test_results.txt").read_text(encoding="utf-8")

    stage4w_gates = {
        "static_gate_pass": regression.get("static_gate_pass") is True,
        "mission_success": coverage.get("success") is True,
        "collision_zero": coverage.get("collision_count") == 0,
        "keepout_violation_zero": coverage.get("keepout_violation_sample_count") == 0,
        "brush_final_false": coverage.get("brush_disabled_on_exit") is True,
        "coverage_at_least_90pct": float(
            coverage.get("empirical_metrics", {}).get("coverage_rate", 0.0)
        ) >= 0.90,
        "coverage_localization_rmse_at_most_0_05m": (
            localization.get("pass_rmse_at_most_0_05m") is True
        ),
        "rosbag_replay": regression.get("rosbag_replay") is True,
    }
    gates = {
        "ros_packages_14_tests_zero_failures": (
            "14 tests, 0 errors, 0 failures, 0 skipped" in colcon_results
        ),
        "synthetic_perception": synthetic.get("synthetic_perception_pass") is True,
        "spot_clean_30_seed_e2e": (
            spot.get("spot_clean_e2e_pass") is True
            and spot.get("valid_trial_count") == 30
        ),
        "gazebo_rgbd_onnx_live_smoke": live.get("live_smoke_pass") is True,
        "ground_truth_registry_and_occlusion_pipeline": (
            live.get("ground_truth_registry_target_count") == 5
            and int(live.get("ground_truth_published_target_count", 0)) > 0
            and live.get("ground_truth_negative_target_count") == 0
            and live.get("ground_truth_occlusion_filter") == "geometry_disc_fallback"
        ),
        "live_2d_3d_map_targets_nonempty": (
            int(live.get("last_detection_count", 0)) > 0
            and int(live.get("last_map_target_count", 0)) > 0
            and live.get("map_targets_fail_closed") is False
        ),
        "formal_rosbag_recorded": (
            live.get("rosbag_required") is True and live.get("rosbag_recorded") is True
        ),
        "ground_truth_never_controls": (
            synthetic.get("ground_truth_control_violation_count") == 0
            and all(
                trial.get("ground_truth_control_violation_count") == 0
                for trial in spot.get("trials", [])
            )
            and live.get("ground_truth_control_violation_count") == 0
        ),
        "stage4w_regression": all(stage4w_gates.values()),
    }
    ready = all(gates.values())

    regression_compact = {
        "schema_version": 1,
        "stage": "Stage4W regression for Stage5A",
        "seed": regression.get("seed"),
        "gates": stage4w_gates,
        "stage4w_regression_pass": all(stage4w_gates.values()),
        "empirical_coverage_rate": coverage.get("empirical_metrics", {}).get("coverage_rate"),
        "collision_count": coverage.get("collision_count"),
        "keepout_violation_sample_count": coverage.get("keepout_violation_sample_count"),
        "brush_disabled_on_exit": coverage.get("brush_disabled_on_exit"),
        "localization_rmse_m": localization.get("rmse_m"),
        "rosbag_replay": regression.get("rosbag_replay"),
    }
    (args.output / "stage4w_regression_compact.json").write_text(
        json.dumps(regression_compact, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    summary = {
        "schema_version": 1,
        "stage": "Stage5A",
        "gates": gates,
        "READY_FOR_GPT_REVIEW_STAGE5A": ready,
        "READY_FOR_STAGE5B": ready,
        "synthetic_scope": synthetic.get("scope"),
        "dataset_scene_count": offline.get("dataset_scene_count"),
        "dataset_split_hash": offline.get("dataset_split_hash"),
        "model_sha256": offline.get("model_sha256"),
        "discrete_macro_precision": synthetic.get("discrete_macro_precision"),
        "discrete_macro_recall": synthetic.get("discrete_macro_recall"),
        "discrete_macro_f1": synthetic.get("discrete_macro_f1"),
        "area_iou": synthetic.get("area_iou"),
        "map_localization_rmse_m": synthetic.get("map_localization_rmse_m"),
        "spot_clean_valid_trial_count": spot.get("valid_trial_count"),
        "spot_clean_mission_success_rate": spot.get("mission_success_rate"),
        "coverage_resume_success_rate": spot.get("coverage_resume_success_rate"),
        "live_inference_frame_count": live.get("inference_frame_count"),
        "competition_perception_pass": False,
        "j6_toolchain_available": j6.get("j6_toolchain_available", False),
        "j6_quantization_pass": False,
        "j6_runtime_pass": False,
        "competition_efficiency_pass": False,
        "theoretical_efficiency_m2_h": offline.get("theoretical_efficiency_m2_h"),
        "target_efficiency_m2_h": offline.get("target_efficiency_m2_h"),
        "scope_boundary": (
            "Stage5A is synthetic-domain readiness only. No real-data accuracy, "
            "J6 quantization/runtime, native GUI, physical vehicle, or competition "
            "efficiency claim is made."
        ),
    }
    (args.output / "stage5a_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    copies = {
        args.formal / "stage5a_offline_report.json": "stage5a_offline_report.json",
        args.formal / "synthetic_perception_report.json": "synthetic_perception_report.json",
        args.formal / "spot_clean_e2e_report.json": "spot_clean_e2e_report.json",
        live_dir / "stage5a_live_smoke_report.json": "stage5a_live_smoke_report.json",
        args.formal / "j6_fail_closed.json": "j6_fail_closed.json",
        args.formal / "colcon_test_results.txt": "colcon_test_results.txt",
        live_dir / "rosbag_info.txt": "rosbag_info.txt",
        args.formal / "stage5a_synthetic_color_prototype.onnx": "stage5a_synthetic_color_prototype.onnx",
        args.formal / "dataset_smoke" / "dataset_manifest.json": "dataset_manifest.json",
        args.formal / "dataset_smoke" / "calibration_dataset_manifest.json": "calibration_dataset_manifest.json",
        args.formal / "dataset_smoke" / "annotations" / "coco.json": "coco_annotations.json",
        args.formal / "dataset_smoke" / "splits" / "train.json": "splits/train.json",
        args.formal / "dataset_smoke" / "splits" / "val.json": "splits/val.json",
        args.formal / "dataset_smoke" / "splits" / "test.json": "splits/test.json",
    }
    root = Path(__file__).resolve().parent.parent
    config = root / "starter_ws" / "src" / "sanitation_perception" / "config"
    for name in (
        "garbage_registry.yaml",
        "registry_schema.json",
        "model_manifest.yaml",
        "preprocess_spec.yaml",
        "postprocess_spec.yaml",
        "operator_inventory.json",
    ):
        copies[config / name] = f"contracts/{name}"
    for source, destination in copies.items():
        if not source.is_file():
            raise SystemExit(f"required evidence missing: {source}")
        copy(source, args.output / destination)
    render_visuals(args.formal, synthetic, spot, args.output)

    files = sorted(
        path for path in args.output.rglob("*")
        if path.is_file() and path.name != "MANIFEST.json"
    )
    manifest = {
        "schema_version": 1,
        "file_count": len(files),
        "files": [
            {
                "path": path.relative_to(args.output).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in files
        ],
    }
    (args.output / "MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
