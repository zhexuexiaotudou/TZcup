#!/usr/bin/env python3
"""Run the ONLINE-X1 FCOS-R50 full static development gate.

This is a development-only gate over VAL and D1-D5.  It never reads G5 and it
does not claim moving-camera, map/tracking, freeze or product readiness.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
try:
    import torch
except ImportError:  # The pure gate contract is tested on Windows without Torch.
    torch = None


ROOT = Path(__file__).resolve().parents[1]
LEARNING = ROOT / "starter_ws/src/sanitation_learning"
sys.path.insert(0, str(LEARNING))

from sanitation_learning.auto04_contract import box_iou  # noqa: E402
from sanitation_learning.g4_data import (  # noqa: E402
    DISCRETE_NAMES,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    area_metrics,
    area_predictions,
    classify_detections,
    discrete_metrics,
    discovery_metrics,
    match_discrete_predictions,
)
from sanitation_learning.g4_models import build_g4_model  # noqa: E402
from sanitation_learning.g4_teacher import (  # noqa: E402
    build_fcos_teacher,
    teacher_predictions,
)


TOP_K = 16
DISCOVERY_THRESHOLD = 0.70
CLASSIFIER_THRESHOLD_GRID = tuple(value / 100.0 for value in range(20, 96, 5))
AREA_THRESHOLDS = (0.85, 0.85)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_checkpoint_model(task: str, path: Path, device: torch.device):
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete":
        raise RuntimeError(f"{task} checkpoint is not training_complete")
    contract = checkpoint.get("model_contract") or {}
    area_architecture = (
        "deeplab_resnet50"
        if "deeplab" in str(contract.get("model_id", ""))
        else "dual_resnet18"
    )
    model = build_g4_model(task, area_architecture=area_architecture).to(device)
    model.load_state_dict(checkpoint["state_dict"], strict=True)
    model.eval()
    return model, {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "checkpoint_status": checkpoint.get("checkpoint_status"),
        "model_contract": contract,
    }


def truncate_top_k(frames: list[dict], top_k: int = TOP_K) -> list[dict]:
    for frame in frames:
        frame["detections"] = sorted(
            frame["detections"], key=lambda item: float(item["score"]), reverse=True
        )[:top_k]
    return frames


def candidate_size_metrics(frames: list[dict]) -> dict:
    small_total = 0
    small_matched = 0
    per_size = {
        "small_lt_18px": {"total": 0, "matched": 0},
        "medium_18_to_48px": {"total": 0, "matched": 0},
        "large_ge_48px": {"total": 0, "matched": 0},
    }
    per_world: dict[str, dict[str, int]] = {}
    for frame in frames:
        detections = frame["detections"]
        world = per_world.setdefault(frame["world_id"], {"total": 0, "matched": 0})
        for truth in frame["truth"]:
            short_side = float(truth.get("native_short_side_px", 0.0))
            if short_side < 18.0:
                bucket = "small_lt_18px"
                small_total += 1
            elif short_side < 48.0:
                bucket = "medium_18_to_48px"
            else:
                bucket = "large_ge_48px"
            matched = any(
                box_iou(
                    tuple(float(value) for value in truth["bbox_xyxy"]),
                    tuple(float(value) for value in detection["bbox_xyxy"]),
                )
                >= 0.5
                for detection in detections
            )
            per_size[bucket]["total"] += 1
            per_size[bucket]["matched"] += int(matched)
            world["total"] += 1
            world["matched"] += int(matched)
            if bucket == "small_lt_18px":
                small_matched += int(matched)
    for record in per_size.values():
        record["recall"] = record["matched"] / max(record["total"], 1)
    return {
        "small_object_candidate_recall": small_matched / max(small_total, 1),
        "per_size": per_size,
        "per_world": {
            world: {**record, "recall": record["matched"] / max(record["total"], 1)}
            for world, record in sorted(per_world.items())
        },
    }


def same_color_specificity(frames: list[dict]) -> dict:
    families: dict[str, dict[str, int]] = {}
    for frame in frames:
        if not frame["negative_only"]:
            continue
        has_fp = bool(frame["detections"])
        for taxonomy in frame["row"].get("taxonomies", ()):
            record = families.setdefault(taxonomy, {"frames": 0, "fp_frames": 0})
            record["frames"] += 1
            record["fp_frames"] += int(has_fp)
    eligible = [
        1.0 - record["fp_frames"] / record["frames"]
        for record in families.values()
        if record["frames"] >= 5
    ]
    return {
        "status": "evaluated" if eligible else "not_evaluated",
        "specificity": min(eligible) if eligible else None,
        "taxonomy_count": len(families),
    }


def combine_area(leaf_frames: list[dict], puddle_frames: list[dict]) -> list[dict]:
    combined = []
    for leaf, puddle in zip(leaf_frames, puddle_frames):
        frame = dict(leaf)
        frame["probabilities"] = np.stack(
            (leaf["probabilities"][0], puddle["probabilities"][1]), axis=0
        )
        frame["boundary_probabilities"] = np.stack(
            (
                leaf["boundary_probabilities"][0],
                puddle["boundary_probabilities"][1],
            ),
            axis=0,
        )
        combined.append(frame)
    return combined


def apply_classifier_threshold(scored_frames: list[dict], threshold: float) -> list[dict]:
    output = []
    for frame in scored_frames:
        predictions = []
        for item in frame["predictions"]:
            candidate_score = float(item["candidate_class_score"])
            background_score = float(item["background_score"])
            accepted = candidate_score >= threshold and candidate_score > background_score
            predictions.append(
                {
                    **item,
                    "class_index": int(item["candidate_class_index"]) if accepted else 0,
                    "class_name": item["candidate_class_name"] if accepted else "background",
                    "score": candidate_score if accepted else background_score,
                }
            )
        output.append({**frame, "predictions": predictions})
    return output


def select_classifier_threshold(scored_frames: list[dict]) -> dict:
    sweep = []
    for threshold in CLASSIFIER_THRESHOLD_GRID:
        metrics = discrete_metrics(
            match_discrete_predictions(apply_classifier_threshold(scored_frames, threshold))
        )
        gates = {
            "macro_precision_at_least_0_90": metrics["macro_precision"] >= 0.90,
            "macro_recall_at_least_0_90": metrics["macro_recall"] >= 0.90,
            "macro_f1_at_least_0_90": metrics["macro_f1"] >= 0.90,
            "paper_precision_at_least_0_80": metrics["paper_precision"] >= 0.80,
            "small_recall_at_least_0_70": metrics["small_object_recall"] >= 0.70,
        }
        sweep.append(
            {"threshold": threshold, "metrics": metrics, "gates": gates, "all_pass": all(gates.values())}
        )
    passing = [item for item in sweep if item["all_pass"]]
    pool = passing or sweep
    selected = max(
        pool,
        key=lambda item: (
            item["metrics"]["macro_f1"],
            item["metrics"]["macro_precision"],
            item["threshold"],
        ),
    )
    return {"selected": selected, "sweep": sweep, "gate_pass": bool(passing)}


def evaluate_rows(
    *,
    name: str,
    rows: list[dict],
    instances: list[dict],
    teacher,
    classifier,
    leaf,
    puddle,
    device: torch.device,
    class_threshold: float | None = None,
) -> tuple[dict, list[dict], list[dict]]:
    started = time.perf_counter()
    by_key = index_instance_records(instances)
    candidate_frames = truncate_top_k(
        teacher_predictions(
            teacher,
            rows,
            by_key,
            device=device,
            score_threshold=DISCOVERY_THRESHOLD,
            batch_size=4,
            input_scale=1,
        )
    )
    candidate = discovery_metrics(candidate_frames)
    candidate.update(candidate_size_metrics(candidate_frames))
    scored = classify_detections(
        classifier,
        candidate_frames,
        device=device,
        class_threshold=0.0,
    )
    calibration = None
    if class_threshold is None:
        calibration = select_classifier_threshold(scored)
        class_threshold = float(calibration["selected"]["threshold"])
    classified = apply_classifier_threshold(scored, class_threshold)
    matched = match_discrete_predictions(classified)
    discrete = discrete_metrics(matched)
    leaf_frames = area_predictions(
        leaf, rows, device=device, thresholds=AREA_THRESHOLDS, task="leaf"
    )
    puddle_frames = area_predictions(
        puddle, rows, device=device, thresholds=AREA_THRESHOLDS, task="puddle"
    )
    area = area_metrics(combine_area(leaf_frames, puddle_frames))
    report = {
        "name": name,
        "rows": len(rows),
        "candidate": candidate,
        "discrete": discrete,
        "area": area,
        "same_color_negative_specificity": same_color_specificity(candidate_frames),
        "classifier_threshold": class_threshold,
        "classifier_calibration": calibration,
        "duration_s": time.perf_counter() - started,
    }
    print(
        f"[{name}] rows={len(rows)} candidate_recall={candidate['all_gt_candidate_recall']:.4f} "
        f"macro_f1={discrete['macro_f1']:.4f} miou={area['macro_miou']:.4f}",
        flush=True,
    )
    return report, candidate_frames, matched


def load_partition(data_root: Path, evidence_dir: Path, allowed_splits=None):
    rows = load_frame_rows(
        evidence_dir / "g4_frame_manifest.jsonl",
        data_root,
        allowed_splits=allowed_splits,
    )
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    instances = load_instance_records(
        evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=keys
    )
    return rows, instances


def static_gate(report: dict) -> dict:
    val = report["splits"]["VAL"]
    cross = report["cross_world_aggregate"]
    gates = {
        "candidate_recall_at_least_0_80": val["candidate"]["all_gt_candidate_recall"] >= 0.80,
        "small_candidate_recall_at_least_0_70": val["candidate"]["small_object_candidate_recall"] >= 0.70,
        "false_candidates_per_min_at_most_2": val["candidate"]["false_candidates_per_min"] <= 2.0,
        "negative_fp_per_frame_at_most_0_05": cross["candidate"]["negative_only_fp_per_frame"] <= 0.05,
        "in_domain_macro_precision_at_least_0_90": val["discrete"]["macro_precision"] >= 0.90,
        "in_domain_macro_recall_at_least_0_90": val["discrete"]["macro_recall"] >= 0.90,
        "in_domain_macro_f1_at_least_0_90": val["discrete"]["macro_f1"] >= 0.90,
        "cross_world_macro_f1_at_least_0_70": cross["discrete"]["macro_f1"] >= 0.70,
        "each_cross_world_recall_at_least_0_70": all(
            cross["discrete"]["per_class"][name]["recall"] >= 0.70
            for name in DISCRETE_NAMES
        ),
        "paper_precision_at_least_0_80": cross["discrete"]["paper_precision"] >= 0.80,
        "small_final_recall_at_least_0_70": cross["discrete"]["small_object_recall"] >= 0.70,
        "leaf_iou_at_least_0_75": val["area"]["iou_by_class"]["leaf_pile"] >= 0.75,
        "puddle_iou_at_least_0_75": val["area"]["iou_by_class"]["puddle"] >= 0.75,
        "area_miou_at_least_0_75": val["area"]["macro_miou"] >= 0.75,
        "boundary_f1_at_least_0_70": val["area"]["boundary_f1"] >= 0.70,
        "negative_area_fp_per_frame_at_most_0_05": cross["area"]["negative_area_fp_per_frame"] <= 0.05,
    }
    return {"gates": gates, "static_gate_pass": all(gates.values())}


def aggregate_area_reports(reports: list[dict]) -> dict:
    intersections = np.sum([item["intersection_pixels"] for item in reports], axis=0)
    unions = np.sum([item["union_pixels"] for item in reports], axis=0)
    boundary_intersections = np.sum(
        [item["boundary_intersection_pixels"] for item in reports], axis=0
    )
    boundary_unions = np.sum(
        [item["boundary_union_pixels"] for item in reports], axis=0
    )
    iou = intersections / np.maximum(unions, 1)
    boundary_f1 = 2 * boundary_intersections / np.maximum(
        boundary_intersections + boundary_unions, 1
    )
    negative_frames = sum(item["negative_only_frames"] for item in reports)
    negative_fp_frames = sum(item["negative_only_fp_frames"] for item in reports)
    return {
        "iou_by_class": {"leaf_pile": float(iou[0]), "puddle": float(iou[1])},
        "macro_miou": float(np.mean(iou)),
        "boundary_f1_by_class": {
            "leaf_pile": float(boundary_f1[0]),
            "puddle": float(boundary_f1[1]),
        },
        "boundary_f1": float(np.mean(boundary_f1)),
        "negative_only_frames": negative_frames,
        "negative_only_fp_frames": negative_fp_frames,
        "negative_area_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
        "intersection_pixels": intersections.astype(int).tolist(),
        "union_pixels": unions.astype(int).tolist(),
        "boundary_intersection_pixels": boundary_intersections.astype(int).tolist(),
        "boundary_union_pixels": boundary_unions.astype(int).tolist(),
    }


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    if torch is None:
        raise RuntimeError("ONLINE-X1 execution requires PyTorch")
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--factorized-root", type=Path, required=True)
    parser.add_argument("--teacher-report", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    teacher_report = json.loads(args.teacher_report.read_text(encoding="utf-8"))
    teacher_path = args.teacher_report.parent / teacher_report["checkpoint"]["path"]
    if sha256(teacher_path) != teacher_report["checkpoint"]["sha256"]:
        raise RuntimeError("teacher checkpoint hash mismatch")
    teacher = build_fcos_teacher(input_scale=1).to(device)
    teacher_checkpoint = torch.load(teacher_path, map_location=device, weights_only=False)
    teacher.load_state_dict(teacher_checkpoint["state_dict"], strict=True)
    teacher.eval()

    classifier, classifier_record = load_checkpoint_model(
        "classifier", args.model_dir / "classifier.pt", device
    )
    leaf, leaf_record = load_checkpoint_model("leaf", args.model_dir / "leaf.pt", device)
    puddle, puddle_record = load_checkpoint_model(
        "puddle", args.model_dir / "puddle.pt", device
    )

    val_rows, val_instances = load_partition(
        args.data_root, args.evidence_dir, allowed_splits={"val"}
    )
    splits = {}
    val_report, _, _ = evaluate_rows(
        name="VAL",
        rows=val_rows,
        instances=val_instances,
        teacher=teacher,
        classifier=classifier,
        leaf=leaf,
        puddle=puddle,
        device=device,
        class_threshold=None,
    )
    splits["VAL"] = val_report
    selected_class_threshold = float(val_report["classifier_threshold"])

    cross_candidate_frames: list[dict] = []
    cross_matched_frames: list[dict] = []
    for index in range(1, 6):
        root = args.factorized_root / f"D{index}"
        rows, instances = load_partition(
            root / "g4_screening_native", root / "evidence/raw_g4_qa"
        )
        split_report, candidate_frames, matched_frames = evaluate_rows(
            name=f"D{index}",
            rows=rows,
            instances=instances,
            teacher=teacher,
            classifier=classifier,
            leaf=leaf,
            puddle=puddle,
            device=device,
            class_threshold=selected_class_threshold,
        )
        splits[f"D{index}"] = split_report
        cross_candidate_frames.extend(candidate_frames)
        cross_matched_frames.extend(matched_frames)

    cross_candidate = discovery_metrics(cross_candidate_frames)
    cross_candidate.update(candidate_size_metrics(cross_candidate_frames))
    cross_report = {
        "name": "D1-D5-AGGREGATE",
        "rows": len(cross_candidate_frames),
        "candidate": cross_candidate,
        "discrete": discrete_metrics(cross_matched_frames),
        "area": aggregate_area_reports([splits[f"D{index}"]["area"] for index in range(1, 6)]),
        "same_color_negative_specificity": same_color_specificity(cross_candidate_frames),
        "classifier_threshold": selected_class_threshold,
        "duration_s": sum(splits[f"D{index}"]["duration_s"] for index in range(1, 6)),
    }
    report = {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-01-X1-STATIC",
        "source_commit": "31be4ce",
        "route": "ONLINE-X1_FCOS_R50",
        "device": str(device),
        "environment": {
            "platform": platform.platform(),
            "torch": torch.__version__,
            "cuda_available": torch.cuda.is_available(),
            "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        },
        "thresholds": {
            "discovery": DISCOVERY_THRESHOLD,
            "classifier": selected_class_threshold,
            "classifier_grid": list(CLASSIFIER_THRESHOLD_GRID),
            "area": list(AREA_THRESHOLDS),
            "top_k": TOP_K,
        },
        "models": {
            "teacher": {"path": teacher_path.as_posix(), "sha256": sha256(teacher_path)},
            "classifier": classifier_record,
            "leaf": leaf_record,
            "puddle": puddle_record,
        },
        "splits": splits,
        "cross_world_aggregate": cross_report,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "moving_camera_gate": "not_run_pending_static_gate",
        "map_tracking_gate": "not_run_pending_moving_camera",
        "PERCEPTION_ONLINE_X86_DEV_PASS": False,
    }
    report["static_decision"] = static_gate(report)
    report["next_action"] = (
        "run_moving_camera_and_export_gates"
        if report["static_decision"]["static_gate_pass"]
        else "X1_failed_enter_X2"
    )
    write_json(args.output, report)
    print(json.dumps(report["static_decision"], indent=2), flush=True)
    return 0 if report["static_decision"]["static_gate_pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
