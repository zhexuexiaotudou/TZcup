#!/usr/bin/env python3
"""AUTO-05R-4 G4 screening gate runner (train/val/test freeze-aware)."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import random
import sys
import time

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING_PACKAGE))

from sanitation_learning.g4_data import (  # noqa: E402
    AREA_FEATURE_COUNT,
    AREA_MODEL_SIZE,
    CLASSIFIER_MODEL_SIZE,
    DISCOVERY_MODEL_SIZE,
    build_classifier_samples,
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
    discovery_predictions,
    evaluate_pipeline,
    match_discrete_predictions,
)
from sanitation_learning.g4_models import (  # noqa: E402
    export_fixed_onnx,
    torch_onnx_parity,
)
from sanitation_learning.g4_train import (  # noqa: E402
    train_area,
    train_classifier,
    train_discovery,
)


SEED = 20260807


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _export_onnx(model, input_shape, output: Path, device) -> dict:
    model = model.cpu().eval()
    dummy = torch.randn(input_shape, generator=torch.Generator().manual_seed(SEED))
    export_fixed_onnx(model, dummy, output, opset=17)
    import onnxruntime as ort

    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    parity = torch_onnx_parity(model, session, dummy)
    return {
        "path": output.name,
        "sha256": sha256(output),
        "bytes": output.stat().st_size,
        "parity": parity,
        "fixed_input": True,
        "custom_ops": 0,
    }


def _select_area_rows(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    task: str,
    positive_limit: int,
    negative_limit: int,
    seed: int,
) -> list[dict]:
    semantic = "leaf_pile" if task == "leaf" else "puddle"
    positives = []
    negatives = []
    for row in rows:
        records = instances_by_key.get(
            (int(row["scene_seed"]), int(row["frame_index"])), []
        )
        labels = {item.get("semantic_class") for item in records}
        if row.get("negative_only"):
            negatives.append(row)
        elif semantic in labels:
            positives.append(row)
    rng = random.Random(seed)
    rng.shuffle(positives)
    rng.shuffle(negatives)
    return positives[:positive_limit] + negatives[:negative_limit]


def _evaluate_split(
    discovery,
    classifier,
    leaf,
    puddle,
    rows,
    instances_by_key,
    device,
    *,
    discovery_threshold: float,
    class_threshold: float,
    area_threshold: float,
) -> dict:
    candidate_frames = discovery_predictions(
        discovery,
        rows,
        instances_by_key,
        device=device,
        threshold=discovery_threshold,
    )
    candidate = discovery_metrics(candidate_frames)
    classified = classify_detections(
        classifier,
        candidate_frames,
        device=device,
        class_threshold=class_threshold,
    )
    matched = match_discrete_predictions(classified)
    discrete = discrete_metrics(matched)
    leaf_preds = area_predictions(
        leaf,
        rows,
        device=device,
        thresholds=(area_threshold, area_threshold),
        task="leaf",
    )
    puddle_preds = area_predictions(
        puddle,
        rows,
        device=device,
        thresholds=(area_threshold, area_threshold),
        task="puddle",
    )
    combined_area = []
    for leaf_frame, puddle_frame in zip(leaf_preds, puddle_preds):
        combined = dict(leaf_frame)
        combined["probabilities"] = np.stack(
            (
                leaf_frame["probabilities"][0],
                puddle_frame["probabilities"][1],
            ),
            axis=0,
        )
        combined["boundary_probabilities"] = np.stack(
            (
                leaf_frame["boundary_probabilities"][0],
                puddle_frame["boundary_probabilities"][1],
            ),
            axis=0,
        )
        combined_area.append(combined)
    area = area_metrics(combined_area)
    return {
        "candidate": candidate,
        "discrete": discrete,
        "area": area,
        "rows": len(rows),
    }


def _cross_world_f1(
    discovery,
    classifier,
    rows,
    instances_by_key,
    device,
    *,
    discovery_threshold: float,
    class_threshold: float,
) -> float:
    worlds = {}
    for row in rows:
        worlds.setdefault(row["world_id"], []).append(row)
    scores = []
    for world_rows in worlds.values():
        frames = discovery_predictions(
            discovery,
            world_rows,
            instances_by_key,
            device=device,
            threshold=discovery_threshold,
        )
        classified = classify_detections(
            classifier,
            frames,
            device=device,
            class_threshold=class_threshold,
        )
        matched = match_discrete_predictions(classified)
        metrics = discrete_metrics(matched)
        scores.append(metrics["macro_f1"])
    return float(np.mean(scores)) if scores else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--classifier-epochs", type=int, default=40)
    parser.add_argument("--area-epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=5e-4)
    parser.add_argument("--max-train-frames", type=int, default=600)
    parser.add_argument("--max-eval-frames", type=int, default=0)
    parser.add_argument("--classifier-positives", type=int, default=200)
    parser.add_argument("--classifier-backgrounds", type=int, default=400)
    parser.add_argument("--area-positive-frames", type=int, default=200)
    parser.add_argument("--area-negative-frames", type=int, default=120)
    parser.add_argument("--discovery-threshold", type=float, default=0.35)
    parser.add_argument("--class-threshold", type=float, default=0.35)
    parser.add_argument("--area-threshold", type=float, default=0.5)
    args = parser.parse_args()

    started = time.perf_counter()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root)
    records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    train_rows = [row for row in rows if row["split"] == "train"][: args.max_train_frames]
    val_rows = [row for row in rows if row["split"] == "val"]
    test_rows = [row for row in rows if row["split"] == "test"]
    if args.max_eval_frames > 0:
        val_rows = val_rows[: args.max_eval_frames]
        test_rows = test_rows[: args.max_eval_frames]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    print(
        f"[screening] train={len(train_rows)} val={len(val_rows)} "
        f"test={len(test_rows)} device={device}",
        flush=True,
    )

    discovery_ckpt = output / "discovery.pt"
    discovery, discovery_training = train_discovery(
        train_rows,
        instances_by_key,
        device=device,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED,
        val_rows=val_rows[:100],
        checkpoint_path=discovery_ckpt,
        early_stopping_patience=0,
        load_best=False,
    )

    classifier_samples = build_classifier_samples(
        train_rows,
        instances_by_key,
        positive_per_class=args.classifier_positives,
        background_per_positive=1,
        negative_only_per_frame=1,
        background_limit=args.classifier_backgrounds,
        seed=SEED,
    )
    classifier_ckpt = output / "classifier.pt"
    classifier, classifier_training = train_classifier(
        classifier_samples,
        device=device,
        epochs=args.classifier_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED,
        checkpoint_path=classifier_ckpt,
        early_stopping_patience=0,
        load_best=False,
        cache_crops=True,
    )

    leaf_rows = _select_area_rows(
        train_rows,
        instances_by_key,
        "leaf",
        args.area_positive_frames,
        args.area_negative_frames,
        SEED,
    )
    puddle_rows = _select_area_rows(
        train_rows,
        instances_by_key,
        "puddle",
        args.area_positive_frames,
        args.area_negative_frames,
        SEED + 1,
    )
    leaf, leaf_training = train_area(
        "leaf",
        leaf_rows,
        device=device,
        epochs=args.area_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED,
        checkpoint_path=output / "leaf.pt",
        early_stopping_patience=0,
        load_best=False,
        cache_frames=True,
    )
    puddle, puddle_training = train_area(
        "puddle",
        puddle_rows,
        device=device,
        epochs=args.area_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED + 1,
        checkpoint_path=output / "puddle.pt",
        early_stopping_patience=0,
        load_best=False,
        cache_frames=True,
    )

    val = _evaluate_split(
        discovery,
        classifier,
        leaf,
        puddle,
        val_rows,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_threshold=args.area_threshold,
    )
    test = _evaluate_split(
        discovery,
        classifier,
        leaf,
        puddle,
        test_rows,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_threshold=args.area_threshold,
    )
    val_cross_world = _cross_world_f1(
        discovery,
        classifier,
        val_rows,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
    )
    test_cross_world = _cross_world_f1(
        discovery,
        classifier,
        test_rows,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
    )
    stress_val = evaluate_pipeline(
        discovery,
        classifier,
        leaf,
        puddle,
        val_rows[:: max(1, len(val_rows) // 20)][:20],
        instances_by_key,
        device=device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_thresholds=(args.area_threshold, args.area_threshold),
        stress_names=("grayscale", "hue_shift", "exposure"),
    )["stress"]["macro_f1"]
    stress_test = evaluate_pipeline(
        discovery,
        classifier,
        leaf,
        puddle,
        test_rows[:: max(1, len(test_rows) // 20)][:20],
        instances_by_key,
        device=device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_thresholds=(args.area_threshold, args.area_threshold),
        stress_names=("grayscale", "hue_shift", "exposure"),
    )["stress"]["macro_f1"]

    onnx = {
        "discovery": _export_onnx(
            discovery,
            (1, 3, DISCOVERY_MODEL_SIZE[1], DISCOVERY_MODEL_SIZE[0]),
            output / "discovery.onnx",
            device,
        ),
        "classifier": _export_onnx(
            classifier,
            (1, 3, CLASSIFIER_MODEL_SIZE[0], CLASSIFIER_MODEL_SIZE[1]),
            output / "classifier.onnx",
            device,
        ),
        "leaf": _export_onnx(
            leaf,
            (1, AREA_FEATURE_COUNT, AREA_MODEL_SIZE[1], AREA_MODEL_SIZE[0]),
            output / "leaf.onnx",
            device,
        ),
        "puddle": _export_onnx(
            puddle,
            (1, AREA_FEATURE_COUNT, AREA_MODEL_SIZE[1], AREA_MODEL_SIZE[0]),
            output / "puddle.onnx",
            device,
        ),
    }
    gates = {
        "discovery_recall_ge_0_80": min(
            val["candidate"]["all_gt_candidate_recall"],
            test["candidate"]["all_gt_candidate_recall"],
        )
        >= 0.80,
        "false_candidates_per_min_le_2": max(
            val["candidate"]["false_candidates_per_min"],
            test["candidate"]["false_candidates_per_min"],
        )
        <= 2.0,
        "negative_only_fp_per_frame_le_0_05": max(
            val["candidate"]["negative_only_fp_per_frame"],
            test["candidate"]["negative_only_fp_per_frame"],
        )
        <= 0.05,
        "in_domain_macro_f1_ge_0_90": min(
            val["discrete"]["macro_f1"], test["discrete"]["macro_f1"]
        )
        >= 0.90,
        "cross_world_macro_f1_ge_0_70": min(
            val_cross_world, test_cross_world
        )
        >= 0.70,
        "paper_precision_ge_0_80": min(
            val["discrete"]["paper_precision"],
            test["discrete"]["paper_precision"],
        )
        >= 0.80,
        "small_object_recall_ge_0_70": min(
            val["discrete"]["small_object_recall"],
            test["discrete"]["small_object_recall"],
        )
        >= 0.70,
        "leaf_iou_ge_0_75": min(
            val["area"]["iou_by_class"]["leaf_pile"],
            test["area"]["iou_by_class"]["leaf_pile"],
        )
        >= 0.75,
        "puddle_iou_ge_0_75": min(
            val["area"]["iou_by_class"]["puddle"],
            test["area"]["iou_by_class"]["puddle"],
        )
        >= 0.75,
        "macro_miou_ge_0_75": min(
            val["area"]["macro_miou"], test["area"]["macro_miou"]
        )
        >= 0.75,
        "boundary_f1_ge_0_70": min(
            val["area"]["boundary_f1"], test["area"]["boundary_f1"]
        )
        >= 0.70,
        "negative_area_fp_le_0_05": max(
            val["area"]["negative_area_fp_per_frame"],
            test["area"]["negative_area_fp_per_frame"],
        )
        <= 0.05,
        "color_material_stress_ge_0_60": min(
            stress_val, stress_test
        )
        >= 0.60,
        "same_color_negative_specificity_ge_0_95": False,
        "full_shift_D6_macro_f1_ge_0_70": False,
        "onnx_parity": all(
            report["parity"]["max_absolute_error"] <= 1e-4
            and report["parity"]["decoded_agreement"] is not None
            and report["parity"]["decoded_agreement"] >= 0.9999
            and report["custom_ops"] == 0
            for report in onnx.values()
        ),
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-4",
        "AUTO_05R_PASS": bool(gates and all(gates.values())),
        "AUTO_05R_BLOCKED": not bool(gates and all(gates.values())),
        "gates": gates,
        "val": val,
        "test": test,
        "cross_world": {
            "val_macro_f1": val_cross_world,
            "test_macro_f1": test_cross_world,
        },
        "stress_macro_f1": {
            "val": stress_val,
            "test": stress_test,
        },
        "missing_gates": [
            "same_color_negative_specificity_ge_0_95",
            "full_shift_D6_macro_f1_ge_0_70",
        ],
        "training": {
            "discovery": discovery_training,
            "classifier": classifier_training,
            "leaf": leaf_training,
            "puddle": puddle_training,
        },
        "onnx": onnx,
        "thresholds": {
            "discovery": args.discovery_threshold,
            "classifier": args.class_threshold,
            "area": args.area_threshold,
        },
        "environment": {
            "python": platform.python_version(),
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0)
            if torch.cuda.is_available()
            else None,
        },
        "duration_s": time.perf_counter() - started,
    }
    report_path = output / "auto05r_screening_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "AUTO_05R_PASS": report["AUTO_05R_PASS"],
                "AUTO_05R_BLOCKED": report["AUTO_05R_BLOCKED"],
                "failed_gates": [
                    name for name, passed in gates.items() if not passed
                ],
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if report["AUTO_05R_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
