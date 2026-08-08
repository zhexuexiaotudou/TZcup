#!/usr/bin/env python3
"""AUTO-05R-3 micro-overfit gate for the G4 model families.

This script runs real training on a small, frozen train subset and evaluates
the task-specific capacity gates before any screening expansion is allowed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import random
import subprocess
import sys
import time
import traceback

import cv2
import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING_PACKAGE))

from sanitation_learning.g4_data import (  # noqa: E402
    AREA_FEATURE_COUNT,
    AREA_NAMES,
    AREA_MODEL_SIZE,
    CLASSIFIER_CLASSES,
    CLASSIFIER_MODEL_SIZE,
    DISCOVERY_MODEL_SIZE,
    DISCRETE_NAMES,
    G4ClassifierDataset,
    build_classifier_samples,
    build_discovery_crop_samples,
    index_instance_records,
    load_classifier_crop,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    area_metrics,
    area_predictions,
    discovery_crop_predictions,
    discovery_metrics,
    discovery_predictions,
)
from sanitation_learning.g4_models import (  # noqa: E402
    build_g4_models,
    export_fixed_onnx,
    torch_onnx_parity,
)
from sanitation_learning.g4_train import (  # noqa: E402
    train_area,
    train_classifier,
    train_discovery_crop,
)


SEED = 20260806


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _select_rows(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    positive_count: int,
    negative_count: int,
    classes: tuple[str, ...],
) -> list[dict]:
    positives = []
    negatives = []
    for row in rows:
        records = instances_by_key.get(
            (int(row["scene_seed"]), int(row["frame_index"])), []
        )
        labels = {record.get("semantic_class") for record in records}
        if row.get("negative_only"):
            negatives.append(row)
        elif labels.intersection(classes):
            positives.append(row)
    positives.sort(
        key=lambda row: (row["scene_seed"], row["frame_index"])
    )
    negatives.sort(
        key=lambda row: (row["scene_seed"], row["frame_index"])
    )
    random.shuffle(positives)
    random.shuffle(negatives)
    selected = positives[:positive_count] + negatives[:negative_count]
    if not positives or (negative_count and not negatives):
        raise RuntimeError("insufficient train rows for micro-overfit")
    return selected


def _classifier_metrics(model, samples: list[dict], device) -> dict:
    model.eval()
    confusion = {name: {"tp": 0, "fp": 0, "fn": 0} for name in CLASSIFIER_CLASSES}
    with torch.no_grad():
        for sample in samples:
            crop = load_classifier_crop(sample, CLASSIFIER_MODEL_SIZE)
            tensor = torch.from_numpy(
                np.ascontiguousarray(crop.transpose(2, 0, 1)[None], dtype=np.float32)
                / 255.0
            ).to(device)
            logits = model(tensor)[0].cpu().numpy()
            predicted = int(np.argmax(logits))
            truth = int(sample["label"])
            if predicted == truth:
                confusion[CLASSIFIER_CLASSES[predicted]]["tp"] += 1
            else:
                confusion[CLASSIFIER_CLASSES[predicted]]["fp"] += 1
                confusion[CLASSIFIER_CLASSES[truth]]["fn"] += 1
    per_class = {}
    for name in CLASSIFIER_CLASSES:
        tp = confusion[name]["tp"]
        fp = confusion[name]["fp"]
        fn = confusion[name]["fn"]
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[name] = {
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }
    positive_classes = DISCRETE_NAMES
    macro_f1 = float(
        np.mean([per_class[name]["f1"] for name in positive_classes])
    )
    return {
        "per_class": per_class,
        "macro_f1": macro_f1,
        "paper_precision": per_class["paper_litter"]["precision"],
        "sample_count": len(samples),
    }


def _select_discovery_threshold(
    model, rows: list[dict], instances_by_key, device
) -> tuple[float, dict]:
    thresholds = (0.35, 0.45, 0.55, 0.65, 0.75, 0.85)
    best = None
    best_threshold = 0.50
    for threshold in thresholds:
        frames = discovery_predictions(
            model,
            rows,
            instances_by_key,
            device=device,
            threshold=threshold,
            max_detections=60,
        )
        metrics = discovery_metrics(frames)
        if (
            metrics["all_gt_candidate_recall"] >= 0.98
            and metrics["negative_only_fp_per_frame"] <= 0.05
        ):
            if best is None or metrics["all_gt_candidate_recall"] > best[
                "all_gt_candidate_recall"
            ]:
                best = metrics
                best_threshold = threshold
    if best is None:
        frames = discovery_predictions(
            model,
            rows,
            instances_by_key,
            device=device,
            threshold=0.55,
            max_detections=60,
        )
        best = discovery_metrics(frames)
        best_threshold = 0.55
    return best_threshold, best


def _select_discovery_crop_threshold(model, samples, device) -> tuple[float, dict]:
    thresholds = (0.35, 0.45, 0.55, 0.65, 0.75, 0.85)
    best = None
    best_threshold = 0.55
    for threshold in thresholds:
        frames = discovery_crop_predictions(
            model,
            samples,
            device=device,
            threshold=threshold,
            max_detections=150,
            nms_iou_threshold=0.5,
            local_maximum_radius=0,
        )
        metrics = discovery_metrics(frames)
        if (
            metrics["all_gt_candidate_recall"] >= 0.98
            and metrics["negative_only_fp_per_frame"] <= 0.05
        ):
            if (
                best is None
                or metrics["all_gt_candidate_recall"] > best["all_gt_candidate_recall"]
                or (
                    metrics["all_gt_candidate_recall"]
                    == best["all_gt_candidate_recall"]
                    and threshold > best_threshold
                )
            ):
                best = metrics
                best_threshold = threshold
    if best is None:
        frames = discovery_crop_predictions(
            model,
            samples,
            device=device,
            threshold=0.55,
            max_detections=150,
            nms_iou_threshold=0.5,
            local_maximum_radius=0,
        )
        best = discovery_metrics(frames)
        best_threshold = 0.55
    return best_threshold, best


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
    }


def run_micro(
    *,
    data_root: Path,
    evidence_dir: Path,
    output: Path,
    model_type: str,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    export_onnx: bool = False,
) -> int:
    output.mkdir(parents=True, exist_ok=True)
    rows = load_frame_rows(evidence_dir / "g4_frame_manifest.jsonl", data_root)
    records = load_instance_records(evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    train_rows = [row for row in rows if row["split"] == "train"]
    print(
        f"[micro] loaded rows={len(rows)} train={len(train_rows)} "
        f"device={torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu'}",
        flush=True,
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    report: dict = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-3",
        "model_type": model_type,
        "executed": True,
        "micro_overfit_pass": False,
        "device": str(device),
        "data_root": str(data_root),
        "train_rows_available": len(train_rows),
    }

    if model_type == "discovery":
        samples = build_discovery_crop_samples(
            train_rows,
            instances_by_key,
            positive_frame_limit=40,
            max_positive_samples=120,
            negative_count=20,
            seed=SEED,
        )
        checkpoint = output / "discovery_micro.pt"
        model, training = train_discovery_crop(
            samples,
            device=device,
            epochs=epochs,
            batch_size=batch_size,
            learning_rate=learning_rate,
            seed=SEED,
            checkpoint_path=checkpoint,
            early_stopping_patience=0,
            load_best=False,
            augment=False,
        )
        print("[micro] discovery training done", flush=True)
        threshold, metrics = _select_discovery_crop_threshold(
            model, samples, device
        )
        print("[micro] discovery threshold scan done", flush=True)
        onnx_report = (
            _export_onnx(
                model,
                (1, 3, DISCOVERY_MODEL_SIZE[1], DISCOVERY_MODEL_SIZE[0]),
                output / "discovery_micro.onnx",
                device,
            )
            if export_onnx
            else None
        )
        gates = {
            "discovery_recall": metrics["all_gt_candidate_recall"] >= 0.98,
            "negative_fp_per_frame": metrics["negative_only_fp_per_frame"] <= 0.05,
        }
        report.update(
            {
                "metrics": metrics,
                "selected_threshold": threshold,
                "training": training,
                "onnx": onnx_report,
                "gates": gates,
                "positive_frames": sum(
                    not sample["negative_only"] for sample in samples
                ),
            }
        )
    elif model_type == "classifier":
        samples = build_classifier_samples(
            train_rows,
            instances_by_key,
            positive_per_class=50,
            background_per_positive=2,
            negative_only_per_frame=3,
            background_limit=120,
            seed=SEED,
        )
        if len(samples) < 250:
            raise RuntimeError(
                f"classifier micro samples insufficient: {len(samples)}"
            )
        checkpoint = output / "classifier_micro.pt"
        model, training = train_classifier(
            samples,
            device=device,
            epochs=epochs,
            batch_size=batch_size,
            learning_rate=learning_rate,
            seed=SEED,
            checkpoint_path=checkpoint,
            early_stopping_patience=0,
            load_best=False,
            cache_crops=True,
        )
        print("[micro] classifier training done", flush=True)
        metrics = _classifier_metrics(model, samples, device)
        print("[micro] classifier metrics done", flush=True)
        onnx_report = (
            _export_onnx(
                model,
                (1, 3, CLASSIFIER_MODEL_SIZE[0], CLASSIFIER_MODEL_SIZE[1]),
                output / "classifier_micro.onnx",
                device,
            )
            if export_onnx
            else None
        )
        gates = {
            "classifier_macro_f1": metrics["macro_f1"] >= 0.98,
            "paper_precision": metrics["paper_precision"] >= 0.98,
            "background_samples": metrics["per_class"]["background"]["tp"]
            + metrics["per_class"]["background"]["fn"]
            >= 100,
        }
        report.update(
            {
                "metrics": metrics,
                "training": training,
                "onnx": onnx_report,
                "gates": gates,
                "sample_count": len(samples),
            }
        )
    elif model_type in ("leaf", "puddle"):
        delegated = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts" / "auto05r_area_crop_micro.py"),
                "--data-root",
                str(data_root),
                "--evidence-dir",
                str(evidence_dir),
                "--output",
                str(output),
                "--task",
                model_type,
                "--epochs",
                str(epochs),
                "--batch-size",
                str(batch_size),
                "--learning-rate",
                str(learning_rate),
                "--arch",
                "simple",
            ],
            check=False,
        )
        if delegated.returncode not in (0, 2):
            raise RuntimeError(
                f"area crop micro failed with exit code {delegated.returncode}"
            )
        delegated_report = json.loads(
            (output / "micro_overfit_report.json").read_text(encoding="utf-8")
        )
        print(
            json.dumps(
                {
                    "micro_overfit_pass": delegated_report[
                        "micro_overfit_pass"
                    ],
                    "model_type": model_type,
                    "gates": delegated_report["gates"],
                    "metrics": delegated_report["metrics"],
                    "report": str(output / "micro_overfit_report.json"),
                },
                indent=2,
            )
        )
        return 0 if delegated_report["micro_overfit_pass"] else 2
        semantic_name = AREA_NAMES[0] if model_type == "leaf" else model_type
        micro_rows = _select_rows(
            train_rows,
            instances_by_key,
            positive_count=80,
            negative_count=80,
            classes=(semantic_name,),
        )
        checkpoint = output / f"{model_type}_micro.pt"
        selected_positive_frames = sum(
            not row.get("negative_only") for row in micro_rows
        )
        selected_negative_frames = sum(
            bool(row.get("negative_only")) for row in micro_rows
        )
        model, training = train_area(
            model_type,
            micro_rows,
            device=device,
            epochs=epochs,
            batch_size=batch_size,
            learning_rate=learning_rate,
            seed=SEED,
            checkpoint_path=checkpoint,
            early_stopping_patience=0,
            load_best=False,
            cache_frames=True,
            crop_mode="full",
        )
        print(f"[micro] {model_type} training done", flush=True)
        predictions = area_predictions(
            model,
            micro_rows,
            device=device,
            thresholds=(0.5, 0.5),
            task=model_type,
        )
        metrics = area_metrics(predictions)
        print(f"[micro] {model_type} metrics done", flush=True)
        onnx_report = (
            _export_onnx(
                model,
                (
                    1,
                    AREA_FEATURE_COUNT,
                    AREA_MODEL_SIZE[1],
                    AREA_MODEL_SIZE[0],
                ),
                output / f"{model_type}_micro.onnx",
                device,
            )
            if export_onnx
            else None
        )
        iou = metrics["iou_by_class"][semantic_name]
        gates = {
            f"{model_type}_iou": iou >= 0.95,
            "negative_area_fp_per_frame": metrics[
                "negative_area_fp_per_frame"
            ]
            <= 0.05,
        }
        report.update(
            {
                "metrics": metrics,
                "training": training,
                "onnx": onnx_report,
                "gates": gates,
                "iou": iou,
                "positive_frames": selected_positive_frames,
                "negative_frames": selected_negative_frames,
            }
        )
    else:
        raise ValueError(f"unknown model_type {model_type}")

    report["micro_overfit_pass"] = bool(
        report["gates"] and all(report["gates"].values())
    )
    report["status"] = "passed" if report["micro_overfit_pass"] else "failed"
    report["environment"] = {
        "python": platform.python_version(),
        "torch": torch.__version__,
        "cuda": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0)
        if torch.cuda.is_available()
        else None,
    }
    report_path = output / "micro_overfit_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "micro_overfit_pass": report["micro_overfit_pass"],
                "model_type": model_type,
                "gates": report["gates"],
                "metrics": report.get("metrics"),
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if report["micro_overfit_pass"] else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--model-type", required=True, choices=("discovery", "classifier", "leaf", "puddle"))
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=6e-4)
    parser.add_argument("--export-onnx", action="store_true")
    args = parser.parse_args()
    started = time.perf_counter()
    try:
        code = run_micro(
            data_root=args.data_root,
            evidence_dir=args.evidence_dir,
            output=args.output,
            model_type=args.model_type,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            export_onnx=args.export_onnx,
        )
    except Exception as exc:
        traceback.print_exc()
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(f"wall_clock_s={time.perf_counter() - started:.1f}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
