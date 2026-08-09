#!/usr/bin/env python3
"""AUTO-05R-4 G4 development screening runner (validation-only).

The operational screening gate trains discovery, classifier, leaf and puddle
with per-epoch validation, EMA, positive early stopping patience, checkpoint
persistence, ``load_best=True`` and constraint-aware selection.  Only
development roles are readable:

- ``train`` / ``train_world_holdout`` / ``val`` / ``D1``-``D5``

The old ``test`` split is renamed ``legacy_G4_D6_diagnostic`` and is recorded
strictly as non-gating diagnostic evidence.  ``G5_SEALED_FINAL`` is never
loaded by this script.  Every ONNX artifact gets task-specific parity plus a
zero-custom-op, fixed-shape, opset-17 contract check.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import random
import shutil
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
    DISCRETE_NAMES,
    build_classifier_samples,
    index_instance_records,
    load_classifier_crop,
    load_frame_rows,
    load_instance_records,
    load_scene_manifests,
)
from sanitation_learning.g4_calibration import (  # noqa: E402
    AREA_THRESHOLD_GRID,
    DISCOVERY_THRESHOLD_GRID,
    select_area_threshold,
    select_discovery_threshold,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    area_metrics,
    area_predictions,
    background_specificity,
    classify_detections,
    discrete_metrics,
    discovery_metrics,
    discovery_predictions,
    match_discrete_predictions,
)
from sanitation_learning.g4_gates import (  # noqa: E402
    evaluate_policy,
    load_policy,
)
from sanitation_learning.g4_models import (  # noqa: E402
    build_g4_model,
    export_fixed_onnx,
    torch_onnx_parity,
)
from sanitation_learning.g4_onnx_parity import (  # noqa: E402
    assert_onnx_contract,
    product_parity_gate,
)
from sanitation_learning.g4_selection import (  # noqa: E402
    area_selector,
    classifier_selector,
    discovery_selector,
)
from sanitation_learning.g4_split_policy import (  # noqa: E402
    DEVELOPMENT_ROLES,
    LEGACY_DIAGNOSTIC_ROLE,
    SEALED_FINAL_ROLE,
    partition_rows,
    screening_decision,
    stratified_row_sample,
)
from sanitation_learning.g4_teacher import require_teacher_dataset_gate  # noqa: E402
from sanitation_learning.g4_train import (  # noqa: E402
    train_area,
    train_classifier,
    train_discovery,
)


SEED = 20260807
CLASSIFIER_THRESHOLD_GRID = tuple(
    round(value / 100.0, 2) for value in range(20, 96, 5)
)
P4_POLICY = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_learning"
    / "config"
    / "perception_p4_screening_policy.yaml"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _selected_validation_metrics(training_report: dict) -> dict:
    """Return metrics for the state actually loaded after training."""
    selection = training_report.get("selection") or {}
    if selection.get("selected"):
        return dict(selection.get("validation_metrics") or {})
    diagnostic = selection.get("diagnostic_checkpoint") or {}
    if diagnostic:
        return dict(diagnostic.get("validation_metrics") or {})
    return dict(selection.get("validation_metrics") or {})


def _selection_product_eligible(training_report: dict) -> bool:
    selection = training_report.get("selection") or {}
    if selection.get("selected"):
        return True
    return bool(selection.get("product_eligible", False))


def _load_reused_model(
    task: str,
    source_dir: Path,
    output_dir: Path,
    device: torch.device,
    *,
    area_architecture: str = "dual_resnet18",
) -> tuple[torch.nn.Module, dict]:
    source = source_dir / f"{task}.pt"
    if not source.is_file():
        raise FileNotFoundError(f"reused {task} checkpoint is missing: {source}")
    checkpoint = torch.load(source, map_location=device, weights_only=False)
    state = checkpoint.get("state_dict")
    if not isinstance(state, dict) or not state:
        raise RuntimeError(f"reused {task} checkpoint has no selected state_dict")
    model = build_g4_model(
        task, area_architecture=area_architecture
    ).to(device)
    model.load_state_dict(state, strict=True)
    model.eval()
    target = output_dir / source.name
    shutil.copy2(source, target)
    return model, {
        "reused_frozen_checkpoint": True,
        "source": str(source),
        "source_sha256": sha256(source),
        "copied_checkpoint_sha256": sha256(target),
        "best_epoch": checkpoint.get("best_epoch"),
        "best_metric": checkpoint.get("best_metric"),
        "selection": checkpoint.get("selection"),
        "device": str(device),
    }


def _holdout_rows(rows: list[dict], fraction: float = 0.2) -> list[dict]:
    """Deterministic per-(world, scene) train-world holdout selection."""
    if not 0.0 < fraction < 1.0:
        raise ValueError("holdout fraction must be in (0, 1)")
    return [
        row
        for row in rows
        if (
            hashlib.sha256(
                f"{row['world_id']}:{int(row['scene_seed'])}".encode("utf-8")
            ).digest()[0]
            % 100
            < int(fraction * 100)
        )
    ]


def _tag_rows_with_scene_metadata(
    rows: list[dict], scene_manifests: dict[int, dict]
) -> list[dict]:
    tagged: list[dict] = []
    for row in rows:
        updated = dict(row)
        scene = scene_manifests.get(int(row["scene_seed"]), {})
        updated["paper_like_hard_negative"] = bool(
            scene.get("paper_like_hard_negative_count", 0) > 0
        )
        taxonomies = sorted(
            {
                str(item.get("taxonomy"))
                for item in scene.get("objects", [])
                if item.get("taxonomy") and not item.get("semantic_label")
            }
        )
        updated["taxonomies"] = taxonomies
        tagged.append(updated)
    return tagged


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
    area_thresholds: tuple[float, float],
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
        thresholds=area_thresholds,
        task="leaf",
    )
    puddle_preds = area_predictions(
        puddle,
        rows,
        device=device,
        thresholds=area_thresholds,
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
    per_class_recall = {
        name: discrete["per_class"][name]["recall"]
        for name in DISCRETE_NAMES
    }
    return {
        "candidate": candidate,
        "discrete": discrete,
        "area": area,
        "per_class_recall": per_class_recall,
        "rows": len(rows),
        "same_color_negative_specificity": _same_color_specificity(
            candidate_frames
        ),
    }


def _same_color_specificity(frames: list[dict]) -> dict:
    """Real per-taxonomy specificity on negative-only frames (fail-closed)."""
    families: dict[str, dict] = {}
    for frame in frames:
        if not frame["negative_only"]:
            continue
        taxonomies = frame["row"].get("taxonomies", ())
        if not taxonomies:
            continue
        has_fp = len(frame["detections"]) > 0
        for taxonomy in taxonomies:
            entry = families.setdefault(
                taxonomy, {"frames": 0, "fp_frames": 0}
            )
            entry["frames"] += 1
            entry["fp_frames"] += int(has_fp)
    if not families:
        return {
            "status": "not_evaluated",
            "reason": "no_taxonomy_tagged_negative_frames",
            "per_taxonomy": {},
        }
    per_taxonomy = {
        taxonomy: {
            "frames": entry["frames"],
            "fp_frames": entry["fp_frames"],
            "specificity": 1.0
            - entry["fp_frames"] / max(entry["frames"], 1),
        }
        for taxonomy, entry in sorted(families.items())
    }
    evaluated = {
        taxonomy: record
        for taxonomy, record in per_taxonomy.items()
        if record["frames"] >= 5
    }
    if not evaluated:
        return {
            "status": "not_evaluated",
            "reason": "insufficient_frames_per_taxonomy",
            "per_taxonomy": per_taxonomy,
        }
    return {
        "status": "evaluated",
        "specificity": float(
            min(record["specificity"] for record in evaluated.values())
        ),
        "per_taxonomy": per_taxonomy,
    }


def _classifier_sample_scores(model, samples: list[dict], device) -> list[dict]:
    """Infer classifier probabilities once for deterministic VAL calibration."""
    records = []
    with torch.no_grad():
        for sample in samples:
            crop = load_classifier_crop(sample, CLASSIFIER_MODEL_SIZE)
            tensor = torch.from_numpy(
                np.ascontiguousarray(
                    crop.transpose(2, 0, 1)[None], dtype=np.float32
                )
                / 255.0
            ).to(device)
            logits = model(tensor)[0].cpu().numpy()
            probabilities = np.exp(logits - logits.max(keepdims=True))
            probabilities /= probabilities.sum(keepdims=True)
            records.append(
                {
                    "truth": int(sample["label"]),
                    "hard_negative": bool(sample.get("hard_negative")),
                    "probabilities": probabilities.tolist(),
                }
            )
    return records


def _classifier_metrics_from_scores(
    records: list[dict], class_threshold: float
) -> dict:
    """Macro F1, paper precision and specificity at one operating point."""
    confusion = {name: {"tp": 0, "fp": 0, "fn": 0} for name in ("background", *DISCRETE_NAMES)}
    hard_negative_total = 0
    hard_negative_misclassified = 0
    names = ("background", *DISCRETE_NAMES)
    for record in records:
        probabilities = np.asarray(record["probabilities"], dtype=np.float64)
        positive_class = int(np.argmax(probabilities[1:])) + 1
        predicted = (
            positive_class
            if float(probabilities[positive_class]) >= class_threshold
            and float(probabilities[positive_class]) > float(probabilities[0])
            else 0
        )
        truth = int(record["truth"])
        if predicted == truth:
            confusion[names[truth]]["tp"] += 1
        else:
            confusion[names[predicted]]["fp"] += 1
            confusion[names[truth]]["fn"] += 1
        if record.get("hard_negative"):
            hard_negative_total += 1
            hard_negative_misclassified += int(predicted != 0)
    per_class = {}
    for name in ("background", *DISCRETE_NAMES):
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
    macro_f1 = float(
        np.mean([per_class[name]["f1"] for name in DISCRETE_NAMES])
    )
    min_discrete_recall = float(
        min(per_class[name]["recall"] for name in DISCRETE_NAMES)
    )
    return {
        "validation_macro_f1": macro_f1,
        "validation_min_discrete_recall": min_discrete_recall,
        "validation_paper_precision": per_class["paper_litter"]["precision"],
        "validation_background_specificity": background_specificity(confusion),
        "validation_hard_negative_specificity": (
            1.0
            - hard_negative_misclassified / max(hard_negative_total, 1)
            if hard_negative_total
            else 0.0
        ),
        "validation_sample_count": len(records),
    }


def _select_classifier_threshold(records: list[dict]) -> dict:
    sweep = []
    for threshold in CLASSIFIER_THRESHOLD_GRID:
        metrics = _classifier_metrics_from_scores(records, threshold)
        paper_shortfall = max(
            0.0, 0.80 - metrics["validation_paper_precision"]
        ) / 0.80
        macro_f1_shortfall = max(
            0.0, 0.90 - metrics["validation_macro_f1"]
        ) / 0.90
        recall_shortfall = max(
            0.0, 0.70 - metrics["validation_min_discrete_recall"]
        ) / 0.70
        background_shortfall = max(
            0.0, 0.95 - metrics["validation_background_specificity"]
        ) / 0.95
        violation = (
            macro_f1_shortfall
            + recall_shortfall
            + paper_shortfall
            + background_shortfall
        )
        sweep.append(
            {
                "threshold": threshold,
                "metrics": metrics,
                "product_eligible": violation == 0.0,
                "constraint_violation": violation,
            }
        )
    selected = min(
        sweep,
        key=lambda item: (
            not item["product_eligible"],
            item["constraint_violation"],
            -item["metrics"]["validation_macro_f1"],
            -item["metrics"]["validation_paper_precision"],
            -item["metrics"]["validation_background_specificity"],
            -item["threshold"],
        ),
    )
    return {
        **selected,
        "selection_split": "val",
        "selection_rule": (
            "macro_f1_min_recall_precision_specificity_constraints_then_macro_f1"
        ),
        "sweep": sweep,
    }


def _discovery_validation_metric_fn(
    rows, instances_by_key, device
):
    from sanitation_learning.g4_losses import discovery_loss
    from sanitation_learning.g4_train import _move_batch

    def metric_fn(model, loader, device):
        total = 0.0
        steps = 0
        model.eval()
        with torch.no_grad():
            for batch in loader:
                inputs, targets = _move_batch(batch, device)
                outputs = model(inputs)
                loss = discovery_loss(outputs, *targets)["total"]
                total += float(loss.detach().cpu())
                steps += 1
        frames = discovery_predictions(
            model,
            rows,
            instances_by_key,
            device=device,
            threshold=min(DISCOVERY_THRESHOLD_GRID),
            max_detections=100,
        )
        operating_point = select_discovery_threshold(frames)
        candidate = operating_point["metrics"]
        return {
            "validation_loss": total / max(steps, 1),
            "validation_all_gt_candidate_recall": candidate[
                "all_gt_candidate_recall"
            ],
            "validation_ap50": candidate["ap50"],
            "validation_precision": candidate["precision"],
            "validation_false_candidates_per_min": candidate[
                "false_candidates_per_min"
            ],
            "validation_negative_only_fp_per_frame": candidate[
                "negative_only_fp_per_frame"
            ],
            "validation_discovery_threshold": operating_point["threshold"],
            "validation_threshold_product_eligible": operating_point[
                "product_eligible"
            ],
            "validation_frames": len(rows),
        }

    return metric_fn


def _classifier_validation_metric_fn(samples, device):
    def metric_fn(model, loader, device):
        from sanitation_learning.g4_losses import classifier_loss
        from sanitation_learning.g4_train import _move_batch

        total = 0.0
        steps = 0
        model.eval()
        with torch.no_grad():
            for batch in loader:
                inputs, targets = _move_batch(batch, device)
                total += float(
                    classifier_loss(model(inputs), targets[0]).detach().cpu()
                )
                steps += 1
        scores = _classifier_sample_scores(model, samples, device)
        operating_point = _select_classifier_threshold(scores)
        return {
            "validation_loss": total / max(steps, 1),
            **operating_point["metrics"],
            "validation_classifier_threshold": operating_point["threshold"],
            "validation_threshold_product_eligible": operating_point[
                "product_eligible"
            ],
        }

    return metric_fn


def _area_validation_metric_fn(rows, device, task: str):
    def metric_fn(model, loader, device):
        from sanitation_learning.g4_losses import area_loss
        from sanitation_learning.g4_train import _move_batch

        total = 0.0
        steps = 0
        model.eval()
        with torch.no_grad():
            for batch in loader:
                inputs, targets = _move_batch(batch, device)
                loss = area_loss(model(inputs), *targets)["total"]
                total += float(loss.detach().cpu())
                steps += 1
        predictions = area_predictions(
            model,
            rows,
            device=device,
            thresholds=(0.5, 0.5),
            task=task,
        )
        operating_point = select_area_threshold(predictions, task)
        metrics = operating_point["metrics"]
        key = "leaf_pile" if task == "leaf" else "puddle"
        return {
            "validation_loss": total / max(steps, 1),
            "validation_iou": metrics["iou_by_class"][key],
            "validation_macro_miou": metrics["macro_miou"],
            "validation_boundary_f1": metrics["boundary_f1_by_class"][key],
            "validation_negative_area_fp_per_frame": metrics[
                "negative_area_fp_per_frame"
            ],
            "validation_area_threshold": operating_point["threshold"],
            "validation_threshold_product_eligible": operating_point[
                "product_eligible"
            ],
            "validation_frames": len(rows),
        }

    return metric_fn


def _stress_macro_f1(
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
    area_thresholds: tuple[float, float],
) -> float | None:
    from sanitation_learning.g4_evaluation import evaluate_pipeline

    if not rows:
        return None
    sampled = rows[:: max(1, len(rows) // 20)][:20]
    return evaluate_pipeline(
        discovery,
        classifier,
        leaf,
        puddle,
        sampled,
        instances_by_key,
        device=device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
        area_thresholds=area_thresholds,
        stress_names=("grayscale", "hue_shift", "exposure"),
    )["stress"]["macro_f1"]


def _export_onnx_task_specific(
    model, input_shape, output: Path, device
) -> dict:
    model = model.cpu().eval()
    dummy = torch.randn(input_shape, generator=torch.Generator().manual_seed(SEED))
    export_fixed_onnx(model, dummy, output, opset=17)
    import onnxruntime as ort

    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    parity = torch_onnx_parity(model, session, dummy)
    contract = assert_onnx_contract(
        output,
        expected_input_shape=input_shape,
        expected_opset=17,
    )
    return {
        "path": output.name,
        "sha256": sha256(output),
        "bytes": output.stat().st_size,
        "parity": parity,
        "opset": contract["opset"],
        "fixed_input": contract["fixed_input"],
        "operator_inventory": contract["operator_inventory"],
        "custom_ops": contract["custom_ops"],
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
    parser.add_argument("--teacher-report", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--classifier-epochs", type=int, default=40)
    parser.add_argument("--area-epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--area-batch-size", type=int, default=4)
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
    parser.add_argument(
        "--area-architecture",
        choices=("dual_resnet18", "deeplab_resnet50"),
        default="dual_resnet18",
    )
    parser.add_argument("--holdout-fraction", type=float, default=0.2)
    parser.add_argument(
        "--reuse-model-dir",
        type=Path,
        help="reuse already-selected A1 checkpoints and rerun evaluation/export only",
    )
    parser.add_argument(
        "--reuse-discrete-model-dir",
        type=Path,
        help=(
            "reuse discovery/classifier checkpoints while retraining an area "
            "architecture candidate"
        ),
    )
    parser.add_argument(
        "--recover-unreported-model-dir",
        type=Path,
        help=(
            "diagnostic-only recovery when training completed but evaluation "
            "failed before the source report was written"
        ),
    )
    args = parser.parse_args()
    reuse_options = (
        args.reuse_model_dir,
        args.reuse_discrete_model_dir,
        args.recover_unreported_model_dir,
    )
    if sum(value is not None for value in reuse_options) > 1:
        raise ValueError(
            "model reuse/recovery options are mutually exclusive"
        )

    started = time.perf_counter()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    dataset_qa = require_teacher_dataset_gate(args.evidence_dir)
    teacher_report = json.loads(args.teacher_report.read_text(encoding="utf-8"))
    if teacher_report.get("teacher_data_learnability_pass") is not True:
        raise RuntimeError("P4 student screening requires a passed P2 teacher")
    teacher_dataset = teacher_report.get("data_policy", {}).get("dataset_qa", {})
    if teacher_dataset.get("sha256") != dataset_qa["sha256"]:
        raise RuntimeError("teacher and student dataset QA SHA-256 differ")
    if teacher_report.get("architecture_role") != (
        "reference_teacher_not_default_deployable"
    ):
        raise RuntimeError("teacher architecture role is not the reference contract")
    # Filter roles while streaming metadata, before resolving paths or loading
    # instance annotations.  The contaminated legacy split and sealed G5 are
    # therefore not read by the development-screening process at all.
    rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=DEVELOPMENT_ROLES,
    )
    allowed_frame_keys = {
        (int(row["scene_seed"]), int(row["frame_index"])) for row in rows
    }
    records = load_instance_records(
        args.evidence_dir / "g4_instance_records.jsonl",
        allowed_frame_keys=allowed_frame_keys,
    )
    instances_by_key = index_instance_records(records)
    scene_manifests = load_scene_manifests(args.data_root, rows)
    rows = _tag_rows_with_scene_metadata(rows, scene_manifests)
    by_role = partition_rows(rows)
    for forbidden_role in (LEGACY_DIAGNOSTIC_ROLE, SEALED_FINAL_ROLE):
        if by_role[forbidden_role]:
            raise AssertionError(
                f"role filter leaked {forbidden_role} into screening"
            )
    train_all = by_role["train"]
    holdout_raw = _holdout_rows(train_all, args.holdout_fraction)
    holdout_keys = {
        (str(row["world_id"]), int(row["scene_seed"]))
        for row in holdout_raw
    }
    holdout = [
        {**row, "split": "train_world_holdout"}
        for row in holdout_raw
    ]
    eligible_train_rows = [
        row
        for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"]))
        not in holdout_keys
    ]
    train_rows = stratified_row_sample(
        eligible_train_rows, args.max_train_frames, seed=SEED + 8
    )
    val_rows = by_role["val"]
    shift_rows = {
        role: by_role[role]
        for role in ("D1", "D2", "D3", "D4", "D5")
    }
    if args.max_eval_frames > 0:
        holdout = holdout[: args.max_eval_frames]
        val_rows = val_rows[: args.max_eval_frames]
    if not train_rows or not holdout or not val_rows:
        raise RuntimeError(
            "screening requires non-empty train, train_world_holdout and val "
            f"rows; got {len(train_rows)}/{len(holdout)}/{len(val_rows)}"
        )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    print(
        f"[screening] train={len(train_rows)} holdout={len(holdout)} "
        f"val={len(val_rows)} legacy_diagnostic=not_read "
        f"shift_counts={ {role: len(items) for role, items in shift_rows.items()} } "
        f"device={device}",
        flush=True,
    )

    discovery_ckpt = output / "discovery.pt"
    selection_val_rows = stratified_row_sample(val_rows, 100, seed=SEED + 9)
    reuse_discrete_source = (
        args.reuse_model_dir
        or args.reuse_discrete_model_dir
        or args.recover_unreported_model_dir
    )
    if reuse_discrete_source:
        previous_report_path = reuse_discrete_source / "auto05r_screening_report.json"
        if args.recover_unreported_model_dir:
            if previous_report_path.exists():
                raise RuntimeError(
                    "recovery mode requires an interrupted run with no source report"
                )
            previous_report = None
        else:
            previous_report = json.loads(
                previous_report_path.read_text(encoding="utf-8")
            )
            if previous_report.get("student_route", {}).get(
                "dataset_qa_sha256"
            ) != dataset_qa["sha256"]:
                raise RuntimeError(
                    "reused checkpoints were selected on a different formal G4 QA"
                )
        discovery, discovery_training = _load_reused_model(
            "discovery", reuse_discrete_source, output, device
        )
    else:
        previous_report = None
        discovery, discovery_training = train_discovery(
            train_rows,
            instances_by_key,
            device=device,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            seed=SEED,
            val_rows=selection_val_rows,
            checkpoint_path=discovery_ckpt,
            early_stopping_patience=8,
            load_best=True,
            selector=discovery_selector(),
            validation_metric_fn=_discovery_validation_metric_fn(
                selection_val_rows,
                instances_by_key,
                device,
            ),
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
    classifier_val_samples = build_classifier_samples(
        selection_val_rows,
        instances_by_key,
        positive_per_class=min(args.classifier_positives // 4, 50),
        background_per_positive=2,
        negative_only_per_frame=2,
        background_limit=min(args.classifier_backgrounds // 2, 200),
        seed=SEED + 1,
    )
    classifier_ckpt = output / "classifier.pt"
    if reuse_discrete_source:
        classifier, classifier_training = _load_reused_model(
            "classifier", reuse_discrete_source, output, device
        )
    else:
        classifier, classifier_training = train_classifier(
            classifier_samples,
            device=device,
            epochs=args.classifier_epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            seed=SEED,
            val_samples=classifier_val_samples,
            checkpoint_path=classifier_ckpt,
            early_stopping_patience=8,
            load_best=True,
            cache_crops=True,
            selector=classifier_selector(),
            validation_metric_fn=_classifier_validation_metric_fn(
                classifier_val_samples, device
            ),
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
    leaf_validation = _select_area_rows(
        val_rows,
        instances_by_key,
        "leaf",
        args.area_positive_frames // 2,
        args.area_negative_frames // 2,
        SEED + 2,
    )
    puddle_validation = _select_area_rows(
        val_rows,
        instances_by_key,
        "puddle",
        args.area_positive_frames // 2,
        args.area_negative_frames // 2,
        SEED + 3,
    )
    if args.reuse_model_dir or args.recover_unreported_model_dir:
        leaf, leaf_training = _load_reused_model(
            "leaf",
            reuse_discrete_source,
            output,
            device,
            area_architecture=args.area_architecture,
        )
        puddle, puddle_training = _load_reused_model(
            "puddle",
            reuse_discrete_source,
            output,
            device,
            area_architecture=args.area_architecture,
        )
    else:
        leaf, leaf_training = train_area(
            "leaf",
            leaf_rows,
            device=device,
            epochs=args.area_epochs,
            batch_size=args.area_batch_size,
            learning_rate=args.learning_rate,
            seed=SEED,
            val_rows=leaf_validation,
            checkpoint_path=output / "leaf.pt",
            early_stopping_patience=8,
            load_best=True,
            cache_frames=True,
            selector=area_selector(),
            validation_metric_fn=_area_validation_metric_fn(
                leaf_validation, device, "leaf"
            ),
            model=build_g4_model(
                "leaf", area_architecture=args.area_architecture
            ),
        )
        puddle, puddle_training = train_area(
            "puddle",
            puddle_rows,
            device=device,
            epochs=args.area_epochs,
            batch_size=args.area_batch_size,
            learning_rate=args.learning_rate,
            seed=SEED + 1,
            val_rows=puddle_validation,
            checkpoint_path=output / "puddle.pt",
            early_stopping_patience=8,
            load_best=True,
            cache_frames=True,
            selector=area_selector(),
            validation_metric_fn=_area_validation_metric_fn(
                puddle_validation, device, "puddle"
            ),
            model=build_g4_model(
                "puddle", area_architecture=args.area_architecture
            ),
        )

    discovery_selection_metrics = _selected_validation_metrics(
        discovery_training
    )
    classifier_selection_metrics = _selected_validation_metrics(
        classifier_training
    )
    leaf_selection_metrics = _selected_validation_metrics(leaf_training)
    puddle_selection_metrics = _selected_validation_metrics(puddle_training)
    discovery_threshold = float(
        discovery_selection_metrics.get(
            "validation_discovery_threshold", args.discovery_threshold
        )
    )
    class_threshold = float(
        classifier_selection_metrics.get(
            "validation_classifier_threshold", args.class_threshold
        )
    )
    area_thresholds = (
        float(
            leaf_selection_metrics.get(
                "validation_area_threshold", args.area_threshold
            )
        ),
        float(
            puddle_selection_metrics.get(
                "validation_area_threshold", args.area_threshold
            )
        ),
    )
    reused_discrete_calibration = None
    if args.reuse_discrete_model_dir:
        calibration_frames = discovery_predictions(
            discovery,
            selection_val_rows,
            instances_by_key,
            device=device,
            threshold=min(DISCOVERY_THRESHOLD_GRID),
            max_detections=100,
        )
        discovery_operating_point = select_discovery_threshold(
            calibration_frames
        )
        classifier_operating_point = _select_classifier_threshold(
            _classifier_sample_scores(
                classifier, classifier_val_samples, device
            )
        )
        discovery_threshold = float(
            discovery_operating_point["threshold"]
        )
        class_threshold = float(classifier_operating_point["threshold"])
        reused_discrete_calibration = {
            "selection_split": "val",
            "diagnostic_area_comparison_only": True,
            "discovery": {
                key: value
                for key, value in discovery_operating_point.items()
                if key != "sweep"
            },
            "classifier": {
                key: value
                for key, value in classifier_operating_point.items()
                if key != "sweep"
            },
        }
    selected_models_product_eligible = all(
        _selection_product_eligible(report)
        for report in (
            discovery_training,
            classifier_training,
            leaf_training,
            puddle_training,
        )
    )
    if args.recover_unreported_model_dir or args.reuse_discrete_model_dir:
        selected_models_product_eligible = False

    in_domain = _evaluate_split(
        discovery,
        classifier,
        leaf,
        puddle,
        holdout,
        instances_by_key,
        device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
        area_thresholds=area_thresholds,
    )
    cross_world = _evaluate_split(
        discovery,
        classifier,
        leaf,
        puddle,
        val_rows,
        instances_by_key,
        device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
        area_thresholds=area_thresholds,
    )
    shift_reports = {
        role: _evaluate_split(
            discovery,
            classifier,
            leaf,
            puddle,
            role_rows,
            instances_by_key,
            device,
            discovery_threshold=discovery_threshold,
            class_threshold=class_threshold,
            area_thresholds=area_thresholds,
        )
        for role, role_rows in shift_rows.items()
        if role_rows
    }
    val_cross_world = _cross_world_f1(
        discovery,
        classifier,
        val_rows,
        instances_by_key,
        device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
    )
    stress_holdout = _stress_macro_f1(
        discovery,
        classifier,
        leaf,
        puddle,
        holdout,
        instances_by_key,
        device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
        area_thresholds=area_thresholds,
    )
    stress_val = _stress_macro_f1(
        discovery,
        classifier,
        leaf,
        puddle,
        val_rows,
        instances_by_key,
        device,
        discovery_threshold=discovery_threshold,
        class_threshold=class_threshold,
        area_thresholds=area_thresholds,
    )
    stress_values = [
        value for value in (stress_holdout, stress_val) if value is not None
    ]
    color_material_stress_macro_f1 = (
        float(min(stress_values)) if stress_values else None
    )

    onnx = {
        "discovery": _export_onnx_task_specific(
            discovery,
            (1, 3, DISCOVERY_MODEL_SIZE[1], DISCOVERY_MODEL_SIZE[0]),
            output / "discovery.onnx",
            device,
        ),
        "classifier": _export_onnx_task_specific(
            classifier,
            (1, 3, CLASSIFIER_MODEL_SIZE[0], CLASSIFIER_MODEL_SIZE[1]),
            output / "classifier.onnx",
            device,
        ),
        "leaf": _export_onnx_task_specific(
            leaf,
            (1, AREA_FEATURE_COUNT, AREA_MODEL_SIZE[1], AREA_MODEL_SIZE[0]),
            output / "leaf.onnx",
            device,
        ),
        "puddle": _export_onnx_task_specific(
            puddle,
            (1, AREA_FEATURE_COUNT, AREA_MODEL_SIZE[1], AREA_MODEL_SIZE[0]),
            output / "puddle.onnx",
            device,
        ),
    }
    onnx_parity_pass = product_parity_gate(onnx)
    onnx_custom_ops_zero = all(
        report["custom_ops"] == 0 for report in onnx.values()
    )

    D1_D5_reports_complete = all(
        role in shift_reports
        and shift_reports[role].get("rows", 0) > 0
        for role in ("D1", "D2", "D3", "D4", "D5")
    )
    specificity_reports = {
        "in_domain": in_domain["same_color_negative_specificity"],
        "cross_world": cross_world["same_color_negative_specificity"],
        **{
            role: item["same_color_negative_specificity"]
            for role, item in shift_reports.items()
        },
    }
    evaluated_specificities = [
        item["specificity"]
        for item in specificity_reports.values()
        if item.get("status") == "evaluated"
    ]
    same_color = {
        "status": (
            "evaluated" if evaluated_specificities else "not_evaluated"
        ),
        "specificity": (
            float(min(evaluated_specificities))
            if evaluated_specificities
            else None
        ),
        "by_split": specificity_reports,
    }
    same_color_value = same_color["specificity"]
    screening_metrics = {
        "candidate": {
            "all_gt_candidate_recall": min(
                in_domain["candidate"]["all_gt_candidate_recall"],
                cross_world["candidate"]["all_gt_candidate_recall"],
            ),
            "false_candidates_per_min": max(
                in_domain["candidate"]["false_candidates_per_min"],
                cross_world["candidate"]["false_candidates_per_min"],
            ),
            "negative_only_fp_per_frame": max(
                in_domain["candidate"]["negative_only_fp_per_frame"],
                cross_world["candidate"]["negative_only_fp_per_frame"],
            ),
        },
        "discrete": {
            "paper_precision": min(
                in_domain["discrete"]["per_class"]["paper_litter"]["precision"],
                cross_world["discrete"]["per_class"]["paper_litter"]["precision"],
            ),
            "small_object_recall": min(
                in_domain["discrete"]["small_object_recall"],
                cross_world["discrete"]["small_object_recall"],
            ),
        },
        "area": {
            "iou_by_class": {
                name: min(
                    in_domain["area"]["iou_by_class"][name],
                    cross_world["area"]["iou_by_class"][name],
                )
                for name in ("leaf_pile", "puddle")
            },
            "macro_miou": min(
                in_domain["area"]["macro_miou"],
                cross_world["area"]["macro_miou"],
            ),
            "boundary_f1": min(
                in_domain["area"]["boundary_f1"],
                cross_world["area"]["boundary_f1"],
            ),
            "negative_area_fp_per_frame": max(
                in_domain["area"]["negative_area_fp_per_frame"],
                cross_world["area"]["negative_area_fp_per_frame"],
            ),
        },
    }
    policy_metrics = {
        "in_domain": {
            "candidate": in_domain["candidate"],
            "discrete": in_domain["discrete"],
            "area": in_domain["area"],
        },
        "cross_world": {
            "macro_f1": val_cross_world,
            "per_class_recall": cross_world["per_class_recall"],
            "discrete": cross_world["discrete"],
        },
        "screening": screening_metrics,
        "color_material_stress_macro_f1": color_material_stress_macro_f1,
        "val_per_class_recall": float(
            min(cross_world["per_class_recall"].values())
        ),
        "same_color_negative_specificity": same_color_value,
        "D1_D5_reports_complete": D1_D5_reports_complete,
        "selected_models_product_eligible": selected_models_product_eligible,
        "onnx_task_specific_parity_pass": onnx_parity_pass,
        "onnx_custom_ops_zero": onnx_custom_ops_zero,
    }
    policy = load_policy(P4_POLICY)
    policy_result = evaluate_policy(policy, policy_metrics)
    decision = screening_decision(
        {
            gate_id: bool(gate["passed"])
            for gate_id, gate in policy_result["gates"].items()
        }
    )

    report = {
        "schema_version": 2,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-4",
        "student_route": {
            "attempt": "A1_FCOS_lite_ResNet18_FPN",
            "teacher_report": str(args.teacher_report),
            "teacher_report_sha256": sha256(args.teacher_report),
            "teacher_checkpoint_sha256": teacher_report.get("checkpoint", {}).get(
                "sha256"
            ),
            "teacher_val_recall": teacher_report["cross_world_val_metrics"][
                "all_gt_candidate_recall"
            ],
            "teacher_val_false_candidates_per_min": teacher_report[
                "cross_world_val_metrics"
            ]["false_candidates_per_min"],
            "dataset_qa_sha256": dataset_qa["sha256"],
            "architecture_attempt_limit": 3,
            "attempts_used": 1,
            "area_architecture": args.area_architecture,
            "reused_selected_checkpoints_for_diagnostic_evaluation": bool(
                args.reuse_model_dir
            ),
            "reused_discrete_checkpoints_for_area_comparison": bool(
                args.reuse_discrete_model_dir
            ),
            "recovered_after_post_training_evaluation_failure": bool(
                args.recover_unreported_model_dir
            ),
            "reuse_source_report_sha256": (
                sha256(reuse_discrete_source / "auto05r_screening_report.json")
                if reuse_discrete_source
                and (reuse_discrete_source / "auto05r_screening_report.json").is_file()
                else None
            ),
        },
        "AUTO_05R_PASS": decision["AUTO_05R_PASS"],
        "AUTO_05R_BLOCKED": decision["AUTO_05R_BLOCKED"],
        "P4_SCREENING_PASS": decision["P4_SCREENING_PASS"],
        "gates": {
            gate_id: {
                "passed": gate["passed"],
                "value": gate["value"],
                "threshold": gate["threshold"],
                "operator": gate["operator"],
                "not_evaluated": gate["not_evaluated"],
            }
            for gate_id, gate in policy_result["gates"].items()
        },
        "in_domain_validation": in_domain,
        "cross_world_validation": cross_world,
        "legacy_G4_D6_diagnostic": {
            "status": "not_read_by_development_screening",
            "included_in_decision": False,
            "runner": "separate_post_freeze_diagnostic_only",
        },
        "legacy_G4_D6_diagnostic_included_in_decision": False,
        "G5_SEALED_FINAL": {
            "status": "not_evaluated",
            "reason": "sealed_final_not_created_and_separately_gated",
            "included_in_decision": False,
        },
        "D1_D5": {
            "reports_complete": D1_D5_reports_complete,
            "counts": {role: len(shift_rows[role]) for role in shift_rows},
            "reports": shift_reports,
        },
        "same_color_negative_specificity": same_color,
        "stress_macro_f1": {
            "in_domain": stress_holdout,
            "cross_world": stress_val,
        },
        "split_roles": {
            "development_readable": list(DEVELOPMENT_ROLES),
            "train_frames": len(train_rows),
            "train_world_holdout_frames": len(holdout),
            "val_frames": len(val_rows),
            "legacy_diagnostic_frames_read": 0,
            "train_world_counts": {
                world_id: sum(
                    1 for row in train_rows if str(row["world_id"]) == world_id
                )
                for world_id in sorted(
                    {str(row["world_id"]) for row in train_rows}
                )
            },
        },
        "selection": {
            "discovery": discovery_training.get("selection"),
            "classifier": classifier_training.get("selection"),
            "leaf": leaf_training.get("selection"),
            "puddle": puddle_training.get("selection"),
            "all_product_eligible": selected_models_product_eligible,
        },
        "training": {
            "discovery": discovery_training,
            "classifier": classifier_training,
            "leaf": leaf_training,
            "puddle": puddle_training,
        },
        "onnx": onnx,
        "onnx_task_specific_parity_pass": onnx_parity_pass,
        "thresholds": {
            "discovery": discovery_threshold,
            "classifier": class_threshold,
            "area": {
                "leaf": area_thresholds[0],
                "puddle": area_thresholds[1],
            },
        },
        "calibration": {
            "selection_split": "val",
            "sealed_final_read": False,
            "discovery_threshold_grid": list(DISCOVERY_THRESHOLD_GRID),
            "classifier_threshold_grid": list(CLASSIFIER_THRESHOLD_GRID),
            "area_threshold_grid": list(AREA_THRESHOLD_GRID),
            "selected_models_product_eligible": selected_models_product_eligible,
            "reused_discrete_operating_points": reused_discrete_calibration,
        },
        "policy": {
            "p4": str(P4_POLICY.relative_to(ROOT)),
            "policy_result": policy_result,
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
                "P4_SCREENING_PASS": report["P4_SCREENING_PASS"],
                "not_evaluated_gates": policy_result["not_evaluated"],
                "failed_gates": [
                    name
                    for name, gate in policy_result["gates"].items()
                    if not gate["passed"]
                ],
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if report["AUTO_05R_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
