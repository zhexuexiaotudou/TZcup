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
    export_fixed_onnx,
    torch_onnx_parity,
)
from sanitation_learning.g4_onnx_parity import (  # noqa: E402
    assert_onnx_contract,
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
)
from sanitation_learning.g4_train import (  # noqa: E402
    train_area,
    train_classifier,
    train_discovery,
)


SEED = 20260807
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


def _classifier_sample_metrics(model, samples: list[dict], device) -> dict:
    """Macro F1, paper precision and specificity over classifier samples."""
    confusion = {name: {"tp": 0, "fp": 0, "fn": 0} for name in ("background", *DISCRETE_NAMES)}
    hard_negative_total = 0
    hard_negative_misclassified = 0
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
            predicted = int(np.argmax(logits))
            truth = int(sample["label"])
            if predicted == truth:
                confusion[("background", *DISCRETE_NAMES)[truth]]["tp"] += 1
            else:
                confusion[("background", *DISCRETE_NAMES)[predicted]]["fp"] += 1
                confusion[("background", *DISCRETE_NAMES)[truth]]["fn"] += 1
            if sample.get("hard_negative"):
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
    return {
        "validation_macro_f1": macro_f1,
        "validation_paper_precision": per_class["paper_litter"]["precision"],
        "validation_background_specificity": background_specificity(confusion),
        "validation_hard_negative_specificity": (
            1.0
            - hard_negative_misclassified / max(hard_negative_total, 1)
            if hard_negative_total
            else 0.0
        ),
        "validation_sample_count": len(samples),
    }


def _discovery_validation_metric_fn(
    rows, instances_by_key, device, threshold: float
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
            threshold=threshold,
            max_detections=100,
        )
        candidate = discovery_metrics(frames)
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
        metrics = _classifier_sample_metrics(model, samples, device)
        return {
            "validation_loss": total / max(steps, 1),
            **metrics,
        }

    return metric_fn


def _area_validation_metric_fn(
    rows, device, task: str, area_threshold: float
):
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
            thresholds=(area_threshold, area_threshold),
            task=task,
        )
        metrics = area_metrics(predictions)
        key = "leaf_pile" if task == "leaf" else "puddle"
        return {
            "validation_loss": total / max(steps, 1),
            "validation_iou": metrics["iou_by_class"][key],
            "validation_macro_miou": metrics["macro_miou"],
            "validation_boundary_f1": metrics["boundary_f1"],
            "validation_negative_area_fp_per_frame": metrics[
                "negative_area_fp_per_frame"
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
    area_threshold: float,
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
        area_thresholds=(area_threshold, area_threshold),
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
    parser.add_argument("--holdout-fraction", type=float, default=0.2)
    args = parser.parse_args()

    started = time.perf_counter()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
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
    train_rows = [
        row
        for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"]))
        not in holdout_keys
    ][: args.max_train_frames]
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
    discovery, discovery_training = train_discovery(
        train_rows,
        instances_by_key,
        device=device,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED,
        val_rows=holdout[:100],
        checkpoint_path=discovery_ckpt,
        early_stopping_patience=8,
        load_best=True,
        selector=discovery_selector(),
        validation_metric_fn=_discovery_validation_metric_fn(
            holdout[:100],
            instances_by_key,
            device,
            threshold=args.discovery_threshold,
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
        holdout,
        instances_by_key,
        positive_per_class=min(args.classifier_positives // 4, 50),
        background_per_positive=2,
        negative_only_per_frame=2,
        background_limit=min(args.classifier_backgrounds // 2, 200),
        seed=SEED + 1,
    )
    classifier_ckpt = output / "classifier.pt"
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
    leaf_holdout = _select_area_rows(
        holdout,
        instances_by_key,
        "leaf",
        args.area_positive_frames // 2,
        args.area_negative_frames // 2,
        SEED + 2,
    )
    puddle_holdout = _select_area_rows(
        holdout,
        instances_by_key,
        "puddle",
        args.area_positive_frames // 2,
        args.area_negative_frames // 2,
        SEED + 3,
    )
    leaf, leaf_training = train_area(
        "leaf",
        leaf_rows,
        device=device,
        epochs=args.area_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED,
        val_rows=leaf_holdout,
        checkpoint_path=output / "leaf.pt",
        early_stopping_patience=8,
        load_best=True,
        cache_frames=True,
        selector=area_selector(),
        validation_metric_fn=_area_validation_metric_fn(
            leaf_holdout, device, "leaf", args.area_threshold
        ),
    )
    puddle, puddle_training = train_area(
        "puddle",
        puddle_rows,
        device=device,
        epochs=args.area_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        seed=SEED + 1,
        val_rows=puddle_holdout,
        checkpoint_path=output / "puddle.pt",
        early_stopping_patience=8,
        load_best=True,
        cache_frames=True,
        selector=area_selector(),
        validation_metric_fn=_area_validation_metric_fn(
            puddle_holdout, device, "puddle", args.area_threshold
        ),
    )

    in_domain = _evaluate_split(
        discovery,
        classifier,
        leaf,
        puddle,
        holdout,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_threshold=args.area_threshold,
    )
    cross_world = _evaluate_split(
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
    shift_reports = {
        role: _evaluate_split(
            discovery,
            classifier,
            leaf,
            puddle,
            role_rows,
            instances_by_key,
            device,
            discovery_threshold=args.discovery_threshold,
            class_threshold=args.class_threshold,
            area_threshold=args.area_threshold,
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
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
    )
    stress_holdout = _stress_macro_f1(
        discovery,
        classifier,
        leaf,
        puddle,
        holdout,
        instances_by_key,
        device,
        discovery_threshold=args.discovery_threshold,
        class_threshold=args.class_threshold,
        area_threshold=args.area_threshold,
    )
    stress_val = _stress_macro_f1(
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
    onnx_parity_pass = all(
        report["parity"]["max_absolute_error"] <= 1e-4
        and report["parity"].get("passed", False)
        and report["custom_ops"] == 0
        and report["opset"] == 17
        for report in onnx.values()
    )
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
        },
        "selection": {
            "discovery": discovery_training.get("selection"),
            "classifier": classifier_training.get("selection"),
            "leaf": leaf_training.get("selection"),
            "puddle": puddle_training.get("selection"),
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
            "discovery": args.discovery_threshold,
            "classifier": args.class_threshold,
            "area": args.area_threshold,
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
