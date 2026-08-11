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
    discrete_boxes_for_frame,
    index_instance_records,
    load_classifier_crop,
    load_frame_rows,
    load_instance_records,
    load_scene_manifests,
)
from sanitation_learning.auto04_contract import box_iou  # noqa: E402
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
from sanitation_learning.g4_teacher import (  # noqa: E402
    build_fcos_teacher,
    require_teacher_dataset_gate,
    teacher_predictions,
)
from sanitation_learning.g4_train import (  # noqa: E402
    train_area,
    train_classifier,
    train_discovery,
)


SEED = 20260807
CLASSIFIER_EXPORT_BATCH = 16
PRODUCT_MAXIMUM_CANDIDATES = CLASSIFIER_EXPORT_BATCH
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


def _prepare_a3_distillation_targets(
    *,
    teacher_report: dict,
    teacher_report_path: Path,
    train_rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    device: torch.device,
    output: Path,
) -> tuple[dict[tuple[int, int], list[dict]], dict]:
    """Freeze teacher detections on train only for A3 soft-quality targets."""
    checkpoint_meta = teacher_report.get("checkpoint") or {}
    checkpoint_path = teacher_report_path.parent / str(
        checkpoint_meta.get("path", "")
    )
    if not checkpoint_path.is_file():
        raise FileNotFoundError(
            f"A3 teacher checkpoint is missing: {checkpoint_path}"
        )
    checkpoint_hash = sha256(checkpoint_path)
    if checkpoint_hash != checkpoint_meta.get("sha256"):
        raise RuntimeError("A3 teacher checkpoint SHA-256 mismatch")
    input_scale = int(teacher_report.get("config", {}).get("input_scale", 1))
    threshold = float(
        teacher_report["frozen_threshold_from_train_world_holdout"]
    )
    teacher = build_fcos_teacher(input_scale).to(device)
    checkpoint = torch.load(
        checkpoint_path, map_location=device, weights_only=False
    )
    teacher.load_state_dict(checkpoint["state_dict"], strict=True)
    frames = teacher_predictions(
        teacher,
        train_rows,
        instances_by_key,
        device=device,
        score_threshold=threshold,
        batch_size=int(
            teacher_report.get("config", {}).get("eval_batch_size", 4)
        ),
        input_scale=input_scale,
    )
    metrics = discovery_metrics(frames)
    targets = {}
    for frame in frames:
        aligned = []
        for truth in frame["truth"]:
            matches = [
                item
                for item in frame["detections"]
                if box_iou(
                    tuple(truth["bbox_xyxy"]), tuple(item["bbox_xyxy"])
                )
                >= 0.5
            ]
            if matches:
                best = max(matches, key=lambda item: float(item["score"]))
                aligned.append(
                    {
                        "class_index": 0,
                        "bbox_xyxy": list(truth["bbox_xyxy"]),
                        "score": float(best["score"]),
                    }
                )
        targets[(int(frame["scene_seed"]), int(frame["frame_index"]))] = aligned
    manifest_path = output / "a3_teacher_distillation_targets.json"
    manifest = {
        "schema_version": 1,
        "role": "train_only_frozen_teacher_soft_quality_targets",
        "teacher_checkpoint_sha256": checkpoint_hash,
        "teacher_threshold": threshold,
        "train_frames": len(frames),
        "metrics": metrics,
        "legacy_G4_D6_read": False,
        "G5_sealed_final_read": False,
        "frames": [
            {
                "scene_seed": int(frame["scene_seed"]),
                "frame_index": int(frame["frame_index"]),
                "quality_targets": targets[
                    (int(frame["scene_seed"]), int(frame["frame_index"]))
                ],
            }
            for frame in frames
        ],
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    del teacher
    torch.cuda.empty_cache()
    return targets, {
        "teacher_checkpoint_sha256": checkpoint_hash,
        "teacher_threshold": threshold,
        "train_frames": len(frames),
        "train_metrics": metrics,
        "target_manifest": manifest_path.name,
        "target_manifest_sha256": sha256(manifest_path),
        "pyramid_target_assignment": "single_level_by_max_side_48_80",
        "quality_target": "frozen_teacher_score_heatmap",
        "quality_target_alignment": "teacher_iou_ge_0.5_score_at_gt_box",
        "matched_quality_targets": sum(len(items) for items in targets.values()),
    }


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


def _selection_fingerprint(selection: dict | None) -> dict:
    selection = selection or {}
    return {
        key: selection.get(key)
        for key in (
            "selected_epoch",
            "selection_score",
            "tie_breaker_score",
            "validation_metrics",
            "violated_constraints",
        )
    }


def _load_qualified_task_reuse_report(
    source_dir: Path,
    dataset_qa_sha256: str,
    tasks: tuple[str, ...],
) -> tuple[dict, dict]:
    """Validate task-level formal checkpoint reuse without promoting a failed run."""
    report_path = source_dir / "auto05r_screening_report.json"
    if not report_path.is_file():
        raise FileNotFoundError(
            f"qualified checkpoint reuse report is missing: {report_path}"
        )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    source_qa = report.get("student_route", {}).get("dataset_qa_sha256")
    if source_qa != dataset_qa_sha256:
        raise RuntimeError(
            "qualified reused checkpoints were selected on a different "
            "formal G4 QA"
        )
    training = report.get("training")
    if not isinstance(training, dict):
        raise RuntimeError("qualified reuse report has no task training records")
    provenance = {
        "source_dir": str(source_dir),
        "source_report": str(report_path),
        "source_report_sha256": sha256(report_path),
        "dataset_qa_sha256": source_qa,
        "tasks": {},
    }
    for task in tasks:
        task_training = training.get(task)
        if not isinstance(task_training, dict):
            raise RuntimeError(
                f"qualified reuse report has no {task} training record"
            )
        if not _selection_product_eligible(task_training):
            raise RuntimeError(
                f"qualified reuse source {task} checkpoint is not product eligible"
            )
        report_selection = report.get("selection", {}).get(task)
        if report_selection != task_training.get("selection"):
            raise RuntimeError(
                f"qualified reuse source {task} selection records disagree"
            )
        checkpoint_path = source_dir / f"{task}.pt"
        if not checkpoint_path.is_file():
            raise FileNotFoundError(
                f"qualified reuse source {task} checkpoint is missing: "
                f"{checkpoint_path}"
            )
        provenance["tasks"][task] = {
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": sha256(checkpoint_path),
            "selection": task_training.get("selection"),
        }
    return report, provenance


def _load_reused_model(
    task: str,
    source_dir: Path,
    output_dir: Path,
    device: torch.device,
    *,
    area_architecture: str = "dual_resnet18",
    discovery_architecture: str = "resnet18_fpn_a1",
    expected_selection: dict | None = None,
) -> tuple[torch.nn.Module, dict]:
    source = source_dir / f"{task}.pt"
    if not source.is_file():
        raise FileNotFoundError(f"reused {task} checkpoint is missing: {source}")
    checkpoint = torch.load(source, map_location=device, weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete":
        raise RuntimeError(
            f"reused {task} checkpoint is not marked training_complete"
        )
    if expected_selection is not None:
        checkpoint_selection = checkpoint.get("selection")
        if _selection_fingerprint(checkpoint_selection) != _selection_fingerprint(
            expected_selection
        ):
            raise RuntimeError(
                f"reused {task} checkpoint selection disagrees with source report"
            )
        if not _selection_product_eligible(
            {"selection": checkpoint_selection or {}}
        ):
            raise RuntimeError(
                f"reused {task} checkpoint is not product eligible"
            )
    state = checkpoint.get("state_dict")
    if not isinstance(state, dict) or not state:
        raise RuntimeError(f"reused {task} checkpoint has no selected state_dict")
    model = build_g4_model(
        task,
        area_architecture=area_architecture,
        discovery_architecture=discovery_architecture,
    ).to(device)
    expected_contract = {
        "model_id": getattr(model, "model_id", None),
        "architecture_role": getattr(model, "architecture_role", None),
        "discovery_architecture": getattr(
            model, "discovery_architecture", None
        ),
    }
    checkpoint_contract = checkpoint.get("model_contract")
    if checkpoint_contract is not None and checkpoint_contract != expected_contract:
        raise RuntimeError(
            f"reused {task} checkpoint model contract mismatch: "
            f"expected {expected_contract}, got {checkpoint_contract}"
        )
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
        "model_contract": checkpoint.get("model_contract"),
        "checkpoint_status": checkpoint.get("checkpoint_status"),
        "device": str(device),
    }


def _freeze_area_backbone(model: torch.nn.Module) -> dict:
    frozen = []
    trainable = []
    for name, parameter in model.named_parameters():
        if name.startswith(("deeplab.backbone.", "geometry_stem.")):
            parameter.requires_grad_(False)
            frozen.append(name)
        else:
            trainable.append(name)
    if not frozen or not trainable:
        raise RuntimeError(
            "area backbone freeze requires both frozen backbone and trainable decoder parameters"
        )
    batch_norm_modules = 0
    for module in model.modules():
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
            module.eval()
            batch_norm_modules += 1
            for parameter in module.parameters():
                parameter.requires_grad_(False)
    model.force_batch_norm_eval = True
    return {
        "frozen_parameter_tensors": len(frozen),
        "trainable_parameter_tensors": len(trainable),
        "frozen_prefixes": ["deeplab.backbone", "geometry_stem"],
        "frozen_batch_norm_modules": batch_norm_modules,
    }


def _freeze_area_refiner_only(model: torch.nn.Module) -> dict:
    frozen = []
    trainable = []
    for name, parameter in model.named_parameters():
        if name.startswith("highres_refiner."):
            parameter.requires_grad_(True)
            trainable.append(name)
        else:
            parameter.requires_grad_(False)
            frozen.append(name)
    if not frozen or not trainable:
        raise RuntimeError(
            "area refiner-only tuning requires a highres_refiner and frozen base"
        )
    batch_norm_modules = 0
    for module in model.modules():
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
            module.eval()
            batch_norm_modules += 1
    model.force_batch_norm_eval = True
    return {
        "frozen_parameter_tensors": len(frozen),
        "trainable_parameter_tensors": len(trainable),
        "trainable_prefixes": ["highres_refiner"],
        "frozen_batch_norm_modules": batch_norm_modules,
    }


def _warm_start_area_model(
    task: str,
    source_dir: Path,
    *,
    area_architecture: str,
    freeze_backbone: bool,
    refiner_only: bool = False,
) -> tuple[torch.nn.Module, dict]:
    source = source_dir / f"{task}.pt"
    if not source.is_file():
        raise FileNotFoundError(f"area warm-start checkpoint is missing: {source}")
    checkpoint = torch.load(source, map_location="cpu", weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete":
        raise RuntimeError(
            f"area warm-start {task} checkpoint is not marked training_complete"
        )
    model = build_g4_model(task, area_architecture=area_architecture)
    expected_contract = {
        "model_id": getattr(model, "model_id", None),
        "architecture_role": getattr(model, "architecture_role", None),
        "discovery_architecture": getattr(
            model, "discovery_architecture", None
        ),
    }
    checkpoint_contract = checkpoint.get("model_contract")
    boundary_refine_upgrade = (
        area_architecture == "deeplab_resnet50_boundary_refine"
        and checkpoint_contract
        and checkpoint_contract.get("architecture_role")
        == "deeplab_resnet50_rgb_preserved_shallow_geometry"
        and checkpoint_contract.get("model_id")
        == f"g4_{task}_segmenter_deeplab_v1"
    )
    if (
        checkpoint_contract is not None
        and checkpoint_contract != expected_contract
        and not boundary_refine_upgrade
    ):
        raise RuntimeError(
            f"area warm-start {task} model contract mismatch: "
            f"expected {expected_contract}, got {checkpoint_contract}"
        )
    state = checkpoint.get("state_dict")
    if not isinstance(state, dict) or not state:
        raise RuntimeError(
            f"area warm-start {task} checkpoint has no selected state_dict"
        )
    if boundary_refine_upgrade:
        incompatible = model.load_state_dict(state, strict=False)
        unexpected = list(incompatible.unexpected_keys)
        invalid_missing = [
            name
            for name in incompatible.missing_keys
            if not name.startswith("highres_refiner.")
        ]
        if unexpected or invalid_missing:
            raise RuntimeError(
                "boundary-refine warm start has incompatible state: "
                f"missing={invalid_missing}, unexpected={unexpected}"
            )
    else:
        model.load_state_dict(state, strict=True)
    freeze = (
        _freeze_area_refiner_only(model)
        if refiner_only
        else (_freeze_area_backbone(model) if freeze_backbone else None)
    )
    return model, {
        "source": str(source),
        "source_sha256": sha256(source),
        "checkpoint_status": checkpoint.get("checkpoint_status"),
        "model_contract": checkpoint_contract,
        "architecture_upgrade": (
            "deeplab_v1_to_highres_boundary_refine_v2"
            if boundary_refine_upgrade
            else None
        ),
        "freeze": freeze,
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


def _row_identity(row: dict) -> tuple[str, int, int]:
    return (
        str(row["world_id"]),
        int(row["scene_seed"]),
        int(row["frame_index"]),
    )


def _prioritized_discovery_row_sample(
    rows: list[dict],
    small_object_frame_keys: set[tuple[str, int, int]],
    limit: int,
    *,
    seed: int,
) -> list[dict]:
    """Retain scarce small-object frames before filling the formal sample.

    The previous generic frame sampler retained only 22 of 102 available
    small objects.  P4 evaluates native short sides below 18 px explicitly,
    so dropping most of their training frames made that fixed gate largely a
    sampling accident.  This helper stays deterministic and uses the existing
    stratified sampler for both the mandatory subset (if it alone is too
    large) and the remaining capacity.
    """
    if limit <= 0 or limit >= len(rows):
        return list(rows)
    small_rows = [
        row for row in rows if _row_identity(row) in small_object_frame_keys
    ]
    if len(small_rows) >= limit:
        return stratified_row_sample(small_rows, limit, seed=seed)
    selected_keys = {_row_identity(row) for row in small_rows}
    remaining = [
        row for row in rows if _row_identity(row) not in selected_keys
    ]
    filler = stratified_row_sample(
        remaining, limit - len(small_rows), seed=seed
    )
    return small_rows + filler


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
        taxonomies = {
            str(item.get("taxonomy"))
            for item in scene.get("objects", [])
            if item.get("taxonomy") and not item.get("semantic_label")
        }
        ground = scene.get("ground_material_executed_by_world")
        lighting = scene.get("lighting_executed_by_world")
        if ground:
            taxonomies.add(f"ground:{ground}")
        if lighting:
            taxonomies.add(f"lighting:{lighting}")
        updated["taxonomies"] = sorted(taxonomies)
        tagged.append(updated)
    return tagged


def _taxonomy_balanced_negative_sample(
    rows: list[dict], limit: int, seed: int
) -> list[dict]:
    """Deterministically cover negative taxonomy before stratified fill."""
    if limit <= 0:
        return []
    rng = random.Random(seed)
    buckets: dict[str, list[dict]] = {}
    for row in rows:
        for taxonomy in row.get("taxonomies", ()) or ("unclassified",):
            buckets.setdefault(str(taxonomy), []).append(row)
    for bucket in buckets.values():
        rng.shuffle(bucket)

    selected: list[dict] = []
    selected_keys: set[tuple[str, int, int]] = set()
    while len(selected) < limit:
        added = False
        for taxonomy in sorted(buckets):
            bucket = buckets[taxonomy]
            while bucket and _row_identity(bucket[-1]) in selected_keys:
                bucket.pop()
            if not bucket:
                continue
            row = bucket.pop()
            key = _row_identity(row)
            selected.append(row)
            selected_keys.add(key)
            added = True
            if len(selected) >= limit:
                break
        if not added:
            break

    if len(selected) < limit:
        remaining = [
            row for row in rows if _row_identity(row) not in selected_keys
        ]
        selected.extend(
            stratified_row_sample(
                remaining, limit - len(selected), seed=seed + 1
            )
        )
    return selected


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
    return positives[:positive_limit] + _taxonomy_balanced_negative_sample(
        negatives, negative_limit, seed + 1
    )


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
        max_detections=PRODUCT_MAXIMUM_CANDIDATES,
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
            max_detections=PRODUCT_MAXIMUM_CANDIDATES,
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
        validation_iou = float(metrics["iou_by_class"][key])
        validation_boundary_f1 = float(
            metrics["boundary_f1_by_class"][key]
        )
        balanced_score = (
            2.0
            * validation_iou
            * validation_boundary_f1
            / max(validation_iou + validation_boundary_f1, 1e-12)
        )
        return {
            "validation_loss": total / max(steps, 1),
            "validation_iou": validation_iou,
            "validation_macro_miou": metrics["macro_miou"],
            "validation_boundary_f1": validation_boundary_f1,
            "validation_area_balanced_score": balanced_score,
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
            max_detections=PRODUCT_MAXIMUM_CANDIDATES,
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
    parser.add_argument(
        "--disable-area-frame-cache",
        action="store_true",
        help="reread Area frames each epoch to bound memory for full TRAIN pools",
    )
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
        "--discovery-architecture",
        choices=(
            "resnet18_fpn_a1",
            "mobilenetv3_small_fpn_a2",
            "teacher_distilled_mobilenetv3_fpn_a3",
        ),
        default="resnet18_fpn_a1",
    )
    parser.add_argument(
        "--area-architecture",
        choices=(
            "dual_resnet18",
            "deeplab_resnet50",
            "deeplab_resnet50_boundary_refine",
        ),
        default="dual_resnet18",
    )
    parser.add_argument(
        "--area-warm-start-dir",
        type=Path,
        help="initialize fresh area training from completed leaf/puddle checkpoints",
    )
    parser.add_argument(
        "--area-freeze-backbone",
        action="store_true",
        help="freeze the warm-start DeepLab backbone and train decoder/boundary heads",
    )
    parser.add_argument(
        "--area-refiner-only",
        action="store_true",
        help="freeze the complete warm-start base and tune only highres_refiner",
    )
    parser.add_argument(
        "--reuse-leaf-model-dir",
        type=Path,
        help="reuse a completed leaf checkpoint while retraining only puddle",
    )
    parser.add_argument(
        "--area-low-light-augmentation",
        action="store_true",
        help="apply deterministic TRAIN-only low-exposure cool-light augmentation",
    )
    parser.add_argument("--area-boundary-loss-weight", type=float, default=0.35)
    parser.add_argument("--area-negative-loss-weight", type=float, default=1.0)
    parser.add_argument("--area-boundary-pixel-weight", type=float, default=2.0)
    parser.add_argument(
        "--area-semantic-boundary-weight", type=float, default=0.0
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
    parser.add_argument(
        "--reuse-qualified-nondiscovery-model-dir",
        type=Path,
        help=(
            "train a fresh discovery candidate while reusing only classifier, "
            "leaf and puddle checkpoints that are task-level product eligible "
            "in a same-QA formal report"
        ),
    )
    args = parser.parse_args()
    reuse_options = (
        args.reuse_model_dir,
        args.reuse_discrete_model_dir,
        args.recover_unreported_model_dir,
        args.reuse_qualified_nondiscovery_model_dir,
    )
    if sum(value is not None for value in reuse_options) > 1:
        raise ValueError(
            "model reuse/recovery options are mutually exclusive"
        )
    if args.area_freeze_backbone and not args.area_warm_start_dir:
        raise ValueError("--area-freeze-backbone requires --area-warm-start-dir")
    if args.area_refiner_only and not args.area_warm_start_dir:
        raise ValueError("--area-refiner-only requires --area-warm-start-dir")
    if args.area_refiner_only and args.area_freeze_backbone:
        raise ValueError(
            "--area-refiner-only and --area-freeze-backbone are mutually exclusive"
        )
    if (
        args.area_refiner_only
        and args.area_architecture != "deeplab_resnet50_boundary_refine"
    ):
        raise ValueError(
            "--area-refiner-only requires deeplab_resnet50_boundary_refine"
        )
    if args.reuse_leaf_model_dir and not args.area_warm_start_dir:
        raise ValueError("--reuse-leaf-model-dir requires --area-warm-start-dir")

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
    _qualified_reuse_report = None
    qualified_reuse_provenance = None
    if args.reuse_qualified_nondiscovery_model_dir:
        _qualified_reuse_report, qualified_reuse_provenance = (
            _load_qualified_task_reuse_report(
                args.reuse_qualified_nondiscovery_model_dir,
                dataset_qa["sha256"],
                ("classifier", "leaf", "puddle"),
            )
        )
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
    small_object_frame_keys = {
        _row_identity(row)
        for row in eligible_train_rows
        if any(
            float(box.get("native_short_side_px", 0.0)) < 18.0
            for box in discrete_boxes_for_frame(row, instances_by_key)
        )
    }
    train_rows = _prioritized_discovery_row_sample(
        eligible_train_rows,
        small_object_frame_keys,
        args.max_train_frames,
        seed=SEED + 8,
    )
    selected_small_object_frames = sum(
        _row_identity(row) in small_object_frame_keys for row in train_rows
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
    teacher_detections_by_key = None
    distillation_summary = None
    if (
        not reuse_discrete_source
        and args.discovery_architecture
        == "teacher_distilled_mobilenetv3_fpn_a3"
    ):
        teacher_detections_by_key, distillation_summary = (
            _prepare_a3_distillation_targets(
                teacher_report=teacher_report,
                teacher_report_path=args.teacher_report,
                train_rows=train_rows,
                instances_by_key=instances_by_key,
                device=device,
                output=output,
            )
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
            "discovery",
            reuse_discrete_source,
            output,
            device,
            discovery_architecture=args.discovery_architecture,
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
            model=build_g4_model(
                "discovery",
                discovery_architecture=args.discovery_architecture,
            ),
            assign_pyramid_by_scale=(
                args.discovery_architecture
                != "resnet18_fpn_a1"
            ),
            teacher_detections_by_key=teacher_detections_by_key,
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
    classifier_reuse_source = (
        reuse_discrete_source or args.reuse_qualified_nondiscovery_model_dir
    )
    if classifier_reuse_source:
        classifier, classifier_training = _load_reused_model(
            "classifier",
            classifier_reuse_source,
            output,
            device,
            expected_selection=(
                qualified_reuse_provenance["tasks"]["classifier"]["selection"]
                if qualified_reuse_provenance
                else None
            ),
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

    # Area recovery uses the full TRAIN-only pool, independently of the
    # discovery frame budget.  This prevents rare wet/reflection/shadow/paint
    # negatives from being discarded by discrete-object sampling.
    leaf_rows = _select_area_rows(
        eligible_train_rows,
        instances_by_key,
        "leaf",
        args.area_positive_frames,
        args.area_negative_frames,
        SEED,
    )
    puddle_rows = _select_area_rows(
        eligible_train_rows,
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
    area_reuse_source = (
        reuse_discrete_source
        if args.reuse_model_dir or args.recover_unreported_model_dir
        else args.reuse_qualified_nondiscovery_model_dir
    )
    if area_reuse_source:
        leaf, leaf_training = _load_reused_model(
            "leaf",
            area_reuse_source,
            output,
            device,
            area_architecture=args.area_architecture,
            expected_selection=(
                qualified_reuse_provenance["tasks"]["leaf"]["selection"]
                if qualified_reuse_provenance
                else None
            ),
        )
        puddle, puddle_training = _load_reused_model(
            "puddle",
            area_reuse_source,
            output,
            device,
            area_architecture=args.area_architecture,
            expected_selection=(
                qualified_reuse_provenance["tasks"]["puddle"]["selection"]
                if qualified_reuse_provenance
                else None
            ),
        )
    else:
        area_warm_start = {}
        puddle_model = build_g4_model(
            "puddle", area_architecture=args.area_architecture
        )
        if args.reuse_leaf_model_dir:
            leaf, leaf_training = _load_reused_model(
                "leaf",
                args.reuse_leaf_model_dir,
                output,
                device,
                area_architecture=args.area_architecture,
            )
        else:
            leaf_model = build_g4_model(
                "leaf", area_architecture=args.area_architecture
            )
        if args.area_warm_start_dir and not args.reuse_leaf_model_dir:
            leaf_model, area_warm_start["leaf"] = _warm_start_area_model(
                "leaf",
                args.area_warm_start_dir,
                area_architecture=args.area_architecture,
                freeze_backbone=args.area_freeze_backbone,
                refiner_only=args.area_refiner_only,
            )
        if args.area_warm_start_dir:
            puddle_model, area_warm_start["puddle"] = _warm_start_area_model(
                "puddle",
                args.area_warm_start_dir,
                area_architecture=args.area_architecture,
                freeze_backbone=args.area_freeze_backbone,
                refiner_only=args.area_refiner_only,
            )
        if not args.reuse_leaf_model_dir:
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
                cache_frames=not args.disable_area_frame_cache,
                selector=area_selector(),
                validation_metric_fn=_area_validation_metric_fn(
                    leaf_validation, device, "leaf"
                ),
                model=leaf_model,
                low_light_appearance_augmentation=(
                    args.area_low_light_augmentation
                ),
                boundary_weight=args.area_boundary_loss_weight,
                negative_weight=args.area_negative_loss_weight,
                boundary_pixel_weight=args.area_boundary_pixel_weight,
                semantic_boundary_weight=args.area_semantic_boundary_weight,
            )
        if device.type == "cuda":
            leaf.to("cpu")
            torch.cuda.empty_cache()
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
            cache_frames=not args.disable_area_frame_cache,
            selector=area_selector(),
            validation_metric_fn=_area_validation_metric_fn(
                puddle_validation, device, "puddle"
            ),
            model=puddle_model,
            low_light_appearance_augmentation=(
                args.area_low_light_augmentation
            ),
            boundary_weight=args.area_boundary_loss_weight,
            negative_weight=args.area_negative_loss_weight,
            boundary_pixel_weight=args.area_boundary_pixel_weight,
            semantic_boundary_weight=args.area_semantic_boundary_weight,
        )
        if device.type == "cuda":
            leaf.to(device)

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
            max_detections=PRODUCT_MAXIMUM_CANDIDATES,
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
            (
                CLASSIFIER_EXPORT_BATCH,
                3,
                CLASSIFIER_MODEL_SIZE[0],
                CLASSIFIER_MODEL_SIZE[1],
            ),
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
            "attempt": {
                "resnet18_fpn_a1": "A1_FCOS_lite_ResNet18_FPN",
                "mobilenetv3_small_fpn_a2": (
                    "A2_FCOS_lite_MobileNetV3_Small_FPN"
                ),
                "teacher_distilled_mobilenetv3_fpn_a3": (
                    "A3_Teacher_Distilled_MobileNetV3_Small_FPN"
                ),
            }[args.discovery_architecture],
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
            "attempts_used": {
                "resnet18_fpn_a1": 1,
                "mobilenetv3_small_fpn_a2": 2,
                "teacher_distilled_mobilenetv3_fpn_a3": 3,
            }[args.discovery_architecture],
            "discovery_architecture": args.discovery_architecture,
            "distillation": distillation_summary,
            "area_architecture": args.area_architecture,
            "area_recovery": {
                "taxonomy_balanced_negative_sampling": True,
                "full_train_only_pool": True,
                "warm_start": area_warm_start if not area_reuse_source else {},
                "backbone_frozen": bool(args.area_freeze_backbone),
                "refiner_only": bool(args.area_refiner_only),
                "frame_cache_enabled": not args.disable_area_frame_cache,
                "reused_leaf_checkpoint": (
                    leaf_training if args.reuse_leaf_model_dir else None
                ),
                "low_light_appearance_augmentation": bool(
                    args.area_low_light_augmentation
                ),
                "loss_weights": {
                    "boundary": args.area_boundary_loss_weight,
                    "negative": args.area_negative_loss_weight,
                    "boundary_pixel": args.area_boundary_pixel_weight,
                    "semantic_boundary": args.area_semantic_boundary_weight,
                },
            },
            "reused_selected_checkpoints_for_diagnostic_evaluation": bool(
                args.reuse_model_dir
            ),
            "reused_discrete_checkpoints_for_area_comparison": bool(
                args.reuse_discrete_model_dir
            ),
            "recovered_after_post_training_evaluation_failure": bool(
                args.recover_unreported_model_dir
            ),
            "reused_qualified_nondiscovery_checkpoints": bool(
                args.reuse_qualified_nondiscovery_model_dir
            ),
            "qualified_nondiscovery_reuse": qualified_reuse_provenance,
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
            "discovery_training_sample": {
                "eligible_frames": len(eligible_train_rows),
                "selected_frames": len(train_rows),
                "available_small_object_frames": len(
                    small_object_frame_keys
                ),
                "selected_small_object_frames": (
                    selected_small_object_frames
                ),
                "small_object_native_short_side_px_lt": 18.0,
                "sampling_policy": (
                    "retain_small_object_frames_then_stratified_fill"
                ),
                "pyramid_scale_assignment": (
                    args.discovery_architecture != "resnet18_fpn_a1"
                ),
            },
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
