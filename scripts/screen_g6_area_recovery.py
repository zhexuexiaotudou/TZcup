#!/usr/bin/env python3
"""Lock G6 Area postprocessing on TRAIN holdout and audit VAL plus D1-D5."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g6_area_recovery import (  # noqa: E402
    AREA_CLASSES,
    G6AreaDataset,
    G6BoundaryAwareAreaNet,
    mask_boundary,
    physical_component_filter,
)


SPLIT_MAP = {
    "VAL": "val",
    "D1": "development_d1",
    "D2": "development_d2",
    "D3": "development_d3",
    "D4": "development_d4",
    "D5": "development_d5",
}
FX = 554.256 * 512.0 / 640.0
FY = 554.256 * 384.0 / 480.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_manifest(root: Path) -> list[dict]:
    path = root / "G6_FRAME_MANIFEST.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def morphology(mask: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(mask, dtype=np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    if name == "none":
        return value.astype(bool)
    if name == "close3":
        return cv2.morphologyEx(value, cv2.MORPH_CLOSE, kernel).astype(bool)
    if name == "open_close3":
        opened = cv2.morphologyEx(value, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel).astype(bool)
    raise ValueError(f"unknown morphology {name}")


def postprocess(probability: np.ndarray, depth_m: np.ndarray, config: dict) -> np.ndarray:
    mask = morphology(probability >= float(config["threshold"]), config["morphology"])
    return physical_component_filter(
        mask,
        depth_m,
        fx=FX,
        fy=FY,
        minimum_area_m2=float(config["minimum_area_m2"]),
        minimum_valid_depth_ratio=0.8,
    )


@torch.no_grad()
def inference_records(model, root: Path, rows: list[dict], device, batch_size: int, workers: int):
    loader = DataLoader(
        G6AreaDataset(root, rows, augment=False),
        batch_size=batch_size,
        shuffle=False,
        num_workers=workers,
        pin_memory=True,
    )
    model.eval()
    offset = 0
    for batch in loader:
        output = model(batch["features"].to(device, non_blocking=True))
        probabilities = torch.sigmoid(output["semantic_logits"]).cpu().numpy()
        boundary_probabilities = torch.sigmoid(output["boundary_logits"]).cpu().numpy()
        count = probabilities.shape[0]
        for index in range(count):
            row = rows[offset + index]
            yield {
                "probabilities": probabilities[index].astype(np.float16),
                "boundary_probabilities": boundary_probabilities[index].astype(np.float16),
                "targets": batch["targets"][index].numpy().astype(bool),
                "negative": batch["negative"][index, 0].numpy().astype(bool),
                "depth_m": batch["depth_m"][index, 0].numpy().astype(np.float16),
                "taxonomy": (row.get("negative_area_taxonomies") or ["none"])[0],
            }
        offset += count


def task_metrics(records: list[dict], task_index: int, config: dict) -> dict:
    intersection = 0
    union = 0
    edge_intersection = 0
    edge_union = 0
    negative_fp_frames = 0
    for record in records:
        predicted = postprocess(
            record["probabilities"][task_index].astype(np.float32),
            record["depth_m"].astype(np.float32),
            config,
        )
        truth = record["targets"][task_index]
        intersection += int((predicted & truth).sum())
        union += int((predicted | truth).sum())
        predicted_edge = mask_boundary(predicted).astype(bool)
        truth_edge = mask_boundary(truth).astype(bool)
        edge_intersection += int((predicted_edge & truth_edge).sum())
        edge_union += int((predicted_edge | truth_edge).sum())
        negative_fp_frames += int((predicted & record["negative"]).sum() >= 32)
    iou = intersection / max(union, 1)
    boundary_f1 = 2 * edge_intersection / max(edge_intersection + edge_union, 1)
    negative_rate = negative_fp_frames / max(len(records), 1)
    return {
        "iou": iou,
        "boundary_f1": boundary_f1,
        "negative_area_fp_per_frame": negative_rate,
        "pixel_totals": {
            "intersection": intersection,
            "union": union,
            "boundary_intersection": edge_intersection,
            "boundary_union": edge_union,
            "negative_frames": len(records),
            "negative_fp_frames": negative_fp_frames,
        },
    }


def select_task_config(records: list[dict], task_index: int) -> tuple[dict, dict, list[dict]]:
    candidates = []
    for threshold in (0.30, 0.40, 0.50, 0.60, 0.70):
        for morph in ("none", "close3", "open_close3"):
            for minimum_area in (0.0005, 0.0010):
                config = {
                    "threshold": threshold,
                    "morphology": morph,
                    "minimum_area_m2": minimum_area,
                }
                metrics = task_metrics(records, task_index, config)
                passed = (
                    metrics["iou"] >= 0.80
                    and metrics["boundary_f1"] >= 0.75
                    and metrics["negative_area_fp_per_frame"] <= 0.05
                )
                normalized = min(
                    metrics["iou"] / 0.80,
                    metrics["boundary_f1"] / 0.75,
                    1.0
                    if metrics["negative_area_fp_per_frame"] <= 0.05
                    else 0.05 / metrics["negative_area_fp_per_frame"],
                )
                candidates.append(
                    {
                        "config": config,
                        "metrics": metrics,
                        "all_constraints_pass": passed,
                        "selection_score": normalized,
                    }
                )
    candidates.sort(
        key=lambda item: (
            item["all_constraints_pass"],
            item["selection_score"],
            item["metrics"]["boundary_f1"],
            item["metrics"]["iou"],
            -item["metrics"]["negative_area_fp_per_frame"],
        ),
        reverse=True,
    )
    selected = candidates[0]
    return selected["config"], selected["metrics"], candidates


def aggregate_selected(records: list[dict], configs: list[dict]) -> dict:
    totals = {
        "intersection": [0, 0],
        "union": [0, 0],
        "boundary_intersection": [0, 0],
        "boundary_union": [0, 0],
        "raw_boundary_intersection": [0, 0],
        "raw_boundary_union": [0, 0],
        "negative_frames": len(records),
        "negative_fp_frames": 0,
    }
    taxonomy = {}
    for record in records:
        predicted_masks = []
        for task_index in range(2):
            predicted = postprocess(
                record["probabilities"][task_index].astype(np.float32),
                record["depth_m"].astype(np.float32),
                configs[task_index],
            )
            predicted_masks.append(predicted)
            truth = record["targets"][task_index]
            totals["intersection"][task_index] += int((predicted & truth).sum())
            totals["union"][task_index] += int((predicted | truth).sum())
            predicted_edge = mask_boundary(predicted).astype(bool)
            truth_edge = mask_boundary(truth).astype(bool)
            totals["boundary_intersection"][task_index] += int(
                (predicted_edge & truth_edge).sum()
            )
            totals["boundary_union"][task_index] += int(
                (predicted_edge | truth_edge).sum()
            )
            raw_edge = record["boundary_probabilities"][task_index] >= 0.5
            totals["raw_boundary_intersection"][task_index] += int(
                (raw_edge & truth_edge).sum()
            )
            totals["raw_boundary_union"][task_index] += int(
                (raw_edge | truth_edge).sum()
            )
        false_actionable = any(
            (predicted & record["negative"]).sum() >= 32
            for predicted in predicted_masks
        )
        totals["negative_fp_frames"] += int(false_actionable)
        bucket = taxonomy.setdefault(record["taxonomy"], {"frames": 0, "fp_frames": 0})
        bucket["frames"] += 1
        bucket["fp_frames"] += int(false_actionable)
    iou = [
        totals["intersection"][index] / max(totals["union"][index], 1)
        for index in range(2)
    ]
    boundary = [
        2 * totals["boundary_intersection"][index]
        / max(
            totals["boundary_intersection"][index]
            + totals["boundary_union"][index],
            1,
        )
        for index in range(2)
    ]
    raw_boundary = [
        2 * totals["raw_boundary_intersection"][index]
        / max(
            totals["raw_boundary_intersection"][index]
            + totals["raw_boundary_union"][index],
            1,
        )
        for index in range(2)
    ]
    for bucket in taxonomy.values():
        bucket["fp_per_frame"] = bucket["fp_frames"] / max(bucket["frames"], 1)
    return {
        "iou_by_class": dict(zip(AREA_CLASSES, iou)),
        "macro_miou": float(np.mean(iou)),
        "postprocessed_mask_boundary_f1_by_class": dict(zip(AREA_CLASSES, boundary)),
        "postprocessed_mask_boundary_f1": float(np.mean(boundary)),
        "raw_network_boundary_head_f1_by_class": dict(zip(AREA_CLASSES, raw_boundary)),
        "raw_network_boundary_head_f1": float(np.mean(raw_boundary)),
        "negative_only_frames": len(records),
        "negative_only_fp_frames": totals["negative_fp_frames"],
        "negative_area_fp_per_frame": totals["negative_fp_frames"] / max(len(records), 1),
        "negative_area_definition": "annotated G6 hard-negative region per frame; actionable overlap >=32 pixels",
        "negative_area_by_taxonomy": taxonomy,
        "pixel_totals": totals,
    }


def load_gate_module():
    path = ROOT / "scripts" / "perception_oprv3_area_gate.py"
    spec = importlib.util.spec_from_file_location("oprv3_area_gate", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--training", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    started = time.perf_counter()
    if not torch.cuda.is_available():
        raise RuntimeError("OPRV3-06 formal Area screening requires CUDA")
    args.output.mkdir(parents=True, exist_ok=False)
    device = torch.device("cuda")
    training_report_path = args.training / "OPRV3_G6_AREA_TRAINING.json"
    training_report = json.loads(training_report_path.read_text(encoding="utf-8"))
    if training_report.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError("training provenance violates sealed G5 boundary")
    checkpoint_path = args.training / training_report["model"]["checkpoint"]
    if sha256(checkpoint_path) != training_report["model"]["checkpoint_sha256"]:
        raise RuntimeError("G6 Area checkpoint hash mismatch")
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = G6BoundaryAwareAreaNet(checkpoint["base_channels"]).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    rows = read_manifest(args.dataset)
    holdout_rows = [
        row
        for row in rows
        if row["split"] == "train" and row["world_id"] == "g6_train_world_10"
    ]
    holdout_records = list(
        inference_records(
            model, args.dataset, holdout_rows, device, args.batch_size, args.workers
        )
    )
    selected_configs = []
    selection_metrics = {}
    selection_search = {}
    for task_index, task in enumerate(("leaf", "puddle")):
        config, metrics, candidates = select_task_config(holdout_records, task_index)
        selected_configs.append(config)
        selection_metrics[task] = metrics
        selection_search[task] = candidates
        print(json.dumps({"task": task, "config": config, "metrics": metrics}), flush=True)

    split_reports = {}
    for public_name, manifest_name in SPLIT_MAP.items():
        split_rows = [row for row in rows if row["split"] == manifest_name]
        records = list(
            inference_records(
                model, args.dataset, split_rows, device, args.batch_size, args.workers
            )
        )
        metrics = aggregate_selected(records, selected_configs)
        split_reports[public_name] = {"development_selected_postprocess": metrics}
        print(json.dumps({"split": public_name, "metrics": metrics}), flush=True)

    audit = {
        "schema_version": 1,
        "stage": "OPRV3-06-G6-AREA-AUDIT",
        "G5_SEALED_FINAL_read": False,
        "G5_V2_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "selection_split": "g6_train_world_10",
        "development_selected_config": {
            "leaf": selected_configs[0],
            "puddle": selected_configs[1],
            "negative_actionable_overlap_pixels": 32,
            "minimum_valid_depth_ratio": 0.8,
            "temporal_persistence": {
                "implemented": True,
                "window": 3,
                "minimum_hits": 2,
                "applied_to_static_gate": False,
                "reason": "G6 scene frames are independently randomized and not registered temporal observations",
            },
        },
        "models": {
            task: {
                "checkpoint_status": "training_complete",
                "path": training_report["onnx"][task]["path"],
                "sha256": training_report["onnx"][task]["sha256"],
                "shared_training_checkpoint_sha256": training_report["model"][
                    "checkpoint_sha256"
                ],
                "onnx_parity_pass": training_report["onnx"][task]["parity_pass"],
            }
            for task in ("leaf", "puddle")
        },
        "selection_metrics": selection_metrics,
        "selection_search": selection_search,
        "splits": split_reports,
        "data_policy": {
            "dataset": args.dataset.as_posix(),
            "development_only": True,
            "world_isolated_selection": True,
            "untouched_evaluation_splits": list(SPLIT_MAP),
            "negative_taxonomy_count": 10,
        },
        "duration_s": time.perf_counter() - started,
    }
    audit_path = args.output / "AREA_BOUNDARY_AUDIT.json"
    audit_path.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    gate_module = load_gate_module()
    gate = gate_module.aggregate(audit)
    gate["input"] = {"path": audit_path.as_posix(), "sha256": sha256(audit_path)}
    gate["training"] = {
        "path": training_report_path.as_posix(),
        "sha256": sha256(training_report_path),
    }
    gate_path = args.output / "OPRV3_AREA_GATE.json"
    gate_path.write_text(json.dumps(gate, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"gate": gate, "gate_sha256": sha256(gate_path)}, indent=2))
    return 0 if gate["OPRV3_06_AREA_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
