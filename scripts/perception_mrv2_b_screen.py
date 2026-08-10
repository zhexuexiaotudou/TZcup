#!/usr/bin/env python3
"""Screen the two bounded MRV2-B ground-tile refinements on fixed partitions."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import (  # noqa: E402
    DISCRETE_NAMES, index_instance_records, load_frame_rows, load_instance_records,
)
from sanitation_learning.g4_direct_fcos import build_direct_fcos  # noqa: E402
from sanitation_learning.g4_evaluation import (  # noqa: E402
    discrete_metrics, discovery_metrics, match_discrete_predictions,
)
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402
from sanitation_learning.g4_tiled_fcos import tiled_direct_predictions  # noqa: E402
from perception_mrv2_a_train import SEED, holdout_rows, sha256  # noqa: E402
from perception_prod_x1_full_pipeline import candidate_size_metrics, load_partition  # noqa: E402


THRESHOLDS = tuple(round(value / 100.0, 2) for value in range(5, 96, 5))


def filter_frames(raw, threshold):
    output = []
    for frame in raw:
        items = [item for item in frame["predictions"] if item["score"] >= threshold][:16]
        output.append({**frame, "predictions": items, "detections": items})
    return output


def metrics_for(raw, threshold):
    frames = filter_frames(raw, threshold)
    discrete = discrete_metrics(match_discrete_predictions(frames))
    candidate = discovery_metrics(frames)
    candidate.update(candidate_size_metrics(frames))
    return {"discrete": discrete, "candidate": candidate}, frames


def select_threshold(raw):
    sweep = []
    for threshold in THRESHOLDS:
        metrics, _ = metrics_for(raw, threshold)
        discrete, candidate = metrics["discrete"], metrics["candidate"]
        metal = discrete["per_class"]["metal_can"]["recall"]
        gates = {
            "candidate_recall_at_least_0_80": candidate["all_gt_candidate_recall"] >= 0.80,
            "small_recall_at_least_0_70": discrete["small_object_recall"] >= 0.70,
            "macro_precision_at_least_0_90": discrete["macro_precision"] >= 0.90,
            "macro_recall_at_least_0_90": discrete["macro_recall"] >= 0.90,
            "metal_recall_at_least_0_90": metal >= 0.90,
            "false_candidates_per_min_at_most_2": candidate["false_candidates_per_min"] <= 2.0,
            "negative_fp_per_frame_at_most_0_05": candidate["negative_only_fp_per_frame"] <= 0.05,
        }
        deficits = (
            max(0.0, 0.80 - candidate["all_gt_candidate_recall"])
            + max(0.0, 0.70 - discrete["small_object_recall"])
            + max(0.0, 0.90 - discrete["macro_precision"])
            + max(0.0, 0.90 - discrete["macro_recall"])
            + max(0.0, 0.90 - metal)
            + max(0.0, candidate["false_candidates_per_min"] - 2.0)
            + max(0.0, candidate["negative_only_fp_per_frame"] - 0.05)
        )
        sweep.append({"threshold": threshold, "metrics": metrics, "gates": gates, "all_pass": all(gates.values()), "constraint_distance": deficits})
    selected = min(sweep, key=lambda item: (not item["all_pass"], item["constraint_distance"], -item["metrics"]["discrete"]["macro_f1"], -item["threshold"]))
    return selected, sweep


def load_main(args):
    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root, allowed_splits=("train", "val"))
    train = [row for row in rows if row["split"] == "train"]
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_rows(train)],
        100, seed=SEED + 1,
    )
    val = [row for row in rows if row["split"] == "val"]
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    instances = index_instance_records(load_instance_records(args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=keys))
    return holdout, val, instances


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--factorized-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal MRV2-B screening requires CUDA")
    payload = torch.load(args.checkpoint, map_location=device, weights_only=False)
    full_input = tuple(payload["input_size"])
    full_model = build_direct_fcos(input_size=full_input).to(device)
    tile_model = build_direct_fcos(input_size=(640, 480)).to(device)
    full_model.load_state_dict(payload["state_dict"], strict=True)
    tile_model.load_state_dict(payload["state_dict"], strict=True)
    full_model.eval(); tile_model.eval()
    holdout, val, main_instances = load_main(args)
    configs = {}
    for mode in ("ground3", "ground2x2"):
        print(f"[MRV2-B] holdout {mode}", flush=True)
        raw = tiled_direct_predictions(full_model, tile_model, holdout, main_instances, device=device, full_input_size=full_input, mode=mode)
        selected, sweep = select_threshold(raw)
        configs[mode] = {"selection": selected, "threshold_sweep": sweep}
    selected_mode = min(
        configs,
        key=lambda mode: (
            not configs[mode]["selection"]["all_pass"],
            configs[mode]["selection"]["constraint_distance"],
            -configs[mode]["selection"]["metrics"]["discrete"]["small_object_recall"],
            0 if mode == "ground3" else 1,
        ),
    )
    selected_threshold = float(configs[selected_mode]["selection"]["threshold"])
    splits = {}; cross_frames = []
    print(f"[MRV2-B] VAL {selected_mode} threshold={selected_threshold}", flush=True)
    raw = tiled_direct_predictions(full_model, tile_model, val, main_instances, device=device, full_input_size=full_input, mode=selected_mode)
    splits["VAL"], _ = metrics_for(raw, selected_threshold)
    for index in range(1, 6):
        name = f"D{index}"; root = args.factorized_root / name
        rows, records = load_partition(root / "g4_screening_native", root / "evidence/raw_g4_qa")
        instances = index_instance_records(records)
        print(f"[MRV2-B] {name} {selected_mode}", flush=True)
        raw = tiled_direct_predictions(full_model, tile_model, rows, instances, device=device, full_input_size=full_input, mode=selected_mode)
        splits[name], filtered = metrics_for(raw, selected_threshold)
        cross_frames.extend(filtered)
    cross_discrete = discrete_metrics(match_discrete_predictions(cross_frames))
    cross_candidate = discovery_metrics(cross_frames); cross_candidate.update(candidate_size_metrics(cross_frames))
    val_d, val_c = splits["VAL"]["discrete"], splits["VAL"]["candidate"]
    gates = {
        "VAL_candidate_recall_at_least_0_80": val_c["all_gt_candidate_recall"] >= 0.80,
        "VAL_small_recall_at_least_0_70": val_d["small_object_recall"] >= 0.70,
        "VAL_false_candidates_per_min_at_most_2": val_c["false_candidates_per_min"] <= 2.0,
        "VAL_macro_precision_at_least_0_90": val_d["macro_precision"] >= 0.90,
        "VAL_macro_recall_at_least_0_90": val_d["macro_recall"] >= 0.90,
        "VAL_macro_f1_at_least_0_90": val_d["macro_f1"] >= 0.90,
        "VAL_metal_can_recall_at_least_0_90": val_d["per_class"]["metal_can"]["recall"] >= 0.90,
        "cross_macro_f1_at_least_0_70": cross_discrete["macro_f1"] >= 0.70,
        "cross_each_class_recall_at_least_0_70": all(cross_discrete["per_class"][name]["recall"] >= 0.70 for name in DISCRETE_NAMES),
        "cross_small_recall_at_least_0_70": cross_discrete["small_object_recall"] >= 0.70,
        "cross_negative_fp_per_frame_at_most_0_05": cross_candidate["negative_only_fp_per_frame"] <= 0.05,
    }
    report = {
        "schema_version": 1, "stage": "MRV2-B-TILED-SCREEN", "route": "MRV2-B",
        "parent_checkpoint": {"path": args.checkpoint.as_posix(), "sha256": sha256(args.checkpoint)},
        "full_input_size": full_input, "tile_input_size": [640, 480],
        "configs": configs, "selected_mode": selected_mode, "selected_threshold": selected_threshold,
        "splits": splits,
        "cross_world_aggregate": {"discrete": cross_discrete, "candidate": cross_candidate},
        "gates": gates, "MRV2_B_DETECTOR_PASS": all(gates.values()),
        "next_action": "repair_area_then_full_static" if all(gates.values()) else "execute_MRV2_C_teacher_assisted_recovery",
        "G5_SEALED_FINAL_read": False, "legacy_G4_D6_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"selected_mode": selected_mode, "gates": gates}, indent=2), flush=True)
    return 0 if all(gates.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
