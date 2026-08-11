#!/usr/bin/env python3
"""Mine classifier proposal crops from OPR-A outputs on G6 TRAIN only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import sys

import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import CLASSIFIER_CLASSES, DISCRETE_NAMES, square_crop  # noqa: E402
from sanitation_learning.g6_small_specialist import build_small_specialist, load_g6_rows  # noqa: E402
from screen_opr_a_classifier import iou, specialist_native_candidates  # noqa: E402


def proposal_samples(rows, instances, candidates):
    samples = []
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        truth = [
            record
            for record in instances.get(key, [])
            if record["class_id"] in DISCRETE_NAMES
            and int(record["bbox_short_side_px"]) < 18
        ]
        for candidate in candidates.get(key, []):
            ranked = sorted(
                ((iou(candidate["bbox_xyxy"], item["bbox_xyxy"]), item) for item in truth),
                reverse=True,
                key=lambda value: value[0],
            )
            matched = ranked[0][1] if ranked and ranked[0][0] >= 0.5 else None
            class_name = matched["class_id"] if matched else "background"
            samples.append(
                {
                    "rgb_path": Path(row["rgb_path"]).as_posix(),
                    "crop": square_crop(
                        640,
                        480,
                        tuple(candidate["bbox_xyxy"]),
                        scale=6.0,
                        minimum_side=64,
                    ),
                    "label": CLASSIFIER_CLASSES.index(class_name),
                    "class_name": class_name,
                    "split": row["split"],
                    "world_id": row["world_id"],
                    "scene_seed": key[0],
                    "frame_index": key[1],
                    "hard_negative": matched is None,
                    "proposal_objectness": candidate["objectness"],
                    "proposal_bbox_xyxy": candidate["bbox_xyxy"],
                    "matched_iou": ranked[0][0] if ranked else 0.0,
                }
            )
    return samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--specialist-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-fit-frames", type=int, default=1200)
    parser.add_argument("--max-holdout-frames", type=int, default=500)
    parser.add_argument("--batch-size", type=int, default=4)
    args = parser.parse_args()
    rows, instances = load_g6_rows(args.g6_root, ("train",))
    holdout_world = sorted({row["world_id"] for row in rows})[-1]
    fit_rows = [row for row in rows if row["world_id"] != holdout_world]
    holdout_rows = [row for row in rows if row["world_id"] == holdout_world]
    rng = random.Random(20260811)
    rng.shuffle(fit_rows)
    rng.shuffle(holdout_rows)
    fit_rows = fit_rows[: args.max_fit_frames]
    holdout_rows = holdout_rows[: args.max_holdout_frames]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPR-A proposal mining requires CUDA")
    model = build_small_specialist(args.base_checkpoint)
    checkpoint = torch.load(
        args.specialist_checkpoint, map_location="cpu", weights_only=False
    )
    model.load_state_dict(checkpoint["state_dict"], strict=True)
    model.to(device)
    threshold = float(checkpoint["objectness_threshold"])
    fit_candidates = specialist_native_candidates(
        model, fit_rows, threshold, device, args.batch_size
    )
    holdout_candidates = specialist_native_candidates(
        model, holdout_rows, threshold, device, args.batch_size
    )
    fit_samples = proposal_samples(fit_rows, instances, fit_candidates)
    holdout_samples = proposal_samples(holdout_rows, instances, holdout_candidates)
    payload = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-A-TRAIN-PROPOSAL-MINING",
        "data_policy": {
            "dataset": "G6_TRAIN",
            "fit_frame_count": len(fit_rows),
            "holdout_frame_count": len(holdout_rows),
            "holdout_world": holdout_world,
            "VAL_read": False,
            "G5_SEALED_FINAL_read": False,
        },
        "objectness_threshold": threshold,
        "fit_counts": {
            name: sum(sample["class_name"] == name for sample in fit_samples)
            for name in CLASSIFIER_CLASSES
        },
        "holdout_counts": {
            name: sum(sample["class_name"] == name for sample in holdout_samples)
            for name in CLASSIFIER_CLASSES
        },
        "fit_samples": fit_samples,
        "holdout_samples": holdout_samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({"fit_counts": payload["fit_counts"], "holdout_counts": payload["holdout_counts"]}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
