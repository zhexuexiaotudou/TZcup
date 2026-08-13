#!/usr/bin/env python3
"""Truthfully recover a completed bounded run after a post-save val failure."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import torch


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--train-coco", type=Path, required=True)
    parser.add_argument("--official-sha256", required=True)
    parser.add_argument("--container-digest", required=True)
    parser.add_argument("--expected-checkpoints", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    checkpoints = sorted(args.run.glob("epoch_*.pth"), key=lambda path: int(path.stem.split("_")[-1]))
    if len(checkpoints) != args.expected_checkpoints:
        raise RuntimeError("cannot recover an incomplete bounded run")
    checkpoint_rows = []
    for checkpoint in checkpoints:
        payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
        if not isinstance(payload.get("state_dict"), dict) or not payload["state_dict"]:
            raise RuntimeError(f"invalid checkpoint state_dict: {checkpoint}")
        checkpoint_rows.append({
            "name": checkpoint.name,
            "sha256": sha256(checkpoint),
            "state_tensor_count": len(payload["state_dict"]),
            "meta_epoch": payload.get("meta", {}).get("epoch"),
        })
    train = json.loads(args.train_coco.read_text())
    negative_count = sum(bool(row.get("negative_only")) for row in train["images"])
    if negative_count == 0:
        raise RuntimeError("negative-only frames were not retained")
    config = args.run / "tgarv9_t2_dino_r50_4scale.py"
    report = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "T2_DINO_TRAIN_RECOVERED",
        "architecture": "official_mmdetection_dino_r50_4scale_improved",
        "official_checkpoint_sha256": args.official_sha256.lower(),
        "config_sha256": sha256(config),
        "container_digest": args.container_digest,
        "train_image_count": len(train["images"]),
        "negative_frame_count": negative_count,
        "negative_frames_retained": True,
        "epochs": args.expected_checkpoints,
        "epoch_checkpoint_count": len(checkpoints),
        "checkpoints": checkpoint_rows,
        "training_complete": True,
        "original_container_exit_code": 1,
        "post_training_failure": "MMEngine attempted an unneeded final G9 val after epoch_6.pth was saved; container path translation failed",
        "recovery_scope": "checkpoint integrity and training completion only; no HOLDOUT result inferred",
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
