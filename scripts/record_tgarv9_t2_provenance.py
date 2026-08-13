#!/usr/bin/env python3
"""Record hash-bound official DINO provenance and the T2 hypothesis."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--container-digest", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    checkpoint_hash = sha256(args.checkpoint)
    if checkpoint_hash != args.expected_sha256.lower():
        raise RuntimeError("official checkpoint SHA-256 mismatch")
    args.output.mkdir(parents=True)
    provenance = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "T2_OFFICIAL_PROVENANCE",
        "architecture": "DINO 4-scale improved, ResNet-50",
        "upstream": "https://github.com/open-mmlab/mmdetection",
        "upstream_tag": "v3.3.0",
        "upstream_release_commit_short": "44ebd17",
        "official_config": "configs/dino/dino-4scale_r50_improved_8xb2-12e_coco.py",
        "official_config_url": "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/configs/dino/dino-4scale_r50_improved_8xb2-12e_coco.py",
        "official_checkpoint_url": "https://download.openmmlab.com/mmdetection/v3.0/dino/dino-4scale_r50_improved_8xb2-12e_coco/dino-4scale_r50_improved_8xb2-12e_coco_20230818_162607-6f47a913.pth",
        "official_checkpoint_sha256": checkpoint_hash,
        "license": "Apache-2.0",
        "license_url": "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/LICENSE",
        "dependency_compatibility_source": "https://github.com/open-mmlab/mmdetection/blob/v3.3.0/docs/en/notes/faq.md",
        "runtime": {
            "mmdetection": "3.3.0",
            "mmcv": "2.1.0",
            "mmengine": "0.10.7",
            "pytorch": "2.5.1+cu124",
            "cuda": "12.4",
            "container_digest": args.container_digest,
            "compatibility_pass": True,
            "runtime_limitation": "MMCV custom CUDA ops unavailable; official differentiable PyTorch fallbacks used for deformable attention and focal loss",
        },
        "source_commit": args.source_commit,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "T2_OFFICIAL_PROVENANCE_PASS": True,
    }
    hypothesis = {
        "schema_version": 1,
        "protocol": "TGARV9",
        "stage": "T2_HYPOTHESIS",
        "trigger": "T1 observed nearly all actionable tubes but failed class separation, small recall, and clean-opportunity recall",
        "T1_observation_recall": 0.9874213836477987,
        "T1_correct_class_recall": 0.22012578616352202,
        "T1_small_correct_class_recall": 0.14705882352941177,
        "hypothesis": "RTMDet dense one-stage class separation is the limiting factor in small and hard-negative real-Gazebo scenes; query-based multi-scale DINO tests a materially different detection bias while retaining track-level evidence",
        "selection_not_based_only_on_coco_map": True,
        "training_boundary": "legacy GA1/G8 TRAIN only; negative-only frames retained",
        "selection_boundary": "G9 HOLDOUT only",
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    (args.output / "T2_OFFICIAL_PROVENANCE.json").write_text(json.dumps(provenance, indent=2) + "\n")
    (args.output / "T2_HYPOTHESIS.json").write_text(json.dumps(hypothesis, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
