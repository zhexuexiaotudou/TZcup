#!/usr/bin/env python3
"""Materialize the auditable CRV6 R1 provenance bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


HISTORICAL_SHA = "481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361"
INIT_SHA = "833e6148f566aed60c27378c4c1f832bb0e3f7532dae780d12ce5424579e2dfa"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-dir", required=True, type=Path)
    parser.add_argument("--prepared", required=True, type=Path)
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--recovery-audit", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--container-image-id", required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    train = json.loads((args.candidate_dir / "D1_TRAIN_REPORT.json").read_text(encoding="utf-8"))
    prep = json.loads((args.prepared / "D1_PREP_REPORT.json").read_text(encoding="utf-8"))
    recovery = json.loads(args.recovery_audit.read_text(encoding="utf-8"))
    checkpoint = next(iter(args.candidate_dir.glob("best_coco_bbox_mAP_epoch_*.pth")))
    candidate_sha = sha256(checkpoint)
    if candidate_sha == HISTORICAL_SHA or train["initial_checkpoint_sha256"] != INIT_SHA:
        raise RuntimeError("R1 identity boundary violated")
    script_path = Path(__file__).parent / "train_ddrv4_d1_rtmdet.py"
    blob = subprocess.run(
        ["git", "hash-object", str(script_path)], capture_output=True, text=True, check=True
    ).stdout.strip()
    qa_files = sorted(args.g7_root.glob("*.json"))
    data_hashes = {
        "prepared": prep["sha256"],
        "G7_QA": {item.name: sha256(item) for item in qa_files},
        "recovery_audit_sha256": sha256(args.recovery_audit),
    }
    config = {
        "architecture": train["architecture"], "epochs": train["epochs"],
        "batch_size": train["batch_size"], "epoch_exposures": prep["fit_images"],
        "sampling_policy_sha256": prep["sha256"]["D1_SAMPLING_POLICY.json"],
        "optimizer_lr": 0.0005, "AMP": True, "grad_clip_max_norm": 10.0,
        "selection": "G7_STATIC_IN_DOMAIN_HOLDOUT_ONLY", "seed": 20260814,
        "G7_static_VAL_used_for_selection": False, "config_sha256": train["config_sha256"],
    }
    provenance = {
        "candidate_id": "D1B_RECON_R1", "route": "R1",
        "historical_D1B_sha256": HISTORICAL_SHA, "candidate_sha256": candidate_sha,
        "candidate_hash_differs_from_historical": True,
        "historical_identity_claimed": False, "initialization_sha256": INIT_SHA,
        "training_script_git_blob_sha": blob, "source_commit": args.source_commit,
        "container_image_id": args.container_image_id,
        "versions": {"mmdetection": "3.3.0", "torch": "2.5.1+cu124", "cuda": "12.4", "mmengine": "0.10.7", "mmcv": "2.1.0"},
    }
    report = {
        "schema_version": 1, "protocol": "CHECKPOINT-RECONSTITUTION-V6", "stage": "CRV6-01",
        **provenance, "HISTORICAL_D1B_CHECKPOINT_LOST": recovery["HISTORICAL_D1B_CHECKPOINT_LOST"],
        "recovery_search_closed": recovery["recovery_search_closed"],
        "best_checkpoint_selection_metric": "G7_STATIC_IN_DOMAIN_HOLDOUT coco/bbox_mAP",
        "best_epoch": int(checkpoint.stem.rsplit("_", 1)[1]),
        "best_checkpoint_relative_path": checkpoint.name,
        "training_report_sha256": sha256(args.candidate_dir / "D1_TRAIN_REPORT.json"),
        "G7_VAL_used": False, "G6_used": False, "G5_read": False, "G5_V2_read": False,
        "CRV6_R1_RECONSTITUTION_COMPLETE": True,
    }
    write(args.output / "RECON_TRAIN_DATA_HASHES.json", data_hashes)
    write(args.output / "RECON_TRAIN_CONFIG.json", config)
    write(args.output / "RECON_MODEL_PROVENANCE.json", provenance)
    write(args.output / "CHECKPOINT_RECONSTITUTION_REPORT.json", report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
