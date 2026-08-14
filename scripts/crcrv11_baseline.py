#!/usr/bin/env python3
"""Freeze the CRCRV11 repository, V10 result, and sealed-data boundary."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


REQUIRED_DIRECTORIES = (
    "baseline", "forensic", "five_view", "c11_data", "r1", "r2", "r3",
    "classifier_gate", "action_verifier", "integrated_holdout", "dev_val",
    "tracker_map", "online_dev", "performance", "freeze", "g5v2",
    "moving_30seed", "spot_clean_30seed", "post_clean", "soak", "faults",
    "replay", "x86_release", "final",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def git(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True, encoding="utf-8",
    ).strip()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--v10-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    repo = args.repo.resolve()
    v10_root = args.v10_root.resolve()
    output = args.output.resolve()
    v10_status_path = v10_root / "final" / "PERCEPTION_TRCRV10_FINAL_STATUS.json"
    v10_blockers_path = v10_root / "final" / "PERCEPTION_TRCRV10_FINAL_BLOCKERS.json"
    v10_boundary_path = v10_root / "baseline" / "UNREAD_DATA_BOUNDARY.json"
    for path in (v10_status_path, v10_blockers_path, v10_boundary_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    v10_status = load_json(v10_status_path)
    v10_blockers = load_json(v10_blockers_path)
    v10_boundary = load_json(v10_boundary_path)
    unread = {
        "G10_DEV_VAL_SEALED_read": bool(v10_boundary.get("G10_DEV_VAL_SEALED_read", False)),
        "VAL_NEW_read": bool(v10_boundary.get("VAL_NEW_read", False)),
        "G5_V2_read": bool(v10_boundary.get("G5_V2_read", False)),
    }
    if any(unread.values()):
        raise RuntimeError(f"V11 cannot start from a consumed sealed boundary: {unread}")

    for relative in REQUIRED_DIRECTORIES:
        (output / relative).mkdir(parents=True, exist_ok=True)

    head = git(repo, "rev-parse", "HEAD")
    status = git(repo, "status", "--short")
    baseline = {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-CLASSIFIER-CONTRACT-RECOVERY-V11",
        "stage": "CRCRV11-00-BASELINE",
        "source_commit": head,
        "source_tree": git(repo, "rev-parse", "HEAD^{tree}"),
        "branch": git(repo, "branch", "--show-current"),
        "worktree_clean_at_freeze": status == "",
        "worktree_status": status.splitlines(),
        "v10_root": str(v10_root),
        "required_directories": list(REQUIRED_DIRECTORIES),
    }
    result_lock = {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-CLASSIFIER-CONTRACT-RECOVERY-V11",
        "stage": "TRCRV10-RESULT-LOCK",
        "immutable_history": {
            "RGDRV8_Route_A_B_C": "failed",
            "TGARV9_T1_T2_T3": "failed",
            "TRCRV10_Proposal": "pass",
            "TRCRV10_C1": "failed",
            "TRCRV10_targeted_recovery": "failed",
            "old_G5": "permanently_consumed",
        },
        "v10_final_status_sha256": sha256(v10_status_path),
        "v10_final_blockers_sha256": sha256(v10_blockers_path),
        "v10_final_status": v10_status,
        "v10_final_blockers": v10_blockers,
    }
    boundary = {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-CLASSIFIER-CONTRACT-RECOVERY-V11",
        "stage": "CRCRV11-SEALED-BOUNDARY",
        **unread,
        "formal_30_seed_read": False,
        "policy": "fail_closed_until_integrated_holdout_pass_and_atomic_access_record",
        "source_v10_boundary_sha256": sha256(v10_boundary_path),
    }
    hypotheses = {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-CLASSIFIER-CONTRACT-RECOVERY-V11",
        "stage": "CRCRV11-HYPOTHESIS-REGISTER",
        "question": (
            "Why does >=18px three-class GT-crop identifiability pass while the "
            "four-class runtime-faithful proposal-crop classifier reaches only "
            "macro-F1 0.6318 and background specificity 0.0833?"
        ),
        "hypotheses": [
            "BACKGROUND_SAMPLE_SCARCITY", "BACKGROUND_MEMORIZATION",
            "NEAR_MISS_LABEL_NOISE", "TRAIN_RUNTIME_VIEW_MISMATCH",
            "AUGMENTATION_TOO_STRONG", "CROP_CONTEXT_MISMATCH",
            "PIXEL_CHANNEL_BUG", "CLASS_INTRINSIC_CONFUSION",
        ],
        "authorized_routes": ["R1", "R2", "R3"],
        "detector_search_authorized": False,
    }
    baseline_dir = output / "baseline"
    write_json(baseline_dir / "REPO_BASELINE.json", baseline)
    write_json(baseline_dir / "TRCRV10_RESULT_LOCK.json", result_lock)
    write_json(baseline_dir / "SEALED_BOUNDARY.json", boundary)
    write_json(baseline_dir / "V11_HYPOTHESIS_REGISTER.json", hypotheses)
    print(json.dumps({
        "output": str(output), "source_commit": head,
        "worktree_clean_at_freeze": baseline["worktree_clean_at_freeze"], **unread,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
