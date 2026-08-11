#!/usr/bin/env python3
"""Create the compact DDRV4-00 baseline without opening either sealed set."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import validate_data_boundary  # noqa: E402


HISTORICAL = {
    "HISTORICAL_X1_X3_FAILED": True,
    "HISTORICAL_X2_RESULT_PRESERVED": True,
    "MRV2_A_B_C_FAILED": True,
    "OPR_A_B_C_FAILED": True,
}
DATA_BOUNDARY = {
    "G5_STATUS": "CONSUMED_FINAL",
    "G5_CAN_BE_USED_FOR_TUNING": False,
    "G6_STATUS": "DEVELOPMENT_HISTORY",
    "G6_CAN_BE_USED_FOR_NEW_ROUTE_TUNING": False,
    "G5_V2_STATUS": "SEALED_NOT_OPENED",
    "G5_V2_CAN_BE_USED_FOR_TUNING": False,
    "DDRV4_G7_DEVELOPMENT_AUTHORIZED": True,
    "selection_dataset_ids": ["G7_DETECTOR_DEVELOPMENT"],
}


def _git(*args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(ROOT), *args], text=True, encoding="utf-8"
    ).strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build(output: Path) -> dict:
    validate_data_boundary(DATA_BOUNDARY)
    output.mkdir(parents=True, exist_ok=False)
    source_commit = _git("rev-parse", "HEAD")
    source_tree = _git("rev-parse", "HEAD^{tree}")
    generated = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    common = {
        "schema_version": 1,
        "protocol": "DETECTOR-DATA-RECOVERY-V4",
        "stage": "DDRV4-00",
        "generated_at_utc": generated,
        "source_commit": source_commit,
        "source_tree": source_tree,
    }
    evidence_paths = [
        "artifacts/online_first_recovery_v3_20260810T042843Z/oprv3_09/OPRV3_SEALED_FINAL_SUMMARY.json",
        "artifacts/online_first_recovery_v3_20260810T042843Z/g6/G6_DEVELOPMENT_SUMMARY.json",
        "artifacts/online_first_recovery_v3_20260810T042843Z/opr_a/OPR_A_DEVELOPMENT_SUMMARY.json",
        "artifacts/online_first_recovery_v3_20260810T042843Z/opr_b/OPR_B_DEVELOPMENT_SUMMARY.json",
        "artifacts/online_first_recovery_v3_20260810T042843Z/opr_c/OPR_C_DEVELOPMENT_SUMMARY.json",
        "artifacts/online_first_recovery_v3_20260810T042843Z/oprv3_06_g6/OPRV3_G6_AREA_SUMMARY.json",
    ]
    baseline = {
        **common,
        "branch": _git("branch", "--show-current"),
        "worktree_clean_before_generation": not bool(_git("status", "--porcelain")),
        "historical_evidence": [
            {"path": item, "sha256": _sha256(ROOT / item)} for item in evidence_paths
        ],
        "sealed_content_opened": False,
    }
    model = {
        **common,
        "detector": {
            "status": "MODEL_BLOCKED_INTERNAL",
            "authorized_routes": ["DDRV4-D1", "DDRV4-D2", "DDRV4-D3"],
            "additional_routes_authorized": False,
        },
        "area": {
            "status": "OPRV3_06_AREA_PASS",
            "retune_authorized": False,
            "exception": "verified integration software bug only",
        },
        "PRODUCT_X86_PERCEPTION_READY": False,
    }
    environment = {
        **common,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "required_runtime": {
            "gpu": "NVIDIA GeForce RTX 4080 Laptop GPU",
            "d1_container": "tzcup/opr-c-rtmdet:v3.3.0-ops",
            "d1_container_digest": "sha256:fefe73290aed0018afe47c725b449b6c3da3db954d9d0ea01c12ed5778a42864",
        },
        "G5_or_G5_V2_content_probed": False,
    }
    records = {
        "REPO_BASELINE.json": baseline,
        "HISTORICAL_ROUTE_STATUS.json": {**common, **HISTORICAL},
        "DATA_BOUNDARY.json": {**common, **DATA_BOUNDARY},
        "MODEL_BOUNDARY.json": model,
        "ENVIRONMENT.json": environment,
    }
    for name, payload in records.items():
        _write(output / name, payload)
    return {"output": output.as_posix(), "files": sorted(records), "sealed_content_opened": False}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(build(args.output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
