#!/usr/bin/env python3
"""Issue the one-time sealed-development access record after a green V10 HOLDOUT."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--integrated-holdout", type=Path, required=True)
    parser.add_argument("--freeze-input", action="append", required=True, help="NAME=path")
    parser.add_argument("--repository-commit", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("sealed access record already exists; one-time access cannot be reissued")
    holdout = json.loads(args.integrated_holdout.read_text(encoding="utf-8"))
    if holdout.get("TRCRV10_INTEGRATED_HOLDOUT_PASS") is not True or holdout.get("sealed_access_authorized_next") is not True:
        raise PermissionError("integrated HOLDOUT is not green; sealed access denied")
    freeze = {}
    for value in args.freeze_input:
        name, raw_path = value.split("=", 1)
        path = Path(raw_path)
        if name in freeze:
            raise ValueError(f"duplicate freeze input name: {name}")
        freeze[name] = {"path": str(path.resolve()), "sha256": sha256(path)}
    required = {"proposal_model", "proposal_threshold", "classifier", "verifier", "reobserve_policy", "g10_manifest"}
    if set(freeze) != required:
        raise ValueError(f"freeze inputs must be exactly {sorted(required)}")
    record = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-08-SEALED-ACCESS-AUTHORIZATION",
        "authorized_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository_commit": args.repository_commit,
        "integrated_holdout": {"path": str(args.integrated_holdout.resolve()), "sha256": sha256(args.integrated_holdout)},
        "freeze_inputs": freeze,
        "access_scope": ["G10_DEV_VAL_SEALED", "historical_VAL_NEW_supported_cross_check_only"],
        "G5_V2_access_authorized": False,
        "one_time": True,
        "immutable_after_issue": True,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output.resolve()), "sha256": sha256(args.output)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
