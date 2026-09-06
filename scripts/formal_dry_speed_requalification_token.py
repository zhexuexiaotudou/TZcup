#!/usr/bin/env python3
"""Create and validate the run-scoped explicit opt-in marker for the test lane."""

from __future__ import annotations

import argparse
import hashlib
import json
import secrets
from pathlib import Path
from typing import Any

import yaml


def _profile(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("requalification profile must be a mapping")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _expected(profile: dict[str, Any], stage_id: str) -> tuple[str, float]:
    activation = profile.get("activation")
    if not isinstance(activation, dict):
        raise ValueError("profile activation is missing")
    if activation.get("required_environment") != "FORMAL_DRY_SPEED_REQUALIFICATION=1":
        raise ValueError("profile does not require explicit opt-in")
    stages = profile.get("qualification_stages")
    if not isinstance(stages, list):
        raise ValueError("profile qualification_stages is missing")
    selected = next(
        (item for item in stages if isinstance(item, dict) and item.get("id") == stage_id),
        None,
    )
    if selected is None:
        raise ValueError("requested qualification stage is not declared by the profile")
    test_cap = selected.get("target_linear_speed_mps")
    if isinstance(test_cap, bool) or not isinstance(test_cap, (int, float)) or test_cap <= 0.0:
        raise ValueError("profile stage speed is invalid")
    profile_id = profile.get("profile_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError("profile_id is missing")
    return profile_id, float(test_cap)


def create(*, profile_path: Path, run_root: Path, output: Path, stage_id: str) -> dict[str, Any]:
    profile = _profile(profile_path)
    profile_id, test_cap = _expected(profile, stage_id)
    result = {
        "schema_version": 1,
        "kind": "TZCUP_DRY_SPEED_REQUALIFICATION_RUN_SCOPED_OPT_IN_MARKER",
        "profile_id": profile_id,
        "profile_sha256": _sha256(profile_path),
        "qualification_stage_id": stage_id,
        "run_root": str(run_root.resolve()),
        "test_only_whole_vehicle_safety_cap_mps": test_cap,
        "nonce": secrets.token_urlsafe(32),
    }
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def validate(*, profile_path: Path, run_root: Path, token_path: Path, requested_cap: float) -> dict[str, Any]:
    profile = _profile(profile_path)
    token = json.loads(token_path.read_text(encoding="utf-8"))
    if not isinstance(token, dict):
        raise ValueError("opt-in marker must be a JSON object")
    if token.get("kind") != "TZCUP_DRY_SPEED_REQUALIFICATION_RUN_SCOPED_OPT_IN_MARKER":
        raise ValueError("wrong opt-in marker kind")
    stage_id = token.get("qualification_stage_id")
    if not isinstance(stage_id, str) or not stage_id:
        raise ValueError("marker stage is invalid")
    profile_id, test_cap = _expected(profile, stage_id)
    if token.get("profile_id") != profile_id or token.get("profile_sha256") != _sha256(profile_path):
        raise ValueError("marker does not bind the current profile")
    if token.get("run_root") != str(run_root.resolve()):
        raise ValueError("marker does not bind this requalification run root")
    if token.get("test_only_whole_vehicle_safety_cap_mps") != test_cap:
        raise ValueError("marker test cap differs from profile")
    if requested_cap != test_cap:
        raise ValueError("requested non-default cap differs from marker")
    nonce = token.get("nonce")
    if not isinstance(nonce, str) or len(nonce) < 32:
        raise ValueError("marker nonce is invalid")
    return token


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--create", action="store_true")
    mode.add_argument("--validate", action="store_true")
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--token", type=Path, required=True)
    parser.add_argument("--requested-cap", type=float)
    parser.add_argument("--stage")
    args = parser.parse_args()
    if args.create:
        if args.token.exists():
            raise SystemExit("refusing to overwrite requalification opt-in marker")
        if not args.stage:
            raise SystemExit("--stage is required with --create")
        print(json.dumps(create(profile_path=args.profile, run_root=args.run_root, output=args.token, stage_id=args.stage), sort_keys=True))
        return 0
    if args.requested_cap is None:
        raise SystemExit("--requested-cap is required with --validate")
    print(json.dumps(validate(profile_path=args.profile, run_root=args.run_root, token_path=args.token, requested_cap=args.requested_cap), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
