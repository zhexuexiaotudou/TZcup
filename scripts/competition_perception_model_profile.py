#!/usr/bin/env python3
"""Resolve hash-bound ONNX models for perception fixtures."""
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path


CONTROLLED_FIXTURE_PROFILE = "controlled_primitive_color_fixture"
PROFILES = {
    CONTROLLED_FIXTURE_PROFILE: {
        "relative_path": "artifacts/stage5a_20260717_review/stage5a_synthetic_color_prototype.onnx",
        "sha256": "f7a10487a2577dc6675abb6784769308a0c21bfb11c0c9eb42af6aa2a5152e73",
        "scope": "synthetic_color_domain_only",
    }
}


class ModelProfileError(ValueError):
    pass


@dataclass(frozen=True)
class ModelProfile:
    profile_id: str
    path: Path
    sha256: str
    scope: str


def resolve_model_profile(source_root: str | Path, profile_id: str) -> ModelProfile:
    if profile_id not in PROFILES:
        raise ModelProfileError(f"unknown controlled perception model profile: {profile_id}")
    spec = PROFILES[profile_id]
    root = Path(source_root).resolve()
    path = (root / str(spec["relative_path"])).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ModelProfileError("model profile path escapes the source root") from exc
    if not path.is_file():
        raise ModelProfileError(f"profile model is missing: {path}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != spec["sha256"]:
        raise ModelProfileError(
            f"profile model hash mismatch: expected {spec['sha256']}, got {actual}"
        )
    return ModelProfile(profile_id, path, actual, str(spec["scope"]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--profile", default=CONTROLLED_FIXTURE_PROFILE)
    parser.add_argument("--field", choices=("json", "path", "sha256", "scope"), default="json")
    args = parser.parse_args()
    profile = resolve_model_profile(args.source, args.profile)
    if args.field == "path":
        print(profile.path)
    elif args.field == "sha256":
        print(profile.sha256)
    elif args.field == "scope":
        print(profile.scope)
    else:
        print(json.dumps({
            "profile_id": profile.profile_id,
            "path": str(profile.path),
            "sha256": profile.sha256,
            "scope": profile.scope,
        }, sort_keys=True))


if __name__ == "__main__":
    main()
