"""Sealed G5 final-set contract and one-shot final evaluator scaffolding.

The G5 final set stays sealed: development code never loads it, and a final
evaluation can only be opened when a validated ``MODEL_FREEZE.json`` exists
and the metadata satisfies the frozen contract (>= 4 unseen worlds,
>= 100 scenes, >= 1000 frames, unseen target and hard-negative assets).

The evaluator is one-shot: the first access/evaluation is recorded
atomically (``O_CREAT|O_EXCL``) and any rerun or partial probing is refused.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from .g4_manifest import load_freeze, validate_freeze_payload
from .g4_manifest import config_hash


SEALED_MIN_WORLDS = 4
SEALED_MIN_SCENES = 100
SEALED_MIN_FRAMES = 1000
ACCESS_RECORD_NAME = "sealed_final_access.json"
RESULT_RECORD_NAME = "sealed_final_result.json"


class SealedFinalReuseError(RuntimeError):
    """Raised when the sealed final set is accessed or evaluated twice."""


def validate_sealed_manifest(
    manifest: dict,
    freeze: dict,
    *,
    development_world_ids,
    development_target_assets,
    development_hard_negative_assets,
) -> dict:
    """Validate the G5 sealed manifest against the frozen contract.

    Fail-closed on any missing count, unseen-asset violation or missing
    freeze evidence.
    """
    if not isinstance(manifest, dict):
        raise ValueError("G5 sealed manifest must be a JSON object")
    if manifest.get("schema_version") != 1:
        raise ValueError("G5 sealed manifest schema_version must be 1")
    if manifest.get("dataset_id") != "G5_SEALED_FINAL":
        raise ValueError(
            "G5 sealed manifest dataset_id must be G5_SEALED_FINAL"
        )
    worlds = manifest.get("worlds")
    scenes = manifest.get("scenes")
    frames = manifest.get("frames")
    if not isinstance(worlds, list) or len(worlds) < SEALED_MIN_WORLDS:
        raise ValueError(
            "G5 sealed final requires at least "
            f"{SEALED_MIN_WORLDS} unseen worlds"
        )
    if not isinstance(scenes, int) or scenes < SEALED_MIN_SCENES:
        raise ValueError(
            "G5 sealed final requires at least "
            f"{SEALED_MIN_SCENES} scenes"
        )
    if not isinstance(frames, int) or frames < SEALED_MIN_FRAMES:
        raise ValueError(
            "G5 sealed final requires at least "
            f"{SEALED_MIN_FRAMES} frames"
        )
    known_worlds = set(development_world_ids)
    overlap = sorted(set(worlds) & known_worlds)
    if overlap:
        raise ValueError(
            "G5 worlds must be unseen in development; overlap: "
            + ", ".join(sorted(overlap))
        )
    for key in ("target_assets", "hard_negative_assets"):
        assets = manifest.get(key)
        if not isinstance(assets, list) or not assets:
            raise ValueError(f"G5 sealed manifest {key} must be a non-empty list")
    seen_targets = set(development_target_assets)
    seen_hard_negatives = set(development_hard_negative_assets)
    target_overlap = sorted(set(manifest["target_assets"]) & seen_targets)
    if target_overlap:
        raise ValueError(
            "G5 target assets must be unseen; overlap: "
            + ", ".join(target_overlap[:20])
        )
    hard_overlap = sorted(
        set(manifest["hard_negative_assets"]) & seen_hard_negatives
    )
    if hard_overlap:
        raise ValueError(
            "G5 hard-negative assets must be unseen; overlap: "
            + ", ".join(hard_overlap[:20])
        )
    if not isinstance(manifest.get("manifest_sha256"), str) or not isinstance(
        manifest.get("sealed_by"), str
    ):
        raise ValueError(
            "G5 sealed manifest must carry manifest_sha256 and sealed_by"
        )
    hash_payload = dict(manifest)
    declared_hash = hash_payload.pop("manifest_sha256")
    if declared_hash != config_hash(hash_payload):
        raise ValueError("G5 sealed manifest SHA-256 mismatch")
    # A valid MODEL_FREEZE.json must already exist; the freeze payload is
    # re-validated so its hashes/fields are machine-checkable.
    validate_freeze_payload(freeze)
    return manifest


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode(
        "utf-8"
    )
    try:
        descriptor = os.open(
            str(path),
            os.O_CREAT | os.O_EXCL | os.O_WRONLY,
            0o644,
        )
    except FileExistsError as exc:
        raise SealedFinalReuseError(
            f"sealed final record already exists: {path}"
        ) from exc
    try:
        os.write(descriptor, encoded)
    finally:
        os.close(descriptor)


class SealedFinalGate:
    """One-shot G5 evaluator that refuses reruns and partial probing."""

    def __init__(self, evidence_dir):
        self.evidence_dir = Path(evidence_dir)
        self.access_path = self.evidence_dir / ACCESS_RECORD_NAME
        self.result_path = self.evidence_dir / RESULT_RECORD_NAME

    def open_once(
        self,
        *,
        freeze_path,
        sealed_manifest: dict,
        development_world_ids,
        development_target_assets,
        development_hard_negative_assets,
    ) -> dict:
        """Record the first (and only) access of the sealed final set."""
        if self.access_path.is_file() or self.result_path.is_file():
            raise SealedFinalReuseError(
                "G5 sealed final set has already been accessed or evaluated; "
                "reruns and partial probing are forbidden"
            )
        freeze = load_freeze(freeze_path)
        validate_sealed_manifest(
            sealed_manifest,
            freeze,
            development_world_ids=development_world_ids,
            development_target_assets=development_target_assets,
            development_hard_negative_assets=development_hard_negative_assets,
        )
        access = {
            "schema_version": 1,
            "event": "sealed_final_first_access",
            "dataset_id": "G5_SEALED_FINAL",
            "freeze_id": freeze["freeze_id"],
            "freeze_config_hash": freeze["config_hash"],
            "manifest_sha256": sealed_manifest.get("manifest_sha256"),
            "access_timestamp_utc": time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
            ),
            "evaluation_count": 0,
        }
        _atomic_write_json(self.access_path, access)
        return access

    def evaluate_once(
        self,
        *,
        metrics: dict,
        freeze_id: str,
    ) -> dict:
        """Write the final one-shot evaluation; refuse any rerun."""
        if not self.access_path.is_file():
            raise SealedFinalReuseError(
                "sealed final access must be recorded before evaluation"
            )
        if self.result_path.is_file():
            raise SealedFinalReuseError(
                "sealed final evaluation already recorded; rerun forbidden"
            )
        access = json.loads(self.access_path.read_text(encoding="utf-8"))
        if access.get("freeze_id") != freeze_id:
            raise ValueError(
                "sealed final evaluation freeze_id does not match access record"
            )
        result = {
            "schema_version": 1,
            "event": "sealed_final_evaluation",
            "dataset_id": "G5_SEALED_FINAL",
            "freeze_id": freeze_id,
            "evaluation_timestamp_utc": time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
            ),
            "metrics": metrics,
            "one_shot": True,
            "rerun_allowed": False,
        }
        _atomic_write_json(self.result_path, result)
        return result


__all__ = [
    "ACCESS_RECORD_NAME",
    "RESULT_RECORD_NAME",
    "SEALED_MIN_FRAMES",
    "SEALED_MIN_SCENES",
    "SEALED_MIN_WORLDS",
    "SealedFinalGate",
    "SealedFinalReuseError",
    "validate_sealed_manifest",
]
