#!/usr/bin/env python3
"""Statically validate the A20 sealing, replay, release, and rollback receipt.

The validator is intentionally pure: it validates supplied JSON only and does
not treat its own success as a runtime or release acceptance result.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

from auto16_release import sha256
from coverage_mcap_replay_audit import validate_product_replay_reports


SCHEMA = "tzcup.a20_release_replay_receipt.v1"
HASH_FIELDS = ("source", "model", "config", "dataset", "dependency")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")


def _is_hash(value: Any) -> bool:
    return isinstance(value, str) and SHA256.fullmatch(value) is not None


def _same_hashes(actual: Any, expected: dict[str, str]) -> bool:
    return isinstance(actual, dict) and actual == expected


def validate_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    """Return a deterministic A20 validation report without filesystem access."""

    errors: list[str] = []
    if receipt.get("schema") != SCHEMA:
        errors.append("unsupported receipt schema")

    hashes = receipt.get("input_hashes")
    if not isinstance(hashes, dict):
        errors.append("missing input_hashes")
        hashes = {}
    hash_values = {name: hashes.get(name) for name in HASH_FIELDS}
    for name, value in hash_values.items():
        if not _is_hash(value):
            errors.append(f"missing or invalid {name} hash")
    typed_hashes = {name: value for name, value in hash_values.items() if isinstance(value, str)}

    frozen = receipt.get("frozen_snapshot")
    if not isinstance(frozen, dict):
        errors.append("missing frozen snapshot record")
        frozen = {}
    for name in ("snapshot_manifest_sha256", "source_inventory_sha256", "expanded_urdf_sha256"):
        if not _is_hash(frozen.get(name)):
            errors.append(f"missing frozen snapshot {name}")
    if frozen.get("source_inventory_sha256") != hashes.get("source"):
        errors.append("frozen snapshot source hash does not match receipt")

    session = receipt.get("sealed_final_session")
    if not isinstance(session, dict):
        errors.append("missing sealed final session record")
        session = {}
    if session.get("report_id") != "tzcup_formal_final_acceptance_session_v1":
        errors.append("sealed final session has wrong identity")
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE":
        errors.append("sealed final session is not complete")
    if session.get("snapshot") != frozen:
        errors.append("sealed final session does not bind the frozen snapshot")

    closure = receipt.get("current_runtime_closure")
    if not isinstance(closure, dict):
        errors.append("missing current runtime closure record")
        closure = {}
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        errors.append("runtime closure is not currently verified")
    if closure.get("passed") is not True or not _is_hash(closure.get("closure_sha256")):
        errors.append("runtime closure has no verified digest")
    binding = session.get("runtime_closure_binding")
    if not isinstance(binding, dict) or binding.get("closure_sha256") != closure.get("closure_sha256"):
        errors.append("sealed final session has a non-current closure")

    replays = receipt.get("product_replays")
    replay_result = validate_product_replay_reports(replays if isinstance(replays, list) else [])
    errors.extend(replay_result["failures"])
    if isinstance(replays, list):
        for index, replay in enumerate(replays):
            if not isinstance(replay, dict):
                continue
            if not _same_hashes(replay.get("input_hashes"), typed_hashes):
                errors.append(f"replays[{index}] input hashes do not match receipt")
            if replay.get("snapshot") != frozen:
                errors.append(f"replays[{index}] does not bind the frozen snapshot")
            if replay.get("closure_sha256") != closure.get("closure_sha256"):
                errors.append(f"replays[{index}] has a non-current closure")

    release = receipt.get("release_artifact")
    if not isinstance(release, dict):
        errors.append("missing release artifact record")
        release = {}
    if release.get("status") != "RELEASE_PACKAGE_ARTIFACT_RECORDED":
        errors.append("release artifact is not recorded")
    if not _is_hash(release.get("archive_sha256")) or not _is_hash(release.get("sbom_sha256")):
        errors.append("release artifact is missing archive or SBOM hash")
    if not isinstance(release.get("main_commit"), str) or GIT_SHA.fullmatch(release["main_commit"]) is None:
        errors.append("release artifact has no exact main commit")
    if not isinstance(release.get("rollback_commit"), str) or GIT_SHA.fullmatch(release["rollback_commit"]) is None:
        errors.append("release artifact has no exact rollback commit")

    rollback = receipt.get("verified_rollback_exercise")
    if not isinstance(rollback, dict):
        errors.append("missing verified rollback exercise")
        rollback = {}
    if rollback.get("status") != "ROLLBACK_EXERCISE_VERIFIED" or rollback.get("verified") is not True:
        errors.append("rollback exercise is not verified")
    if rollback.get("rollback_commit") != release.get("rollback_commit"):
        errors.append("rollback exercise does not bind the declared rollback commit")
    if not isinstance(rollback.get("rollback_commit"), str) or GIT_SHA.fullmatch(rollback["rollback_commit"]) is None:
        errors.append("rollback exercise has no exact rollback commit")
    if not _is_hash(rollback.get("verification_report_sha256")):
        errors.append("rollback exercise has no verification report hash")

    return {
        "schema": "tzcup.a20_release_replay_receipt_validation.v1",
        "status": "A20_RECEIPT_STATIC_VALID" if not errors else "A20_RECEIPT_STATIC_BLOCKED",
        "valid": not errors,
        "errors": errors,
        "replay": replay_result,
        "release_runtime_pass": False,
        "claim_boundary": (
            "Static receipt validation does not prove a release artifact exists, "
            "a runtime executed, or any product acceptance passed."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        receipt = json.loads(args.receipt.read_text(encoding="utf-8"))
        if not isinstance(receipt, dict):
            raise ValueError("receipt root is not an object")
        report = validate_receipt(receipt)
        report["receipt_sha256"] = sha256(args.receipt)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        report = {
            "schema": "tzcup.a20_release_replay_receipt_validation.v1",
            "status": "A20_RECEIPT_STATIC_BLOCKED",
            "valid": False,
            "errors": [str(exc)],
            "release_runtime_pass": False,
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
