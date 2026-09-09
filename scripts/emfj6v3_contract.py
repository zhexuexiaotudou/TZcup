#!/usr/bin/env python3
"""Validate the fail-closed EMFJ6V3 candidate and phase-order contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "config" / "existing_model_candidates_v3.yaml"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
REVISION_RE = re.compile(r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$")
ROLES = {"detector", "classifier", "area"}
LICENSE_STATES = {
    "development_only",
    "competition_open_source",
    "commercial_permissive",
    "blocked_license",
}
REQUIRED_CANDIDATE_FIELDS = {
    "model_id",
    "role",
    "source_uri",
    "revision",
    "files",
    "architecture",
    "class_order",
    "class_order_source",
    "class_semantics",
    "license",
    "training_data",
    "weight_distribution",
    "input_output",
    "framework",
    "onnx_exportability",
    "journey6_preflight_status",
    "disposition",
    "reason",
}


class ContractError(ValueError):
    """The V3 order, source, or evidence contract is invalid."""


def load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ContractError("registry must be a YAML mapping")
    return payload


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_registry(payload: dict[str, Any]) -> dict[str, int]:
    if payload.get("schema_version") != 3 or payload.get("protocol_id") != "EMFJ6V3":
        raise ContractError("registry must declare EMFJ6V3 schema version 3")
    if payload.get("sealed_access_allowed") is not False:
        raise ContractError("sealed access must remain disabled")
    frozen = payload.get("inventory_frozen")
    if not isinstance(frozen, bool):
        raise ContractError("inventory_frozen must be boolean")
    discovery_status = payload.get("discovery_status")
    expected_roles = {"detector", "classifier", "area"}
    if not isinstance(discovery_status, dict) or set(discovery_status) != expected_roles:
        raise ContractError("discovery_status must declare detector, classifier, and area")
    if any(not isinstance(value, bool) for value in discovery_status.values()):
        raise ContractError("discovery_status values must be boolean")
    discovery_complete = all(discovery_status.values())
    if frozen != discovery_complete:
        raise ContractError("inventory freeze requires all discovery roles complete")
    completed_at = payload.get("discovery_completed_at")
    if frozen:
        if not isinstance(completed_at, str) or not completed_at.strip():
            raise ContractError("frozen inventory requires discovery_completed_at")
        try:
            datetime.fromisoformat(completed_at)
        except ValueError as exc:
            raise ContractError("discovery_completed_at must be ISO-8601") from exc
    elif completed_at is not None:
        raise ContractError("unfrozen inventory must not declare discovery_completed_at")
    states = payload.get("states")
    if not isinstance(states, dict):
        raise ContractError("registry states must be a mapping")
    if states.get("EMF_EXISTING_MODEL_INVENTORY_READY") is not frozen:
        raise ContractError("inventory ready state must match inventory freeze")
    limits = payload.get("candidate_limits")
    if limits != {"detector": 12, "classifier": 6, "area": 3}:
        raise ContractError("candidate limits must remain detector=12 classifier=6 area=3")
    exclusions = payload.get("discovery_exclusions")
    if not isinstance(exclusions, list):
        raise ContractError("discovery_exclusions must be a list")
    for exclusion in exclusions:
        if not isinstance(exclusion, dict):
            raise ContractError("discovery exclusions must be mappings")
        if exclusion.get("role") not in ROLES:
            raise ContractError("discovery exclusion role is invalid")
        if not isinstance(exclusion.get("source_uri"), str) or not exclusion["source_uri"]:
            raise ContractError("discovery exclusion source_uri is required")
        if not isinstance(exclusion.get("reason"), str) or not exclusion["reason"]:
            raise ContractError("discovery exclusion reason is required")
    candidates = payload.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ContractError("registry must contain candidates")
    counts = {role: 0 for role in ROLES}
    identifiers: set[str] = set()
    for candidate in candidates:
        if not isinstance(candidate, dict):
            raise ContractError("candidate entries must be mappings")
        missing = REQUIRED_CANDIDATE_FIELDS - set(candidate)
        if missing:
            raise ContractError(f"candidate missing fields: {sorted(missing)}")
        model_id = candidate["model_id"]
        if not isinstance(model_id, str) or not model_id or model_id in identifiers:
            raise ContractError("candidate model_id must be unique and non-empty")
        identifiers.add(model_id)
        role = candidate["role"]
        if role not in ROLES:
            raise ContractError(f"unsupported role: {role}")
        counts[role] += 1
        if counts[role] > limits[role]:
            raise ContractError(f"candidate cap exceeded for {role}")
        revision = candidate["revision"]
        if not isinstance(revision, str) or not REVISION_RE.fullmatch(revision.lower()):
            raise ContractError(f"{model_id}: revision must be an immutable 40/64 hex id")
        if candidate["license"] not in LICENSE_STATES:
            raise ContractError(f"{model_id}: invalid license state")
        if not isinstance(candidate["class_order"], list) or not candidate["class_order"]:
            raise ContractError(f"{model_id}: class order must be explicit")
        if not isinstance(candidate["class_order_source"], str) or not candidate["class_order_source"]:
            raise ContractError(f"{model_id}: class order source must be explicit")
        files = candidate["files"]
        if not isinstance(files, list) or not files:
            raise ContractError(f"{model_id}: at least one artifact file is required")
        for artifact in files:
            digest = artifact.get("sha256") if isinstance(artifact, dict) else None
            if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest.lower()):
                raise ContractError(f"{model_id}: artifact SHA-256 is required")
    return counts


def training_allowed(state: dict[str, Any]) -> bool:
    return all(
        (
            state.get("EMF_EXISTING_MODEL_SCREENING_COMPLETE") is True,
            state.get("EMF_NONTRAINING_ADJUSTMENT_COMPLETE") is True,
            state.get("EMF_TRANSFER_LEARNING_REQUIRED") is True,
            state.get("sealed_access_allowed") is False,
        )
    )


def build_inventory(payload: dict[str, Any], registry_path: Path) -> dict[str, Any]:
    counts = validate_registry(payload)
    try:
        registry_relative = registry_path.resolve().relative_to(ROOT).as_posix()
    except ValueError as exc:
        raise ContractError("registry must be inside the repository") from exc
    frozen = payload.get("inventory_frozen") is True
    completed_at = payload.get("discovery_completed_at")
    discovery_status = payload["discovery_status"]
    ready = frozen and all(discovery_status.values())
    return {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "registry_id": payload["registry_id"],
        "registry_path": registry_relative,
        "registry_sha256": sha256(registry_path),
        "inventory_frozen": frozen,
        "discovery_completed_at": completed_at,
        "discovery_status": discovery_status,
        "candidate_limits": payload["candidate_limits"],
        "candidate_counts": counts,
        "candidate_count_total": sum(counts.values()),
        "discovery_exclusions": payload["discovery_exclusions"],
        "development_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "sealed_access_allowed": False,
        "EMF_EXISTING_MODEL_INVENTORY_READY": ready,
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "candidates": payload["candidates"],
        "truth_boundary": (
            "Inventory readiness freezes bounded discovery and source metadata only. "
            "It does not prove artifact availability, inference, screening, product "
            "fitness, release licensing, Journey 6 conversion, or training authority."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--state", type=Path)
    parser.add_argument("--authorize-training", action="store_true")
    parser.add_argument("--output-inventory", type=Path)
    parser.add_argument("--require-frozen", action="store_true")
    arguments = parser.parse_args()
    payload = load_yaml(arguments.registry)
    counts = validate_registry(payload)
    if arguments.authorize_training:
        state = payload if arguments.state is None else json.loads(
            arguments.state.read_text(encoding="utf-8")
        )
        if not training_allowed(state):
            raise ContractError(
                "training blocked until existing-model screening and non-training "
                "adjustment are complete and transfer learning is explicitly required"
            )
    inventory = build_inventory(payload, arguments.registry.resolve())
    if arguments.require_frozen and not inventory["EMF_EXISTING_MODEL_INVENTORY_READY"]:
        raise ContractError("existing-model inventory is not frozen and complete")
    if arguments.output_inventory is not None:
        arguments.output_inventory.parent.mkdir(parents=True, exist_ok=True)
        arguments.output_inventory.write_text(
            json.dumps(inventory, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(json.dumps({
        "valid": True,
        "candidate_counts": counts,
        "inventory_ready": inventory["EMF_EXISTING_MODEL_INVENTORY_READY"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
