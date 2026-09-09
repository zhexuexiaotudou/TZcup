#!/usr/bin/env python3
"""Create a fail-closed license audit for pinned pretrained candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "config" / "pretrained_model_sources.yaml"
REQUIRED_COMPONENTS = (
    "model_card_spdx",
    "architecture",
    "training_code",
    "training_data",
    "weight_distribution",
)
UNRESOLVED_MARKERS = {None, "", "unknown", "unresolved"}
ALLOWED_STATUSES = {
    "competition_open_source",
    "commercial_permissive",
    "research_only",
    "blocked_license",
}


class LicenseAuditError(RuntimeError):
    pass


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unresolved(value: Any) -> bool:
    return value in UNRESOLVED_MARKERS or (
        isinstance(value, str) and value.startswith("unresolved")
    )


def audit_entry(
    model_id: str,
    entry: dict[str, Any],
    model_card_root: Path | None = None,
) -> dict[str, Any]:
    license_info = entry.get("license", {})
    declared_status = license_info.get("status")
    if declared_status not in ALLOWED_STATUSES:
        raise LicenseAuditError(f"{model_id}: invalid license status")
    unresolved_components = sorted(
        component
        for component in REQUIRED_COMPONENTS
        if unresolved(license_info.get(component))
    )
    card = entry.get("model_card", {})
    card_digest = card.get("sha256")
    card_digest_verified: bool | None = None
    card_path: str | None = None
    if model_card_root is not None:
        candidate = model_card_root / model_id / str(card.get("filename", "README.md"))
        card_path = str(candidate)
        card_digest_verified = (
            candidate.is_file()
            and isinstance(card_digest, str)
            and file_sha256(candidate) == card_digest
        )
    component_complete = not unresolved_components
    status_consistent = (
        declared_status == "blocked_license"
        if not component_complete
        else declared_status != "blocked_license"
    )
    release_allowed = (
        component_complete
        and status_consistent
        and declared_status in {"competition_open_source", "commercial_permissive"}
        and card_digest_verified is not False
    )
    return {
        "model_id": model_id,
        "candidate": entry.get("candidate"),
        "declared_status": declared_status,
        "components": {
            component: license_info.get(component)
            for component in REQUIRED_COMPONENTS
        },
        "unresolved_components": unresolved_components,
        "component_complete": component_complete,
        "status_consistent": status_consistent,
        "model_card_sha256": card_digest,
        "model_card_path": card_path,
        "model_card_digest_verified": card_digest_verified,
        "release_allowed": release_allowed,
        "competition_claim_allowed": release_allowed
        and declared_status == "competition_open_source",
    }


def build_audit(registry: dict[str, Any], model_card_root: Path | None = None) -> dict:
    models = registry.get("models")
    if not isinstance(models, dict) or not models:
        raise LicenseAuditError("registry contains no models")
    records = [
        audit_entry(model_id, entry, model_card_root)
        for model_id, entry in models.items()
    ]
    return {
        "schema_version": 1,
        "registry_id": registry.get("registry_id"),
        "models": records,
        "all_components_resolved": all(
            record["component_complete"] for record in records
        ),
        "all_statuses_consistent": all(
            record["status_consistent"] for record in records
        ),
        "release_allowed": bool(records)
        and all(record["release_allowed"] for record in records),
        "truth_boundary": (
            "A model-card SPDX label alone does not resolve architecture, "
            "training-code, dataset, or weight-distribution obligations."
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--model-card-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        registry = yaml.safe_load(args.registry.read_text(encoding="utf-8"))
        report = build_audit(
            registry,
            args.model_card_root.resolve() if args.model_card_root else None,
        )
    except (OSError, yaml.YAMLError, LicenseAuditError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["release_allowed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
