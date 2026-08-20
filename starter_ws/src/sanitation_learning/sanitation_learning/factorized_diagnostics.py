"""AUTO-05R factorized diagnostics contract.

D1-D6 split model generalization failures across world, asset, material,
geometry, lighting, negative asset, and trajectory axes.  This module only
implements the config/report contracts; it never fabricates model metrics.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping

import yaml


REQUIRED_DIAGNOSIS_IDS = ("D1", "D2", "D3", "D4", "D5", "D6")
REQUIRED_METRICS = (
    "macro_f1",
    "per_class_precision",
    "per_class_recall",
    "per_class_f1",
    "ap50",
    "ap50_95",
    "negative_fp_per_frame",
    "discovery_recall",
    "leaf_iou",
    "puddle_iou",
)
KNOWN_FILTER_KEYS = {
    "world_seen",
    "asset_seen",
    "geometry_seen",
    "material_seen",
    "lighting_seen",
    "trajectory_seen",
    "negative_only",
    "negative_asset_unseen",
}


def load_diagnostic_config(path: str | Path) -> dict:
    """Load and validate the AUTO-05R factorized diagnostics YAML contract."""
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("factorized diagnostics config must be a YAML mapping")
    if config.get("schema_version") != 1:
        raise ValueError("factorized diagnostics config schema_version must be 1")
    if config.get("stage") != "AUTO-05R":
        raise ValueError("factorized diagnostics config stage must be AUTO-05R")
    if config.get("legacy_g3_test_used_as_selection") is not False:
        raise ValueError(
            "legacy_g3_test_used_as_selection must be false; "
            "the legacy G3 test must not become a new model selection set"
        )
    diagnoses = config.get("diagnoses")
    if not isinstance(diagnoses, dict):
        raise ValueError("config must define a diagnoses mapping")
    missing = [item for item in REQUIRED_DIAGNOSIS_IDS if item not in diagnoses]
    if missing:
        raise ValueError(f"factorized diagnostics missing diagnoses: {', '.join(missing)}")
    for diagnosis_id in REQUIRED_DIAGNOSIS_IDS:
        diagnosis = diagnoses[diagnosis_id]
        _validate_diagnosis(diagnosis_id, diagnosis)
    _validate_seen_metadata(config.get("seen_metadata", {}))
    return config


def _validate_diagnosis(diagnosis_id: str, diagnosis: object) -> None:
    if not isinstance(diagnosis, dict):
        raise ValueError(f"{diagnosis_id} must be a mapping")
    for field in ("name", "description", "filter", "required_metrics"):
        if field not in diagnosis:
            raise ValueError(f"{diagnosis_id} missing required field {field!r}")
    if not isinstance(diagnosis["name"], str) or not diagnosis["name"].strip():
        raise ValueError(f"{diagnosis_id} name must be a non-empty string")
    if not isinstance(diagnosis["description"], str) or not diagnosis["description"].strip():
        raise ValueError(f"{diagnosis_id} description must be a non-empty string")
    row_filter = diagnosis["filter"]
    if not isinstance(row_filter, dict):
        raise ValueError(f"{diagnosis_id} filter must be a mapping")
    unknown = set(row_filter) - KNOWN_FILTER_KEYS
    if unknown:
        raise ValueError(
            f"{diagnosis_id} filter has unknown keys: {', '.join(sorted(unknown))}"
        )
    for key, value in row_filter.items():
        if not isinstance(value, bool):
            raise ValueError(f"{diagnosis_id} filter {key!r} must be a boolean")
    metrics = diagnosis["required_metrics"]
    if not isinstance(metrics, list):
        raise ValueError(f"{diagnosis_id} required_metrics must be a list")
    missing_metrics = [item for item in REQUIRED_METRICS if item not in metrics]
    if missing_metrics:
        raise ValueError(
            f"{diagnosis_id} required_metrics missing: {', '.join(missing_metrics)}"
        )


def _validate_seen_metadata(seen_metadata: object) -> None:
    if not isinstance(seen_metadata, dict):
        raise ValueError("seen_metadata must be a mapping")
    expected = {
        "seen_world_ids",
        "seen_asset_families",
        "seen_geometry_families",
        "seen_material_ids",
        "seen_lighting_families",
        "seen_negative_asset_families",
        "seen_trajectory_ids",
    }
    for name in expected:
        if name not in seen_metadata:
            raise ValueError(f"seen_metadata missing {name!r}")
        if not isinstance(seen_metadata[name], list):
            raise ValueError(f"seen_metadata {name!r} must be a list")


def _row_boolean(row: Mapping[str, object], key: str, seen_values: list) -> bool:
    value = row.get(key)
    if value is None:
        return False
    return value in set(seen_values)


def diagnosis_ids_for_row(
    row: Mapping[str, object],
    config: Mapping[str, object],
) -> list[str]:
    """Return the D1-D6 diagnosis ids that apply to a row's metadata."""
    seen_metadata = config["seen_metadata"]
    booleans = {
        "world_seen": _row_boolean(row, "world_id", seen_metadata["seen_world_ids"]),
        "asset_seen": _row_boolean(
            row, "asset_family", seen_metadata["seen_asset_families"]
        ),
        "geometry_seen": _row_boolean(
            row, "geometry_family", seen_metadata["seen_geometry_families"]
        ),
        "material_seen": _row_boolean(
            row, "material_id", seen_metadata["seen_material_ids"]
        ),
        "lighting_seen": _row_boolean(
            row, "lighting_family", seen_metadata["seen_lighting_families"]
        ),
        "trajectory_seen": _row_boolean(
            row, "trajectory_id", seen_metadata["seen_trajectory_ids"]
        ),
        "negative_only": bool(row.get("negative_only", False)),
        "negative_asset_unseen": _negative_asset_unseen(
            row, seen_metadata["seen_negative_asset_families"]
        ),
    }
    applicable = []
    for diagnosis_id in REQUIRED_DIAGNOSIS_IDS:
        row_filter = config["diagnoses"][diagnosis_id]["filter"]
        if all(booleans[key] is value for key, value in row_filter.items()):
            applicable.append(diagnosis_id)
    return applicable


def _negative_asset_unseen(row: Mapping[str, object], seen_families: list) -> bool:
    family = row.get("negative_asset_family")
    return bool(family) and family not in set(seen_families)


def validate_factorized_metrics_report(
    report: Mapping[str, object],
    config: Mapping[str, object],
) -> list[str]:
    """Validate a factorized metrics report against the D1-D6 contract.

    Returns a list of error strings; an empty list means the report schema is
    valid.  Metric values may be null only when explicitly set to null; any
    non-null value must be a finite number.
    """
    errors: list[str] = []
    if not isinstance(report, dict):
        return ["factorized metrics report must be a mapping"]
    diagnoses = report.get("diagnoses")
    if not isinstance(diagnoses, dict):
        diagnoses = report
    for diagnosis_id in REQUIRED_DIAGNOSIS_IDS:
        if diagnosis_id not in diagnoses:
            errors.append(f"report missing diagnosis {diagnosis_id}")
            continue
        entry = diagnoses[diagnosis_id]
        if not isinstance(entry, dict):
            errors.append(f"report {diagnosis_id} must be a mapping")
            continue
        for metric in REQUIRED_METRICS:
            if metric not in entry:
                errors.append(f"report {diagnosis_id} missing metric {metric!r}")
                continue
            value = entry[metric]
            if value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append(
                    f"report {diagnosis_id} metric {metric!r} must be numeric or null"
                )
                continue
            if not math.isfinite(float(value)):
                errors.append(
                    f"report {diagnosis_id} metric {metric!r} must be finite"
                )
    return errors
