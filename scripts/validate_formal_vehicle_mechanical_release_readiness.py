#!/usr/bin/env python3
"""Fail-closed validation for the formal vehicle mechanical-release package."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

from generate_formal_vehicle_snapshot import SnapshotError, verify_snapshot


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/high_fidelity_vehicle/mechanical_release_readiness.yaml"
NOT_READY = "NOT_READY_FOR_MECHANICAL_MANUFACTURING_RELEASE"
BASELINE_MASS_KG = 160.007583
MASS_TOLERANCE_KG = 1.0e-6
SHA256_HEX_LENGTH = 64
REQUIRED_BOM_COLUMNS = {
    "part_id",
    "subsystem",
    "description",
    "quantity",
    "source_class",
    "make_or_buy",
    "material_or_model",
    "process_or_supplier",
    "nominal_mass_kg",
    "mass_basis",
    "mass_rollup",
    "manufacturing_release_state",
    "evidence_path",
}
REQUIRED_RELEASE_ITEMS = {
    "step_models",
    "drawings_2d",
    "gdt",
    "materials",
    "fasteners",
    "joints_welds_or_connections",
    "surface_treatment",
    "assembly_process",
    "inspection_and_quality",
    "weighing_and_inertia",
    "structural_fea",
    "waterproofing",
    "maintenance_drawings",
}
REQUIRED_CANDIDATE_IDS = {
    "project_authored_arm_pedestal_and_adapter",
    "project_authored_bodywork_shells_and_non_safety_covers",
    "project_authored_storage_frame_and_bin_structure",
    "project_authored_sensor_tower_and_mounts",
    "project_authored_cleaning_head_brackets_and_guards",
}
NON_RELEASED_BOM_STATES = {
    "reference_only_not_released",
    "design_required_not_released",
    "pending_vendor_measurement",
    "pending_owned_board_measurement",
    "nominal_baseline_allocation",
}


def _mapping(value: Any, label: str, errors: list[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        errors.append(f"{label} must be a mapping")
        return {}
    return value


def _number(value: Any, label: str, errors: list[str]) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f"{label} must be a finite number")
        return None
    result = float(value)
    if not math.isfinite(result):
        errors.append(f"{label} must be a finite number")
        return None
    return result


def _root_file(root: Path, relative: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(relative, str) or not relative:
        errors.append(f"{label} must be a non-empty repository-relative path")
        return None
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError:
        errors.append(f"{label} must not escape the repository root")
        return None
    if not candidate.is_file():
        errors.append(f"{label} is missing")
        return None
    return candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bound_hash(path: Path | None, expected: Any, label: str, errors: list[str]) -> str | None:
    if not isinstance(expected, str) or len(expected) != SHA256_HEX_LENGTH:
        errors.append(f"{label} must be a SHA-256 hex digest")
        return None
    try:
        int(expected, 16)
    except ValueError:
        errors.append(f"{label} must be a SHA-256 hex digest")
        return None
    if path is None:
        return None
    actual = _sha256(path)
    if actual != expected:
        errors.append(f"{label} does not match the current file")
    return actual


def _verify_current_snapshot(baseline: dict[str, Any], root: Path, errors: list[str]) -> dict[str, Any] | None:
    manifest_relative = baseline.get("source_snapshot_path")
    manifest_path = _root_file(root, manifest_relative, "baseline.source_snapshot_path", errors)
    _bound_hash(
        manifest_path,
        baseline.get("source_snapshot_sha256"),
        "baseline.source_snapshot_sha256",
        errors,
    )
    if baseline.get("source_snapshot_state") != "current_verified_snapshot":
        errors.append("baseline.source_snapshot_state must equal current_verified_snapshot")
    if manifest_path is None:
        return None
    try:
        manifest = verify_snapshot(root, manifest_path=manifest_path)
    except SnapshotError as exc:
        errors.append(f"current source snapshot verification failed: {exc}")
        return None

    if manifest.get("source_inventory_sha256") != baseline.get("source_inventory_sha256"):
        errors.append("baseline.source_inventory_sha256 does not match the verified current snapshot")
    if manifest.get("output_inventory_sha256") != baseline.get("output_inventory_sha256"):
        errors.append("baseline.output_inventory_sha256 does not match the verified current snapshot")

    urdf_relative = baseline.get("expanded_urdf_path")
    urdf_path = _root_file(root, urdf_relative, "baseline.expanded_urdf_path", errors)
    actual_urdf_sha = _bound_hash(
        urdf_path,
        baseline.get("expanded_urdf_sha256"),
        "baseline.expanded_urdf_sha256",
        errors,
    )
    outputs = manifest.get("outputs")
    if not isinstance(outputs, dict) or not isinstance(urdf_relative, str):
        errors.append("verified snapshot has no usable expanded-URDF output inventory")
    else:
        entry = outputs.get(urdf_relative)
        if not isinstance(entry, dict):
            errors.append("baseline.expanded_urdf_path is not an output of the verified snapshot")
        elif entry.get("sha256") != baseline.get("expanded_urdf_sha256") or entry.get("sha256") != actual_urdf_sha:
            errors.append("baseline expanded-URDF hash does not match the verified snapshot output")
    return manifest


def _csv_positive_int(value: str, label: str, errors: list[str]) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        errors.append(f"{label} must be a positive integer")
        return None
    if str(parsed) != value or parsed <= 0:
        errors.append(f"{label} must be a positive integer")
        return None
    return parsed


def _csv_mass(value: str, label: str, errors: list[str]) -> float | None:
    if value == "":
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        errors.append(f"{label} must be blank for pending mass or a finite number")
        return None
    if not math.isfinite(parsed) or parsed < 0.0:
        errors.append(f"{label} must be blank for pending mass or a finite number")
        return None
    return parsed


def _validate_bom(bom_path: Path, errors: list[str]) -> tuple[float | None, int]:
    try:
        with bom_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = set(reader.fieldnames or [])
            if fieldnames != REQUIRED_BOM_COLUMNS:
                errors.append("manufacturing BOM must contain exactly the required columns")
                return None, 0
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as exc:
        errors.append(f"cannot read manufacturing BOM: {exc}")
        return None, 0

    if not rows:
        errors.append("manufacturing BOM must contain at least one row")
        return None, 0
    part_ids: set[str] = set()
    allocated_total = 0.0
    for line, row in enumerate(rows, start=2):
        label = f"manufacturing BOM line {line}"
        part_id = row["part_id"]
        if not part_id or part_id in part_ids:
            errors.append(f"{label}.part_id must be unique and non-empty")
        part_ids.add(part_id)
        for field in (
            "subsystem",
            "description",
            "source_class",
            "make_or_buy",
            "material_or_model",
            "process_or_supplier",
            "mass_basis",
            "mass_rollup",
            "manufacturing_release_state",
            "evidence_path",
        ):
            if not row[field]:
                errors.append(f"{label}.{field} must be non-empty")
        _csv_positive_int(row["quantity"], f"{label}.quantity", errors)
        mass = _csv_mass(row["nominal_mass_kg"], f"{label}.nominal_mass_kg", errors)
        if mass is None and row["nominal_mass_kg"] == "" and not row["mass_basis"].startswith("pending_"):
            errors.append(f"{label}.mass_basis must mark an unknown mass as pending")
        if row["mass_rollup"] == "baseline_allocated":
            if mass is None:
                errors.append(f"{label}.baseline_allocated mass must be finite")
            else:
                allocated_total += mass
        elif row["mass_rollup"] != "reference_only_not_additive":
            errors.append(f"{label}.mass_rollup must be baseline_allocated or reference_only_not_additive")
        if row["manufacturing_release_state"] not in NON_RELEASED_BOM_STATES:
            errors.append(f"{label}.manufacturing_release_state must remain fail-closed")
    return allocated_total, len(rows)


def _release_evidence_is_acceptable(item: str, path_text: str, root: Path) -> bool:
    suffix = Path(path_text).suffix.lower()
    if item == "step_models":
        return suffix in {".step", ".stp"} and (root / path_text).is_file()
    if item == "drawings_2d":
        return suffix in {".pdf", ".dxf", ".dwg"} and (root / path_text).is_file()
    return bool(path_text) and (root / path_text).is_file()


def validate(config_path: Path = DEFAULT_CONFIG, *, root: Path = ROOT) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        return {"valid": False, "errors": [f"cannot read mechanical-release config: {exc}"]}

    errors: list[str] = []
    config = _mapping(payload, "mechanical-release config", errors)
    if config.get("schema_version") != 2 or isinstance(config.get("schema_version"), bool):
        errors.append("schema_version must equal integer 2")
    if config.get("status") != NOT_READY:
        errors.append(f"status must equal {NOT_READY}")
    if type(config.get("ready")) is not bool or config.get("ready") is not False:
        errors.append("ready must be boolean false")
    if config.get("nominal_mass_declaration") != "nominal_model_allocation_not_actual_weighed":
        errors.append("nominal_mass_declaration must preserve the nominal-model boundary")

    baseline = _mapping(config.get("baseline"), "baseline", errors)
    mass = _number(baseline.get("expanded_urdf_mass_kg"), "baseline.expanded_urdf_mass_kg", errors)
    if mass is not None and not math.isclose(mass, BASELINE_MASS_KG, abs_tol=MASS_TOLERANCE_KG):
        errors.append("baseline.expanded_urdf_mass_kg must equal 160.007583")
    _verify_current_snapshot(baseline, root, errors)
    if baseline.get("bom_rollup_policy") != "only_baseline_allocated_rows_sum_to_nominal_expanded_urdf_mass":
        errors.append("baseline.bom_rollup_policy must preserve non-additive reference rows")
    bom_relative = baseline.get("bom_path")
    bom_path = _root_file(root, bom_relative, "baseline.bom_path", errors)
    _bound_hash(bom_path, baseline.get("bom_sha256"), "baseline.bom_sha256", errors)
    if bom_path is None:
        bom_total, bom_row_count = None, 0
    else:
        bom_total, bom_row_count = _validate_bom(bom_path, errors)
    if bom_total is not None and mass is not None and not math.isclose(bom_total, mass, abs_tol=MASS_TOLERANCE_KG):
        errors.append("baseline allocated BOM mass must equal baseline.expanded_urdf_mass_kg")

    release_items = _mapping(config.get("release_items"), "release_items", errors)
    if set(release_items) != REQUIRED_RELEASE_ITEMS:
        errors.append("release_items must contain exactly every mechanical release item")
    else:
        for item in REQUIRED_RELEASE_ITEMS:
            row = _mapping(release_items.get(item), f"release_items.{item}", errors)
            state = row.get("state")
            evidence_path = row.get("evidence_path")
            if state != "blocked":
                errors.append(f"release_items.{item}.state must remain blocked")
            if not isinstance(evidence_path, str) or not evidence_path:
                errors.append(f"release_items.{item}.evidence_path must be non-empty")
            if state == "ready":
                if evidence_path.endswith(".stl"):
                    errors.append(f"release_items.{item} cannot use a visual STL as manufacturing evidence")
                elif not _release_evidence_is_acceptable(item, evidence_path, root):
                    errors.append(f"release_items.{item} ready evidence is missing or has the wrong manufacturing format")

    mitigation = _mapping(config.get("mass_mitigation"), "mass_mitigation", errors)
    if mitigation.get("status") != "design_required_not_credited":
        errors.append("mass_mitigation.status must equal design_required_not_credited")
    minimum = _number(mitigation.get("minimum_margin_kg"), "mass_mitigation.minimum_margin_kg", errors)
    recommended = _number(mitigation.get("recommended_margin_kg"), "mass_mitigation.recommended_margin_kg", errors)
    minimum_reduction = _number(mitigation.get("conservative_required_reduction_minimum_kg"), "mass_mitigation.conservative_required_reduction_minimum_kg", errors)
    recommended_reduction = _number(mitigation.get("conservative_required_reduction_recommended_kg"), "mass_mitigation.conservative_required_reduction_recommended_kg", errors)
    if minimum is not None and not math.isclose(minimum, 5.0, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_mitigation.minimum_margin_kg must equal 5.0")
    if recommended is not None and not math.isclose(recommended, 10.0, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_mitigation.recommended_margin_kg must equal 10.0")
    if minimum_reduction is not None and not math.isclose(minimum_reduction, 4.969583, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_mitigation.conservative_required_reduction_minimum_kg must equal 4.969583")
    if recommended_reduction is not None and not math.isclose(recommended_reduction, 9.969583, abs_tol=MASS_TOLERANCE_KG):
        errors.append("mass_mitigation.conservative_required_reduction_recommended_kg must equal 9.969583")
    if type(mitigation.get("direct_urdf_mass_edit_is_not_a_credit")) is not bool or mitigation.get("direct_urdf_mass_edit_is_not_a_credit") is not True:
        errors.append("mass_mitigation.direct_urdf_mass_edit_is_not_a_credit must be boolean true")

    platform = _mapping(config.get("platform_reselection"), "platform_reselection", errors)
    five = _number(platform.get("rated_payload_minimum_for_5kg_margin_kg"), "platform_reselection.rated_payload_minimum_for_5kg_margin_kg", errors)
    ten = _number(platform.get("rated_payload_minimum_for_10kg_margin_kg"), "platform_reselection.rated_payload_minimum_for_10kg_margin_kg", errors)
    if five is not None and not math.isclose(five, 107.02, abs_tol=0.01):
        errors.append("platform_reselection.rated_payload_minimum_for_5kg_margin_kg must equal 107.02")
    if ten is not None and not math.isclose(ten, 112.58, abs_tol=0.01):
        errors.append("platform_reselection.rated_payload_minimum_for_10kg_margin_kg must equal 112.58")
    if type(platform.get("stability_must_pass_independently")) is not bool or platform.get("stability_must_pass_independently") is not True:
        errors.append("platform_reselection.stability_must_pass_independently must be boolean true")

    return {
        "schema_version": 2,
        "status": NOT_READY,
        "ready": False,
        "valid": not errors,
        "errors": errors,
        "computed": {"baseline_allocated_bom_mass_kg": bom_total, "bom_row_count": bom_row_count},
        "claim_boundary": "Validation binds the nominal mechanical baseline to the current frozen digital snapshot while preserving a fail-closed manufacturing-release declaration; it is not fabrication approval.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    result = validate(args.config, root=args.root)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
