#!/usr/bin/env python3
"""Audit non-sealed Journey 6 calibration frames and second-pass ROIs.

The tool never discovers data implicitly.  It consumes an explicit, SHA-locked
record inventory and fails closed on forbidden split/path tokens, insufficient
counts, incomplete stratification, hash mismatches, or preprocessing drift.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re
from typing import Iterable

import yaml


FORBIDDEN_TOKENS = ("g5_v2", "sealed_final", "dev_val")
ROLES = ("detector_frame", "second_pass_roi")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".npy"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normalized(value: object) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value).casefold()).strip("_")


def forbidden_tokens(value: object) -> list[str]:
    normalized = _normalized(value)
    return [token for token in FORBIDDEN_TOKENS if token in normalized]


def load_document(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        value = json.loads(text)
    else:
        value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError(f"expected an object document: {path}")
    return value


def load_records(path: Path) -> list[dict]:
    if path.suffix.lower() == ".jsonl":
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    else:
        value = json.loads(path.read_text(encoding="utf-8"))
        rows = value.get("records", []) if isinstance(value, dict) else value
    if not isinstance(rows, list) or not all(isinstance(row, dict) for row in rows):
        raise ValueError("record inventory must be a JSON list or JSONL objects")
    return rows


def validate_preprocess(config: dict) -> list[str]:
    failures: list[str] = []
    preprocess = config.get("preprocess", {})
    letterbox = preprocess.get("letterbox", {})
    nv12 = preprocess.get("nv12", {})
    width = preprocess.get("input_width")
    height = preprocess.get("input_height")
    if preprocess.get("source_color") != "rgb":
        failures.append("preprocess_source_color_must_be_rgb")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        failures.append("preprocess_input_shape_invalid")
    elif width % 2 or height % 2:
        failures.append("nv12_input_dimensions_must_be_even")
    if letterbox != {
        "enabled": True,
        "preserve_aspect_ratio": True,
        "placement": "center",
        "pad_value": 114,
        "interpolation": "bilinear",
    }:
        failures.append("letterbox_contract_not_frozen")
    required_nv12 = {
        "layout": "nv12",
        "matrix": "bt601",
        "value_range": "limited",
        "chroma_order": "uv",
        "width_alignment": 2,
        "height_alignment": 2,
    }
    if nv12 != required_nv12:
        failures.append("nv12_contract_not_frozen")
    return failures


def _within(root: Path, candidate: Path) -> bool:
    try:
        candidate.relative_to(root)
        return True
    except ValueError:
        return False


def audit(
    *,
    source_config: Path,
    records_path: Path,
    data_root: Path,
    detector_minimum: int = 1000,
    second_pass_minimum: int = 1000,
) -> tuple[dict, dict, list[str]]:
    source_config = source_config.resolve()
    records_path = records_path.resolve()
    data_root = data_root.resolve()
    early_blockers = []
    for path_name, path in (("data_root", data_root), ("records", records_path), ("source_config", source_config)):
        hits = forbidden_tokens(path)
        if hits:
            early_blockers.append({"code": "forbidden_path_token", "field": path_name, "tokens": hits})
    if early_blockers:
        minimums = {"detector_frame": detector_minimum, "second_pass_roi": second_pass_minimum}
        manifest = {
            "schema_version": 1,
            "target_family": "journey6",
            "status": "blocked_external",
            "calibration_ready": False,
            "sealed_access_allowed": False,
            "forbidden_sources": ["G5_V2", "SEALED_FINAL", "DEV_VAL"],
            "source": None,
            "preprocess": None,
            "stratification": None,
            "counts": {role: 0 for role in ROLES},
            "records": [],
            "blockers": early_blockers,
            "truth_boundary": "Forbidden paths are rejected before source config, inventory, or payload content is read.",
        }
        distribution = {
            "schema_version": 1,
            "status": "blocked_external",
            "stratification_pass": False,
            "required_dimensions": [],
            "role_counts": {role: 0 for role in ROLES},
            "minimum_role_counts": minimums,
            "distribution": {role: {} for role in ROLES},
        }
        return manifest, distribution, []
    config = load_document(source_config)
    records = load_records(records_path)
    blockers: list[dict] = []

    def block(code: str, **details: object) -> None:
        blockers.append({"code": code, **details})

    source = config.get("source", {})
    if config.get("schema_version") != 1:
        block("unsupported_source_config_schema")
    if source.get("sealed_access_allowed") is not False:
        block("sealed_access_must_be_false")
    for field in ("source_id", "provenance_uri"):
        hits = forbidden_tokens(source.get(field, ""))
        if hits:
            block("forbidden_source_token", field=field, tokens=hits)
    for split in source.get("allowed_splits", []):
        hits = forbidden_tokens(split)
        if hits:
            block("forbidden_split_allowlist_token", split=split, tokens=hits)
    inventory_digest = sha256(records_path)
    expected_inventory = source.get("record_inventory_sha256")
    if not isinstance(expected_inventory, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_inventory):
        block("record_inventory_sha256_missing_or_invalid")
    elif inventory_digest != expected_inventory:
        block("record_inventory_sha256_mismatch", expected=expected_inventory, observed=inventory_digest)
    for failure in validate_preprocess(config):
        block(failure)

    stratification = config.get("stratification", {})
    dimensions = stratification.get("required_dimensions", [])
    minimum_distinct = stratification.get("minimum_distinct_values", {})
    minimum_per_value = stratification.get("minimum_per_value", {})
    if not dimensions or not all(isinstance(item, str) and item for item in dimensions):
        block("stratification_dimensions_missing")
        dimensions = []

    role_counts: Counter[str] = Counter()
    distribution: dict[str, dict[str, Counter[str]]] = defaultdict(lambda: defaultdict(Counter))
    locked_records: list[dict] = []
    checksum_rows: list[str] = []
    seen: set[tuple[str, str]] = set()
    for index, row in enumerate(records):
        role = row.get("role")
        relative_value = row.get("relative_path")
        split = row.get("split")
        if role not in ROLES:
            block("invalid_calibration_role", index=index, role=role)
            continue
        if not isinstance(relative_value, str) or not relative_value:
            block("relative_path_missing", index=index)
            continue
        if "\n" in relative_value or "\r" in relative_value:
            block("relative_path_contains_control_character", index=index)
            continue
        hits = forbidden_tokens(f"{relative_value} {split}")
        if hits:
            block("forbidden_calibration_record", index=index, tokens=hits, relative_path=relative_value)
            continue
        if split not in source.get("allowed_splits", []):
            block("calibration_split_not_allowlisted", index=index, split=split)
            continue
        relative = Path(relative_value)
        candidate = (data_root / relative).resolve()
        if relative.is_absolute() or not _within(data_root, candidate):
            block("calibration_path_escapes_root", index=index, relative_path=relative_value)
            continue
        key = (role, relative.as_posix())
        if key in seen:
            block("duplicate_calibration_record", index=index, role=role, relative_path=relative.as_posix())
            continue
        seen.add(key)
        if candidate.suffix.lower() not in IMAGE_SUFFIXES:
            block("unsupported_calibration_file_type", index=index, relative_path=relative.as_posix())
            continue
        if not candidate.is_file():
            block("calibration_file_missing", index=index, relative_path=relative.as_posix())
            continue
        observed = sha256(candidate)
        declared = row.get("sha256")
        if not isinstance(declared, str) or not re.fullmatch(r"[0-9a-f]{64}", declared):
            block("calibration_file_sha256_missing", index=index, relative_path=relative.as_posix())
            continue
        if declared != observed:
            block("calibration_file_sha256_mismatch", index=index, relative_path=relative.as_posix(), expected=declared, observed=observed)
            continue
        strata = row.get("strata")
        if not isinstance(strata, dict) or any(not isinstance(strata.get(dimension), str) or not strata[dimension] for dimension in dimensions):
            block("calibration_strata_incomplete", index=index, relative_path=relative.as_posix())
            continue
        role_counts[role] += 1
        for dimension in dimensions:
            distribution[role][dimension][strata[dimension]] += 1
        locked_records.append({
            "relative_path": relative.as_posix(),
            "role": role,
            "split": split,
            "sha256": observed,
            "bytes": candidate.stat().st_size,
            "strata": {dimension: strata[dimension] for dimension in dimensions},
        })
        checksum_rows.append(f"{observed}  source/{relative.as_posix()}")

    minimums = {"detector_frame": detector_minimum, "second_pass_roi": second_pass_minimum}
    for role, minimum in minimums.items():
        if role_counts[role] < minimum:
            block("calibration_count_below_minimum", role=role, observed=role_counts[role], required=minimum)
        for dimension in dimensions:
            counts = distribution[role][dimension]
            required_distinct = int(minimum_distinct.get(dimension, 1))
            required_per_value = int(minimum_per_value.get(dimension, 1))
            if len(counts) < required_distinct:
                block("calibration_stratum_distinct_values_below_minimum", role=role, dimension=dimension, observed=len(counts), required=required_distinct)
            underfilled = {value: count for value, count in counts.items() if count < required_per_value}
            if underfilled:
                block("calibration_stratum_value_underfilled", role=role, dimension=dimension, required_per_value=required_per_value, values=underfilled)

    distribution_report = {
        "schema_version": 1,
        "status": "ready" if not blockers else "blocked_external",
        "stratification_pass": not any(
            "stratum" in item["code"]
            or "strata" in item["code"]
            or item["code"] == "calibration_count_below_minimum"
            for item in blockers
        ),
        "required_dimensions": dimensions,
        "role_counts": {role: role_counts[role] for role in ROLES},
        "minimum_role_counts": minimums,
        "distribution": {
            role: {
                dimension: dict(sorted(distribution[role][dimension].items()))
                for dimension in dimensions
            }
            for role in ROLES
        },
    }
    manifest = {
        "schema_version": 1,
        "target_family": "journey6",
        "status": "ready" if not blockers else "blocked_external",
        "calibration_ready": not blockers,
        "sealed_access_allowed": False,
        "forbidden_sources": ["G5_V2", "SEALED_FINAL", "DEV_VAL"],
        "source": {
            "source_id": source.get("source_id"),
            "provenance_uri": source.get("provenance_uri"),
            "record_inventory_path": str(records_path),
            "record_inventory_sha256": inventory_digest,
            "data_root": str(data_root),
        },
        "preprocess": config.get("preprocess"),
        "stratification": stratification,
        "counts": {role: role_counts[role] for role in ROLES},
        "records": sorted(locked_records, key=lambda item: (item["role"], item["relative_path"])),
        "blockers": blockers,
        "truth_boundary": "Only explicit non-sealed calibration records are audited; no data is copied or inferred.",
    }
    return manifest, distribution_report, sorted(checksum_rows)


def write_outputs(output_dir: Path, manifest: dict, distribution: dict, source_checksums: Iterable[str], *, replace: bool = False) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "J6_CALIBRATION_MANIFEST.json"
    distribution_path = output_dir / "J6_CALIBRATION_DISTRIBUTION.json"
    sums_path = output_dir / "J6_CALIBRATION_SHA256SUMS"
    outputs = (manifest_path, distribution_path, sums_path)
    if not replace and any(path.exists() for path in outputs):
        raise FileExistsError("calibration output exists; use --replace explicitly")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    distribution_path.write_text(json.dumps(distribution, indent=2) + "\n", encoding="utf-8")
    rows = list(source_checksums)
    rows.extend((
        f"{sha256(manifest_path)}  evidence/J6_CALIBRATION_MANIFEST.json",
        f"{sha256(distribution_path)}  evidence/J6_CALIBRATION_DISTRIBUTION.json",
    ))
    sums_path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-config", type=Path, required=True)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--detector-minimum", type=int, default=1000)
    parser.add_argument("--second-pass-minimum", type=int, default=1000)
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    manifest, distribution, sums = audit(
        source_config=args.source_config,
        records_path=args.records,
        data_root=args.data_root,
        detector_minimum=args.detector_minimum,
        second_pass_minimum=args.second_pass_minimum,
    )
    write_outputs(args.output_dir, manifest, distribution, sums, replace=args.replace)
    print(json.dumps({
        "status": manifest["status"],
        "calibration_ready": manifest["calibration_ready"],
        "counts": manifest["counts"],
        "blockers": manifest["blockers"],
        "output_dir": str(args.output_dir.resolve()),
    }, indent=2))
    return 0 if manifest["calibration_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
