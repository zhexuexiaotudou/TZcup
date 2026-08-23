#!/usr/bin/env python3
"""Evaluate the SHA-locked C1/C3 classifier HOLDOUT inference grid.

This is an offline development evaluator.  It consumes raw native-class
probabilities and never loads/trains a model.  A selected point, when one
exists, is only a classifier development threshold; it is not evidence for
tracking, CLEAN_NOW, runtime stability, a functional candidate, or a product
candidate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[1]
PRODUCT_ACCEPTANCE_PATH = ROOT / "config/product_acceptance_v1.json"
PRODUCT_ACCEPTANCE_SHA256 = (
    "5b4ed908132d048e89d0f481e5180ec0ce8c5db83aef947ffe3a1cd3bd1d8a1e"
)
BACKGROUND_SPECIFICITY_MINIMUM = 0.995
DOMAIN_MANIFEST_SHA256 = "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
HOLDOUT_WORLDS = {
    "g10v15_val_w01_07_service_road",
    "g10v15_val_w02_08_mixed_curb_vegetation",
    "g10v15_val_w03_09_light_paver_pedestrian",
}

PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
TARGET_CLASSES = PRODUCT_CLASSES[1:]
MANIFEST_CLASS_NAMES = (*TARGET_CLASSES, "background_or_unknown")
MODEL_CONTRACTS = {
    "c1_wastewise_yolov8n_cls": {
        "model_sha256": "2b46d491091dbc0ed98a0f1eaee7fe5739c8fd3eb5bd5935396c3b2712e1f7a6",
        "artifact_sha256": None,
        "class_order": [
            "battery", "biological", "cardboard", "glass", "metal", "paper",
            "plastic", "trash",
        ],
    },
    "c3_vasantvohra_trashnet": {
        "model_sha256": None,
        "artifact_sha256": "013afdc86a673cb2354f4559c165301d5abda1c5878bb523a5995e483d4cc90a",
        "class_order": ["cardboard", "glass", "metal", "paper", "plastic", "trash"],
    },
}
MODEL_ORDER = tuple(MODEL_CONTRACTS)
EXPECTED_CLASS_MAPPING = {
    "metal": "metal_can",
    "paper": "paper_litter",
    "plastic": "plastic_bottle",
}

# This is the already-reviewed bounded grid.  There are intentionally no CLI
# options that can change it.
TARGET_PROBABILITY_MINIMUMS = (0.05, 0.10, 0.20, 0.30, 0.40)
MAXIMUM_UNKNOWN_PROBABILITIES = (0.50, 0.65, 0.80, 0.90, 1.00)

SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
FORBIDDEN_PATTERN = re.compile(
    r"(?:^|[^A-Z0-9])(?:G5(?:_V2)?|VAL_NEW|DEV_VAL|SEALED)(?:$|[^A-Z0-9])",
    re.IGNORECASE,
)


class EvaluationError(ValueError):
    """Raised when evidence violates the fixed HOLDOUT contract."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _sha(value: Any, *, field: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise EvaluationError(f"{field} must be a lowercase SHA-256")
    return value


def _exact_bool(value: Any, *, expected: bool, field: str) -> None:
    if type(value) is not bool or value is not expected:
        raise EvaluationError(f"{field} must be {expected}")


def reject_forbidden(value: str, *, field: str) -> None:
    normalized = re.sub(r"[^A-Z0-9]+", "_", value.upper()).split("_")
    if FORBIDDEN_PATTERN.search(value) or any(
        token == "G5" or token.startswith("G5V2") for token in normalized
    ):
        raise EvaluationError(f"forbidden data marker in {field}")


def _load_locked_json(path: Path, expected_sha256: str, *, field: str) -> tuple[dict, str]:
    reject_forbidden(str(path.absolute()), field=f"{field} path")
    _sha(expected_sha256, field=f"expected {field} SHA-256")
    if not path.is_file():
        raise EvaluationError(f"missing {field}: {path}")
    try:
        raw_bytes = path.read_bytes()
        raw = raw_bytes.decode("utf-8")
    except (OSError, UnicodeError) as exc:
        raise EvaluationError(f"cannot read {field}") from exc
    observed_sha256 = hashlib.sha256(raw_bytes).hexdigest()
    if observed_sha256 != expected_sha256:
        raise EvaluationError(f"{field} raw evidence SHA-256 contract mismatch")
    reject_forbidden(raw, field=f"{field} payload")
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise EvaluationError(f"invalid {field} JSON") from exc
    if not isinstance(payload, dict):
        raise EvaluationError(f"{field} must be a JSON object")
    return payload, observed_sha256


def _verify_acceptance_source() -> dict[str, Any]:
    # The full product contract legitimately documents sealed-final gates, so
    # the HOLDOUT forbidden-marker scan applies to evaluation evidence paths
    # and payloads, not to this immutable policy source.
    if sha256(PRODUCT_ACCEPTANCE_PATH) != PRODUCT_ACCEPTANCE_SHA256:
        raise EvaluationError("product acceptance V1 SHA-256 contract mismatch")
    payload = json.loads(PRODUCT_ACCEPTANCE_PATH.read_text(encoding="utf-8"))
    if payload.get("contract_id") != "TZCUP-SIMULATION-PRODUCT-ACCEPTANCE-V1":
        raise EvaluationError("unexpected product acceptance contract")
    matches = [
        check
        for gate in payload.get("gates", [])
        if isinstance(gate, dict)
        for check in gate.get("checks", [])
        if isinstance(check, dict) and check.get("id") == "E-07"
    ]
    if len(matches) != 1 or matches[0].get("metric") != "background_specificity":
        raise EvaluationError("product acceptance E-07 contract is missing or ambiguous")
    if matches[0].get("op") != "gte" or matches[0].get("threshold") != BACKGROUND_SPECIFICITY_MINIMUM:
        raise EvaluationError("product acceptance E-07 threshold changed")
    return matches[0]


def load_manifest(path: Path, expected_sha256: str) -> dict[str, Any]:
    payload, observed_sha256 = _load_locked_json(
        path, expected_sha256, field="HOLDOUT manifest"
    )
    expected_header = {
        "schema_version": "emfj6v3.classifier_holdout_gt.v1",
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT",
        "source_split": "G10_HOLDOUT",
    }
    for field, expected in expected_header.items():
        if payload.get(field) != expected:
            raise EvaluationError(f"HOLDOUT manifest {field} contract mismatch")
    if payload.get("g10_domain_manifest_sha256") != DOMAIN_MANIFEST_SHA256:
        raise EvaluationError("HOLDOUT domain manifest SHA-256 contract mismatch")
    if payload.get("holdout_world_ids") != sorted(HOLDOUT_WORLDS):
        raise EvaluationError("HOLDOUT world allowlist contract mismatch")
    for field, expected in (
        ("offline_gt_development_only", True),
        ("production_runtime_gt_forbidden", True),
        ("training_performed", False),
        ("threshold_selected", False),
        ("threshold_frozen", False),
        ("formal_product_evidence", False),
        ("pass", True),
    ):
        _exact_bool(payload.get(field), expected=expected, field=f"manifest.{field}")

    declared_canonical = _sha(
        payload.get("canonical_manifest_sha256"), field="manifest canonical SHA-256"
    )
    canonical_payload = dict(payload)
    canonical_payload.pop("canonical_manifest_sha256")
    if canonical_sha256(canonical_payload) != declared_canonical:
        raise EvaluationError("HOLDOUT manifest canonical SHA-256 mismatch")

    records = payload.get("records")
    if not isinstance(records, list) or not records:
        raise EvaluationError("HOLDOUT manifest records must be non-empty")
    declared_identity = _sha(
        payload.get("identity_lock_sha256"), field="manifest identity lock"
    )
    if canonical_sha256(records) != declared_identity:
        raise EvaluationError("HOLDOUT manifest identity lock mismatch")

    seen: set[str] = set()
    normalized_records = []
    observed_worlds: set[str] = set()
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise EvaluationError(f"manifest record {index} must be an object")
        record_id = record.get("record_id")
        if not isinstance(record_id, str) or not record_id.startswith("emf-holdout-"):
            raise EvaluationError(f"manifest record {index} has invalid record_id")
        if record_id in seen:
            raise EvaluationError(f"duplicate HOLDOUT record_id: {record_id}")
        seen.add(record_id)
        class_name = record.get("class_name")
        if class_name not in MANIFEST_CLASS_NAMES:
            raise EvaluationError(f"{record_id} has an invalid class_name")
        crop_sha256 = _sha(record.get("crop_sha256"), field=f"{record_id}.crop_sha256")
        identity_sha256 = _sha(
            record.get("source_identity_sha256"),
            field=f"{record_id}.source_identity_sha256",
        )
        source_identity = record.get("source_identity")
        if not isinstance(source_identity, dict):
            raise EvaluationError(f"{record_id}.source_identity must be an object")
        if source_identity.get("source_split") != "G10_HOLDOUT":
            raise EvaluationError(f"{record_id} is not G10_HOLDOUT")
        world_id = source_identity.get("world_id")
        if world_id not in HOLDOUT_WORLDS:
            raise EvaluationError(f"{record_id} world is outside the frozen HOLDOUT set")
        observed_worlds.add(world_id)
        if class_name == "background_or_unknown" and source_identity.get("negative_only") is not True:
            raise EvaluationError(f"{record_id} background is not independent negative-only")
        if class_name != "background_or_unknown" and source_identity.get("negative_only") is not False:
            raise EvaluationError(f"{record_id} target is marked negative-only")
        _exact_bool(
            record.get("offline_gt_development_only"),
            expected=True,
            field=f"{record_id}.offline_gt_development_only",
        )
        _exact_bool(
            record.get("production_runtime_eligible"),
            expected=False,
            field=f"{record_id}.production_runtime_eligible",
        )
        normalized_records.append(
            {
                "record_id": record_id,
                "class_name": class_name,
                "actual_product_class": (
                    "background" if class_name == "background_or_unknown" else class_name
                ),
                "crop_sha256": crop_sha256,
                "source_identity_sha256": identity_sha256,
            }
        )

    counts = Counter(record["class_name"] for record in normalized_records)
    declared_counts = payload.get("counts")
    if not isinstance(declared_counts, dict) or declared_counts != dict(sorted(counts.items())):
        raise EvaluationError("HOLDOUT manifest counts do not match records")
    if any(counts[name] != 60 for name in TARGET_CLASSES):
        raise EvaluationError("HOLDOUT requires exactly 60 records per target class")
    negative_frames = payload.get("negative_only_frame_count")
    if type(negative_frames) is not int or negative_frames <= 0:
        raise EvaluationError("HOLDOUT requires independent negative-only frames")
    if counts["background_or_unknown"] != negative_frames:
        raise EvaluationError("background count must equal negative-only frame count")
    if observed_worlds != HOLDOUT_WORLDS:
        raise EvaluationError("manifest records do not cover the exact frozen HOLDOUT world set")
    return {
        "file_sha256": observed_sha256,
        "canonical_manifest_sha256": declared_canonical,
        "identity_lock_sha256": declared_identity,
        "counts": dict(sorted(counts.items())),
        "g10_domain_manifest_sha256": DOMAIN_MANIFEST_SHA256,
        "holdout_world_ids": sorted(HOLDOUT_WORLDS),
        "records": sorted(normalized_records, key=lambda row: row["record_id"]),
        "negative_only_frame_count": negative_frames,
    }


def _probability(value: Any, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvaluationError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise EvaluationError(f"{field} must be finite and in [0, 1]")
    return result


def load_inference(
    path: Path,
    expected_sha256: str,
    manifest: dict[str, Any],
) -> dict[str, Any]:
    payload, observed_sha256 = _load_locked_json(
        path, expected_sha256, field="classifier HOLDOUT inference"
    )
    expected_header = {
        "schema_version": "emfj6v3.classifier_holdout_raw_inference.v1",
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_RAW_INFERENCE",
        "source_split": "G10_HOLDOUT",
    }
    for field, expected in expected_header.items():
        if payload.get(field) != expected:
            raise EvaluationError(f"raw inference {field} contract mismatch")
    for field, expected in (
        ("training_performed", False),
        ("raw_probabilities_only", True),
        ("threshold_applied", False),
        ("offline_gt_development_only", True),
        ("production_runtime_eligible", False),
    ):
        _exact_bool(payload.get(field), expected=expected, field=f"raw.{field}")

    model_id = payload.get("model_id")
    if model_id not in MODEL_CONTRACTS:
        raise EvaluationError(f"unsupported raw inference model_id: {model_id!r}")
    contract = MODEL_CONTRACTS[model_id]
    if payload.get("model_sha256") != contract["model_sha256"]:
        raise EvaluationError(f"{model_id} model SHA-256 contract mismatch")
    expected_artifact = contract["artifact_sha256"]
    artifact = payload.get("artifact")
    if expected_artifact is not None and (
        not isinstance(artifact, dict) or artifact.get("sha256") != expected_artifact
    ):
        raise EvaluationError(f"{model_id} artifact SHA-256 contract mismatch")
    if payload.get("class_order") != contract["class_order"]:
        raise EvaluationError(f"{model_id} native class_order contract mismatch")
    if payload.get("class_mapping") != EXPECTED_CLASS_MAPPING:
        raise EvaluationError(f"{model_id} product class_mapping contract mismatch")

    source = payload.get("source_manifest")
    expected_source = {
        key: manifest[key]
        for key in (
            "file_sha256", "canonical_manifest_sha256", "identity_lock_sha256",
            "g10_domain_manifest_sha256", "holdout_world_ids", "counts"
        )
    }
    if source != expected_source:
        raise EvaluationError(f"{model_id} source manifest identity/counts mismatch")

    rows = payload.get("rows")
    if not isinstance(rows, list):
        raise EvaluationError(f"{model_id} rows must be a list")
    manifest_by_id = {row["record_id"]: row for row in manifest["records"]}
    if len(rows) != len(manifest_by_id):
        raise EvaluationError(f"{model_id} row count does not match HOLDOUT manifest")
    seen: set[str] = set()
    normalized = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise EvaluationError(f"{model_id} row {index} must be an object")
        record_id = row.get("record_id")
        if record_id not in manifest_by_id or record_id in seen:
            raise EvaluationError(f"{model_id} has unknown or duplicate record_id")
        seen.add(record_id)
        identity = manifest_by_id[record_id]
        if row.get("crop_sha256") != identity["crop_sha256"]:
            raise EvaluationError(f"{model_id} {record_id} crop SHA mismatch")
        if row.get("source_identity_sha256") != identity["source_identity_sha256"]:
            raise EvaluationError(f"{model_id} {record_id} source identity mismatch")
        probabilities = row.get("probabilities")
        class_order = contract["class_order"]
        if not isinstance(probabilities, dict) or set(probabilities) != set(class_order):
            raise EvaluationError(f"{model_id} {record_id} probabilities do not match class_order")
        native = {
            name: _probability(probabilities[name], field=f"{record_id}.{name}")
            for name in class_order
        }
        if not math.isclose(sum(native.values()), 1.0, rel_tol=0.0, abs_tol=1e-4):
            raise EvaluationError(f"{model_id} {record_id} probabilities must sum to 1")
        targets = {
            product_class: native[source_class]
            for source_class, product_class in EXPECTED_CLASS_MAPPING.items()
        }
        normalized.append(
            {
                **identity,
                "target_probabilities": targets,
                "unknown_probability": 1.0 - sum(targets.values()),
            }
        )
    if seen != set(manifest_by_id):
        raise EvaluationError(f"{model_id} does not cover the exact HOLDOUT identity set")
    return {
        "model_id": model_id,
        "source_file_sha256": observed_sha256,
        "model_sha256": contract["model_sha256"],
        "artifact_sha256": contract["artifact_sha256"],
        "rows": sorted(normalized, key=lambda row: row["record_id"]),
    }


def predict(row: dict[str, Any], target_minimum: float, unknown_maximum: float) -> str:
    target_class, probability = max(
        row["target_probabilities"].items(), key=lambda item: (item[1], item[0])
    )
    if probability < target_minimum or row["unknown_probability"] > unknown_maximum:
        return "background"
    return target_class


def classification_metrics(truth: list[str], predicted: list[str]) -> dict[str, Any]:
    confusion = {
        actual: {choice: 0 for choice in PRODUCT_CLASSES} for actual in PRODUCT_CLASSES
    }
    for actual, choice in zip(truth, predicted):
        confusion[actual][choice] += 1
    per_class = {}
    for class_name in PRODUCT_CLASSES:
        tp = confusion[class_name][class_name]
        fp = sum(confusion[name][class_name] for name in PRODUCT_CLASSES if name != class_name)
        fn = sum(confusion[class_name][name] for name in PRODUCT_CLASSES if name != class_name)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2.0 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_name] = {
            "true_positive": tp,
            "false_positive": fp,
            "false_negative": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "support": sum(confusion[class_name].values()),
        }
    background_support = sum(confusion["background"].values())
    background_correct = confusion["background"]["background"]
    background_flood = sum(
        confusion["background"][name] for name in TARGET_CLASSES
    )
    specificity = background_correct / max(background_support, 1)
    return {
        "confusion": confusion,
        "per_target": {name: per_class[name] for name in TARGET_CLASSES},
        "independent_negative_only_background": {
            "support": background_support,
            "correctly_rejected_as_background": background_correct,
            "accepted_as_target_count": background_flood,
            "specificity": specificity,
            "flood_ratio": 1.0 - specificity,
        },
        "macro_f1": sum(float(per_class[name]["f1"]) for name in PRODUCT_CLASSES) / 4.0,
        "target_macro_f1": sum(float(per_class[name]["f1"]) for name in TARGET_CLASSES) / 3.0,
        "background_specificity": specificity,
    }


def _candidate(inference: dict[str, Any], target_minimum: float, unknown_maximum: float) -> dict[str, Any]:
    truth = [row["actual_product_class"] for row in inference["rows"]]
    predicted = [predict(row, target_minimum, unknown_maximum) for row in inference["rows"]]
    metrics = classification_metrics(truth, predicted)
    target_tp_positive = all(
        metrics["per_target"][name]["true_positive"] > 0 for name in TARGET_CLASSES
    )
    background_ok = metrics["background_specificity"] >= BACKGROUND_SPECIFICITY_MINIMUM
    reasons = []
    if not target_tp_positive:
        reasons.append("one_or_more_target_classes_have_zero_true_positive")
    if not background_ok:
        reasons.append("background_specificity_below_product_acceptance_v1_E-07")
    return {
        "candidate_id": (
            f"{inference['model_id']}:target_min={target_minimum:.2f}:"
            f"unknown_max={unknown_maximum:.2f}"
        ),
        "model_id": inference["model_id"],
        "target_probability_minimum": target_minimum,
        "maximum_unknown_probability": unknown_maximum,
        "metrics": metrics,
        "eligibility": {
            "each_target_true_positive_gt_zero": target_tp_positive,
            "background_specificity_gte_0_995": background_ok,
            "eligible": target_tp_positive and background_ok,
            "rejection_reasons": reasons,
        },
        "selected": False,
        "frozen": False,
        "classifier_development_only": True,
    }


def selection_key(candidate: dict[str, Any]) -> tuple:
    """Higher tuple wins; this ordering is preregistered before HOLDOUT reads."""
    metrics = candidate["metrics"]
    return (
        metrics["target_macro_f1"],
        metrics["macro_f1"],
        metrics["background_specificity"],
        candidate["target_probability_minimum"],
        -candidate["maximum_unknown_probability"],
        -MODEL_ORDER.index(candidate["model_id"]),
    )


def evaluate_files(
    manifest_path: Path,
    manifest_sha256: str,
    input_specs: list[tuple[Path, str]],
) -> dict[str, Any]:
    acceptance_e07 = _verify_acceptance_source()
    manifest = load_manifest(manifest_path, manifest_sha256)
    if len(input_specs) != 2:
        raise EvaluationError("exactly two SHA-locked C1/C3 inputs are required")
    inferences = [load_inference(path, expected, manifest) for path, expected in input_specs]
    model_ids = [item["model_id"] for item in inferences]
    if len(set(model_ids)) != 2 or set(model_ids) != set(MODEL_ORDER):
        raise EvaluationError("exactly one C1 and one C3 raw inference are required")
    inferences.sort(key=lambda item: MODEL_ORDER.index(item["model_id"]))
    if [row["record_id"] for row in inferences[0]["rows"]] != [
        row["record_id"] for row in inferences[1]["rows"]
    ]:
        raise EvaluationError("C1/C3 HOLDOUT identities are not exactly aligned")

    candidates = [
        _candidate(inference, target_minimum, unknown_maximum)
        for inference in inferences
        for target_minimum in TARGET_PROBABILITY_MINIMUMS
        for unknown_maximum in MAXIMUM_UNKNOWN_PROBABILITIES
    ]
    eligible = [candidate for candidate in candidates if candidate["eligibility"]["eligible"]]
    selected = max(eligible, key=selection_key) if eligible else None
    for candidate in candidates:
        if candidate is selected:
            candidate["selected"] = True
            candidate["frozen"] = True
            candidate["selection_disposition"] = "selected_highest_preregistered_rank"
        elif candidate["eligibility"]["eligible"]:
            candidate["selection_disposition"] = "not_selected_lower_preregistered_rank"
        else:
            candidate["selection_disposition"] = "ineligible"

    return {
        "schema_version": "emfj6v3.classifier_holdout_evaluation.v1",
        "protocol_id": "EMFJ6V3",
        "stage": "A5_CLASSIFIER_HOLDOUT_FIXED_GRID",
        "source_split": "G10_HOLDOUT",
        "evidence_role": "classifier_development_threshold_only",
        "source_manifest": {
            key: manifest[key]
            for key in (
                "file_sha256", "canonical_manifest_sha256", "identity_lock_sha256",
                "g10_domain_manifest_sha256", "holdout_world_ids", "counts",
                "negative_only_frame_count",
            )
        },
        "inputs": [
            {
                "model_id": item["model_id"],
                "raw_evidence_sha256": item["source_file_sha256"],
                "model_sha256": item["model_sha256"],
                "artifact_sha256": item["artifact_sha256"],
                "record_count": len(item["rows"]),
            }
            for item in inferences
        ],
        "grid_contract": {
            "target_probability_minimums": list(TARGET_PROBABILITY_MINIMUMS),
            "maximum_unknown_probabilities": list(MAXIMUM_UNKNOWN_PROBABILITIES),
            "candidate_count_per_model": 25,
            "candidate_count_total": 50,
            "cli_grid_override_allowed": False,
        },
        "selection_rule": {
            "preregistered_before_new_holdout_raw_read": True,
            "eligibility": [
                "each of plastic_bottle, metal_can, paper_litter has TP > 0",
                "independent negative-only background specificity >= 0.995",
            ],
            "background_rule_source": {
                "path": "config/product_acceptance_v1.json",
                "sha256": PRODUCT_ACCEPTANCE_SHA256,
                "check_id": "E-07",
                "metric": acceptance_e07["metric"],
                "operator": acceptance_e07["op"],
                "threshold": acceptance_e07["threshold"],
                "note": "traceable classifier non-flood floor; not an ActionVerifier ratio",
            },
            "eligible_ranking_high_to_low": [
                "target_macro_f1", "macro_f1", "background_specificity",
                "target_probability_minimum", "inverse_maximum_unknown_probability",
                "registry_model_order_C1_then_C3",
            ],
            "macro_f1_0_98_is_not_an_A5_eligibility_requirement": True,
            "deterministic": True,
        },
        "candidates": candidates,
        "threshold_selection": {
            "selected": selected is not None,
            "frozen": selected is not None,
            "candidate_id": selected["candidate_id"] if selected else None,
            "model_id": selected["model_id"] if selected else None,
            "reason": (
                "highest preregistered lexicographic rank among eligible candidates"
                if selected else
                "no candidate satisfies target TP and independent background non-flood eligibility"
            ),
            "scope": "classifier_development_threshold_only",
            "functional_candidate": False,
            "product_candidate": False,
        },
        "training": False,
        "training_authorized": False,
        "forbidden_data_read": False,
        "track_gate_evaluated": False,
        "clean_now_gate_evaluated": False,
        "runtime_stability_gate_evaluated": False,
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
    }


def _parse_input_sha(values: list[str]) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise EvaluationError("--input-sha256 must be MODEL_ID=SHA256")
        model_id, expected = value.split("=", 1)
        if model_id not in MODEL_CONTRACTS or model_id in parsed:
            raise EvaluationError("--input-sha256 has unknown or duplicate model_id")
        parsed[model_id] = _sha(expected, field=f"{model_id} expected raw SHA-256")
    return parsed


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    if path.exists():
        raise EvaluationError("output path already exists")
    if not path.parent.is_dir():
        raise EvaluationError("output parent must already exist")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise EvaluationError("output path appeared during atomic write")
        os.replace(temporary, path)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--input", type=Path, action="append", required=True)
    parser.add_argument("--input-sha256", action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    reject_forbidden(str(args.output.absolute()), field="output path")
    input_hashes = _parse_input_sha(args.input_sha256)

    # Identify each file from its validated payload header only after its
    # physical hash is locked by the caller.  A raw path cannot be paired with
    # an arbitrary model hash by position.
    if len(args.input) != 2 or set(input_hashes) != set(MODEL_ORDER):
        raise EvaluationError("exactly two inputs and both model SHA locks are required")
    specs = []
    for path in args.input:
        observed = sha256(path) if path.is_file() else ""
        matching = [model_id for model_id, expected in input_hashes.items() if expected == observed]
        if len(matching) != 1:
            raise EvaluationError("each input must match exactly one declared raw evidence SHA")
        specs.append((path, input_hashes[matching[0]]))
    report = evaluate_files(args.manifest, args.manifest_sha256, specs)
    write_json_atomic(args.output, report)
    print(json.dumps(report["threshold_selection"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
