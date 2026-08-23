#!/usr/bin/env python3
"""Run the one bounded EMFJ6V3 classifier non-training diagnostic grid.

This evaluator consumes already-produced C1/C4 TRAIN smoke probabilities.  It
does not load a model, train, select a threshold, or authorize later gates.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
TARGET_CLASSES = PRODUCT_CLASSES[1:]
MODEL_CONTRACTS = {
    "c1_wastewise_yolov8n_cls": {
        "model_sha256": "2b46d491091dbc0ed98a0f1eaee7fe5739c8fd3eb5bd5935396c3b2712e1f7a6",
        "artifacts": None,
        "class_order": [
            "battery",
            "biological",
            "cardboard",
            "glass",
            "metal",
            "paper",
            "plastic",
            "trash",
        ],
    },
    "c4_prithiv_trash_net_siglip2": {
        "model_sha256": None,
        "artifacts": {
            "config.json": "341bb75a50a7dbd13034e189a02ad7cf54e8a6af28357d66c138a397e0d28c6e",
            "model.safetensors": "a67e2f6a82914be03cfb85218bc4e7683c8e81fe3fc4a5f9bed3abc8e93757c8",
            "preprocessor_config.json": "9b36b57ebaf20f09bf4c22100ccc21877ea6bfe5aead0c00c59f8af8ccefacfc",
        },
        "class_order": ["cardboard", "glass", "metal", "paper", "plastic", "trash"],
    },
}
SUPPORTED_MODELS = set(MODEL_CONTRACTS)
EXPECTED_CLASS_MAPPING = {
    "metal": "metal_can",
    "paper": "paper_litter",
    "plastic": "plastic_bottle",
}
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "2f226ed4925779348e218c49c16d0b33ba719958289e493948fd5427e94e3166"
)

# One deliberately small, immutable TRAIN-only grid.  These constants are part
# of the evidence contract; callers cannot expand or override them from CLI.
TARGET_PROBABILITY_MINIMUMS = (0.05, 0.10, 0.20, 0.30, 0.40)
MAXIMUM_UNKNOWN_PROBABILITIES = (0.50, 0.65, 0.80, 0.90, 1.00)

FORBIDDEN_DATA_PATTERN = re.compile(
    r"(?:^|[^A-Z0-9])(?:G5(?:_V2)?|VAL_NEW|DEV_VAL|SEALED)(?:$|[^A-Z0-9])",
    re.IGNORECASE,
)


class EvaluationError(ValueError):
    """Raised when an input violates the fixed diagnostic contract."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reject_forbidden_data(value: str, *, field: str) -> None:
    normalized_words = re.sub(r"[^A-Z0-9]+", "_", value.upper()).split("_")
    if FORBIDDEN_DATA_PATTERN.search(value) or any(
        word.startswith("G5V2") or word == "G5" for word in normalized_words
    ):
        raise EvaluationError(f"forbidden evaluation data marker in {field}")


def _validate_probability(value: Any, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvaluationError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or result > 1.0:
        raise EvaluationError(f"{field} must be finite and in [0, 1]")
    return result


def load_smoke(path: Path) -> dict[str, Any]:
    reject_forbidden_data(str(path.resolve()), field="input path")
    raw = path.read_text(encoding="utf-8")
    reject_forbidden_data(raw, field="input payload")
    payload = json.loads(raw)
    if payload.get("protocol_id") != "EMFJ6V3":
        raise EvaluationError("input protocol_id must be EMFJ6V3")
    if payload.get("schema_version") != 1:
        raise EvaluationError("input schema_version must be 1")
    model_id = payload.get("model_id")
    if model_id not in SUPPORTED_MODELS:
        raise EvaluationError(f"unsupported classifier smoke model_id: {model_id!r}")
    contract = MODEL_CONTRACTS[model_id]
    if payload.get("model_sha256") != contract["model_sha256"]:
        raise EvaluationError(f"{model_id} model SHA-256 contract mismatch")
    if payload.get("artifacts") != contract["artifacts"]:
        raise EvaluationError(f"{model_id} artifact SHA-256 contract mismatch")
    rows = payload.get("rows")
    if not isinstance(rows, list) or not rows:
        raise EvaluationError("classifier smoke rows must be a non-empty list")
    class_order = payload.get("class_order")
    class_mapping = payload.get("class_mapping")
    if class_order != contract["class_order"]:
        raise EvaluationError(f"{model_id} native class_order contract mismatch")
    if class_mapping != EXPECTED_CLASS_MAPPING:
        raise EvaluationError(f"{model_id} product class_mapping contract mismatch")

    normalized_rows = []
    seen_ids: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise EvaluationError(f"row {index} must be an object")
        record_id = row.get("record_id")
        if not isinstance(record_id, str) or not record_id.startswith("G10_TRAIN:"):
            raise EvaluationError("only fixed G10_TRAIN smoke rows are allowed")
        if record_id in seen_ids:
            raise EvaluationError(f"duplicate record_id: {record_id}")
        seen_ids.add(record_id)
        source_image_sha256 = row.get("source_image_sha256")
        if not isinstance(source_image_sha256, str) or not re.fullmatch(
            r"[0-9a-f]{64}", source_image_sha256
        ):
            raise EvaluationError(
                f"row {record_id} has an invalid source image SHA-256"
            )
        bbox = row.get("bbox_xyxy")
        if (
            not isinstance(bbox, list)
            or len(bbox) != 4
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in bbox
            )
        ):
            raise EvaluationError(f"row {record_id} has an invalid bbox_xyxy")
        relative_file = row.get("relative_file")
        if not isinstance(relative_file, str) or not relative_file:
            raise EvaluationError(f"row {record_id} has an invalid relative_file")
        reject_forbidden_data(relative_file, field=f"{record_id}.relative_file")
        actual = row.get("actual_product_class")
        if actual not in PRODUCT_CLASSES:
            raise EvaluationError(
                f"row {record_id} has an invalid actual product class"
            )
        probabilities = row.get("probabilities")
        if not isinstance(probabilities, dict) or set(probabilities) != set(
            class_order
        ):
            raise EvaluationError(
                f"row {record_id} probabilities do not match class_order"
            )
        normalized_probabilities = {
            name: _validate_probability(
                probabilities[name], field=f"{record_id}.{name}"
            )
            for name in class_order
        }
        total = sum(normalized_probabilities.values())
        if not math.isclose(total, 1.0, rel_tol=0.0, abs_tol=1e-4):
            raise EvaluationError(f"row {record_id} probabilities must sum to 1")
        target_probabilities = {
            product_class: normalized_probabilities[source_class]
            for source_class, product_class in class_mapping.items()
        }
        normalized_rows.append(
            {
                "record_id": record_id,
                "actual_product_class": actual,
                "identity": {
                    "record_id": record_id,
                    "source_image_sha256": source_image_sha256,
                    "bbox_xyxy": [float(value) for value in bbox],
                    "relative_file": relative_file,
                    "actual_product_class": actual,
                },
                "target_probabilities": target_probabilities,
                "unknown_probability": 1.0 - sum(target_probabilities.values()),
            }
        )
    source_manifest_sha256 = payload.get("source_manifest_sha256")
    if source_manifest_sha256 != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise EvaluationError(
            "input source manifest SHA-256 does not match the fixed crop bank"
        )
    return {
        "model_id": model_id,
        "source_manifest_sha256": source_manifest_sha256,
        "source_file_sha256": sha256(path),
        "rows": normalized_rows,
    }


def predict(
    row: dict[str, Any],
    *,
    target_probability_minimum: float,
    maximum_unknown_probability: float,
) -> str:
    target_class, target_probability = max(
        row["target_probabilities"].items(), key=lambda item: (item[1], item[0])
    )
    if (
        target_probability < target_probability_minimum
        or row["unknown_probability"] > maximum_unknown_probability
    ):
        return "background"
    return target_class


def classification_metrics(truth: list[str], predicted: list[str]) -> dict[str, Any]:
    if not truth or len(truth) != len(predicted):
        raise EvaluationError("truth and predicted must be non-empty and aligned")
    confusion = {
        actual: {choice: 0 for choice in PRODUCT_CLASSES} for actual in PRODUCT_CLASSES
    }
    for actual, choice in zip(truth, predicted):
        if actual not in confusion or choice not in confusion[actual]:
            raise EvaluationError("metric input contains an unknown product class")
        confusion[actual][choice] += 1
    per_class: dict[str, dict[str, float | int]] = {}
    for class_name in PRODUCT_CLASSES:
        true_positive = confusion[class_name][class_name]
        false_positive = sum(
            confusion[actual][class_name]
            for actual in PRODUCT_CLASSES
            if actual != class_name
        )
        false_negative = sum(
            confusion[class_name][choice]
            for choice in PRODUCT_CLASSES
            if choice != class_name
        )
        precision = true_positive / max(true_positive + false_positive, 1)
        recall = true_positive / max(true_positive + false_negative, 1)
        f1 = 2.0 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_name] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "support": sum(confusion[class_name].values()),
        }
    return {
        "confusion": confusion,
        "per_class": per_class,
        "macro_f1": sum(float(per_class[name]["f1"]) for name in PRODUCT_CLASSES)
        / len(PRODUCT_CLASSES),
        "target_macro_f1": sum(float(per_class[name]["f1"]) for name in TARGET_CLASSES)
        / len(TARGET_CLASSES),
        "background_specificity": float(per_class["background"]["recall"]),
    }


def evaluate_smoke(smoke: dict[str, Any]) -> list[dict[str, Any]]:
    truth = [row["actual_product_class"] for row in smoke["rows"]]
    candidates = []
    for target_minimum in TARGET_PROBABILITY_MINIMUMS:
        for unknown_maximum in MAXIMUM_UNKNOWN_PROBABILITIES:
            predicted = [
                predict(
                    row,
                    target_probability_minimum=target_minimum,
                    maximum_unknown_probability=unknown_maximum,
                )
                for row in smoke["rows"]
            ]
            candidates.append(
                {
                    "candidate_id": (
                        f"{smoke['model_id']}:target_min={target_minimum:.2f}:"
                        f"unknown_max={unknown_maximum:.2f}"
                    ),
                    "model_id": smoke["model_id"],
                    "target_probability_minimum": target_minimum,
                    "maximum_unknown_probability": unknown_maximum,
                    "metrics": classification_metrics(truth, predicted),
                    "diagnostic_only": True,
                    "selected": False,
                    "frozen": False,
                }
            )
    return candidates


def upper_bound_summary(candidates: list[dict[str, Any]]) -> dict[str, Any]:
    by_model: dict[str, list[dict[str, Any]]] = {}
    for candidate in candidates:
        by_model.setdefault(candidate["model_id"], []).append(candidate)

    def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
        maximum = max(row["metrics"]["macro_f1"] for row in rows)
        return {
            "maximum_train_macro_f1": maximum,
            "candidate_ids_at_maximum": sorted(
                row["candidate_id"]
                for row in rows
                if math.isclose(row["metrics"]["macro_f1"], maximum, abs_tol=1e-12)
            ),
            "maximum_train_target_macro_f1": max(
                row["metrics"]["target_macro_f1"] for row in rows
            ),
            "maximum_train_background_specificity": max(
                row["metrics"]["background_specificity"] for row in rows
            ),
            "not_selection_evidence": True,
        }

    return {
        "per_model": {
            model_id: summarize(rows) for model_id, rows in sorted(by_model.items())
        },
        "global": summarize(candidates),
        "interpretation": (
            "Optimistic TRAIN-only observed maxima across the one fixed grid; metrics may come "
            "from different candidates and cannot select or freeze a threshold."
        ),
    }


def evaluate_files(paths: list[Path]) -> dict[str, Any]:
    if not paths:
        raise EvaluationError("both C1/C4 smoke inputs are required")
    smokes = [load_smoke(path) for path in paths]
    model_ids = [smoke["model_id"] for smoke in smokes]
    if len(model_ids) != len(set(model_ids)):
        raise EvaluationError("each supported model may be supplied only once")
    if set(model_ids) != SUPPORTED_MODELS:
        raise EvaluationError(
            "the bounded diagnostic requires exactly one C1 and one C4 input"
        )
    identity_sets = [
        sorted(
            (row["identity"] for row in smoke["rows"]),
            key=lambda identity: identity["record_id"],
        )
        for smoke in smokes
    ]
    if identity_sets[0] != identity_sets[1]:
        raise EvaluationError("C1 and C4 crop identities are not exactly aligned")
    source_hashes = {smoke["source_manifest_sha256"] for smoke in smokes}
    if None in source_hashes or len(source_hashes) != 1:
        raise EvaluationError("all inputs must bind the same source manifest SHA-256")
    candidates = [candidate for smoke in smokes for candidate in evaluate_smoke(smoke)]
    return {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "stage": "A5_NONTRAINING_ADJUSTMENT_TRAIN_DIAGNOSTIC",
        "source_split": "G10_TRAIN",
        "source_manifest_sha256": next(iter(source_hashes)),
        "inputs": [
            {
                "model_id": smoke["model_id"],
                "source_file_sha256": smoke["source_file_sha256"],
                "record_count": len(smoke["rows"]),
            }
            for smoke in smokes
        ],
        "grid_contract": {
            "target_probability_minimums": list(TARGET_PROBABILITY_MINIMUMS),
            "maximum_unknown_probabilities": list(MAXIMUM_UNKNOWN_PROBABILITIES),
            "candidate_count_per_model": (
                len(TARGET_PROBABILITY_MINIMUMS) * len(MAXIMUM_UNKNOWN_PROBABILITIES)
            ),
            "decision": (
                "Accept the highest mapped target only when its source probability meets the "
                "minimum and total unmapped probability does not exceed the maximum; otherwise "
                "return background."
            ),
            "cli_grid_override_allowed": False,
        },
        "candidates": candidates,
        "upper_bound_summary": upper_bound_summary(candidates),
        "threshold_selection": {
            "selected": False,
            "frozen": False,
            "reason": (
                "A5 threshold selection requires an untouched complete HOLDOUT; the current "
                "HOLDOUT has no plastic_bottle examples."
            ),
        },
        "diagnostic_only": True,
        "training": False,
        "training_authorized": False,
        "forbidden_data_read": False,
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reject_forbidden_data(str(args.output.resolve()), field="output path")
    report = evaluate_files(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "models": [item["model_id"] for item in report["inputs"]],
                "candidates": len(report["candidates"]),
                "threshold_selected": False,
                "training": False,
                "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
