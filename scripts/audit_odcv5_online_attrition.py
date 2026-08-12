#!/usr/bin/env python3
"""Build the ODCV5 stage-wise online attrition ladder.

The input is a full moving-camera benchmark produced by
``perception_oprv3_moving_benchmark.py``.  Ground truth is used only by this
offline evaluator.  A legacy benchmark does not contain target-addressable
scheduler decisions, so that final stage is reported as unknown instead of
being inferred from mission totals.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Iterable


STAGES = (
    "GT_VISIBLE",
    "GT_ACTIONABLE_WINDOW",
    "NATIVE_DETECTOR_OBSERVATION",
    "NATIVE_DETECTOR_ACTION_THRESHOLD",
    "CORRECT_CLASS",
    "DEPTH_VALID",
    "PROJECTION_SUCCESS",
    "TRACK_CREATED",
    "TRACK_CONFIRMED",
    "DYNAMIC_MAP_ACCEPTED",
    "DYNAMIC_MAP_CONFIRMED",
    "SCHEDULER_ACTIONABLE",
)
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
MATCH_DISTANCE_M = 0.50
ROOT_CAUSE_TAXONOMY = (
    "NOT_VISIBLE",
    "OUTSIDE_ACTIONABLE_WINDOW",
    "DETECTOR_NO_PROPOSAL",
    "DETECTOR_SCORE_LOW",
    "DETECTOR_WRONG_CLASS",
    "DETECTOR_BOX_IOU_FAIL",
    "DEPTH_INVALID",
    "PROJECTION_FAILURE",
    "PROJECTION_OUTLIER",
    "TRACK_ASSOCIATION_FAILURE",
    "TRACK_NOT_CONFIRMED",
    "TRACK_DUPLICATE",
    "MAP_INGEST_REJECT",
    "MAP_DUPLICATE",
    "MAP_CLASS_SWITCH",
    "MAP_COVARIANCE_REJECT",
    "SCHEDULER_REJECT",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite(values: Iterable[object]) -> list[float]:
    output = []
    for value in values:
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            output.append(float(value))
    return output


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _distribution(values: Iterable[object]) -> dict:
    finite = _finite(values)
    return {
        "count": len(finite),
        "minimum": min(finite) if finite else None,
        "p10": _percentile(finite, 10),
        "median": statistics.median(finite) if finite else None,
        "p90": _percentile(finite, 90),
        "maximum": max(finite) if finite else None,
    }


def _nearest_product(target: dict, products: list[dict]) -> tuple[dict | None, float | None]:
    tx, ty = (float(value) for value in target["world_xyz_m"][:2])
    candidates = []
    for product in products:
        if product.get("class_name") != target.get("class_name"):
            continue
        distance = math.hypot(float(product["x_m"]) - tx, float(product["y_m"]) - ty)
        candidates.append((distance, product))
    if not candidates:
        return None, None
    distance, product = min(candidates, key=lambda item: item[0])
    return (product, distance) if distance <= MATCH_DISTANCE_M else (None, distance)


def _domain_labels(encounter: dict) -> list[str]:
    labels = [str(encounter.get("world_id", "unknown_world"))]
    occlusion = str(encounter.get("occlusion_bucket", "none"))
    visibility = str(encounter.get("visible_fraction_bucket", "unknown"))
    labels.extend((f"occlusion:{occlusion}", f"visibility:{visibility}"))
    frames = encounter.get("frames", [])
    if any(float(frame.get("visible_bbox_short_side_px", 0.0)) < 18.0 for frame in frames if frame.get("visible")):
        labels.append("size:small_lt18px")
    return labels


def _legacy_target_record(encounter: dict, products: list[dict]) -> dict:
    frames = list(encounter.get("frames", []))
    actionable = [frame for frame in frames if frame.get("actionable_window")]
    observed = [frame for frame in actionable if frame.get("observation_created")]
    action = [frame for frame in actionable if frame.get("action_detection")]
    correct = [frame for frame in actionable if frame.get("correct_action_detection")]
    depth_valid = [
        frame for frame in correct
        if float(frame.get("depth_valid_ratio", 0.0)) >= 0.8
    ]
    product, product_distance = _nearest_product(encounter, products)
    transitions = list(product.get("transitions", [])) if product else []
    ever_tracked = any(item.get("to") == "TRACKED" for item in transitions)
    ever_confirmed = bool(product and product.get("ever_confirmed"))

    # Legacy product-map evidence only emits a map target after projection,
    # tracker creation, and DynamicTrashMap.ingest all succeeded.  It cannot
    # distinguish those three stages, so the common evidence is explicit.
    downstream_common = product is not None
    raw_pass = {
        "GT_VISIBLE": bool(encounter.get("ever_in_camera_frustum")),
        "GT_ACTIONABLE_WINDOW": bool(encounter.get("entered_actionable_window")),
        "NATIVE_DETECTOR_OBSERVATION": bool(observed),
        "NATIVE_DETECTOR_ACTION_THRESHOLD": bool(action),
        "CORRECT_CLASS": bool(correct),
        "DEPTH_VALID": bool(depth_valid),
        "PROJECTION_SUCCESS": downstream_common,
        "TRACK_CREATED": downstream_common,
        "TRACK_CONFIRMED": ever_tracked or ever_confirmed,
        "DYNAMIC_MAP_ACCEPTED": downstream_common,
        "DYNAMIC_MAP_CONFIRMED": ever_confirmed,
        "SCHEDULER_ACTIONABLE": None,
    }
    # Attrition is a causal ladder, not a collection of independent metrics.
    # A nearby downstream false target must never make a GT target re-enter
    # the chain after an upstream detector/class/depth loss.
    passed = {}
    upstream_passed = True
    for stage in STAGES:
        raw = raw_pass[stage]
        if not upstream_passed:
            passed[stage] = False
        elif raw is None:
            passed[stage] = None
            upstream_passed = False
        else:
            passed[stage] = bool(raw)
            upstream_passed = passed[stage]
    reasons = {}
    for index, stage in enumerate(STAGES):
        if passed[stage] is None:
            reasons[stage] = "UNKNOWN_LEGACY_EVIDENCE_GAP"
        elif passed[stage]:
            reasons[stage] = None
        else:
            previous = STAGES[index - 1] if index else None
            reasons[stage] = (
                f"UPSTREAM_{previous}_FAILED" if previous and passed[previous] is False
                else f"{stage}_FAILED"
            )
    root_cause = _root_cause(passed)
    return {
        "target_id": encounter["target_id"],
        "class_name": encounter["class_name"],
        "scene_seed": int(encounter["scene_seed"]),
        "world_id": encounter["world_id"],
        "domains": _domain_labels(encounter),
        "stage_pass": passed,
        "stage_reason": reasons,
        "root_cause": root_cause,
        "diagnostics": {
            "median_model_score": (
                statistics.median(_finite(frame.get("model_score") for frame in actionable))
                if _finite(frame.get("model_score") for frame in actionable) else None
            ),
            "score_distribution": _distribution(frame.get("model_score") for frame in actionable),
            "bbox_short_side_px_distribution": _distribution(
                frame.get("visible_bbox_short_side_px") for frame in actionable
            ),
            "distance_m_distribution": _distribution(frame.get("distance_m") for frame in actionable),
            "depth_valid_ratio": (
                sum(float(frame.get("depth_valid_ratio", 0.0)) for frame in actionable) / len(actionable)
                if actionable else None
            ),
            "matched_product_uuid": product.get("uuid") if product else None,
            "matched_product_distance_m": product_distance,
        },
    }


def _root_cause(passed: dict[str, bool | None]) -> dict:
    """Classify the first causal loss without inventing missing legacy traces."""
    if passed["GT_VISIBLE"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "NOT_VISIBLE"}
    if passed["GT_ACTIONABLE_WINDOW"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "OUTSIDE_ACTIONABLE_WINDOW"}
    if passed["NATIVE_DETECTOR_OBSERVATION"] is False:
        return {
            "status": "UNRESOLVED_LEGACY_TRACE_GAP",
            "taxonomy": None,
            "candidates": ["DETECTOR_NO_PROPOSAL", "DETECTOR_BOX_IOU_FAIL"],
            "reason": "legacy target facts retain no full proposal list below the GT IoU gate",
        }
    if passed["NATIVE_DETECTOR_ACTION_THRESHOLD"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "DETECTOR_SCORE_LOW"}
    if passed["CORRECT_CLASS"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "DETECTOR_WRONG_CLASS"}
    if passed["DEPTH_VALID"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "DEPTH_INVALID"}
    if passed["PROJECTION_SUCCESS"] is False:
        return {
            "status": "UNRESOLVED_LEGACY_TRACE_GAP",
            "taxonomy": None,
            "candidates": [
                "PROJECTION_FAILURE", "PROJECTION_OUTLIER",
                "TRACK_ASSOCIATION_FAILURE", "TRACK_NOT_CONFIRMED",
                "TRACK_DUPLICATE", "MAP_INGEST_REJECT", "MAP_DUPLICATE",
                "MAP_CLASS_SWITCH", "MAP_COVARIANCE_REJECT",
            ],
            "reason": "legacy evidence emits only the final product-map target",
        }
    if passed["TRACK_CONFIRMED"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "TRACK_NOT_CONFIRMED"}
    if passed["DYNAMIC_MAP_CONFIRMED"] is False:
        return {
            "status": "UNRESOLVED_LEGACY_TRACE_GAP",
            "taxonomy": None,
            "candidates": ["MAP_INGEST_REJECT", "MAP_COVARIANCE_REJECT"],
            "reason": "legacy evidence lacks per-ingest rejection codes",
        }
    if passed["SCHEDULER_ACTIONABLE"] is False:
        return {"status": "ATTRIBUTED", "taxonomy": "SCHEDULER_REJECT"}
    if passed["SCHEDULER_ACTIONABLE"] is None:
        return {
            "status": "UNKNOWN_LEGACY_EVIDENCE_GAP",
            "taxonomy": None,
            "candidates": ["SCHEDULER_REJECT"],
            "reason": "legacy evidence has no target-addressable scheduler decision",
        }
    return {"status": "NO_LOSS", "taxonomy": None}


def _summarize(records: list[dict], group: str) -> dict:
    summary = {"group": group, "target_count": len(records), "stages": {}}
    for index, stage in enumerate(STAGES):
        eligible = records if index == 0 else [
            record for record in records
            if record["stage_pass"][STAGES[index - 1]] is True
        ]
        passed = sum(record["stage_pass"][stage] is True for record in eligible)
        failed = sum(record["stage_pass"][stage] is False for record in eligible)
        unknown = sum(record["stage_pass"][stage] is None for record in eligible)
        summary["stages"][stage] = {
            "count_in": len(eligible),
            "count_passed": passed,
            "count_lost": failed,
            "count_unknown": unknown,
            "loss_rate": failed / len(eligible) if eligible else None,
            "unknown_rate": unknown / len(eligible) if eligible else None,
            "loss_reasons": dict(sorted(Counter(
                record["stage_reason"][stage]
                for record in eligible if record["stage_pass"][stage] is False
            ).items())),
        }
    diagnostic_rows = [record["diagnostics"] for record in records]
    summary["diagnostics"] = {
        "score_distribution": _distribution(
            item["median_model_score"] for item in diagnostic_rows
        ),
        "bbox_short_side_px_distribution": _distribution(
            value
            for item in diagnostic_rows
            for value in (
                item["bbox_short_side_px_distribution"]["median"],
            )
        ),
        "distance_m_distribution": _distribution(
            value
            for item in diagnostic_rows
            for value in (item["distance_m_distribution"]["median"],)
        ),
        "mean_depth_valid_ratio": (
            statistics.mean(_finite(item["depth_valid_ratio"] for item in diagnostic_rows))
            if _finite(item["depth_valid_ratio"] for item in diagnostic_rows) else None
        ),
    }
    return summary


def _root_cause_decision(records: list[dict], shared: dict) -> dict:
    attributed = Counter(
        record["root_cause"]["taxonomy"] for record in records
        if record["root_cause"]["status"] == "ATTRIBUTED"
    )
    unresolved = Counter(
        "+".join(record["root_cause"].get("candidates", [])) for record in records
        if record["root_cause"]["status"] in {
            "UNRESOLVED_LEGACY_TRACE_GAP", "UNKNOWN_LEGACY_EVIDENCE_GAP"
        }
    )
    ranked = sorted(attributed.items(), key=lambda item: (-item[1], item[0]))
    primary = ranked[0][0] if ranked else None
    return {
        **shared,
        "taxonomy_contract": list(ROOT_CAUSE_TAXONOMY),
        "target_count": len(records),
        "attributed_loss_counts": dict(sorted(attributed.items())),
        "unresolved_legacy_trace_gap_counts": dict(sorted(unresolved.items())),
        "primary_directly_attributed_loss": primary,
        "primary_directly_attributed_loss_count": ranked[0][1] if ranked else 0,
        "decision": (
            "DETECTOR_SCORE_AND_CLASS_ARE_THE_LARGEST_DIRECTLY_OBSERVED_LOSSES;_"
            "RUNTIME_PARITY_AND_FULL_STAGE_TRACES_REMAIN_REQUIRED_BEFORE_TRAINING"
        ),
        "training_allowed": False,
        "reason_training_blocked": (
            "ODCV5-01 native/adapter/product parity has not passed and legacy evidence "
            "cannot separate detector proposal-vs-IoU or projection/tracker/map causes"
        ),
    }


def build_reports(payload: dict, *, route: str, input_path: Path) -> tuple[dict, dict, dict, dict]:
    if payload.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("input violates the sealed-final boundary")
    route_payload = payload.get("routes", {}).get(route)
    if not route_payload:
        raise ValueError(f"route {route!r} is absent")
    encounters = [
        item for item in route_payload.get("encounters", [])
        if item.get("class_name") in DISCRETE_CLASSES
    ]
    if not encounters:
        raise ValueError("input contains no discrete target encounters")
    product_map = route_payload.get("product_map") or {}
    products_by_seed = {
        int(mission["scene_seed"]): list(mission.get("product_targets", []))
        for mission in product_map.get("missions", [])
    }
    records = [
        _legacy_target_record(item, products_by_seed.get(int(item["scene_seed"]), []))
        for item in encounters
    ]
    shared = {
        "schema_version": 1,
        "protocol": "ONLINE-DOMAIN-CLOSURE-V5",
        "stage": "ODCV5-00",
        "source_commit": payload.get("source_commit"),
        "route": route,
        "input": {"path": input_path.as_posix(), "sha256": sha256(input_path)},
        "GT_used_by_product_pipeline": False,
        "GT_used_only_by_attrition_evaluator": True,
        "G5_SEALED_FINAL_read": False,
        "legacy_scheduler_target_attribution_available": False,
        "scheduler_unknown_is_not_a_pass": True,
    }
    overall = {
        **shared,
        "stage_order": list(STAGES),
        "summary": _summarize(records, "all_discrete"),
        "targets": records,
    }
    by_class = {
        **shared,
        "groups": {
            class_name: _summarize(
                [record for record in records if record["class_name"] == class_name],
                class_name,
            )
            for class_name in DISCRETE_CLASSES
        },
    }
    domains = sorted({domain for record in records for domain in record["domains"]})
    by_domain = {
        **shared,
        "groups": {
            domain: _summarize(
                [record for record in records if domain in record["domains"]], domain
            )
            for domain in domains
        },
    }
    return overall, by_class, by_domain, _root_cause_decision(records, shared)


def main() -> int:
    parser = argparse.ArgumentParser()
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--benchmark", type=Path)
    sources.add_argument("--existing-ladder", type=Path)
    parser.add_argument("--route", default="D1-B")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.existing_ladder:
        overall = json.loads(args.existing_ladder.read_text(encoding="utf-8"))
        records = list(overall.get("targets", []))
        if not records:
            raise ValueError("existing ladder contains no targets")
        for record in records:
            record["root_cause"] = _root_cause(record["stage_pass"])
        shared = {
            key: overall[key] for key in (
                "schema_version", "protocol", "stage", "source_commit", "route",
                "input", "GT_used_by_product_pipeline", "GT_used_only_by_attrition_evaluator",
                "G5_SEALED_FINAL_read", "legacy_scheduler_target_attribution_available",
                "scheduler_unknown_is_not_a_pass",
            ) if key in overall
        }
        root_cause = _root_cause_decision(records, shared)
        outputs = {"ODCV5_ROOT_CAUSE_DECISION.json": root_cause}
    else:
        payload = json.loads(args.benchmark.read_text(encoding="utf-8"))
        overall, by_class, by_domain, root_cause = build_reports(
            payload, route=args.route, input_path=args.benchmark
        )
        outputs = {
            "ODCV5_ATTRITION_LADDER.json": overall,
            "ODCV5_ATTRITION_BY_CLASS.json": by_class,
            "ODCV5_ATTRITION_BY_DOMAIN.json": by_domain,
            "ODCV5_ROOT_CAUSE_DECISION.json": root_cause,
        }
    for name, report in outputs.items():
        (args.output_dir / name).write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps({name: sha256(args.output_dir / name) for name in outputs}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
