#!/usr/bin/env python3
"""Fail-closed evaluator for ODCV5 native/adapter/product parity traces."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


EXPECTED_CLASSES = ["plastic_bottle", "metal_can", "paper_litter"]
EXPECTED_CHECKPOINT = "481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checkpoint_preflight(path: Path | None, expected_sha256: str = EXPECTED_CHECKPOINT) -> dict:
    if path is None or not path.is_file():
        return {
            "available": False,
            "expected_sha256": expected_sha256,
            "actual_sha256": None,
            "pass": False,
            "reason": "D1_B_CHECKPOINT_MISSING",
        }
    actual = sha256(path)
    return {
        "available": True,
        "path": path.as_posix(),
        "expected_sha256": expected_sha256,
        "actual_sha256": actual,
        "pass": actual == expected_sha256,
        "reason": None if actual == expected_sha256 else "D1_B_CHECKPOINT_HASH_MISMATCH",
    }


def _contract(trace: dict) -> dict:
    return {
        key: trace.get(key)
        for key in (
            "checkpoint_sha256", "class_names", "input_color_order",
            "resize", "keep_ratio", "pad", "mean", "std",
            "observation_threshold", "action_threshold", "nms", "top_k",
        )
    }


def _detection_key(item: dict) -> tuple:
    return (
        str(item["class_name"]),
        -float(item["score"]),
        *(float(value) for value in item["bbox_xyxy"]),
    )


def evaluate(manifest: dict, traces: dict[str, dict], checkpoint: dict) -> dict:
    required = ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")
    missing = [name for name in required if name not in traces]
    manifest_ids = [item["frame_id"] for item in manifest.get("frames", [])]
    manifest_ok = (
        manifest.get("positive_frames", 0) >= 100
        and manifest.get("negative_frames", 0) >= 50
        and len(manifest_ids) == len(set(manifest_ids))
        and manifest.get("exact_rgb_duplicates") == 0
        and manifest.get("selection_independent_of_model_output") is True
        and manifest.get("required_coverage_complete") is True
        and manifest.get("G5_SEALED_FINAL_read") is False
        and manifest.get("G5_V2_read") is False
    )
    contracts = {name: _contract(trace) for name, trace in traces.items()}
    contract_agreement = not missing and all(
        contracts[name] == contracts["P0_NATIVE"] for name in required[1:]
    )
    expected_contract = bool(
        not missing
        and contracts["P0_NATIVE"].get("checkpoint_sha256")
        == checkpoint.get("expected_sha256", EXPECTED_CHECKPOINT)
    )
    expected_contract &= bool(not missing and contracts["P0_NATIVE"].get("class_names") == EXPECTED_CLASSES)
    expected_contract &= bool(not missing and contracts["P0_NATIVE"].get("input_color_order") == "BGR")
    expected_contract &= bool(not missing and float(contracts["P0_NATIVE"].get("observation_threshold", -1)) == 0.05)
    expected_contract &= bool(not missing and float(contracts["P0_NATIVE"].get("action_threshold", -1)) == 0.53)

    decoded_total = 0
    decoded_equal = 0
    maximum_bbox_delta = 0.0
    maximum_score_delta = 0.0
    frame_maps = {
        name: {frame["frame_id"]: frame for frame in trace.get("frames", [])}
        for name, trace in traces.items()
    }
    frames_complete = not missing and all(set(frame_maps[name]) == set(manifest_ids) for name in required)
    if frames_complete:
        for frame_id in manifest_ids:
            native = sorted(frame_maps["P0_NATIVE"][frame_id].get("detections", []), key=_detection_key)
            for pipeline in required[1:]:
                other = sorted(frame_maps[pipeline][frame_id].get("detections", []), key=_detection_key)
                decoded_total += max(len(native), len(other), 1)
                if len(native) != len(other):
                    continue
                if not native:
                    decoded_equal += 1
                    continue
                for left, right in zip(native, other):
                    score_delta = abs(float(left["score"]) - float(right["score"]))
                    bbox_delta = max(abs(float(a) - float(b)) for a, b in zip(left["bbox_xyxy"], right["bbox_xyxy"]))
                    maximum_score_delta = max(maximum_score_delta, score_delta)
                    maximum_bbox_delta = max(maximum_bbox_delta, bbox_delta)
                    if left["class_name"] == right["class_name"] and score_delta <= 1e-6 and bbox_delta <= 1.0:
                        decoded_equal += 1
    agreement = decoded_equal / decoded_total if decoded_total else None

    product_stages = [] if missing else traces["P2_PRODUCT"].get("stage_trace", [])
    valid_correct = [item for item in product_stages if item.get("correct_class") and item.get("depth_valid")]
    projected = [item for item in valid_correct if item.get("projection_success")]
    projection_rate = len(projected) / len(valid_correct) if valid_correct else None
    projection_reasons = sorted({
        str(item.get("projection_fail_reason")) for item in valid_correct
        if not item.get("projection_success")
    })
    allowed_reasons = {"NO_VALID_DEPTH", "BAD_DEPTH_PATCH", "INVALID_TRANSFORM", "PROJECTION_OUTLIER", "COVARIANCE_TOO_HIGH"}
    reasons_valid = set(projection_reasons) <= allowed_reasons
    pass_value = all((
        checkpoint.get("pass") is True,
        manifest_ok,
        not missing,
        contract_agreement,
        expected_contract,
        frames_complete,
        agreement is not None and agreement >= 0.999,
        maximum_bbox_delta <= 1.0,
        projection_rate is not None and projection_rate >= 0.98,
        reasons_valid,
    ))
    blockers = []
    if checkpoint.get("pass") is not True:
        blockers.append(checkpoint.get("reason"))
    if missing:
        blockers.append("MISSING_PARITY_TRACES:" + ",".join(missing))
    if not manifest_ok:
        blockers.append("GOLDEN_MANIFEST_CONTRACT_FAILED")
    if not pass_value and not blockers:
        blockers.append("PARITY_NUMERIC_OR_CONTRACT_GATE_FAILED")
    return {
        "schema_version": 1,
        "protocol": "ONLINE-DOMAIN-CLOSURE-V5",
        "stage": "ODCV5-01",
        "checkpoint_preflight": checkpoint,
        "manifest_contract_pass": manifest_ok,
        "pipeline_contracts": contracts,
        "pipeline_contract_agreement": contract_agreement,
        "expected_runtime_contract": expected_contract,
        "frames_complete": frames_complete,
        "decoded_agreement": agreement,
        "maximum_bbox_delta_px": maximum_bbox_delta,
        "maximum_score_delta": maximum_score_delta,
        "valid_depth_correct_detections": len(valid_correct),
        "projection_success_count": len(projected),
        "projection_success_rate": projection_rate,
        "projection_fail_reasons": projection_reasons,
        "projection_fail_reasons_valid": reasons_valid,
        "RUNTIME_CONTRACT_BUG": (
            None if missing or checkpoint.get("pass") is not True
            else not (contract_agreement and expected_contract and agreement is not None and agreement >= 0.999)
        ),
        "ODCV5_01_PASS": pass_value,
        "training_allowed": False,
        "blockers": blockers,
        "G5_SEALED_FINAL_read": False,
        "G5_V2_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--expected-checkpoint-sha256", default=EXPECTED_CHECKPOINT)
    parser.add_argument("--trace", action="append", type=Path, default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    traces = {}
    for path in args.trace:
        trace = json.loads(path.read_text(encoding="utf-8"))
        name = trace.get("pipeline")
        if name in traces:
            raise ValueError(f"duplicate trace pipeline: {name}")
        traces[name] = trace
    report = evaluate(manifest, traces, checkpoint_preflight(args.checkpoint, args.expected_checkpoint_sha256))
    if args.expected_checkpoint_sha256 != EXPECTED_CHECKPOINT:
        report["protocol"] = "CHECKPOINT-RECONSTITUTION-V6"
        report["stage"] = "CRV6-03"
        report["CRV6_GOLDEN_PARITY_PASS"] = report["ODCV5_01_PASS"]
    report["inputs"] = {
        "manifest": {"path": args.manifest.as_posix(), "sha256": sha256(args.manifest)},
        "traces": [{"path": path.as_posix(), "sha256": sha256(path)} for path in args.trace],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["ODCV5_01_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
