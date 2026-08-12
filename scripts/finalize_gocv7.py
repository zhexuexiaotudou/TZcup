#!/usr/bin/env python3
"""Create the mandatory fail-closed GOCV7 evidence bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(path: Path) -> dict:
    return {"path": path.as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)}


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--crv6-online", type=Path, required=True)
    parser.add_argument("--g6-area-gate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    trace_path = args.root / "real_gazebo_trace/GOCV7_ROOT_CAUSE_DECISION.json"
    prep_path = args.root / "ga1_prepared_v2/GOCV7_GA1_DATA_PREP.json"
    train_path = args.root / "ga1_training/GOCV7_GA1_TRAIN_REPORT.json"
    selection_path = args.root / "ga1_selection_v2.json"
    checkpoint_path = args.root / "ga1_training/best_coco_bbox_mAP_epoch_6.pth"
    inputs = [trace_path, prep_path, train_path, selection_path, checkpoint_path, args.crv6_online, args.g6_area_gate]
    trace, prep, train, selection, crv6, area = (
        load(trace_path), load(prep_path), load(train_path), load(selection_path),
        load(args.crv6_online), load(args.g6_area_gate),
    )
    if prep["repository_commit"] != train["repository_commit"]:
        raise RuntimeError("GA1 prep/training source commit mismatch")
    if train["best_checkpoint_sha256"] != sha256(checkpoint_path):
        raise RuntimeError("GA1 checkpoint/report hash mismatch")
    if selection["checkpoint_sha256"] != sha256(checkpoint_path):
        raise RuntimeError("GA1 selection/checkpoint hash mismatch")
    if selection.get("existing_24_mission_read_before_selection_freeze") is not False:
        raise RuntimeError("formal 24-mission boundary was violated")
    detector_pass = selection.get("GOCV7_GA1_HOLDOUT_PASS") is True
    if detector_pass:
        raise RuntimeError("blocked finalizer is only valid after GA1 exhaustion")
    area_metrics = area["cross_world_aggregate"]["area"]
    old_map = crv6["sections"]["tracker_map"]["metrics"]
    status = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "source_commit": args.source_commit,
        "P0_P1_P2_exact_parity": trace["P0_P1_P2_exact_parity"],
        "RUNTIME_CONTRACT_BUG": trace["RUNTIME_CONTRACT_BUG"],
        "GA1_REQUIRED": trace["GA1_REQUIRED"],
        "GA1_PREP_PASS": prep["GA1_PREP_PASS"],
        "GA1_CHECKPOINT_SHA256": train["best_checkpoint_sha256"],
        "GOCV7_GA1_HOLDOUT_PASS": False,
        "GOCV7_DETECTOR_GAZEBO_PASS": False,
        "GOCV7_TRACKER_PASS": False,
        "GOCV7_DYNAMIC_MAP_PASS": False,
        "GOCV7_AREA_PASS": area["OPRV3_06_AREA_PASS"],
        "GOCV7_X86_DEV_PASS": False,
        "GOCV7_PERFORMANCE_PASS": False,
        "MODEL_FREEZE_X86_CREATED": False,
        "G5_V2_read": False,
        "G5_V2_PASS": False,
        "DYNAMIC_MAP_30SEED_PASS": False,
        "SPOT_CLEAN_PRODUCT_PASS": False,
        "SOAK_2H_PASS": False,
        "MCAP_REPLAY_PASS": False,
        "RELEASE_BUNDLE_PASS": False,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "MODEL_BLOCKED_INTERNAL": True,
        "PR_READY_ALLOWED": False,
        "deployment_allowed": False,
    }
    blockers = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "source_commit": args.source_commit,
        "stopping_rule": "unique_GA1_fine_tune_and_bounded_threshold_repair_exhausted",
        "internal": [{
            "stage": "GOCV7-01",
            "failed_gates": ["eventual_correct_class_recall", "small_target_correct_recall", "actionable_precision", "wrong_actionable_rate"],
            "selected_threshold": selection["selected_threshold"],
            "selected_metrics": selection["selected_metrics"],
        }],
        "locked_not_run": ["formal_24_mission_GOCV7", "performance", "x86_freeze", "G5_V2", "30_seed_dynamic_map", "spot_cleaning", "2h_soak", "MCAP_replay", "release", "Ready/Merge", "deployment"],
        "minimum_next_research": "A new protocol and independent real-Gazebo TRAIN/HOLDOUT pack focused on unmatched actionable false positives and small paper, because the prompt-authorized single GA1 fine-tune is exhausted.",
        "external_after_internal_closure": ["J6 board/toolchain acceptance", "authorized calibrated field RGB-D and independent map GT"],
    }
    registry = {
        "schema_version": 1, "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "models": [
            {"id": "MA1", "sha256": trace["checkpoint_sha256"], "status": "real_gazebo_native_failed"},
            {"id": "GA1", "checkpoint": record(checkpoint_path), "initial_model": "MA1", "selection": record(selection_path), "holdout_pass": False, "frozen_for_product": False},
            {"id": "G6_AREA", "shared_checkpoint_sha256": area["models"]["leaf"]["shared_training_checkpoint_sha256"], "gate": record(args.g6_area_gate), "cross_world_metrics": area_metrics, "frozen_for_gocv7_development": True},
        ],
    }
    release = {
        "schema_version": 1, "protocol": "GAZEBO-ONLINE-CLOSURE-V7", "source_commit": args.source_commit,
        "release_created": False, "release_zip": None, "freeze_manifest": None,
        "reason": "GOCV7_GA1_HOLDOUT_PASS is false", "rollback_point": "589a52b", "G5_V2_read": False,
    }
    answers = {
        "1": "MA1 overfit the G7-MOVING rendering distribution; real Gazebo trace attributes eight targets to score calibration mismatch and two to image-domain shift.",
        "2": trace["P0_native_metrics"],
        "3": False,
        "4": "Yes; the unique GA1 fine-tune ran and remained blocked on HOLDOUT.",
        "5": "not rerun; detector prerequisite failed",
        "6": old_map["discrete_product_target_precision"],
        "7": old_map["discrete_map_coverage"],
        "8": area_metrics["boundary_f1"],
        "9": area_metrics["negative_area_fp_per_frame"],
        "10": "not run; GA1 HOLDOUT failed before formal access",
        "11": "not run",
        "12": None,
        "13": "not read",
        "14": "not run",
        "15": "not run",
        "16": "not run",
        "17": "not run",
        "18": "not run",
        "19": None,
        "20": False,
        "21": "Draft",
        "22": "No; detector remains an internal blocker, in addition to later external J6/field gates.",
    }
    args.output.mkdir(parents=True)
    paths = {
        "status": args.output / "PERCEPTION_GOCV7_FINAL_STATUS.json",
        "blockers": args.output / "PERCEPTION_GOCV7_FINAL_BLOCKERS.json",
        "registry": args.output / "PERCEPTION_GOCV7_MODEL_REGISTRY.json",
        "release": args.output / "PERCEPTION_GOCV7_RELEASE_MANIFEST.json",
    }
    for name, payload in (("status", status), ("blockers", blockers), ("registry", registry), ("release", release)):
        write_json(paths[name], payload)
    report_path = args.output / "GAZEBO_ONLINE_CLOSURE_V7_REPORT.md"
    lines = ["# GAZEBO ONLINE CLOSURE V7 REPORT", "", "GOCV7 stopped fail-closed after the unique GA1 fine-tune and bounded threshold repair failed the world-isolated HOLDOUT gate.", "", "## Mandatory answers", ""]
    for key, value in answers.items():
        rendered = json.dumps(value, ensure_ascii=False, sort_keys=True) if isinstance(value, (dict, list, bool)) or value is None else str(value)
        lines.append(f"{key}. {rendered}")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    generated = [*paths.values(), report_path]
    index_path = args.output / "PERCEPTION_GOCV7_EVIDENCE_INDEX.md"
    index_lines = ["# PERCEPTION GOCV7 EVIDENCE INDEX", "", f"Source commit: `{args.source_commit}`", "", "| Artifact | Bytes | SHA-256 |", "|---|---:|---|"]
    for path in [*inputs, *generated]:
        item = record(path)
        index_lines.append(f"| `{item['path']}` | {item['bytes']} | `{item['sha256']}` |")
    index_path.write_text("\n".join(index_lines) + "\n", encoding="utf-8")
    print(json.dumps({"output": args.output.as_posix(), "files": 6, "simulation_complete": False}, indent=2))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
