#!/usr/bin/env python3
"""Create the immutable TGARV9 baseline and gate-provenance bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(path: Path) -> dict:
    return {"path": str(path.resolve()), "size_bytes": path.stat().st_size, "sha256": sha256(path)}


def write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--v8-root", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    status = subprocess.check_output(["git", "status", "--porcelain=v2", "--branch"], text=True)
    v8 = args.v8_root
    candidates = {
        "route_a_detector": v8 / "route_a/training/run/epoch_4.pth",
        "route_a_config": v8 / "route_a/training/run/rgdrv8_route_a_rtmdet_s.py",
        "route_b_verifier": v8 / "route_b/verifier/run/verifier.pt",
        "route_c_specialist": v8 / "route_c/specialist/run/specialist.pt",
        "v8_final_status": v8 / "final/PERCEPTION_RGDRV8_FINAL_STATUS.json",
        "v8_final_blockers": v8 / "final/PERCEPTION_RGDRV8_FINAL_BLOCKERS.json",
        "pipeline_manifest": Path("starter_ws/src/sanitation_perception/config/perception_pipeline_manifest.yaml"),
        "projection_code": Path("starter_ws/src/sanitation_perception/sanitation_perception/projection.py"),
        "tracker_code": Path("starter_ws/src/sanitation_perception/sanitation_perception/tracker_v2.py"),
        "dynamic_map_code": Path("starter_ws/src/sanitation_perception/sanitation_perception/dynamic_trash_map.py"),
        "scheduler_code": Path("starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/cleaning_task_scheduler.py"),
    }
    missing = [name for name, path in candidates.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"baseline artifacts missing: {missing}")
    registry = {name: artifact(path) for name, path in candidates.items()}
    write(args.output / "REPO_BASELINE.json", {"schema_version": 1, "protocol": "TGARV9", "source_commit": commit, "worktree_status": status.splitlines(), "expected_v8_commit": "3d1c37740cb8f978949bfcd977f51adffe6d307b", "baseline_exact": commit == "3d1c37740cb8f978949bfcd977f51adffe6d307b"})
    write(args.output / "HISTORICAL_RESULTS.json", {"schema_version": 1, "immutable": True, "RGDRV8": {"route_A_wrong_actionable": 0.022857142857142857, "route_A_required_max": 0.01, "route_B_macro_f1": 0.8838177466394646, "route_C_specialist_recall": 0.9207119741100324, "routes": {"A": "FAILED", "B": "FAILED", "C": "FAILED"}, "SIMULATION_PRODUCT_COMPLETE": False}})
    metrics = [
        ("raw_frame_detector_wrong_actionable", 0.01, "EXISTING_INTERNAL_DIAGNOSTIC", "frame"),
        ("eventual_observation_recall", 0.97, "TGARV9_DEVELOPMENT_GATE", "encounter"),
        ("eventual_correct_class_recall", 0.95, "TGARV9_DEVELOPMENT_GATE", "track"),
        ("small_eventual_correct_class_recall", 0.90, "TGARV9_DEVELOPMENT_GATE", "track"),
        ("confirmed_actionable_precision_hard_minimum", 0.95, "PRODUCT_SAFETY_GATE", "track"),
        ("confirmed_actionable_precision_preferred", 0.99, "PRODUCT_SAFETY_GATE", "track"),
        ("wrong_confirmed_actionable_rate_maximum", 0.01, "PRODUCT_SAFETY_GATE", "track"),
        ("negative_only_confirmed_actionable_rate_maximum", 0.01, "PRODUCT_SAFETY_GATE", "track"),
        ("wrong_target_cleaning_maximum", 0.0, "PRODUCT_SAFETY_GATE", "cleaning action"),
        ("false_candidate_cleaning_maximum", 0.0, "PRODUCT_SAFETY_GATE", "cleaning action"),
    ]
    write(args.output / "GATE_PROVENANCE_V9.json", {"schema_version": 1, "competition_material_search": {"paths_checked": ["STAGE_GATES.md", "PROJECT_SPEC.md", "README.md", "docs/"], "explicit_raw_single_frame_gate_found": False, "interpretation": "The repository formal G5 gate is encounter/product-chain based. The 1% raw-frame gate remains reported as an internal detector diagnostic and cannot be confused with track/action safety."}, "semantic_layers": ["RAW_FRAME_DETECTOR_DIAGNOSTIC", "TRACK_LEVEL_CONFIRMATION_GATE", "PRODUCT_ACTIONABLE_TARGET_GATE", "CLEAN_ACTION_GATE"], "metrics": [{"name": name, "threshold": threshold, "source": source, "unit_of_evaluation": unit} for name, threshold, source, unit in metrics]})
    write(args.output / "UNREAD_DATA_BOUNDARY.json", {"schema_version": 1, "VAL_NEW_read": False, "G5_V2_read": False, "formal_30_seed_read": False, "rules": ["T1/T2/T3 selection uses G9 HOLDOUT only", "VAL_NEW is opened atomically once after a route is frozen", "G5_V2 remains sealed until downstream freeze"]})
    write(args.output / "MODEL_REGISTRY_BASELINE.json", {"schema_version": 1, "artifacts": registry})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
