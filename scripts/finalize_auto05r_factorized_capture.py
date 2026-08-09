#!/usr/bin/env python3
"""Finalize one D1-D5 native capture without misclaiming the formal G4 gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(PACKAGE))

from sanitation_learning.g4_qa import finalize_g4_dataset  # noqa: E402


ESSENTIAL_GATES = (
    "annotation_completeness_100_percent",
    "four_sensor_sync_100_percent",
    "camera_info_valid_100_percent",
    "tf_valid_100_percent",
    "semantic_instance_error_zero",
    "scene_pose_reset_contract_100_percent",
    "manifest_pixel_target_consistency_100_percent",
    "declared_target_sequence_visibility_100_percent",
)
G4_CONTRACT = PACKAGE / "config" / "auto05r_g4_contract.yaml"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finalize_factorized_capture(data_root: Path, output: Path, role: str) -> dict:
    plan_path = data_root / "factorized_capture_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("role") != role:
        raise RuntimeError(f"capture plan role mismatch: {plan.get('role')} != {role}")
    raw_output = output / "raw_g4_qa"
    raw = finalize_g4_dataset(
        data_root,
        raw_output,
        contract_path=G4_CONTRACT,
        strict=False,
    )
    frames_path = raw_output / "g4_frame_manifest.jsonl"
    frames = [
        json.loads(line) for line in frames_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    scenes = sorted((data_root / "scenes").glob("scene_*"))
    factor_errors: list[str] = []
    positive_scenes = 0
    for scene_dir in scenes:
        scene = json.loads(
            (scene_dir / "scene_manifest.json").read_text(encoding="utf-8")
        )
        factor = scene.get("factorized_diagnostic", {})
        if scene.get("split") != role or factor.get("role") != role:
            factor_errors.append(f"{scene_dir.name}: role mismatch")
        if factor.get("single_factor_capture") is not True:
            factor_errors.append(f"{scene_dir.name}: not single-factor capture")
        positives = [item for item in scene.get("objects", []) if item.get("semantic_label")]
        positive_scenes += int(bool(positives))
        if role == "D5":
            if positives or scene.get("negative_only") is not True:
                factor_errors.append(f"{scene_dir.name}: D5 is not negative-only")
            if any(item.get("split_eligibility") != ["val"] for item in scene.get("objects", [])):
                factor_errors.append(f"{scene_dir.name}: D5 contains seen negatives")
        elif positives:
            expected = "val" if role == "D1" else "train"
            if any(item.get("split_eligibility") != [expected] for item in positives):
                factor_errors.append(
                    f"{scene_dir.name}: {role} positive asset source mismatch"
                )
    if role != "D5" and positive_scenes == 0:
        factor_errors.append(f"{role}: no positive diagnostic scenes")
    if len(scenes) != 10:
        factor_errors.append(f"{role}: expected 10 scenes, got {len(scenes)}")
    if len(frames) != 100:
        factor_errors.append(f"{role}: expected 100 frames, got {len(frames)}")
    if any(row.get("split") != role for row in frames):
        factor_errors.append(f"{role}: frame manifest role mismatch")
    essential = {
        name: bool(raw.get("gates", {}).get(name, False)) for name in ESSENTIAL_GATES
    }
    passed = (
        not raw.get("errors")
        and not factor_errors
        and all(essential.values())
        and len(scenes) == 10
        and len(frames) == 100
    )
    report = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "role": role,
        "factorized_diagnostic_pass": passed,
        "formal_G4_gate_claimed": False,
        "scene_count": len(scenes),
        "frame_count": len(frames),
        "positive_scene_count": positive_scenes,
        "essential_gates": essential,
        "raw_qa_errors": raw.get("errors", []),
        "factor_contract_errors": factor_errors,
        "capture_plan_sha256": _sha256(plan_path),
        "frame_manifest_sha256": _sha256(frames_path),
        "instance_records_sha256": _sha256(
            raw_output / "g4_instance_records.jsonl"
        ),
    }
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "factorized_diagnostic_qa.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if not passed:
        raise RuntimeError(
            f"{role} factorized diagnostic QA failed: "
            + "; ".join(factor_errors or [str(raw.get('errors', []))])
        )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--role", required=True, choices=("D1", "D2", "D3", "D4", "D5"))
    args = parser.parse_args()
    print(json.dumps(finalize_factorized_capture(args.data_root, args.output, args.role), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
