#!/usr/bin/env python3
"""Summarize MRV2-00 evidence into a machine decision and evidence manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    audits = args.root / "audits"
    small = json.loads((audits / "SMALL_OBJECT_AUDIT.json").read_text(encoding="utf-8"))
    metal = json.loads((audits / "METAL_CAN_DOMAIN_AUDIT.json").read_text(encoding="utf-8"))
    area = json.loads((audits / "AREA_BOUNDARY_AUDIT.json").read_text(encoding="utf-8"))
    performance = json.loads((audits / "PERFORMANCE_BUDGET.json").read_text(encoding="utf-8"))
    grounding_path = args.root / "grounding_dino/GROUNDING_DINO_PROVENANCE.json"
    grounding = json.loads(grounding_path.read_text(encoding="utf-8"))
    partitions = small["partitions"]
    split_metal = metal["aggregate"]["split"]
    decision = {
        "schema_version": 1,
        "stage": "MRV2-00-DIAGNOSTIC-DECISION",
        "diagnosis_complete": True,
        "small_object": {
            "effective_X3_batch_frames": partitions["TRAIN_effective_X3_batch"]["frames"],
            "effective_X3_batch_small_frames": partitions["TRAIN_effective_X3_batch"]["small_lt_18_frames"],
            "effective_X3_batch_small_frame_ratio": partitions["TRAIN_effective_X3_batch"]["small_frame_ratio"],
            "VAL_raw_recall": small["top_k_threshold_effect"]["VAL"]["raw_score_0_01_top100"]["small_lt_18"]["recall"],
            "VAL_frozen_recall": small["top_k_threshold_effect"]["VAL"]["frozen_threshold_top16"]["small_lt_18"]["recall"],
            "top_k_is_primary_blocker": False,
            "primary_causes": ["5.67_percent_effective_small_frame_exposure", "no_training_augmentation", "no_P2_stride4", "score_threshold_suppression"],
        },
        "metal_can": {
            "recall_by_split": {name: record["recall"] for name, record in split_metal.items()},
            "primary_failure_mode": "score_below_threshold",
            "wrong_class_is_primary_failure": False,
            "box_localization_is_primary_failure": False,
        },
        "area": {
            "VAL_current_boundary_f1": area["splits"]["VAL"]["current"]["postprocessed_mask_boundary_f1"],
            "VAL_raw_boundary_head_f1": area["splits"]["VAL"]["current"]["raw_network_boundary_head_f1"],
            "VAL_selected_postprocess_boundary_f1": area["splits"]["VAL"]["development_selected_postprocess"]["postprocessed_mask_boundary_f1"],
            "D4_negative_area_fp_per_frame": area["splits"]["D4"]["current"]["negative_area_fp_per_frame"],
            "simple_postprocess_sufficient": False,
        },
        "performance": {
            "serial_full_frame_p95_ms": performance["end_to_end_serial_estimate_ms_p95"],
            "within_200ms_budget": performance["end_to_end_serial_estimate_ms_p95"] <= 200,
        },
        "grounding_dino": {
            "official_checkpoint_provenance_verified": grounding["GROUNDING_DINO_OFFICIAL_CHECKPOINT_PROVENANCE_VERIFIED"],
            "benchmark_executed": grounding["benchmark_executed"],
        },
        "selected_next_route": "MRV2-A",
        "MRV2_A_design": {
            "epoch_ratios": {"small_object": 0.30, "negative_only": 0.20, "metal_can": 0.15, "general": 0.35},
            "augmentation": ["target_centered_crop_scale", "TRAIN_only_instance_mask_copy_paste", "metal_photometric_highlight_shadow", "horizontal_flip"],
            "resolution": "R960_formal_with_R640_control",
            "R1280_status": "bounded_rejected_no_raw_small_recall_gain",
            "threshold": "constraint_aware_train_world_holdout_only",
        },
        "historical_X1_pass": False,
        "historical_X2_status": "BLOCKED_EXTERNAL_NETWORK_ASSET",
        "historical_X3_pass": False,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "MRV2_X86_STATIC_PASS": False,
    }
    decision_path = audits / "MRV2_00_DECISION.json"
    decision_path.write_bytes((json.dumps(decision, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    artifacts = []
    for path in sorted(args.root.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json" or path.suffix not in (".json", ".md"):
            continue
        artifacts.append({"path": path.relative_to(args.root).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)})
    (args.root / "artifact_manifest.json").write_bytes(
        (json.dumps({"schema_version": 1, "stage": "MRV2-EVIDENCE-MANIFEST", "artifacts": artifacts}, indent=2) + "\n").encode("utf-8")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
