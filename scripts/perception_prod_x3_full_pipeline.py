#!/usr/bin/env python3
"""Run the full ONLINE-X3 static gate on VAL and D1-D5 only."""

from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path
import sys
import time

import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import index_instance_records  # noqa: E402
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    MRV2_C_P2_ARCHITECTURE,
    build_direct_fcos,
    build_p2_direct_fcos,
    direct_predictions,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    area_metrics,
    area_predictions,
    discrete_metrics,
    discovery_metrics,
    match_discrete_predictions,
)
from perception_prod_x1_full_pipeline import (  # noqa: E402
    AREA_THRESHOLDS,
    aggregate_area_reports,
    candidate_size_metrics,
    combine_area,
    load_checkpoint_model,
    load_partition,
    same_color_specificity,
    sha256,
    static_gate,
    write_json,
)


def evaluate_rows(
    *, name, rows, instances, detector, leaf, puddle, device, threshold,
    input_size=(640, 480),
):
    started = time.perf_counter()
    by_key = index_instance_records(instances)
    frames = direct_predictions(
        detector,
        rows,
        by_key,
        device=device,
        score_threshold=threshold,
        batch_size=4,
        input_size=input_size,
        top_k=16,
    )
    candidate = discovery_metrics(frames)
    candidate.update(candidate_size_metrics(frames))
    matched = match_discrete_predictions(frames)
    discrete = discrete_metrics(matched)
    leaf_frames = area_predictions(
        leaf, rows, device=device, thresholds=AREA_THRESHOLDS, task="leaf"
    )
    puddle_frames = area_predictions(
        puddle, rows, device=device, thresholds=AREA_THRESHOLDS, task="puddle"
    )
    area = area_metrics(combine_area(leaf_frames, puddle_frames))
    report = {
        "name": name,
        "rows": len(rows),
        "candidate": candidate,
        "discrete": discrete,
        "area": area,
        "same_color_negative_specificity": same_color_specificity(frames),
        "direct_detector_threshold": threshold,
        "duration_s": time.perf_counter() - started,
    }
    print(
        f"[{name}] rows={len(rows)} candidate_recall={candidate['all_gt_candidate_recall']:.4f} "
        f"macro_f1={discrete['macro_f1']:.4f} fp_min={candidate['false_candidates_per_min']:.2f}",
        flush=True,
    )
    return report, frames, matched


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--factorized-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal X3 static gate requires CUDA")
    payload = torch.load(args.checkpoint, map_location=device, weights_only=False)
    if payload.get("checkpoint_status") not in (
        "training_complete", "training_complete_candidate_not_frozen"
    ):
        raise RuntimeError("direct FCOS checkpoint is not training complete")
    if payload.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError("X3 checkpoint violates sealed-final policy")
    threshold = float(payload["frozen_threshold_from_train_world_holdout"])
    input_size = tuple(payload.get("input_size", (640, 480)))
    detector = (
        build_p2_direct_fcos(input_size=input_size)
        if payload.get("architecture") == MRV2_C_P2_ARCHITECTURE
        else build_direct_fcos(input_size=input_size)
    ).to(device)
    detector.load_state_dict(payload["state_dict"], strict=True)
    detector.eval()
    leaf, leaf_record = load_checkpoint_model(
        "leaf", args.model_dir / "leaf.pt", device
    )
    puddle, puddle_record = load_checkpoint_model(
        "puddle", args.model_dir / "puddle.pt", device
    )
    val_rows, val_instances = load_partition(
        args.data_root, args.evidence_dir, allowed_splits={"val"}
    )
    splits = {}
    splits["VAL"], _, _ = evaluate_rows(
        name="VAL",
        rows=val_rows,
        instances=val_instances,
        detector=detector,
        leaf=leaf,
        puddle=puddle,
        device=device,
        threshold=threshold,
        input_size=input_size,
    )
    cross_frames = []
    cross_matched = []
    for index in range(1, 6):
        root = args.factorized_root / f"D{index}"
        rows, instances = load_partition(
            root / "g4_screening_native", root / "evidence/raw_g4_qa"
        )
        split, frames, matched = evaluate_rows(
            name=f"D{index}",
            rows=rows,
            instances=instances,
            detector=detector,
            leaf=leaf,
            puddle=puddle,
            device=device,
            threshold=threshold,
            input_size=input_size,
        )
        splits[f"D{index}"] = split
        cross_frames.extend(frames)
        cross_matched.extend(matched)
    cross_candidate = discovery_metrics(cross_frames)
    cross_candidate.update(candidate_size_metrics(cross_frames))
    report = {
        "schema_version": 1,
        "stage": (
            f"{payload['route']}-STATIC"
            if payload.get("route") in ("MRV2-A", "MRV2-C")
            else "PERCEPTION-PROD-02-X3-STATIC"
        ),
        "source_commit": "d958854",
        "route": payload.get("route", "ONLINE-X3_TORCHVISION_FCOS_R50_DIRECT_3CLASS"),
        "device": str(device),
        "environment": {
            "platform": platform.platform(),
            "torch": torch.__version__,
            "cuda_available": torch.cuda.is_available(),
            "gpu": torch.cuda.get_device_name(0),
        },
        "thresholds": {
            "direct_detector_from_train_world_holdout": threshold,
            "area": list(AREA_THRESHOLDS),
            "top_k": 16,
            "input_size": list(input_size),
        },
        "models": {
            "detector": {
                "path": args.checkpoint.as_posix(),
                "sha256": sha256(args.checkpoint),
                "provenance": payload.get("provenance"),
            },
            "leaf": leaf_record,
            "puddle": puddle_record,
        },
        "splits": splits,
        "cross_world_aggregate": {
            "name": "D1-D5-AGGREGATE",
            "rows": len(cross_frames),
            "candidate": cross_candidate,
            "discrete": discrete_metrics(cross_matched),
            "area": aggregate_area_reports(
                [splits[f"D{index}"]["area"] for index in range(1, 6)]
            ),
            "same_color_negative_specificity": same_color_specificity(cross_frames),
            "direct_detector_threshold": threshold,
            "duration_s": sum(splits[f"D{index}"]["duration_s"] for index in range(1, 6)),
        },
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "moving_camera_gate": "not_run_pending_static_gate",
        "PERCEPTION_ONLINE_X86_DEV_PASS": False,
    }
    report["static_decision"] = static_gate(report)
    report["next_action"] = (
        "run_moving_camera_and_export_gates"
        if report["static_decision"]["static_gate_pass"]
        else (
            "execute_MRV2_B_tiled_refinement"
            if payload.get("route") == "MRV2-A"
            else (
                "MODEL_BLOCKED_INTERNAL_all_MRV2_routes_exhausted"
                if payload.get("route") == "MRV2-C"
                else "all_three_routes_exhausted_model_blocked_internal"
            )
        )
    )
    write_json(args.output, report)
    print(json.dumps(report["static_decision"], indent=2), flush=True)
    return 0 if report["static_decision"]["static_gate_pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
