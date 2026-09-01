#!/usr/bin/env python3
"""Audit whether saved product evidence can support ground-dirt counterfactuals."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


EXPECTED_INTERMEDIATES = {
    "timestamp_aligned_rgb_depth_camera_info": (
        "product_rgb_depth_frames.npz",
        "product_camera_info.jsonl",
    ),
    "runtime_post_nms_detections": ("product_detections.jsonl",),
    "selected_edgesam_prompts": ("edgesam_prompts.jsonl",),
    "per_prompt_edgesam_masks": ("edgesam_prompt_masks.npz",),
    "ground_plane_filter_state": ("ground_plane_filter.jsonl",),
    "per_class_projected_rasters": ("ground_dirt_per_class_rasters.npz",),
    "final_union_rasters": ("ground_dirt_union_rasters.npz",),
    "public_map_and_tf_used": ("product_projection_context.npz",),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_episode(episode_root: Path) -> dict:
    available = {}
    missing = {}
    for group, names in EXPECTED_INTERMEDIATES.items():
        found = [name for name in names if (episode_root / name).is_file()]
        available[group] = found
        missing[group] = [name for name in names if name not in found]
    raw_path = episode_root / "dosod_raw_diagnostic.json"
    acceptance_path = episode_root / "perception_acceptance.json"
    image_path = episode_root / "best_front_frame.png"
    return {
        "episode_root": str(episode_root),
        "saved_evidence": {
            "best_front_rgb": image_path.is_file(),
            "best_front_rgb_sha256": _sha256(image_path) if image_path.is_file() else None,
            "offline_raw_dosod_best_frame": raw_path.is_file(),
            "offline_raw_dosod_best_frame_sha256": _sha256(raw_path) if raw_path.is_file() else None,
            "aggregate_acceptance_counts": acceptance_path.is_file(),
            "aggregate_acceptance_counts_sha256": (
                _sha256(acceptance_path) if acceptance_path.is_file() else None
            ),
            "available_product_intermediates": available,
            "missing_product_intermediates": missing,
        },
        "counterfactual_iou_replay_possible": all(not rows for rows in missing.values()),
    }


def build_report(
    episode_roots: list[Path],
    *,
    product_adapter_path: Path,
    projection_path: Path,
    rescore_path: Path,
) -> dict:
    adapter_source = product_adapter_path.read_text(encoding="utf-8")
    projection_source = projection_path.read_text(encoding="utf-8")
    rescore = json.loads(rescore_path.read_text(encoding="utf-8"))
    episodes = [audit_episode(path) for path in episode_roots]
    replay_possible = bool(episodes) and all(
        episode["counterfactual_iou_replay_possible"] for episode in episodes
    )
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_r7_ground_dirt_candidate_evidence_audit_v1",
        "claim_boundary": {
            "evaluator_only_offline_diagnostic": True,
            "eligible_as_formal_product_acceptance": False,
            "truth_used_to_choose_product_parameters": False,
            "product_threshold_prompt_or_weight_changed": False,
            "counterfactual_iou_claim_allowed": replay_possible,
        },
        "source_contract": {
            "product_adapter_sha256": _sha256(product_adapter_path),
            "projection_sha256": _sha256(projection_path),
            "fixed_ground_z_filter_present": "on_ground =" in projection_source,
            "fixed_ground_tolerance_default_m": 0.25,
            "public_grid_carries_occupancy_values": "data:" in projection_source,
            "large_prompt_rejection_present": "max_area_fraction: float = 0.45" in adapter_source,
            "per_class_raster_union_present": "per_class_raster" in projection_source,
        },
        "saved_r7_summary": {
            "ground_two_episode_metrics": rescore["saved_evidence_aggregate"][
                "ground_two_episode_confusion_counts"
            ],
            "cube_dosod_capability_is_not_the_ground_bottleneck": rescore[
                "capability_assessment"
            ]["cube_best_saved_frames_both_meet_0_8"],
        },
        "episodes": episodes,
        "evidence_decision": {
            "sufficient_for_quantitative_candidate_iou_replay": replay_possible,
            "reason": (
                "r7 saved only one RGB/raw-DOSOD diagnostic and aggregate dirt counts per scored "
                "episode; it did not save aligned depth, runtime prompts, per-prompt EdgeSAM masks, "
                "projection context, per-class rasters or final union rasters."
            ),
        },
        "candidate_mechanisms": [
            {
                "priority": 1,
                "name": "depth_ground_plane_and_cleanable_roi",
                "current_state": (
                    "Projection already rejects points outside map z=0 +/-0.25m, but PublicGrid "
                    "contains geometry only and cannot reject occupied/unknown/non-cleanable cells."
                ),
                "truth_free_mechanism": (
                    "Estimate or validate the ground plane from aligned public depth/TF, then retain "
                    "only finite ground-supported pixels in public free/cleanable map cells."
                ),
                "expected_effect": "reduce background, curb and non-floor mask spill before map union",
                "risks": [
                    "TF drift can move valid dirt outside a narrow plane band",
                    "slopes and height discontinuities can be mistaken for non-cleanable space",
                    "occupancy unknown must remain fail-closed rather than silently treated as free",
                ],
                "quantified_from_r7": False,
            },
            {
                "priority": 2,
                "name": "cube_priority_cross_class_same_box_dedup",
                "current_state": (
                    "r7 raw diagnostics contain cube/dirt shared anchors and an ep0 cube-dust "
                    "postprocess pair at IoU 0.978, while cube geometry itself meets the saved-frame gate."
                ),
                "truth_free_mechanism": (
                    "Before EdgeSAM, suppress a ground-dirt prompt only when its box is the same "
                    "anchor or nearly identical geometry as a higher-confidence accepted cube box."
                ),
                "expected_effect": "remove cube-shaped false dirt prompts without changing cube output",
                "risks": [
                    "real dirt directly beneath or beside a cube may be suppressed",
                    "cross-class confidence magnitudes are not calibrated probabilities",
                    "an overlap cutoff cannot be selected from evaluator IoU without truth leakage",
                ],
                "quantified_from_r7": False,
            },
            {
                "priority": 3,
                "name": "per_class_mask_union_and_large_box_rejection",
                "current_state": (
                    "Prompts above 45% image area are already rejected, but accepted dirt masks from "
                    "all classes are written directly into one maximum-valued raster."
                ),
                "truth_free_mechanism": (
                    "Keep leaves/soil/puddle rasters separate through prompt sanitation and ground/ROI "
                    "projection; reject invalid boxes before EdgeSAM and union only the surviving cells."
                ),
                "expected_effect": "prevent one ambiguous class mask from contaminating every dirt class",
                "risks": [
                    "medium-area false boxes remain below the existing 45% guard",
                    "class separation alone does not correct a bad mask",
                    "extra raster state increases memory and evidence volume",
                ],
                "quantified_from_r7": False,
            },
        ],
        "minimum_next_capture": {
            "selection_policy": (
                "capture the first N or fixed-rate frames before evaluator scoring; never select frames "
                "or parameters by evaluator truth"
            ),
            "required_per_frame": [
                "RGB and depth arrays with original ROS stamps and encodings",
                "CameraInfo K/width/height and exact map_from_camera transform used",
                "all runtime post-NMS boxes, class ids, scores and source anchor ids",
                "selected/rejected EdgeSAM prompt indices with deterministic rejection reason",
                "one compressed binary mask plus quality for every selected prompt",
                "valid-depth, ground-plane, in-grid and public-free/cleanable pixel masks or counts",
                "per-class projected rasters before union and the final published raster",
                "public occupancy-grid data, origin, resolution and map stamp/hash",
            ],
            "offline_evaluation_policy": (
                "Apply predeclared truth-free candidate rules to the captured product intermediates, "
                "hash the candidate outputs, and only then let the evaluator compare those frozen "
                "rasters with private truth. Do not use resulting IoU to tune candidate parameters."
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-root", action="append", required=True, type=Path)
    parser.add_argument("--product-adapter", required=True, type=Path)
    parser.add_argument("--projection", required=True, type=Path)
    parser.add_argument("--rescore", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = build_report(
        args.episode_root,
        product_adapter_path=args.product_adapter,
        projection_path=args.projection,
        rescore_path=args.rescore,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(args.output), "sufficient": report["evidence_decision"]["sufficient_for_quantitative_candidate_iou_replay"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
