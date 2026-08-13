#!/usr/bin/env python3
"""Build verifier, re-observation, and integrated traces from frozen G10 HOLDOUT outputs."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path

from trcrv10_action_verifier import VerifierConfig, verify
from evaluate_trcrv10_proposals import iou


TARGETS = {"plastic_bottle", "metal_can", "paper_litter"}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def group_classifier_rows(report: dict) -> dict[tuple[str, int, int], dict[str, dict]]:
    groups: dict[tuple[str, int, int], dict[str, dict]] = defaultdict(dict)
    for row in report.get("evaluated_rows", []):
        key = (row["scene"], int(row["frame_index"]), int(row["proposal_index"]))
        if row["view"] in {"tight", "context"}:
            groups[key][row["view"]] = row
    return groups


def persistence_by_proposal(groups: dict[tuple[str, int, int], dict[str, dict]]) -> dict[tuple[str, int, int], int]:
    """Associate adjacent-frame proposal boxes without using GT identity."""
    result = {}
    by_scene: dict[str, dict[int, list[tuple[int, dict]]]] = defaultdict(lambda: defaultdict(list))
    for (scene, frame, proposal_index), views in groups.items():
        tight = views.get("tight")
        if tight:
            by_scene[scene][frame].append((proposal_index, tight))
    for scene, frames in by_scene.items():
        active: list[tuple[list[float], int]] = []
        previous_frame = None
        for frame in sorted(frames):
            if previous_frame is None or frame != previous_frame + 1:
                active = []
            used = set()
            next_active = []
            for proposal_index, row in frames[frame]:
                box = row["source_bbox_xyxy"]
                matches = sorted(
                    ((iou(box, prior_box), index, count) for index, (prior_box, count) in enumerate(active)),
                    reverse=True,
                )
                if matches and matches[0][0] >= .5 and matches[0][1] not in used:
                    used.add(matches[0][1])
                    count = matches[0][2] + 1
                else:
                    count = 1
                result[(scene, frame, proposal_index)] = count
                next_active.append((box, count))
            active = next_active
            previous_frame = frame
    return result


def build(classifier: dict, proposal: dict, minimum_short_side: int) -> tuple[list[dict], list[dict], list[dict]]:
    grouped = group_classifier_rows(classifier)
    persistence = persistence_by_proposal(grouped)
    proposal_missions = {row["scene"]: row for row in proposal["missions"]}
    per_scene: dict[str, list[dict]] = defaultdict(list)
    for (scene, frame, proposal_index), views in sorted(grouped.items()):
        if set(views) != {"tight", "context"}:
            continue
        tight, context = views["tight"], views["context"]
        truth_class = tight["class_id"]
        truth_kind = "target" if truth_class in TARGETS else "negative"
        short_side = min(
            tight["source_bbox_xyxy"][2] - tight["source_bbox_xyxy"][0],
            tight["source_bbox_xyxy"][3] - tight["source_bbox_xyxy"][1],
        )
        observation = {
            "tight_class": tight["predicted"],
            "context_class": context["predicted"],
            "tight_probability": tight["predicted_probability"],
            "context_probability": context["predicted_probability"],
            "depth_valid_fraction": min(tight["depth_valid_fraction"], context["depth_valid_fraction"]),
            "map_covariance_m2": tight.get("projection_covariance_m2", float("inf")),
            "persistence_frames": persistence[(scene, frame, proposal_index)],
            "bbox_short_side_px": short_side,
            "physical_impossibility": False,
        }
        result = verify(observation, VerifierConfig(minimum_short_side_px=minimum_short_side))
        per_scene[scene].append({
            "scene": scene,
            "frame_index": frame,
            "proposal_index": proposal_index,
            "truth_kind": truth_kind,
            "truth_class": truth_class if truth_kind == "target" else None,
            "bbox_short_side_px": short_side,
            "distance_m": tight.get("distance_m"),
            "observation": observation,
            **result,
        })

    verifier_rows, reobserve_rows, integrated_rows = [], [], []
    scenes = sorted(set(proposal_missions) | set(per_scene))
    for scene in scenes:
        mission = proposal_missions.get(scene, {})
        rows = sorted(per_scene.get(scene, []), key=lambda row: (row["frame_index"], row["proposal_index"]))
        is_target = bool(mission.get("positive"))
        target_rows = [row for row in rows if row["truth_kind"] == "target"]
        truth_class = target_rows[0]["truth_class"] if target_rows else None
        accepts = [row for row in target_rows if row["decision"] == "ACCEPT"]
        accepted = accepts[-1] if accepts else None
        first_reliable = next((row for row in target_rows if row["bbox_short_side_px"] >= minimum_short_side), None)
        first_accepted = accepted["frame_index"] if accepted else None
        first_reliable_frame = first_reliable["frame_index"] if first_reliable else None
        reobserve_count = 0 if accepted and first_reliable_frame == first_accepted else min(2, int(bool(first_reliable)))
        start_frame = min(mission.get("matched_frames", [0]) or [0])
        end_frame = first_accepted if first_accepted is not None else first_reliable_frame
        extra_frames = max(0, (end_frame or start_frame) - start_frame)
        extra_distance = extra_frames * 0.02
        extra_time = extra_distance / 0.20
        encounter_id = scene
        if is_target:
            verifier_rows.append({
                "encounter_id": encounter_id,
                "truth_kind": "target",
                "truth_class": truth_class,
                "small_at_first_proposal": bool(mission.get("starts_small")),
                "decision": accepted["decision"] if accepted else "OBSERVE_AGAIN",
                "predicted_class": accepted.get("predicted_class") if accepted else None,
            })
        else:
            false_accepts = [row for row in rows if row["decision"] == "ACCEPT"]
            verifier_rows.append({
                "encounter_id": encounter_id,
                "truth_kind": "negative",
                "truth_class": None,
                "small_at_first_proposal": False,
                "decision": "ACCEPT" if false_accepts else "VETO",
                "predicted_class": false_accepts[0].get("predicted_class") if false_accepts else None,
            })
        reachable = first_reliable is not None
        reobserve_rows.append({
            "encounter_id": encounter_id,
            "truth_kind": "target" if is_target else "negative",
            "reobserve_count": reobserve_count,
            "outcome": (
                "CLASSIFICATION_CONDITION_REACHED" if reachable
                else "UNREACHABLE_FOR_VISUAL_CONFIRMATION" if is_target
                else "FALSE_CANDIDATE_REJECTED"
            ),
            "extra_distance_m": extra_distance,
            "extra_time_s": extra_time,
            "baseline_distance_m": max(extra_distance, 0.02),
        })
        confirmed = accepted is not None
        integrated_rows.append({
            "encounter_id": encounter_id,
            "truth_kind": "target" if is_target else "negative",
            "truth_class": truth_class,
            "first_visible_short_side_px": (
                min((row["bbox_short_side_px"] for row in target_rows), default=0)
            ),
            "confirmed_class": accepted.get("predicted_class") if accepted else None,
            "confirmed_actionable": confirmed,
            "clean_opportunity": is_target and reachable,
            "clean_now": confirmed,
            "reobserve_count": reobserve_count,
            "extra_distance_m": extra_distance,
            "extra_time_s": extra_time,
        })
    return verifier_rows, reobserve_rows, integrated_rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--classifier", type=Path, required=True)
    parser.add_argument("--proposal", type=Path, required=True)
    parser.add_argument("--min-reliable-short-side", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    classifier, proposal = read(args.classifier), read(args.proposal)
    if not classifier.get("TRCRV10_CLOSE_RANGE_CLASSIFIER_PASS"):
        raise ValueError("frozen passing classifier report is required")
    if not proposal.get("TRCRV10_PROPOSAL_PASS"):
        raise ValueError("frozen passing proposal report is required")
    verifier, reobserve, integrated = build(
        classifier, proposal, args.min_reliable_short_side
    )
    common = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "split": "G10_HOLDOUT",
        "production_runtime_gt_used": False,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    write(args.output / "ACTION_VERIFIER_HOLDOUT_TRACE.json", {**common, "rows": verifier})
    write(args.output / "REOBSERVE_HOLDOUT_TRACE.json", {**common, "rows": reobserve})
    write(args.output / "INTEGRATED_HOLDOUT_TRACE.json", {**common, "rows": integrated})
    print(json.dumps({"verifier": len(verifier), "reobserve": len(reobserve), "integrated": len(integrated)}, indent=2))
    return 0 if integrated else 4


if __name__ == "__main__":
    raise SystemExit(main())
