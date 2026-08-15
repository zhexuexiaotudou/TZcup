#!/usr/bin/env python3
"""Bounded, safety-first active re-observation policy for TRCRV10."""

from __future__ import annotations

from dataclasses import dataclass


MAX_REOBSERVE = 2


@dataclass(frozen=True)
class ReobserveConfig:
    minimum_reliable_short_side_px: int
    maximum_map_covariance_m2: float = .04
    minimum_depth_valid_fraction: float = .80
    maximum_severe_occlusion_ratio: float = .60


def decide(state: dict, config: ReobserveConfig) -> dict:
    if state.get("action_verifier_decision") == "ACCEPT":
        return {"decision": "CONFIRMED", "reason": "action_verified"}
    safety_failures = [
        key for key in (
            "dynamic_obstacle", "localization_unhealthy", "keepout_conflict",
            "nav2_path_unavailable", "candidate_covariance_exploded",
        ) if state.get(key) is True
    ]
    if safety_failures:
        return {"decision": "DEFER", "reason": "safety_precondition_failed", "details": safety_failures}
    if state.get("reachable_for_visual_confirmation") is False:
        return {"decision": "UNREACHABLE_FOR_VISUAL_CONFIRMATION", "reason": "safe_observation_pose_unavailable"}
    count = int(state.get("reobserve_count", 0))
    if count >= MAX_REOBSERVE:
        return {"decision": "DEFER", "reason": "maximum_reobserve_exhausted"}
    conditions = {
        "bbox_reliable": int(state.get("bbox_short_side_px", 0)) >= config.minimum_reliable_short_side_px,
        "depth_valid": float(state.get("depth_valid_fraction", 0.0)) >= config.minimum_depth_valid_fraction,
        "occlusion_acceptable": float(state.get("occlusion_ratio", 1.0)) <= config.maximum_severe_occlusion_ratio,
        "covariance_healthy": float(state.get("map_covariance_m2", float("inf"))) <= config.maximum_map_covariance_m2,
    }
    if all(conditions.values()) and state.get("action_verifier_decision") == "VETO":
        return {"decision": "REJECT", "reason": "reliable_observation_vetoed"}
    mode = "CONTINUE_COVERAGE_APPROACH" if state.get("coverage_path_improves_view") else "SHORT_SAFE_DETOUR"
    return {"decision": "OBSERVE_AGAIN", "reobserve_count": count + 1, "mode": mode,
            "goal": "reach_reliable_image_evidence", "conditions": conditions}
