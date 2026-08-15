#!/usr/bin/env python3
"""Auditable V1 action verifier for TRCRV10 close-range observations."""

from __future__ import annotations

from dataclasses import dataclass


TARGET_CLASSES = {"plastic_bottle", "metal_can", "paper_litter"}
DECISIONS = {"ACCEPT", "VETO", "OBSERVE_AGAIN"}


@dataclass(frozen=True)
class VerifierConfig:
    minimum_class_probability: float = .97
    maximum_posterior_delta: float = .10
    minimum_depth_valid_fraction: float = .80
    maximum_map_covariance_m2: float = .04
    minimum_persistence_frames: int = 3
    minimum_short_side_px: int = 64


def verify(observation: dict, config: VerifierConfig) -> dict:
    tight_class = observation.get("tight_class", "background_or_unknown")
    context_class = observation.get("context_class", "background_or_unknown")
    reasons = []
    if tight_class not in TARGET_CLASSES or context_class not in TARGET_CLASSES:
        return {"decision": "VETO", "reasons": ["classifier_unknown_or_background"]}
    if tight_class != context_class:
        return {"decision": "OBSERVE_AGAIN", "reasons": ["tight_context_disagree"]}
    tight_probability = float(observation.get("tight_probability", 0.0))
    context_probability = float(observation.get("context_probability", 0.0))
    if min(tight_probability, context_probability) < config.minimum_class_probability:
        reasons.append("class_probability_below_threshold")
    if abs(tight_probability - context_probability) > config.maximum_posterior_delta:
        reasons.append("class_posterior_unstable")
    if float(observation.get("depth_valid_fraction", 0.0)) < config.minimum_depth_valid_fraction:
        reasons.append("depth_invalid")
    if float(observation.get("map_covariance_m2", float("inf"))) > config.maximum_map_covariance_m2:
        reasons.append("map_covariance_too_high")
    if int(observation.get("persistence_frames", 0)) < config.minimum_persistence_frames:
        reasons.append("track_not_persistent")
    if int(observation.get("bbox_short_side_px", 0)) < config.minimum_short_side_px:
        reasons.append("below_reliable_visual_size")
    # Physical veto is an external RGB-D plausibility check.  It may only flag
    # obvious impossibility; paper is never required to have non-zero height,
    # and low height cannot by itself veto a crushed/sideways can.
    if observation.get("physical_impossibility") is True:
        return {"decision": "VETO", "reasons": ["physical_plausibility_impossible"]}
    if reasons:
        return {"decision": "OBSERVE_AGAIN", "reasons": reasons}
    return {"decision": "ACCEPT", "predicted_class": tight_class, "reasons": []}
