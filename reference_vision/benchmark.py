"""Reference-only moving-camera benchmark validation and deterministic ranking."""

from __future__ import annotations


REQUIRED_SCENARIO_TAGS = {
    "near_target",
    "far_target",
    "turning",
    "occlusion",
    "light_shadow",
    "negative_only",
    "late_fov_entry",
}


def validate_moving_camera_manifest(manifest: dict) -> None:
    sequences = manifest.get("sequences", [])
    if len(sequences) < 10:
        raise ValueError("reference benchmark requires at least 10 sequences")
    seen_tags = set()
    for sequence in sequences:
        if sequence.get("source") != "gazebo_onboard_rgb":
            raise ValueError("reference input must be the onboard Gazebo RGB stream")
        if float(sequence.get("duration_s", 0.0)) < 60.0:
            raise ValueError("each reference sequence must be at least 60 seconds")
        if sequence.get("fixed_overhead") or sequence.get("pre_cropped"):
            raise ValueError("fixed overhead and pre-cropped inputs are forbidden")
        seen_tags.update(sequence.get("tags", []))
    missing = REQUIRED_SCENARIO_TAGS - seen_tags
    if missing:
        raise ValueError(f"moving-camera scenario tags missing: {sorted(missing)}")


def ranking_key(metrics: dict) -> tuple:
    if metrics.get("pre_fov_false_discovery", 1) != 0:
        return (1,)
    return (
        0,
        -float(metrics["recall"]),
        -float(metrics["small_object_recall"]),
        float(metrics["false_candidates_per_min"]),
        -float(metrics["temporal_stability"]),
        float(metrics["latency_ms"]),
        float(metrics["memory_mb"]),
        0 if metrics.get("license_deployment_suitable") else 1,
    )
