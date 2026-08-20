"""Deterministic, quota-bound sampling for MODEL-RECOVERY-V2 route A."""

from __future__ import annotations

import random


MRV2_BUCKET_RATIOS = {
    "small_object": 0.30,
    "negative_only": 0.20,
    "metal_can": 0.15,
    "general": 0.35,
}


def row_key(row: dict) -> tuple[str, int, int]:
    return str(row["world_id"]), int(row["scene_seed"]), int(row["frame_index"])


def _sample(pool: list[dict], count: int, rng: random.Random) -> list[dict]:
    if count <= 0:
        return []
    if not pool:
        raise ValueError("MRV2 quota pool is empty")
    shuffled = list(pool)
    rng.shuffle(shuffled)
    return [dict(shuffled[index % len(shuffled)]) for index in range(count)]


def build_mrv2_epoch_rows(
    rows: list[dict], *, small_keys: set[tuple], metal_keys: set[tuple],
    frame_count: int, seed: int,
) -> tuple[list[dict], dict]:
    """Build mutually exclusive 30/20/15/35 percent epoch buckets.

    Repetition is intentional when a scarce pool cannot fill its quota.  This
    keeps every small frame and increases its effective batch exposure without
    removing the fixed negative quota.
    """
    if frame_count <= 0:
        raise ValueError("frame_count must be positive")
    pools = {name: [] for name in MRV2_BUCKET_RATIOS}
    for row in rows:
        key = row_key(row)
        if key in small_keys:
            bucket = "small_object"
        elif bool(row.get("negative_only")):
            bucket = "negative_only"
        elif key in metal_keys:
            bucket = "metal_can"
        else:
            bucket = "general"
        pools[bucket].append(row)
    quotas = {
        "small_object": int(round(frame_count * MRV2_BUCKET_RATIOS["small_object"])),
        "negative_only": int(round(frame_count * MRV2_BUCKET_RATIOS["negative_only"])),
        "metal_can": int(round(frame_count * MRV2_BUCKET_RATIOS["metal_can"])),
    }
    quotas["general"] = frame_count - sum(quotas.values())
    rng = random.Random(seed)
    selected = []
    for bucket in MRV2_BUCKET_RATIOS:
        selected.extend(
            {**row, "mrv2_sampling_bucket": bucket}
            for row in _sample(pools[bucket], quotas[bucket], rng)
        )
    rng.shuffle(selected)
    report = {
        "frame_count": len(selected),
        "ratios": {
            bucket: sum(row["mrv2_sampling_bucket"] == bucket for row in selected)
            / max(len(selected), 1)
            for bucket in MRV2_BUCKET_RATIOS
        },
        "quotas": quotas,
        "unique_pool_frames": {bucket: len(pool) for bucket, pool in pools.items()},
        "unique_selected_frames": len({row_key(row) for row in selected}),
        "replacement_used": {
            bucket: quotas[bucket] > len(pools[bucket]) for bucket in MRV2_BUCKET_RATIOS
        },
    }
    return selected, report


__all__ = ["MRV2_BUCKET_RATIOS", "build_mrv2_epoch_rows", "row_key"]
