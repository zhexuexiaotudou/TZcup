#!/usr/bin/env python3
"""Audit all fixed public train/val pedestrian paths without hidden inputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PACKAGE = ROOT / "starter_ws/src/sanitation_campus_scenario"
sys.path.insert(0, str(SCENARIO_PACKAGE))

from sanitation_campus_scenario.generator import (  # noqa: E402
    _start_exclusion,
    derive_field_dimensions,
    generate_assets,
    generate_cubes,
    generate_dirt,
    generate_pedestrians,
    load_config,
    pedestrian_paths_clear,
    seeds_for,
    segment_clearance_to_asset,
    segment_clearance_to_cube,
    validate_episode_geometry,
)


DEFAULT_CONFIG = SCENARIO_PACKAGE / "config/default_scenario.yaml"
PUBLIC_SPLITS = ("train", "val")
MISSIONS_PER_MAP = 20


def audit_public_geometry(
    config: dict[str, Any],
    *,
    split_map_counts: Iterable[tuple[str, int]] | None = None,
    missions_per_map: int = MISSIONS_PER_MAP,
) -> dict[str, Any]:
    """Return an exact collision audit for public fixed-seed geometry only."""
    requested = tuple(split_map_counts or (
        (split, int(config["split"][split]["map_count"]))
        for split in PUBLIC_SPLITS
    ))
    if not requested or any(split not in PUBLIC_SPLITS for split, _ in requested):
        raise ValueError("audit accepts only public train/val splits")
    if missions_per_map <= 0:
        raise ValueError("missions_per_map must be positive")

    base_profile = config["profiles"]["formal"]
    episode = config["episode"]
    expected_static = sum(
        int(base_profile[key])
        for key in ("building_count", "pole_count", "bin_count", "tree_count", "bench_count")
    )
    summary: dict[str, Any] = {
        "schema_version": 1,
        "scope": "public_train_val_only",
        "hidden_accessed": False,
        "missions_per_map": missions_per_map,
        "map_count": 0,
        "episode_count": 0,
        "pedestrian_path_count": 0,
        "pedestrian_pair_count": 0,
        "pedestrian_pair_violation_count": 0,
        "pedestrian_static_collision_path_count": 0,
        "pedestrian_cube_collision_path_count": 0,
        "expected_counts_per_episode": {
            "static_assets": expected_static,
            "dirt_patches": int(episode["dirt_patch_count"]),
            "discrete_cubes": int(episode["cube_count"]),
            "pedestrians": int(episode["pedestrian_count"]),
        },
        "field_area_m2": float(base_profile["width_m"] * base_profile["height_m"]),
        "seed_determinism_passed": True,
        "object_counts_passed": True,
        "field_area_passed": True,
        "splits": {},
    }

    for split, map_count in requested:
        split_episodes = 0
        split_paths = 0
        summary["map_count"] += map_count
        for map_index in range(map_count):
            for mission_index in range(missions_per_map):
                seeds = seeds_for(config, split, map_index, mission_index)
                summary["seed_determinism_passed"] &= seeds == seeds_for(
                    config, split, map_index, mission_index
                )
                width, height = derive_field_dimensions(base_profile, map_index, seeds.layout)
                profile = {**base_profile, "width_m": width, "height_m": height}
                summary["field_area_passed"] &= abs(width * height - summary["field_area_m2"]) <= 1e-9
                assets = generate_assets(profile, seeds.layout)
                exclusions = [*assets, _start_exclusion(profile)]
                cubes = generate_cubes(profile, episode, exclusions, seeds.cubes)
                dirt = generate_dirt(profile, episode, exclusions, seeds.dirt)
                pedestrians = generate_pedestrians(
                    profile, episode, exclusions, cubes, seeds.pedestrians
                )
                validate_episode_geometry(profile, assets, cubes, pedestrians)
                counts = (len(assets), len(dirt), len(cubes), len(pedestrians))
                summary["object_counts_passed"] &= counts == (
                    expected_static,
                    int(episode["dirt_patch_count"]),
                    int(episode["cube_count"]),
                    int(episode["pedestrian_count"]),
                )
                for pedestrian in pedestrians:
                    segments = [
                        ((x1, y1), (x2, y2))
                        for (_, x1, y1), (_, x2, y2) in zip(
                            pedestrian.waypoints, pedestrian.waypoints[1:]
                        )
                    ]
                    if any(
                        not segment_clearance_to_asset(start, end, pedestrian.radius_m, asset)
                        for start, end in segments
                        for asset in assets
                    ):
                        summary["pedestrian_static_collision_path_count"] += 1
                    if any(
                        not segment_clearance_to_cube(start, end, pedestrian.radius_m, cube)
                        for start, end in segments
                        for cube in cubes
                    ):
                        summary["pedestrian_cube_collision_path_count"] += 1
                for right_index, pedestrian in enumerate(pedestrians):
                    for other in pedestrians[:right_index]:
                        summary["pedestrian_pair_count"] += 1
                        if not pedestrian_paths_clear(pedestrian, other):
                            summary["pedestrian_pair_violation_count"] += 1
                summary["episode_count"] += 1
                summary["pedestrian_path_count"] += len(pedestrians)
                split_episodes += 1
                split_paths += len(pedestrians)
        summary["splits"][split] = {
            "map_count": map_count,
            "episode_count": split_episodes,
            "pedestrian_path_count": split_paths,
            "pedestrian_pair_count": split_episodes
            * int(episode["pedestrian_count"])
            * (int(episode["pedestrian_count"]) - 1)
            // 2,
        }

    summary["passed"] = all((
        summary["seed_determinism_passed"],
        summary["object_counts_passed"],
        summary["field_area_passed"],
        summary["pedestrian_static_collision_path_count"] == 0,
        summary["pedestrian_cube_collision_path_count"] == 0,
        summary["pedestrian_pair_violation_count"] == 0,
    ))
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit_public_geometry(load_config(args.config))
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
