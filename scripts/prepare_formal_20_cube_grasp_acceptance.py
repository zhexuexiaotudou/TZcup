#!/usr/bin/env python3
"""Materialize the deterministic 20-cube target-conditioned grasp manifest."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random


MASS_KG = {"paperboard": 0.0189, "PP": 0.0243, "PET": 0.03726, "aluminum": 0.0729}
CUBE_EDGE_M = 0.030
GRID_COLUMNS = 5
GRID_ROWS = 4
GRID_X_PITCH_M = 0.060
GRID_Y_PITCH_M = 0.070
GRID_CENTER_X_M = 0.300
GRID_CENTER_Y_M = -0.855
ARM_BASE_XY_M = (0.100, -0.200)
MAXIMUM_PLANAR_REACH_M = 0.850


def _slots() -> list[tuple[int, int, float, float]]:
    """Return a single-layer array with physical finger clearance between cubes."""

    slots = []
    for row in range(GRID_ROWS):
        for column in range(GRID_COLUMNS):
            x_m = GRID_CENTER_X_M + (column - (GRID_COLUMNS - 1) / 2.0) * GRID_X_PITCH_M
            y_m = GRID_CENTER_Y_M + (row - (GRID_ROWS - 1) / 2.0) * GRID_Y_PITCH_M
            slots.append((row, column, round(x_m, 6), round(y_m, 6)))
    return slots


def build_manifest(seed: int) -> dict:
    rng = random.Random(seed)
    color_rng = random.Random(seed ^ 0x20C0BE)
    materials = list(MASS_KG) * 5
    rng.shuffle(materials)
    slots = _slots()
    rng.shuffle(slots)
    requests = []
    cumulative = 0.0
    maximum_reach = 0.0
    for index, (material, slot) in enumerate(zip(materials, slots)):
        row, column, x_m, y_m = slot
        yaw = rng.uniform(-math.pi, math.pi)
        planar_reach = math.hypot(x_m - ARM_BASE_XY_M[0], y_m - ARM_BASE_XY_M[1])
        if planar_reach > MAXIMUM_PLANAR_REACH_M:
            raise RuntimeError(f"generated slot ({row}, {column}) is outside nominal arm reach")
        maximum_reach = max(maximum_reach, planar_reach)
        cumulative += MASS_KG[material]
        requests.append(
            {
                "schema_version": 2,
                "target_id": f"perception-track-20cube-{index + 1:02d}",
                "frame_id": "base_link",
                "pose": {
                    "x_m": x_m,
                    "y_m": y_m,
                    "z_m": 0.015,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": round(math.sin(yaw / 2.0), 9),
                    "qw": round(math.cos(yaw / 2.0), 9),
                },
                "size_m": [CUBE_EDGE_M, CUBE_EDGE_M, CUBE_EDGE_M],
                "material": "unknown",
                "confidence": 0.99,
                "truth_used": False,
                "acceptance": {
                    "scene_model_name": f"object_{index + 1:02d}",
                    "actual_material_evaluator_only": material,
                    # Colour is deliberately independent of material so the
                    # product camera cannot infer paperboard / polymer / metal
                    # from a fixed simulator palette.  Material remains an
                    # evaluator-only inertial property until the bin load
                    # increment is measured after release.
                    "random_color_rgb_evaluator_only": [
                        round(color_rng.uniform(0.12, 0.92), 6),
                        round(color_rng.uniform(0.12, 0.92), 6),
                        round(color_rng.uniform(0.12, 0.92), 6),
                    ],
                    "expected_increment_kg": MASS_KG[material],
                    "expected_count_after": index + 1,
                    "expected_cumulative_mass_kg": round(cumulative, 9),
                    "single_layer_slot": {"row": row, "column": column},
                    "planar_distance_from_arm_base_m": round(planar_reach, 9),
                },
            }
        )
    return {
        "schema_version": 1,
        "manifest_id": "tzcup_formal_target_conditioned_20_cube_grasp_v1",
        "seed": seed,
        "task_count": 20,
        "evaluator_materials": sorted(MASS_KG),
        "material_assignment": {
            "method": "seeded_random_permutation",
            "seed": seed,
            "count_per_material": 5,
            "evaluator_only": True,
        },
        "product_material_contract": "unknown",
        "expected_final_physical_resident_mass_kg": round(cumulative, 9),
        "expected_final_aggregate_dry_mass_kg": 0.0,
        "dry_payload_accounting": {
            "mode": "physical_resident",
            "aggregate_dry_mass_must_remain_kg": 0.0,
            "nonzero_aggregate_input_rejected": True,
            "load_transfer": "independent_rigid_bodies_contact",
        },
        "scene_contract": {
            "world_name": "formal_cube_manipulation",
            "vehicle_model_name": "tzcup_formal_sanitation_vehicle",
            "litter_model_prefix": "object_",
            "physical_rigid_bodies_retained_after_deposit": True,
            "initial_contained_object_count": 0,
            "initial_contained_mass_kg": 0.0,
        },
        "dry_bin_capacity_contract": {
            "maximum_count": 20,
            "single_layer": True,
            "stacking_allowed": False,
            "minimum_inter_cube_spacing_m": 0.005,
            "cube_edge_m": CUBE_EDGE_M,
            "grid_rows": GRID_ROWS,
            "grid_columns": GRID_COLUMNS,
            "grid_x_pitch_m": GRID_X_PITCH_M,
            "grid_y_pitch_m": GRID_Y_PITCH_M,
            "maximum_task_mass_kg": round(20 * max(MASS_KG.values()), 9),
            "structural_payload_margin_checked_at_runtime": True,
        },
        "arm_reach_contract": {
            "reference_frame": "base_link",
            "arm_base_xy_m": list(ARM_BASE_XY_M),
            "maximum_planar_reach_m": MAXIMUM_PLANAR_REACH_M,
            "maximum_generated_planar_reach_m": round(maximum_reach, 9),
            "final_reachability_authority": "runtime_moveit_ik_and_collision_check",
        },
        "runtime_requirements": {
            "maximum_attempts_per_target": 2,
            "retry_requires_safe_transport_restored": True,
            "retry_requires_unchanged_evaluator_payload": True,
            "duplicate_payload_accounting_forbidden": True,
            "move_group_action": "/move_action",
            "compute_ik_service": "/compute_ik",
            "cartesian_path_service": "/compute_cartesian_path",
            "wrist_recheck_topic": "/perception/wrist/grasp_recheck",
            "base_motion_inhibit_topic": "/manipulation/base_motion_inhibited",
            "physical_contact_and_bin_monitor_required": True,
        },
        "requests": requests,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=6020)
    args = parser.parse_args()
    payload = build_manifest(args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "tasks": 20, "mass_kg": payload["expected_final_physical_resident_mass_kg"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
