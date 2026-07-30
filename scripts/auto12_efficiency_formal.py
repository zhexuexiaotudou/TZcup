#!/usr/bin/env python3
"""Execute the AUTO-12 design search and formal offline matrix."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_tasks"))
from sanitation_tasks.efficiency import (  # noqa: E402
    aggregate_runs,
    search_designs,
    select_design,
    simulate_formal_run,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    profile_path = (
        ROOT
        / "starter_ws/src/sanitation_navigation/config/auto12_efficiency_v1.yaml"
    )
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    xacro_text = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/"
        "sanitation_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    sim_launch_text = (
        ROOT / "starter_ws/src/sanitation_bringup/launch/sim.launch.py"
    ).read_text(encoding="utf-8")
    nav2 = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_navigation/config/nav2_auto12.yaml"
        ).read_text(encoding="utf-8")
    )
    coverage = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_coverage/config/coverage_auto12.yaml"
        ).read_text(encoding="utf-8")
    )
    mission = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_tasks/config/demo_area_auto12.yaml"
        ).read_text(encoding="utf-8")
    )
    search = search_designs()
    design = select_design(search)
    runs = [simulate_formal_run(design, seed) for seed in range(10)]
    aggregate = aggregate_runs(runs)

    configured = profile["efficiency_design"]
    synchronization = {
        "physical_model_width_matches": (
            configured["physical_model"]["cleaning_width_m"]
            == design.cleaning_width_m
            and '<xacro:arg name="cleaning_width" default="0.65"/>' in xacro_text
            and 'value="$(arg cleaning_width)"' in xacro_text
            and '" cleaning_width:=", cleaning_width' in sim_launch_text
        ),
        "cleaning_footprint_width_matches": configured["cleaning_footprint"]["width_m"]
        == design.cleaning_width_m,
        "collision_width_matches": (
            configured["collision_geometry"]["width_m"]
            == design.cleaning_width_m
            and nav2["local_costmap"]["local_costmap"]["ros__parameters"][
                "footprint"
            ]
            == "[[0.72, 0.66], [0.72, -0.66], [-0.58, -0.66], [-0.58, 0.66]]"
            and nav2["global_costmap"]["global_costmap"]["ros__parameters"][
                "footprint"
            ]
            == "[[0.72, 0.66], [0.72, -0.66], [-0.58, -0.66], [-0.58, 0.66]]"
        ),
        "dynamics_speed_matches": configured["dynamics"]["max_cleaning_speed_m_s"]
        == design.speed_m_s,
        "energy_model_present": configured["energy"]["schema_version"] == 1,
        "costmap_width_matches": (
            configured["costmap"]["width_m"] == design.cleaning_width_m
            and nav2["velocity_smoother"]["ros__parameters"]["max_velocity"][0]
            == design.speed_m_s
            and nav2["velocity_smoother"]["ros__parameters"]["max_accel"][0]
            == design.acceleration_m_s2
            and abs(
                nav2["velocity_smoother"]["ros__parameters"]["max_decel"][0]
            )
            == design.deceleration_m_s2
        ),
        "coverage_width_matches": (
            configured["coverage"]["operation_width_m"]
            == design.cleaning_width_m
            and coverage["coverage_server"]["ros__parameters"]["operation_width"]
            == design.cleaning_width_m
            and mission["operation_width_m"] == design.cleaning_width_m
            and mission["robot_footprint"]
            == configured["collision_geometry"]["footprint_xy_m"]
        ),
        "auto02_equivalent_safety_regression": (
            configured["collision_geometry"]["contains_all_extended_brush_geometry"]
            and configured["safety"]["fail_closed_on_extension_state_unknown"]
            and design.braking_distance_m
            <= configured["safety"]["braking_envelope_m"]
        ),
    }
    checks = {
        "cleaning_width_at_least_0_60_m": design.cleaning_width_m >= 0.60,
        "theoretical_rate_at_least_3800_m2_h": design.theoretical_rate_m2_h
        >= 3800.0,
        "control_stable_at_candidate_speed": all(
            row["control_stable"] for row in runs
        ),
        "braking_distance_within_safety_envelope": design.braking_distance_m
        <= design.safety_braking_envelope_m,
        "all_width_consumers_synchronized": all(synchronization.values()),
        "formal_runs_at_least_10": aggregate["formal_run_count"] >= 10,
        "mean_rate_at_least_3500_m2_h": aggregate[
            "mean_effective_cleaning_rate_m2_h"
        ]
        >= 3500.0,
        "rate_95ci_lower_at_least_3500_m2_h": aggregate[
            "rate_95ci_lower_m2_h"
        ]
        >= 3500.0,
        "each_run_at_least_3300_m2_h": aggregate["minimum_run_rate_m2_h"]
        >= 3300.0,
        "empirical_coverage_at_least_0_90": aggregate[
            "minimum_empirical_coverage"
        ]
        >= 0.90,
        "missed_cleanable_area_at_most_0_05": aggregate[
            "maximum_missed_cleanable_area_ratio"
        ]
        <= 0.05,
        "overlap_ratio_at_most_0_15": aggregate["maximum_overlap_ratio"] <= 0.15,
        "collision_zero": aggregate["collision_count"] == 0,
        "keepout_zero": aggregate["keepout_violation_count"] == 0,
        "trajectory_xy_rmse_at_most_0_05_m": aggregate[
            "maximum_trajectory_xy_rmse_m"
        ]
        <= 0.05,
        "brush_final_false": aggregate["brush_final_false_count"] == len(runs),
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-12",
        "attempt_id": "AUTO-12-EFFICIENCY-V1",
        "implementation_commit": args.implementation_commit,
        "source_level": "OFFLINE_TIME_STEP_DYNAMICS_AND_RASTER_SIMULATION",
        "truth_boundary": (
            "No Gazebo or physical-vehicle measurement is claimed. Effective rate "
            "uses raster-verified swept area and includes turns, obstacle stops, "
            "interruptions, and in-mission staging."
        ),
        "selected_design": asdict(design),
        "selected_theoretical_rate_m2_h": design.theoretical_rate_m2_h,
        "selected_braking_distance_m": design.braking_distance_m,
        "design_search": search,
        "configuration_synchronization": synchronization,
        "runs": runs,
        "aggregate": aggregate,
        "checks": checks,
        "auto12_gate_pass": all(checks.values()),
        "competition_efficiency_pass": all(checks.values()),
    }
    (output / "formal_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (output / "run_metrics.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in runs),
        encoding="utf-8",
    )
    (output / "design_search.json").write_text(
        json.dumps(search, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"aggregate": aggregate, "checks": checks}, indent=2))
    return 0 if report["auto12_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
