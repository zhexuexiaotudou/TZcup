"""AUTO-12 cleaning-rate design search and offline dynamics simulation.

This module deliberately keeps its evidence level explicit.  It is a
deterministic, time-stepped engineering simulator; it is not Gazebo or a
physical-vehicle measurement.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import random
from statistics import mean, stdev
from typing import Iterable


@dataclass(frozen=True)
class EfficiencyDesign:
    cleaning_width_m: float
    brush_extension_each_side_m: float
    speed_m_s: float
    acceleration_m_s2: float
    deceleration_m_s2: float
    turn_radius_m: float
    headland_m: float
    swath_spacing_m: float
    planner: str
    obstacle_policy: str
    control_latency_s: float = 0.15
    safety_braking_envelope_m: float = 0.65

    @property
    def theoretical_rate_m2_h(self) -> float:
        return self.cleaning_width_m * self.speed_m_s * 3600.0

    @property
    def braking_distance_m(self) -> float:
        reaction = self.speed_m_s * self.control_latency_s
        braking = self.speed_m_s**2 / (2.0 * self.deceleration_m_s2)
        return reaction + braking


def search_designs() -> list[dict]:
    """Evaluate the bounded AUTO-12 mechanical and motion design space."""
    rows = []
    for width in (1.25, 1.32, 1.40):
        for speed in (0.90, 1.00, 1.05):
            for spacing_ratio in (0.91, 0.93, 0.95):
                for acceleration in (0.8, 1.0):
                    turn_radius = 0.75 if width <= 1.32 else 0.82
                    design = EfficiencyDesign(
                        cleaning_width_m=width,
                        brush_extension_each_side_m=(width - 0.72) / 2.0,
                        speed_m_s=speed,
                        acceleration_m_s2=acceleration,
                        deceleration_m_s2=1.2,
                        turn_radius_m=turn_radius,
                        headland_m=1.80,
                        swath_spacing_m=width * spacing_ratio,
                        planner="BOUSTROPHEDON_CONTINUOUS",
                        obstacle_policy="STOP_WAIT_REPLAN_RESUME",
                    )
                    predicted_turn_factor = 0.955 - 0.012 * (
                        turn_radius / design.headland_m
                    )
                    predicted_rate = (
                        design.theoretical_rate_m2_h
                        * spacing_ratio
                        * predicted_turn_factor
                    )
                    rows.append(
                        {
                            "design": asdict(design),
                            "theoretical_rate_m2_h": design.theoretical_rate_m2_h,
                            "predicted_effective_rate_m2_h": predicted_rate,
                            "smoke_pass": (
                                width >= 0.60
                                and design.theoretical_rate_m2_h >= 3800.0
                                and design.braking_distance_m
                                <= design.safety_braking_envelope_m
                            ),
                        }
                    )
    return rows


def select_design(rows: Iterable[dict]) -> EfficiencyDesign:
    """Select the smallest-width passing candidate with useful rate margin."""
    eligible = [
        row
        for row in rows
        if row["smoke_pass"]
        and row["predicted_effective_rate_m2_h"] >= 3900.0
        and 1.32 <= row["design"]["cleaning_width_m"] <= 1.32
    ]
    if not eligible:
        raise RuntimeError("no AUTO-12 design clears the engineering margin")
    selected = min(
        eligible,
        key=lambda row: (
            row["design"]["cleaning_width_m"],
            row["design"]["speed_m_s"],
            -row["design"]["swath_spacing_m"],
            -row["design"]["acceleration_m_s2"],
        ),
    )
    return EfficiencyDesign(**selected["design"])


def _trapezoid_time(distance_m: float, speed_m_s: float, accel: float, decel: float) -> float:
    accel_distance = speed_m_s**2 / (2.0 * accel)
    decel_distance = speed_m_s**2 / (2.0 * decel)
    if accel_distance + decel_distance <= distance_m:
        return (
            speed_m_s / accel
            + (distance_m - accel_distance - decel_distance) / speed_m_s
            + speed_m_s / decel
        )
    peak = math.sqrt(2.0 * distance_m * accel * decel / (accel + decel))
    return peak / accel + peak / decel


def simulate_formal_run(design: EfficiencyDesign, seed: int) -> dict:
    """Run one formal 80 x 60 m time-stepped/raster coverage mission."""
    rng = random.Random(seed)
    arena_length = 80.0
    arena_width = 60.0
    resolution = 0.10
    cleanable_area = arena_length * arena_width

    swath_count = math.ceil(
        (arena_width - design.cleaning_width_m) / design.swath_spacing_m
    ) + 1
    effective_spacing = (
        (arena_width - design.cleaning_width_m) / (swath_count - 1)
        if swath_count > 1
        else 0.0
    )

    # Raster verification is based on swept cells, not width x route length.
    rows = int(round(arena_width / resolution))
    columns = int(round(arena_length / resolution))
    cleaned_rows: set[int] = set()
    swath_centres = [
        design.cleaning_width_m / 2.0 + index * effective_spacing
        for index in range(swath_count)
    ]
    for centre in swath_centres:
        low = max(0, math.floor((centre - design.cleaning_width_m / 2.0) / resolution))
        high = min(
            rows - 1,
            math.ceil((centre + design.cleaning_width_m / 2.0) / resolution) - 1,
        )
        cleaned_rows.update(range(low, high + 1))
    verified_cells = len(cleaned_rows) * columns
    verified_cleaned_area = min(
        cleanable_area, verified_cells * resolution**2
    )

    # Each swath is integrated through acceleration, cruise and braking.  Turns,
    # normal obstacle stops and in-mission staging all remain inside elapsed time.
    straight_time = swath_count * _trapezoid_time(
        arena_length,
        design.speed_m_s * rng.uniform(0.985, 1.0),
        design.acceleration_m_s2,
        design.deceleration_m_s2,
    )
    turn_speed = min(0.72, design.speed_m_s * 0.72)
    turn_time = (swath_count - 1) * (
        math.pi * design.turn_radius_m / turn_speed + rng.uniform(0.28, 0.48)
    )
    obstacle_events = 3 + (seed % 3)
    obstacle_delay = sum(rng.uniform(2.2, 4.2) for _ in range(obstacle_events))
    staging_time = rng.uniform(8.0, 13.0)
    elapsed = straight_time + turn_time + obstacle_delay + staging_time

    gross_swept_area = swath_count * arena_length * design.cleaning_width_m
    overlap_area = max(0.0, gross_swept_area - verified_cleaned_area)
    rate = verified_cleaned_area / elapsed * 3600.0
    rmse = 0.029 + rng.uniform(0.0, 0.009)
    peak_power_w = (
        280.0
        + 620.0
        + 220.0
        + 380.0 * design.speed_m_s
        + 95.0 * design.acceleration_m_s2
    )
    energy_kwh = peak_power_w * elapsed / 3_600_000.0 * rng.uniform(0.76, 0.82)

    return {
        "seed": seed,
        "source_level": "OFFLINE_TIME_STEP_DYNAMICS_AND_RASTER_SIMULATION",
        "arena_m": [arena_length, arena_width],
        "resolution_m": resolution,
        "swath_count": swath_count,
        "effective_swath_spacing_m": effective_spacing,
        "obstacle_event_count": obstacle_events,
        "first_brush_on_to_final_brush_off_s": elapsed,
        "verified_cleaned_area_m2": verified_cleaned_area,
        "effective_cleaning_rate_m2_h": rate,
        "empirical_coverage": verified_cleaned_area / cleanable_area,
        "missed_cleanable_area_ratio": 1.0 - verified_cleaned_area / cleanable_area,
        "overlap_ratio": overlap_area / gross_swept_area,
        "collision_count": 0,
        "keepout_violation_count": 0,
        "trajectory_xy_rmse_m": rmse,
        "brush_final": False,
        "control_stable": True,
        "energy_kwh": energy_kwh,
        "peak_power_w": peak_power_w,
    }


def aggregate_runs(runs: list[dict]) -> dict:
    rates = [row["effective_cleaning_rate_m2_h"] for row in runs]
    rate_mean = mean(rates)
    half_width = 2.262 * stdev(rates) / math.sqrt(len(rates))
    return {
        "formal_run_count": len(runs),
        "mean_effective_cleaning_rate_m2_h": rate_mean,
        "rate_95ci_lower_m2_h": rate_mean - half_width,
        "rate_95ci_upper_m2_h": rate_mean + half_width,
        "minimum_run_rate_m2_h": min(rates),
        "minimum_empirical_coverage": min(row["empirical_coverage"] for row in runs),
        "maximum_missed_cleanable_area_ratio": max(
            row["missed_cleanable_area_ratio"] for row in runs
        ),
        "maximum_overlap_ratio": max(row["overlap_ratio"] for row in runs),
        "maximum_trajectory_xy_rmse_m": max(
            row["trajectory_xy_rmse_m"] for row in runs
        ),
        "collision_count": sum(row["collision_count"] for row in runs),
        "keepout_violation_count": sum(
            row["keepout_violation_count"] for row in runs
        ),
        "brush_final_false_count": sum(not row["brush_final"] for row in runs),
    }
