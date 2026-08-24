import math
from pathlib import Path
import time

from sanitation_active_cleaning.evaluation import evaluate_paired
from sanitation_active_cleaning.models import TaskConfig


def trivial_config():
    return TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [4, 0], [4, 3], [0, 3]],
            "start": {"x": 1.0, "y": 1.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 20.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.1,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 10,
        }
    )


def test_paired_report_uses_same_seeds_and_excludes_return_time_and_energy():
    report = evaluate_paired(trivial_config(), seeds=[11, 12, 13])
    assert report["paired_seeds"] == [11, 12, 13]
    assert report["truth_boundary"] == "evaluation_token_only"
    assert report["time_energy_ignored"] is True
    assert set(report["summaries"]) == {"full_coverage", "sensing_greedy", "oracle"}
    assert len(report["episodes"]) == 9
    assert all(row["return_distance_included"] is False for row in report["episodes"])
    assert all(set(row["role_seeds"]) == {"layout", "dynamics", "grasp", "policy"} for row in report["episodes"])
    assert all(row["distance_gate"] for row in report["episodes"])
    assert all(row["success"] for row in report["episodes"])
    for summary in report["summaries"].values():
        assert set(summary["task_distance"]) == {"mean", "ci95_low", "ci95_high", "p10", "worst"}


def test_demo_seed_is_bounded_and_finishes_quickly():
    config_path = Path(__file__).parents[1] / "config" / "demo_task.json"
    task = TaskConfig.from_json(config_path)
    started = time.perf_counter()
    report = evaluate_paired(task, seeds=[101])
    elapsed = time.perf_counter() - started

    assert elapsed < 10.0
    assert all(row["steps"] <= task.max_steps for row in report["episodes"])
    assert all(row["success"] for row in report["episodes"])
