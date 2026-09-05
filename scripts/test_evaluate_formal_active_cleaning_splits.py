import importlib.util
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/evaluate_formal_active_cleaning_splits.py"
SPEC = importlib.util.spec_from_file_location("formal_split_eval", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_materializes_disjoint_formal_train_val_hidden_samples(tmp_path):
    tasks = MODULE.materialize_tasks(
        ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
        tmp_path,
        grid_resolution=2.0,
        sensing_radius=8.0,
        sensing_fov_rad=math.radians(86.0),
        max_steps=10,
    )
    assert tuple(task.split for task in tasks) == ("train", "val", "hidden")
    assert len({task.public_manifest["map_id"] for task in tasks}) == 3
    assert all(task.public_manifest["profile"] == "formal" for task in tasks)
    assert all(task.public_manifest["field"]["area_m2"] == 20_000.0 for task in tasks)
    assert all(len(task.layout.pedestrians) == 8 for task in tasks)
    assert all(len(task.layout.ground_dirt_polygons) == 18 for task in tasks)
    assert all(len(task.layout.discrete_targets) == 20 for task in tasks)
    assert all((task.directory / "public/world.sdf").is_file() for task in tasks)
    assert all((task.directory / "evaluator/ground_truth.json").is_file() for task in tasks)


def test_rectangular_dirty_region_and_formal_geometry_contract(tmp_path):
    task = MODULE.materialize_tasks(
        ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml",
        tmp_path,
        grid_resolution=2.0,
        sensing_radius=8.0,
        sensing_fov_rad=math.radians(86.0),
        max_steps=10,
    )[0]
    assert task.config.cleaning_width == 1.32
    assert task.config.observation_threshold == 0.95
    assert task.config.ground_clear_threshold == 0.95
    assert task.config.discrete_clear_threshold == 0.95
    polygon = task.layout.ground_dirt_polygons[0]
    doubled_area = abs(
        sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1])
        )
    )
    assert doubled_area / 2.0 == pytest.approx(1.0)
    assert task.public_manifest["vehicle"]["included"] is False


def test_report_contract_never_claims_product_perception():
    source = SCRIPT.read_text(encoding="utf-8")
    assert '"status": "BLOCKED_RESEARCH_ONLY"' in source
    assert '"formal_product_ready": False' in source
    assert '"product_perception_used": False' in source
    assert '"gazebo_sensor_streams_used": False' in source
    assert '"nav2_execution_used": False' in source
    assert '"truth_access_used": False' in source
