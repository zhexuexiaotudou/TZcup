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
    with pytest.raises(ValueError, match="must not materialize hidden"):
        MODULE.materialize_tasks(ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml", tmp_path, grid_resolution=2.0, sensing_radius=8.0, sensing_fov_rad=math.radians(86.0), max_steps=10)


def test_rectangular_dirty_region_and_formal_geometry_contract(tmp_path):
    with pytest.raises(ValueError, match="must not materialize hidden"):
        MODULE.materialize_tasks(ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml", tmp_path, grid_resolution=2.0, sensing_radius=8.0, sensing_fov_rad=math.radians(86.0), max_steps=10)


def test_report_contract_never_claims_product_perception():
    source = SCRIPT.read_text(encoding="utf-8")
    assert '"status": "BLOCKED_RESEARCH_ONLY"' in source
    assert '"formal_product_ready": False' in source
    assert '"product_perception_used": False' in source
    assert '"gazebo_sensor_streams_used": False' in source
    assert '"nav2_execution_used": False' in source
    assert '"truth_access_used": False' in source
