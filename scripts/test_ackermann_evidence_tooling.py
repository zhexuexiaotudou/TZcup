import json
from pathlib import Path
import sys

from ackermann_acceptance_status import GATES, derive_status
from ackermann_runtime_audit import SCENARIOS, evaluate, manifest
from generate_ackermann_inventory import main as generate_inventory


ROOT = Path(__file__).resolve().parents[1]


def _inventory(tmp_path, monkeypatch):
    monkeypatch.setattr(
        sys, "argv", ["generate_ackermann_inventory.py", "--output", str(tmp_path)]
    )
    assert generate_inventory() == 0
    return tmp_path


def test_all_required_source_generated_inventories_exist_and_are_traceable(
    tmp_path, monkeypatch
):
    inventory = _inventory(tmp_path, monkeypatch)
    expected = {
        "vehicle_geometry.json", "wheel_joint_inventory.json",
        "drive_plugin_inventory.json", "ros_gz_bridge_inventory.json",
        "odometry_tf_inventory.json", "localization_inventory.json",
        "nav2_inventory.json", "coverage_connector_inventory.json",
        "task_geometry_inventory.json", "regression_inventory.json",
    }
    assert expected.issubset({path.name for path in inventory.glob("*.json")})
    for name in expected:
        payload = json.loads((inventory / name).read_text(encoding="utf-8"))
        assert payload["generated_by"] == "scripts/generate_ackermann_inventory.py"
        assert payload.get("source") or payload.get("sources")


def test_inventory_preserves_frozen_geometry_and_cleanable_contract(tmp_path, monkeypatch):
    inventory = _inventory(tmp_path, monkeypatch)
    geometry = json.loads((inventory / "vehicle_geometry.json").read_text(encoding="utf-8"))
    assert geometry["wheelbase_m"] == 0.76
    assert geometry["front_track_m"] == 0.8
    assert geometry["frozen_steering"]["virtual_max_deg"] == 28.0
    assert geometry["frozen_steering"]["center_turning_radius_m"] > 1.429
    task = json.loads((inventory / "task_geometry_inventory.json").read_text(encoding="utf-8"))
    assert task["cleanable_area_m2"] == 12.0
    assert task["target_count"] == 10
    assert task["turning_apron_is_cleanable"] is False


def test_new_runtime_assets_are_installed_by_their_ros_packages():
    tasks_setup = (ROOT / "starter_ws/src/sanitation_tasks/setup.py").read_text(encoding="utf-8")
    assert '"config/competition_ackermann_demo_area.yaml"' in tasks_setup
    navigation_cmake = (ROOT / "starter_ws/src/sanitation_navigation/CMakeLists.txt").read_text(encoding="utf-8")
    assert "behavior_trees" in navigation_cmake
    visualization_package = (ROOT / "starter_ws/src/sanitation_gazebo_visualization/package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>sensor_msgs</exec_depend>" in visualization_package


def test_status_is_fail_closed_and_default_is_exact_conjunction(tmp_path):
    status = derive_status(tmp_path)
    assert status["first_blocking_layer"] == GATES[0]
    assert status["ACKERMANN_DEFAULT_PROFILE_READY"] is False
    for gate in GATES:
        (tmp_path / f"{gate.lower()}.json").write_text(
            json.dumps({"passed": True}), encoding="utf-8"
        )
    status = derive_status(tmp_path)
    assert status["ACKERMANN_DEFAULT_PROFILE_READY"] is True
    assert status["first_blocking_layer"] is None


def test_runtime_manifest_contains_full_formal_matrix_and_missing_fails(tmp_path):
    required_runs = {name: item["runs"] for name, item in manifest()["scenarios"].items()}
    assert required_runs == {
        "straight_5m": 1, "circle_matrix": 12,
        "steering_step_slalom": 1, "zero_speed_steering": 1,
        "three_point_turn": 1, "wheel_odometry": 1,
        "localization_30_seed": 30, "coverage_30_seed": 30,
        "dynamic_30_seed": 30, "estop_30": 30, "mcap_replay": 1,
    }
    report = evaluate(tmp_path)
    assert report["all_pass"] is False
    assert report["first_failure"] == next(iter(SCENARIOS))
