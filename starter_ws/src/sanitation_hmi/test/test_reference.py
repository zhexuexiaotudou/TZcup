import csv
import json

from sanitation_hmi.reference import load_real_replay, load_reference


def test_reference_loads_repository_truth_without_claiming_sensor_truth():
    root = __import__("pathlib").Path(__file__).resolve().parents[2]
    reference = load_reference(
        root / "sanitation_perception" / "config" / "garbage_registry.yaml",
        root / "sanitation_ground_truth" / "config" / "stage5a_scene.yaml",
        root / "sanitation_tasks" / "config" / "demo_area.yaml",
    )
    assert reference["truth_targets"]
    assert reference["mission"]["outer_polygon"]
    assert reference["scene"]["truth_boundary"].startswith("Gazebo")
    assert all(item["source"] == "simulation_truth" for item in reference["truth_targets"])


def test_replay_preserves_failed_execution_boundary(tmp_path):
    trajectory = tmp_path / "coverage_trajectory.csv"
    with trajectory.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["stamp_sec", "base_x_m", "base_y_m", "yaw_rad", "brush_enabled"],
        )
        writer.writeheader()
        writer.writerow({"stamp_sec": 1, "base_x_m": 0, "base_y_m": 0, "yaw_rad": 0, "brush_enabled": "false"})
        writer.writerow({"stamp_sec": 2, "base_x_m": 1, "base_y_m": 0, "yaw_rad": 0, "brush_enabled": "true"})
    report = tmp_path / "report.json"
    report.write_text(
        json.dumps({"success": False, "execution_boundary": "navigation_failed"}),
        encoding="utf-8",
    )
    replay = load_real_replay(trajectory, report)
    assert replay["success"] is False
    assert replay["execution_boundary"] == "navigation_failed"
    assert replay["samples"][1]["brush"] is True
    assert "不是实时运行" in replay["warning"]
