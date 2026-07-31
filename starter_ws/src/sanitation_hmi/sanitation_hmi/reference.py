"""Load human-readable reference geometry without feeding it into control."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import yaml


def load_reference(
    registry_path: str | Path,
    scene_path: str | Path,
    mission_path: str | Path,
) -> dict[str, Any]:
    registry = yaml.safe_load(Path(registry_path).read_text(encoding="utf-8"))
    scene = yaml.safe_load(Path(scene_path).read_text(encoding="utf-8"))
    mission = yaml.safe_load(Path(mission_path).read_text(encoding="utf-8"))
    tx, ty = (float(value) for value in scene.get("world_to_map_translation", [0, 0]))

    truth = []
    for name, item in scene.get("objects", {}).items():
        model = registry.get("models", {}).get(name)
        if not model:
            continue
        pose = item.get("pose_world", [0, 0, 0, 0])
        truth.append(
            {
                "uuid": str(model["uuid"]),
                "name": name,
                "class_id": str(model["class_id"]),
                "target_type": str(model.get("target_type", "unknown")),
                "position": [float(pose[0]) + tx, float(pose[1]) + ty, float(pose[2])],
                "yaw": float(pose[3]),
                "size": [float(value) for value in model.get("size_m", [0.1, 0.1, 0.1])],
                "source": "simulation_truth",
            }
        )

    obstacles = []
    for name, item in scene.get("negative_objects", {}).items():
        pose = item.get("pose_world", [0, 0])
        model = registry.get("negative_models", {}).get(name, {})
        obstacles.append(
            {
                "uuid": str(model.get("uuid", name)),
                "name": name,
                "class_id": str(model.get("class_id", "obstacle")),
                "position": [float(pose[0]) + tx, float(pose[1]) + ty],
                "radius_m": float(item.get("radius_m", 0.35)),
                "dynamic": "dynamic" in name or "pedestrian" in name,
                "source": "simulation_truth",
            }
        )

    return {
        "scene": {
            "id": str(scene.get("world_name", "unknown")),
            "name": "园区环卫示范道路",
            "frame_id": str(mission.get("frame_id", "map")),
            "truth_boundary": "Gazebo 配置参考真值，仅用于显示和评测",
        },
        "mission": {
            "id": str(mission.get("mission_id", "unknown")),
            "outer_polygon": mission.get("outer_polygon", []),
            "keepout_polygons": mission.get("keepout_polygons", []),
            "exclusion_polygons": mission.get("exclusion_polygons", []),
            "operation_width_m": float(mission.get("operation_width_m", 0.65)),
            "speed_zones": mission.get("speed_zones", []),
        },
        "truth_targets": truth,
        "obstacles": obstacles,
        "map_semantics": {
            "reference": "Gazebo world 和项目配置",
            "slam": "车辆传感器与 SLAM 输出 /map",
            "operation": "任务语义、规划结果与实际执行结果叠加",
        },
    }

def load_real_replay(
    trajectory_csv: str | Path,
    report_json: str | Path | None = None,
    *,
    max_samples: int = 900,
) -> dict[str, Any]:
    """Load a recorded trajectory and preserve its original success boundary."""

    trajectory_path = Path(trajectory_csv)
    rows: list[dict[str, Any]] = []
    with trajectory_path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append(
                {
                    "t": float(row["stamp_sec"]),
                    "x": float(row["base_x_m"]),
                    "y": float(row["base_y_m"]),
                    "yaw": float(row["yaw_rad"]),
                    "brush": row["brush_enabled"].strip().lower() == "true",
                }
            )
    if not rows:
        raise ValueError("replay trajectory is empty")
    stride = max(1, (len(rows) + max_samples - 1) // max_samples)
    sampled = rows[::stride]
    if sampled[-1] != rows[-1]:
        sampled.append(rows[-1])

    report: dict[str, Any] = {}
    if report_json and Path(report_json).is_file():
        report = json.loads(Path(report_json).read_text(encoding="utf-8"))
    return {
        "id": trajectory_path.stem,
        "label": "Stage4R 历史真实记录",
        "source": str(trajectory_path),
        "mode_label": "历史回放",
        "samples": sampled,
        "sample_count_original": len(rows),
        "success": report.get("success"),
        "execution_boundary": report.get("execution_boundary", "未提供任务报告"),
        "planned_metrics": report.get("planned_metrics"),
        "empirical_metrics": report.get("empirical_metrics"),
        "warning": (
            "这是历史真实记录，不是实时运行。记录中的失败状态原样保留，"
            "不得把规划覆盖率解释为实际清扫覆盖率。"
        ),
    }
