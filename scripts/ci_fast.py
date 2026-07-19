#!/usr/bin/env python3
"""Fast, ROS-independent validation used by the 开发工作流 CI gate."""

from __future__ import annotations

import compileall
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "starter_ws" / "src"


def require_project_files() -> None:
    required = (
        ROOT / "AGENTS.md",
        ROOT / "README.md",
        ROOT / "README_FIRST.md",
        ROOT / "PROJECT_SPEC.md",
        ROOT / "STAGE_GATES.md",
        ROOT / "docs" / "development-workflow.md",
        ROOT / ".github" / "workflows" / "development-workflow.yml",
    )
    missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"missing required project files: {', '.join(missing)}")


def validate_python() -> None:
    targets = (SOURCE_ROOT, ROOT / "scripts")
    failed = [str(path) for path in targets if not compileall.compile_dir(path, quiet=1)]
    if failed:
        raise RuntimeError(f"Python compilation failed under: {', '.join(failed)}")


def validate_structured_files() -> None:
    for path in sorted(SOURCE_ROOT.rglob("*.json")):
        json.loads(path.read_text(encoding="utf-8"))

    for pattern in ("*.yaml", "*.yml"):
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            yaml.safe_load(path.read_text(encoding="utf-8"))

    xml_patterns = ("package.xml", "*.xacro", "*.sdf", "*.urdf")
    seen: set[Path] = set()
    for pattern in xml_patterns:
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            if path not in seen:
                ET.parse(path)
                seen.add(path)


def validate_stage4w_runtime_contract() -> None:
    nav2_path = SOURCE_ROOT / "sanitation_navigation" / "config" / "nav2.yaml"
    nav2 = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    controller = nav2["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    if progress["plugin"] != "nav2_controller::PoseProgressChecker":
        raise RuntimeError("Stage4W requires PoseProgressChecker")
    if float(controller["failure_tolerance"]) != 5.0:
        raise RuntimeError("Stage4W controller failure_tolerance must be 5.0 s")
    for costmap_name in ("local_costmap", "global_costmap"):
        obstacle_scan = nav2[costmap_name][costmap_name]["ros__parameters"][
            "obstacle_layer"
        ]["scan"]
        if obstacle_scan.get("inf_is_valid") is not True:
            raise RuntimeError(
                f"Stage4W {costmap_name} must clear infinite-range laser rays"
            )


def run_ros_independent_tests() -> None:
    coverage_package = SOURCE_ROOT / "sanitation_coverage"
    tasks_package = SOURCE_ROOT / "sanitation_tasks"
    gnss_package = SOURCE_ROOT / "sanitation_gnss_sim"
    perception_package = SOURCE_ROOT / "sanitation_perception"
    dataset_package = SOURCE_ROOT / "sanitation_dataset"
    ground_truth_package = SOURCE_ROOT / "sanitation_ground_truth"
    spot_cleaning_package = SOURCE_ROOT / "sanitation_spot_cleaning"
    learning_package = SOURCE_ROOT / "sanitation_learning"
    sys.path.insert(0, str(coverage_package))
    sys.path.insert(0, str(tasks_package))
    sys.path.insert(0, str(gnss_package))
    sys.path.insert(0, str(perception_package))
    sys.path.insert(0, str(dataset_package))
    sys.path.insert(0, str(ground_truth_package))
    sys.path.insert(0, str(spot_cleaning_package))
    sys.path.insert(0, str(learning_package))
    test_paths = (
        coverage_package / "test" / "test_metrics.py",
        coverage_package / "test" / "test_stage4w_geometry.py",
        tasks_package / "test" / "test_localization_metrics.py",
        tasks_package / "test" / "test_stage4t_localization_aggregate.py",
        tasks_package / "test" / "test_stage4v_localization_aggregate.py",
        tasks_package / "test" / "test_dynamic_geometry.py",
        tasks_package / "test" / "test_stage4w_dynamic_aggregate.py",
        gnss_package / "test" / "test_model.py",
        perception_package / "test" / "test_registry.py",
        perception_package / "test" / "test_projection.py",
        perception_package / "test" / "test_tracking.py",
        perception_package / "test" / "test_backends.py",
        dataset_package / "test" / "test_synthetic.py",
        ground_truth_package / "test" / "test_visibility.py",
        spot_cleaning_package / "test" / "test_coordinator.py",
        learning_package / "test" / "test_assets.py",
        learning_package / "test" / "test_rendered.py",
    )
    result = pytest.main(["-q", *(str(path) for path in test_paths)])
    if result != pytest.ExitCode.OK:
        raise RuntimeError(f"ROS-independent pytest gate failed with exit code {int(result)}")


def main() -> int:
    require_project_files()
    validate_python()
    validate_structured_files()
    validate_stage4w_runtime_contract()
    run_ros_independent_tests()
    print("development workflow fast validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
