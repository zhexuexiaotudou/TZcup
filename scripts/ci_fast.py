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
AUTONOMY_CONFIG_ROOT = ROOT / "config" / "autonomy"
REPORTS_ROOT = ROOT / "reports"


def require_project_files() -> None:
    required = (
        ROOT / "AGENTS.md",
        ROOT / "README.md",
        ROOT / "README_FIRST.md",
        ROOT / "PROJECT_SPEC.md",
        ROOT / "STAGE_GATES.md",
        ROOT / "docs" / "development-workflow.md",
        ROOT / "docs" / "artifact-policy.md",
        ROOT / ".github" / "workflows" / "development-workflow.yml",
        AUTONOMY_CONFIG_ROOT / "README.md",
        AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_STATE.json",
        AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_RUN_PLAN.json",
        ROOT / "config" / "autonomous_stage_registry.yaml",
        REPORTS_ROOT / "README.md",
        REPORTS_ROOT / "release" / "FINAL_AUTONOMOUS_STATUS.json",
        REPORTS_ROOT / "release" / "FINAL_BLOCKER_REGISTER.json",
        ROOT / "scripts" / "autonomous_runner.py",
        ROOT / "scripts" / "verify_evidence_manifest.py",
        ROOT / "scripts" / "verify_state_invariants.py",
        ROOT / "scripts" / "scan_secrets.py",
        ROOT / "docs" / "pre-urdf-readiness.md",
        ROOT / "config" / "high_fidelity_vehicle" / "pre_urdf_contract.yaml",
        ROOT / "config" / "high_fidelity_vehicle" / "mass_budget.csv",
        ROOT / "config" / "high_fidelity_vehicle" / "power_budget.csv",
        ROOT / "config" / "high_fidelity_vehicle" / "capacity_budget.csv",
        ROOT / "config" / "high_fidelity_vehicle" / "throughput_budget.csv",
        ROOT / "repos" / "high_fidelity_vehicle.repos",
        REPORTS_ROOT / "engineering" / "pre_urdf_readiness.json",
        ROOT / "scripts" / "validate_pre_urdf_readiness.py",
        ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_layout.yaml",
        SOURCE_ROOT / "sanitation_vehicle_description" / "urdf" / "formal_competition_vehicle.urdf.xacro",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "formal_vehicle" / "formal_vehicle_layout.scad",
        REPORTS_ROOT / "engineering" / "formal_competition_vehicle.urdf",
        REPORTS_ROOT / "engineering" / "formal_vehicle_layout_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_urdf_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_runtime_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_preview.png",
        ROOT / "scripts" / "validate_formal_vehicle_urdf.py",
        ROOT / "scripts" / "render_formal_vehicle_preview.py",
    )
    missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"missing required project files: {', '.join(missing)}")


def validate_repository_hygiene() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    readme_first = (ROOT / "README_FIRST.md").read_text(encoding="utf-8")
    if len(readme.splitlines()) > 180:
        raise RuntimeError("README.md must remain a concise project front door")
    if len(readme_first.splitlines()) > 180:
        raise RuntimeError("README_FIRST.md must remain a concise startup guide")
    forbidden_headings = ("## 最近同步", "## AUTO-", "## Stage")
    found = [heading for heading in forbidden_headings if heading in readme]
    if found:
        raise RuntimeError(
            "README.md contains progress-log headings: " + ", ".join(found)
        )

    root_json = sorted(path.name for path in ROOT.glob("*.json"))
    allowed_legacy_reviews = {"GPT_REVIEW_STAGE4V.md"}
    root_reviews = sorted(
        path.name
        for pattern in ("GPT_REVIEW_*.md", "ENGINEERING_WAIVER_*.md")
        for path in ROOT.glob(pattern)
        if path.name not in allowed_legacy_reviews
    )
    if root_json or root_reviews:
        raise RuntimeError(
            "root-level generated files must use config/autonomy or reports/: "
            + ", ".join(root_json + root_reviews)
        )

    artifact_root = ROOT / "artifacts"
    raw_patterns = (
        "stage0_*",
        "stage1_*",
        "stage2_*",
        "stage3_*",
        "stage4_[0-9]*",
        "stage4r_[0-9]*",
        "stage4s_gt_*",
        "stage4s_fit_*",
        "stage4s_friction_*",
        "stage4s_motion_*",
    )
    raw_dirs = sorted(
        str(path.relative_to(ROOT))
        for pattern in raw_patterns
        for path in artifact_root.glob(pattern)
        if path.is_dir()
    )
    raw_payloads = sorted(
        str(path.relative_to(ROOT))
        for suffix in ("*.mcap", "*.db3")
        for path in artifact_root.rglob(suffix)
    )
    if raw_dirs or raw_payloads:
        raise RuntimeError(
            "raw artifacts must stay outside Git: "
            + ", ".join(raw_dirs + raw_payloads)
        )


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

    json.loads(
        (AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_STATE.json").read_text(
            encoding="utf-8"
        )
    )
    json.loads(
        (AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_RUN_PLAN.json").read_text(
            encoding="utf-8"
        )
    )
    yaml.safe_load(
        (ROOT / "config" / "autonomous_stage_registry.yaml").read_text(encoding="utf-8")
    )
    yaml.safe_load(
        (ROOT / "config" / "high_fidelity_vehicle" / "pre_urdf_contract.yaml").read_text(
            encoding="utf-8"
        )
    )
    yaml.safe_load(
        (ROOT / "repos" / "high_fidelity_vehicle.repos").read_text(encoding="utf-8")
    )
    json.loads(
        (REPORTS_ROOT / "engineering" / "pre_urdf_readiness.json").read_text(
            encoding="utf-8"
        )
    )

    xml_patterns = ("package.xml", "*.xacro", "*.sdf", "*.urdf", "*.srdf")
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
    hmi_package = SOURCE_ROOT / "sanitation_hmi"
    manipulation_package = SOURCE_ROOT / "sanitation_manipulation"
    debug_visualization_package = SOURCE_ROOT / "sanitation_debug_visualization"
    gazebo_visualization_package = SOURCE_ROOT / "sanitation_gazebo_visualization"
    active_cleaning_package = SOURCE_ROOT / "sanitation_active_cleaning"
    campus_scenario_package = SOURCE_ROOT / "sanitation_campus_scenario"
    research_demo_package = SOURCE_ROOT / "sanitation_research_demo"
    sys.path.insert(0, str(coverage_package))
    sys.path.insert(0, str(tasks_package))
    sys.path.insert(0, str(gnss_package))
    sys.path.insert(0, str(perception_package))
    sys.path.insert(0, str(dataset_package))
    sys.path.insert(0, str(ground_truth_package))
    sys.path.insert(0, str(spot_cleaning_package))
    sys.path.insert(0, str(learning_package))
    sys.path.insert(0, str(hmi_package))
    sys.path.insert(0, str(manipulation_package))
    sys.path.insert(0, str(debug_visualization_package))
    sys.path.insert(0, str(gazebo_visualization_package))
    sys.path.insert(0, str(active_cleaning_package))
    sys.path.insert(0, str(campus_scenario_package))
    sys.path.insert(0, str(research_demo_package))
    test_paths = (
        coverage_package / "test" / "test_metrics.py",
        coverage_package / "test" / "test_stage4w_geometry.py",
        tasks_package / "test" / "test_localization_metrics.py",
        tasks_package / "test" / "test_stage4t_localization_aggregate.py",
        tasks_package / "test" / "test_stage4v_localization_aggregate.py",
        tasks_package / "test" / "test_dynamic_geometry.py",
        tasks_package / "test" / "test_stage4w_dynamic_aggregate.py",
        tasks_package / "test" / "test_auto11_large_map.py",
        gnss_package / "test" / "test_model.py",
        perception_package / "test" / "test_registry.py",
        perception_package / "test" / "test_projection.py",
        perception_package / "test" / "test_tracking.py",
        perception_package / "test" / "test_backends.py",
        perception_package / "test" / "test_preprocessing.py",
        perception_package / "test" / "test_j6_runtime.py",
        perception_package / "test" / "test_open_vocab.py",
        dataset_package / "test" / "test_synthetic.py",
        ground_truth_package / "test" / "test_visibility.py",
        spot_cleaning_package / "test" / "test_coordinator.py",
        spot_cleaning_package / "test" / "test_active_observation.py",
        spot_cleaning_package / "test" / "test_observation_pose_planner.py",
        spot_cleaning_package / "test" / "test_stage5br6w_engineering.py",
        spot_cleaning_package / "test" / "test_auto03_contract.py",
        spot_cleaning_package / "test" / "test_auto03_replay_audit.py",
        learning_package / "test" / "test_assets.py",
        learning_package / "test" / "test_rendered.py",
        learning_package / "test" / "test_gazebo_g1.py",
        learning_package / "test" / "test_g1_collector.py",
        learning_package / "test" / "test_g2_contract.py",
        learning_package / "test" / "test_g2_metrics.py",
        learning_package / "test" / "test_gazebo_g2.py",
        learning_package / "test" / "test_stage5br6_handoff.py",
        learning_package / "test" / "test_auto04_contract.py",
        learning_package / "test" / "test_g3_contract.py",
        learning_package / "test" / "test_auto13_real_domain.py",
        hmi_package / "test" / "test_dsl.py",
        hmi_package / "test" / "test_gateway.py",
        hmi_package / "test" / "test_live_state.py",
        hmi_package / "test" / "test_state.py",
        hmi_package / "test" / "test_reference.py",
        hmi_package / "test" / "test_ros_adapter.py",
        hmi_package / "test" / "test_server.py",
        *sorted((manipulation_package / "test").glob("test_*.py")),
        debug_visualization_package / "test" / "test_debug_visualization_model.py",
        gazebo_visualization_package / "test" / "test_coverage_telemetry_v2.py",
        *sorted((active_cleaning_package / "test").glob("test_*.py")),
        *sorted((campus_scenario_package / "test").glob("test_*.py")),
        *sorted((research_demo_package / "test").glob("test_*.py")),
        active_cleaning_package / "test" / "test_performance.py",
        spot_cleaning_package / "test" / "test_auto01_geometry.py",
        ROOT / "scripts" / "test_autonomous_runner.py",
        ROOT / "scripts" / "test_auto02_tools.py",
        ROOT / "scripts" / "test_auto03_matrix.py",
        ROOT / "scripts" / "test_auto10_formal.py",
        ROOT / "scripts" / "test_auto10_speech.py",
        ROOT / "scripts" / "test_auto15_competition_matrix.py",
        ROOT / "scripts" / "test_auto16_release.py",
        ROOT / "scripts" / "test_visual_demo_summary.py",
        ROOT / "scripts" / "test_dashboard_telemetry_frames.py",
        ROOT / "scripts" / "test_run_visual_demo_contract.py",
        ROOT / "scripts" / "test_gazebo_cleaning_demo_contract.py",
        ROOT / "scripts" / "test_coverage_dynamic_matrix_report.py",
        ROOT / "scripts" / "test_gazebo_viewport_probe.py",
        ROOT / "scripts" / "test_human_visualization_gate.py",
        ROOT / "scripts" / "test_gazebo_scene_contract.py",
        ROOT / "scripts" / "test_pre_urdf_readiness.py",
        ROOT / "scripts" / "test_formal_vehicle_urdf.py",
    )
    result = pytest.main(["-q", *(str(path) for path in test_paths)])
    if result != pytest.ExitCode.OK:
        raise RuntimeError(f"ROS-independent pytest gate failed with exit code {int(result)}")


def main() -> int:
    require_project_files()
    validate_repository_hygiene()
    validate_python()
    validate_structured_files()
    validate_stage4w_runtime_contract()
    run_ros_independent_tests()
    from autonomous_runner import build_plan, load_json, load_registry, validate_registry, validate_state

    registry = load_registry()
    state = load_json(AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_STATE.json")
    errors = validate_registry(registry) + validate_state(state, registry)
    if load_json(AUTONOMY_CONFIG_ROOT / "AUTONOMOUS_RUN_PLAN.json") != build_plan(registry):
        errors.append("AUTONOMOUS_RUN_PLAN.json differs from registry")
    if errors:
        raise RuntimeError("autonomous control-plane validation failed: " + "; ".join(errors))
    print("development workflow fast validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
