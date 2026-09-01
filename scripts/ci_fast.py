#!/usr/bin/env python3
"""Fast, ROS-independent validation used by the 开发工作流 CI gate."""

from __future__ import annotations

import compileall
import json
import os
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
        ROOT / "config" / "high_fidelity_vehicle" / "formal_functional_acceptance_contract.yaml",
        ROOT / "config" / "dosod_s100p_hbm_compile_contract.json",
        ROOT / "config" / "dosod_s100p_hbm_compile_contract.schema.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "urdf" / "formal_competition_vehicle.urdf.xacro",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "formal_vehicle" / "formal_vehicle_layout.scad",
        REPORTS_ROOT / "engineering" / "formal_competition_vehicle.urdf",
        REPORTS_ROOT / "engineering" / "formal_vehicle_snapshot_manifest.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_layout_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_urdf_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_runtime_report.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_preview.png",
        ROOT / "scripts" / "validate_formal_vehicle_urdf.py",
        ROOT / "scripts" / "run_formal_vehicle_static_engineering_preflight.py",
        ROOT / "scripts" / "run_native_cadquery_serial_export.py",
        ROOT / "scripts" / "validate_native_cadquery_serial_export_contract.py",
        ROOT / "scripts" / "validate_s100p_formal_board_bundle.py",
        ROOT / "scripts" / "validate_component_addressable_native_cad_assembly_draft.py",
        ROOT / "scripts" / "validate_per_part_native_cad_release_gap_register.py",
        ROOT / "scripts" / "audit_native_brep_source_coverage.py",
        ROOT / "scripts" / "audit_formal_four_chain_runtime_readiness.py",
        ROOT / "scripts" / "formal_runtime_gate_binding.py",
        ROOT / "scripts" / "validate_native_brep_first_batch_contract.py",
        ROOT / "scripts" / "validate_native_brep_cleaning_recovery_second_batch_contract.py",
        ROOT / "scripts" / "validate_native_brep_storage_service_third_batch_contract.py",
        ROOT / "scripts" / "validate_native_brep_body_sensor_power_fourth_batch_contract.py",
        ROOT / "scripts" / "validate_native_brep_storage_service_seventh_batch_contract.py",
        ROOT / "scripts" / "validate_native_brep_pending_batch.py",
        ROOT / "scripts" / "generate_static_functional_chain_audit.py",
        ROOT / "scripts" / "validate_static_functional_chain_audit.py",
        ROOT / "scripts" / "audit_formal_requirement_coverage.py",
        ROOT / "scripts" / "validate_formal_requirement_coverage_gap_register.py",
        ROOT / "scripts" / "validate_s100p_mechanical_electrical_evidence.py",
        ROOT / "scripts" / "validate_formal_mechanical_interface_datums.py",
        ROOT / "scripts" / "validate_s100p_offline_predeploy.py",
        ROOT / "scripts" / "validate_s100p_final_predeploy.py",
        ROOT / "scripts" / "audit_s100p_windows_connectivity_dependencies.py",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.schema.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_cleaning_recovery_second_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_storage_service_third_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_body_sensor_power_fourth_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_bodywork_fifth_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_cleaning_mechanisms_sixth_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_storage_service_seventh_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_brep_power_distribution_eighth_batch_contract.json",
        ROOT / "config" / "high_fidelity_vehicle" / "component_addressable_native_cad_assembly_manifest_draft.json",
        ROOT / "config" / "high_fidelity_vehicle" / "per_part_native_cad_release_gap_register.json",
        ROOT / "config" / "high_fidelity_vehicle" / "s100p_mechanical_electrical_evidence.json",
        ROOT / "config" / "high_fidelity_vehicle" / "native_cadquery_serial_export_contract.json",
        ROOT / "config" / "s100p_formal_board_bundle_manifest.json",
        ROOT / "config" / "s100p_product_artifact_bundle.json",
        ROOT / "config" / "s100p_product_board_launch_parameters.json",
        ROOT / "config" / "s100p_product_overlay_packages.json",
        REPORTS_ROOT / "engineering" / "native_brep_source_coverage_audit.json",
        REPORTS_ROOT / "engineering" / "formal_four_chain_runtime_readiness.json",
        REPORTS_ROOT / "engineering" / "formal_vehicle_static_engineering_preflight.json",
        REPORTS_ROOT / "engineering" / "native_cadquery_serial_export_contract_audit.json",
        REPORTS_ROOT / "engineering" / "s100p_formal_board_bundle_audit.json",
        REPORTS_ROOT / "engineering" / "s100p_final_predeploy_audit.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_cleaning_recovery_second_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_cleaning_recovery_second_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_storage_service_third_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_storage_service_third_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_body_sensor_power_fourth_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_body_sensor_power_fourth_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_bodywork_fifth_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_bodywork_fifth_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_cleaning_mechanisms_sixth_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_cleaning_mechanisms_sixth_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_storage_service_seventh_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_storage_service_seventh_batch_source_manifest.json",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_power_distribution_eighth_batch.py",
        SOURCE_ROOT / "sanitation_vehicle_description" / "cad" / "native_brep" / "formal_vehicle" / "native_brep_power_distribution_eighth_batch_source_manifest.json",
        ROOT / "config" / "high_fidelity_vehicle" / "formal_mechanical_interface_datums.yaml",
        ROOT / "config" / "s100p_offline_predeploy_plan.json",
        ROOT / "scripts" / "validate_formal_vehicle_visual_fidelity.py",
        ROOT / "scripts" / "validate_formal_auxiliary_runtime.py",
        ROOT / "scripts" / "run_formal_auxiliary_runtime.sh",
        ROOT / "scripts" / "formal_vehicle_mesh_manifest.py",
        ROOT / "scripts" / "collect_formal_service_door_runtime.py",
        ROOT / "scripts" / "run_formal_service_door_runtime.sh",
        ROOT / "scripts" / "validate_formal_service_door_runtime.py",
        ROOT / "scripts" / "run_formal_manipulator_trajectory_runtime.sh",
        ROOT / "scripts" / "run_formal_function_positions_runtime.sh",
        ROOT / "scripts" / "publish_integrated_basic_functional_acceptance.py",
        ROOT / "scripts" / "run_integrated_functional_acceptance.sh",
        ROOT / "scripts" / "validate_formal_side_brush_sdf_surface.py",
        ROOT / "scripts" / "formal_cleaning_motor_telemetry.py",
        SOURCE_ROOT / "sanitation_gazebo_control" / "include" / "sanitation_gazebo_control" / "CleaningActuatorMotorCore.hh",
        SOURCE_ROOT / "sanitation_gazebo_control" / "src" / "CleaningActuatorMotorCore.cc",
        SOURCE_ROOT / "sanitation_gazebo_control" / "src" / "CleaningActuatorMotorSystem.cc",
        SOURCE_ROOT / "sanitation_gazebo_control" / "src" / "CleaningActuatorVectorBridge.cc",
        SOURCE_ROOT / "sanitation_vehicle_description" / "launch" / "formal_vehicle_sim.launch.py",
        ROOT / "config" / "high_fidelity_vehicle" / "cleaning_actuator_motor_realism_contract.yaml",
        ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml",
        ROOT / "scripts" / "formal_final_runtime_closure.py",
        ROOT / "scripts" / "build_formal_final_runtime.sh",
        ROOT / "scripts" / "build_formal_final_runtime_windows.ps1",
        ROOT / "scripts" / "prepare_formal_runtime_source_cache_hygiene.ps1",
        ROOT / "scripts" / "validate_formal_windows_cold_gate_evidence.py",
        ROOT / "scripts" / "formal_memory_watchdog.sh",
        ROOT / "scripts" / "formal_windows_memory_probe.py",
        ROOT / "scripts" / "formal_wsl_entry_memory_guard.ps1",
        ROOT / "scripts" / "run_formal_runtime_isolation.sh",
        ROOT / "scripts" / "run_formal_final_acceptance.py",
        ROOT / "scripts" / "validate_dosod_s100p_hbm_compile_contract.py",
        ROOT / "scripts" / "collect_dosod_s100p_compiler_identity.py",
        ROOT / "scripts" / "test_collect_dosod_s100p_compiler_identity.py",
        ROOT / "scripts" / "test_validate_dosod_s100p_hbm_compile_contract.py",
        ROOT / "scripts" / "test_formal_final_runtime_closure.py",
        ROOT / "scripts" / "test_formal_memory_watchdog.py",
        ROOT / "scripts" / "test_formal_windows_memory_probe.py",
        ROOT / "scripts" / "test_formal_windows_runtime_dry_run.py",
        ROOT / "scripts" / "test_prepare_formal_runtime_source_cache_hygiene.py",
        ROOT / "scripts" / "test_formal_wsl_entry_memory_guard.py",
        ROOT / "scripts" / "test_build_formal_final_runtime_windows_guard.py",
        ROOT / "scripts" / "test_validate_formal_windows_cold_gate_evidence.py",
        ROOT / "scripts" / "test_formal_runtime_isolation.py",
        ROOT / "scripts" / "test_formal_runtime_orchestration.py",
        ROOT / "scripts" / "test_run_formal_final_acceptance.py",
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
    json.loads(
        (ROOT / "config" / "dosod_s100p_hbm_compile_contract.json").read_text(
            encoding="utf-8"
        )
    )
    json.loads(
        (ROOT / "config" / "dosod_s100p_hbm_compile_contract.schema.json").read_text(
            encoding="utf-8"
        )
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "native_brep_first_batch_contract.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "per_part_native_cad_release_gap_register.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "component_addressable_native_cad_assembly_manifest_draft.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "native_cadquery_serial_export_contract.json"
        ).read_text(encoding="utf-8")
    )
    for path in (
        ROOT / "config" / "s100p_formal_board_bundle_manifest.json",
        ROOT / "config" / "s100p_product_artifact_bundle.json",
        ROOT / "config" / "s100p_product_board_launch_parameters.json",
        ROOT / "config" / "s100p_product_overlay_packages.json",
    ):
        json.loads(path.read_text(encoding="utf-8"))
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "native_brep_first_batch_contract.schema.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "native_brep_storage_service_third_batch_contract.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "native_brep_body_sensor_power_fourth_batch_contract.json"
        ).read_text(encoding="utf-8")
    )
    json.loads(
        (ROOT / "config" / "s100p_offline_predeploy_plan.json").read_text(
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
        (
            ROOT
            / "config"
            / "high_fidelity_vehicle"
            / "formal_mechanical_interface_datums.yaml"
        ).read_text(encoding="utf-8")
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


def validate_s100p_mechanical_electrical_evidence_contract() -> None:
    """Run the fail-closed S100P evidence validator before the pytest suite."""
    from validate_s100p_mechanical_electrical_evidence import DEFAULT, validate

    validate(json.loads(DEFAULT.read_text(encoding="utf-8")), ROOT)


def validate_formal_static_engineering_preflight_report() -> None:
    """Reject a stale static report instead of trusting file existence."""

    from run_formal_vehicle_static_engineering_preflight import build_report

    report_path = (
        REPORTS_ROOT / "engineering" / "formal_vehicle_static_engineering_preflight.json"
    )
    stored = json.loads(report_path.read_text(encoding="utf-8"))
    live = build_report(ROOT)
    if stored != live:
        raise RuntimeError(
            "formal vehicle static engineering preflight report is stale; regenerate it"
        )
    if (
        live.get("static_check_count") != 14
        or live.get("static_check_completed_count") != 14
        or live.get("static_preflight_complete") is not True
        or live.get("manufacturing_release_ready") is not False
        or live.get("native_export_ready") is not False
        or live.get("deployment_ready") is not False
    ):
        raise RuntimeError("formal vehicle fourteen-check static preflight is invalid")


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
    formal_campus_package = SOURCE_ROOT / "sanitation_formal_campus_integration"
    product_demo_package = SOURCE_ROOT / "sanitation_product_demo_integration"
    safety_package = SOURCE_ROOT / "sanitation_safety"
    power_system_package = SOURCE_ROOT / "sanitation_power_system"
    localization_package = SOURCE_ROOT / "sanitation_localization"
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
    sys.path.insert(0, str(formal_campus_package))
    sys.path.insert(0, str(product_demo_package))
    sys.path.insert(0, str(safety_package))
    sys.path.insert(0, str(power_system_package))
    sys.path.insert(0, str(localization_package))
    sys.path.insert(0, str(research_demo_package))
    sys.path.insert(0, str(ROOT / "scripts"))
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
        perception_package / "test" / "test_formal_contract.py",
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
        *sorted((formal_campus_package / "test").glob("test_*.py")),
        *sorted((product_demo_package / "test").glob("test_*.py")),
        *sorted((safety_package / "test").glob("test_*.py")),
        *sorted((power_system_package / "test").glob("test_*.py")),
        *sorted((localization_package / "test").glob("test_*.py")),
        *sorted((research_demo_package / "test").glob("test_*.py")),
        active_cleaning_package / "test" / "test_performance.py",
        spot_cleaning_package / "test" / "test_auto01_geometry.py",
        ROOT / "scripts" / "test_autonomous_runner.py",
        ROOT / "scripts" / "test_auto02_tools.py",
        ROOT / "scripts" / "test_auto03_matrix.py",
        ROOT / "scripts" / "test_auto14_onnx_preflight.py",
        ROOT / "scripts" / "test_collect_dosod_s100p_compiler_identity.py",
        ROOT / "scripts" / "test_validate_dosod_s100p_hbm_compile_contract.py",
        ROOT / "scripts" / "test_auto10_formal.py",
        ROOT / "scripts" / "test_auto10_speech.py",
        ROOT / "scripts" / "test_auto15_competition_matrix.py",
        ROOT / "scripts" / "test_auto16_release.py",
        ROOT / "scripts" / "test_upstream_patch_contract.py",
        ROOT / "scripts" / "test_locked_repository_audit.py",
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
        ROOT / "scripts" / "test_formal_motion_cleaning_profile.py",
        ROOT / "scripts" / "test_validate_formal_operation_speed_profiles.py",
        ROOT / "scripts" / "test_validate_formal_vehicle_real_world_build_readiness.py",
        ROOT / "scripts" / "test_validate_formal_vehicle_mechanical_release_readiness.py",
        ROOT / "scripts" / "test_validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py",
        ROOT / "scripts" / "test_native_cad_readiness.py",
        ROOT / "scripts" / "test_component_addressable_native_cad_preflight_integration.py",
        ROOT / "scripts" / "test_validate_component_addressable_native_cad_assembly_draft.py",
        ROOT / "scripts" / "test_validate_per_part_native_cad_release_gap_register.py",
        ROOT / "scripts" / "test_native_brep_source_coverage_audit.py",
        ROOT / "scripts" / "test_audit_formal_four_chain_runtime_readiness.py",
        ROOT / "scripts" / "test_formal_runtime_gate_binding.py",
        ROOT / "scripts" / "test_native_brep_reconstruction_manifest.py",
        ROOT / "scripts" / "test_native_brep_first_batch_contract.py",
        ROOT / "scripts" / "test_native_brep_cleaning_recovery_second_batch_sources.py",
        ROOT / "scripts" / "test_validate_native_brep_cleaning_recovery_second_batch_contract.py",
        ROOT / "scripts" / "test_native_brep_storage_service_third_batch_sources.py",
        ROOT / "scripts" / "test_native_brep_body_sensor_power_fourth_batch_sources.py",
        ROOT / "scripts" / "test_validate_native_brep_later_batches_contract.py",
        ROOT / "scripts" / "test_native_brep_bodywork_fifth_batch_sources.py",
        ROOT / "scripts" / "test_native_brep_cleaning_mechanisms_sixth_batch.py",
        ROOT / "scripts" / "test_native_brep_storage_service_seventh_batch_contract.py",
        ROOT / "scripts" / "test_native_brep_power_distribution_eighth_batch_sources.py",
        ROOT / "scripts" / "test_formal_mechanical_interface_datums.py",
        ROOT / "scripts" / "test_formal_vehicle_static_engineering_preflight.py",
        ROOT / "scripts" / "test_native_cadquery_serial_export.py",
        ROOT / "scripts" / "test_validate_native_cadquery_serial_export_contract.py",
        ROOT / "scripts" / "test_static_functional_chain_audit.py",
        ROOT / "scripts" / "test_formal_requirement_coverage_gap_register.py",
        ROOT / "scripts" / "test_s100p_mechanical_electrical_evidence.py",
        ROOT / "scripts" / "test_validate_s100p_formal_board_bundle.py",
        ROOT / "scripts" / "test_validate_s100p_offline_predeploy.py",
        ROOT / "scripts" / "test_validate_s100p_final_predeploy.py",
        ROOT / "scripts" / "test_audit_s100p_windows_connectivity_dependencies.py",
        ROOT / "scripts" / "test_validate_electrical_harness_thermal_readiness.py",
        ROOT / "scripts" / "test_formal_cleaning_lift_kinematic_contract.py",
        ROOT / "scripts" / "test_formal_vehicle_snapshot_manifest.py",
        ROOT / "scripts" / "test_formal_vehicle_visual_fidelity.py",
        ROOT / "scripts" / "test_formal_vehicle_product_design.py",
        ROOT / "scripts" / "test_capture_formal_vehicle_visual_acceptance.py",
        ROOT / "scripts" / "test_formal_vehicle_component_register.py",
        ROOT / "scripts" / "test_formal_encoder_feedback_contract.py",
        ROOT / "scripts" / "test_formal_vehicle_service_door_wrist_contract.py",
        ROOT / "scripts" / "test_formal_service_door_runtime.py",
        ROOT / "scripts" / "test_generate_formal_rl_multimap_report.py",
        ROOT / "scripts" / "test_validate_formal_dynamic_obstacle_avoidance.py",
        ROOT / "scripts" / "test_prepare_formal_dynamic_obstacle_schedule.py",
        ROOT / "scripts" / "test_prepare_formal_dynamic_runtime_world.py",
        ROOT / "scripts" / "test_formal_functional_acceptance_contract.py",
        ROOT / "scripts" / "test_formal_acceptance_session.py",
        ROOT / "scripts" / "test_formal_final_runtime_closure.py",
        ROOT / "scripts" / "test_formal_memory_watchdog.py",
        ROOT / "scripts" / "test_formal_windows_memory_probe.py",
        ROOT / "scripts" / "test_prepare_formal_runtime_source_cache_hygiene.py",
        ROOT / "scripts" / "test_formal_wsl_entry_memory_guard.py",
        ROOT / "scripts" / "test_build_formal_final_runtime_windows_guard.py",
        ROOT / "scripts" / "test_validate_formal_windows_cold_gate_evidence.py",
        ROOT / "scripts" / "test_run_formal_final_acceptance.py",
        ROOT / "scripts" / "test_formal_s100_live_acceptance.py",
        ROOT / "scripts" / "test_validate_formal_end_to_end_cleaning_mission.py",
        ROOT / "scripts" / "test_aggregate_formal_single_episode_cleaning_mission.py",
        ROOT / "scripts" / "test_run_formal_single_episode_cleaning_mission.py",
        ROOT / "scripts" / "test_formal_manipulator_control_contract.py",
        ROOT / "scripts" / "test_formal_missing_gate_runners.py",
        ROOT / "scripts" / "test_dynamic_payload_lumped_inertia_contract.py",
        ROOT / "scripts" / "test_dry_bin_monitor_system.py",
        ROOT / "scripts" / "test_ground_dirt_cleaning_system.py",
        ROOT / "scripts" / "test_cleaning_actuator_motor_contract.py",
        ROOT / "scripts" / "test_validate_formal_cleaning_actuator_motor_runtime.py",
        ROOT / "scripts" / "test_run_formal_cleaning_actuator_motor_runtime.py",
        ROOT / "scripts" / "test_whole_vehicle_actuator_interlock_contract.py",
        ROOT / "scripts" / "test_formal_auxiliary_product_interfaces.py",
        ROOT / "scripts" / "test_formal_squeegee_compliance_core.py",
        ROOT / "scripts" / "test_validate_formal_auxiliary_runtime.py",
        ROOT / "scripts" / "test_formal_fov_occlusion.py",
        ROOT / "scripts" / "test_scan_formal_vehicle_inertia_and_swept_volume.py",
        ROOT / "scripts" / "test_evaluate_formal_active_cleaning_splits.py",
        ROOT / "scripts" / "test_aggregate_formal_random_scene_perception.py",
        ROOT / "scripts" / "test_formal_random_scene_perception_runner.py",
        ROOT / "scripts" / "test_validate_formal_vehicle_mobility_runtime.py",
        ROOT / "scripts" / "test_run_formal_vehicle_mobility_runtime.py",
        ROOT / "scripts" / "test_collect_formal_vehicle_sensor_runtime.py",
        ROOT / "scripts" / "test_validate_formal_water_recovery_runtime.py",
        ROOT / "scripts" / "test_formal_water_motor_metrics.py",
        ROOT / "scripts" / "test_formal_cleaning_motor_telemetry.py",
        ROOT / "scripts" / "test_cleaning_actuator_vector_bridge_contract.py",
        ROOT / "scripts" / "test_validate_formal_side_brush_sdf_surface.py",
        ROOT / "scripts" / "test_validate_formal_cube_pick_place_runtime.py",
        ROOT / "scripts" / "test_validate_formal_grasp_executor_runtime.py",
        ROOT / "scripts" / "test_formal_20_cube_grasp_acceptance.py",
        ROOT / "scripts" / "test_run_formal_water_recovery_runtime.py",
        ROOT / "scripts" / "test_formal_vehicle_evaluator_interface_isolation.py",
        ROOT / "scripts" / "test_functional_acceptance_package_dependencies.py",
        ROOT / "scripts" / "test_integrated_functional_acceptance.py",
        ROOT / "scripts" / "test_publish_integrated_basic_functional_acceptance.py",
        ROOT / "scripts" / "test_formal_storage_collision_clearance.py",
        ROOT / "scripts" / "test_cadquery_windows_bootstrap.py",
        ROOT / "scripts" / "test_formal_storage_service_mechanics.py",
    )
    # Several independent ROS packages legitimately use the same conventional
    # test filename (for example test_formal_contract.py). Importlib mode keeps
    # their module identities path-scoped instead of causing pytest's default
    # top-level import cache to report a false collection mismatch.
    storage_source_only_before = os.environ.get(
        "TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY"
    )
    if os.name == "nt":
        # ci_fast is explicitly ROS-independent.  Do not let the storage test's
        # optional Windows convenience path start WSL behind the memory gate.
        os.environ["TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY"] = "1"
    try:
        result = pytest.main([
            "-q",
            "--import-mode=importlib",
            *(str(path) for path in test_paths),
        ])
    finally:
        if storage_source_only_before is None:
            os.environ.pop("TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY", None)
        else:
            os.environ["TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY"] = (
                storage_source_only_before
            )
    if result != pytest.ExitCode.OK:
        raise RuntimeError(f"ROS-independent pytest gate failed with exit code {int(result)}")


def main() -> int:
    require_project_files()
    validate_repository_hygiene()
    validate_python()
    validate_structured_files()
    validate_stage4w_runtime_contract()
    validate_s100p_mechanical_electrical_evidence_contract()
    validate_formal_static_engineering_preflight_report()
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
