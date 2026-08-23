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
        ROOT / "docs" / "current-status.md",
        ROOT / "docs" / "development-workflow.md",
        ROOT / "docs" / "artifact-policy.md",
        ROOT / "docs" / "product-acceptance-spec-v1.md",
        ROOT / "config" / "product_acceptance_v1.json",
        ROOT / ".github" / "workflows" / "development-workflow.yml",
        ROOT / "scripts" / "product_acceptance.py",
        ROOT / "scripts" / "scan_secrets.py",
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

    forbidden_ledger_files = [ROOT / "docs" / "progress.md", ROOT / "CODEX_MASTER_PROMPT.md"]
    forbidden_ledger_files.extend(ROOT.glob("GPT_REVIEW_STAGE*.md"))
    forbidden_ledger_files.extend(
        ROOT / "docs" / name
        for name in (
            "auto05-attempts.md",
            "auto05r-2-3-models-training-status.md",
            "auto05r-p2-data-integrity-recovery.md",
        )
    )
    present_ledgers = sorted(
        str(path.relative_to(ROOT)) for path in forbidden_ledger_files if path.exists()
    )
    if present_ledgers:
        raise RuntimeError(
            "chronological task ledgers must stay in Git/PR history, not the project front door: "
            + ", ".join(present_ledgers)
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
    json.loads(
        (ROOT / "config" / "product_acceptance_v1.json").read_text(
            encoding="utf-8"
        )
    )
    for path in sorted(SOURCE_ROOT.rglob("*.json")):
        json.loads(path.read_text(encoding="utf-8"))

    for pattern in ("*.yaml", "*.yml"):
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            yaml.safe_load(path.read_text(encoding="utf-8"))

    xml_patterns = ("package.xml", "*.xacro", "*.sdf", "*.urdf", "*.srdf")
    seen: set[Path] = set()
    for pattern in xml_patterns:
        for path in sorted(SOURCE_ROOT.rglob(pattern)):
            if path not in seen:
                ET.parse(path)
                seen.add(path)


def validate_product_acceptance_contract() -> None:
    sys.path.insert(0, str(ROOT / "scripts"))
    from product_acceptance import validate_contract

    validate_contract(ROOT / "config" / "product_acceptance_v1.json")


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
    safety_package = SOURCE_ROOT / "sanitation_safety"
    journey6_hil_package = SOURCE_ROOT / "journey6_hil_gateway"
    sys.path.insert(0, str(ROOT))
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
    sys.path.insert(0, str(safety_package))
    sys.path.insert(0, str(journey6_hil_package))
    test_paths = (
        coverage_package / "test" / "test_metrics.py",
        coverage_package / "test" / "test_stage4w_geometry.py",
        coverage_package / "test" / "test_ackermann_geometry.py",
        coverage_package / "test" / "test_ackermann_turn_planner.py",
        coverage_package / "test" / "test_coverage_components.py",
        coverage_package / "test" / "test_oriented_swath_router.py",
        coverage_package / "test" / "test_product_task_drain.py",
        tasks_package / "test" / "test_localization_metrics.py",
        tasks_package / "test" / "test_stage4t_localization_aggregate.py",
        tasks_package / "test" / "test_stage4v_localization_aggregate.py",
        tasks_package / "test" / "test_dynamic_geometry.py",
        tasks_package / "test" / "test_frontier_core.py",
        tasks_package / "test" / "test_tf_continuity_probe.py",
        tasks_package / "test" / "test_navigation_probe_waypoints.py",
        tasks_package / "test" / "test_stage4w_dynamic_aggregate.py",
        tasks_package / "test" / "test_auto11_large_map.py",
        gnss_package / "test" / "test_model.py",
        gnss_package / "test" / "test_dual_navsat.py",
        perception_package / "test" / "test_registry.py",
        perception_package / "test" / "test_projection.py",
        perception_package / "test" / "test_tracking.py",
        perception_package / "test" / "test_tracker_v2.py",
        perception_package / "test" / "test_frame_synchronizer.py",
        perception_package / "test" / "test_lifecycle_health.py",
        perception_package / "test" / "test_map_projection_v2.py",
        perception_package / "test" / "test_model_registry.py",
        perception_package / "test" / "test_model_activation.py",
        perception_package / "test" / "test_inference_engine.py",
        perception_package / "test" / "test_product_pipeline_contract.py",
        perception_package / "test" / "test_grid_safety.py",
        perception_package / "test" / "test_action_verifier.py",
        perception_package / "test" / "test_second_pass_provider.py",
        perception_package / "test" / "test_second_pass_product_integration.py",
        perception_package / "test" / "test_legacy_area_development.py",
        perception_package / "test" / "test_performance_monitor.py",
        perception_package / "test" / "test_backends.py",
        perception_package / "test" / "test_pipeline_manifest.py",
        perception_package / "test" / "test_preprocessing.py",
        perception_package / "test" / "test_j6_runtime.py",
        perception_package / "test" / "test_journey6_contract.py",
        perception_package / "test" / "test_journey6_nv12.py",
        perception_package / "test" / "test_journey6_provider.py",
        perception_package / "test" / "test_journey6_hil.py",
        perception_package / "test" / "test_onnx_provider.py",
        perception_package / "test" / "test_pretrained_contracts.py",
        perception_package / "test" / "test_dynamic_trash_map.py",
        perception_package / "test" / "test_no_preknown_targets.py",
        perception_package / "test" / "test_fov_visibility_contract.py",
        perception_package / "test" / "test_online_observation_fusion.py",
        perception_package / "test" / "test_dynamic_insertion.py",
        perception_package / "test" / "test_online_replay.py",
        perception_package / "test" / "test_target_appears_only_after_fov_entry.py",
        perception_package / "test" / "test_target_expiry_after_removal.py",
        dataset_package / "test" / "test_synthetic.py",
        ground_truth_package / "test" / "test_visibility.py",
        spot_cleaning_package / "test" / "test_coordinator.py",
        spot_cleaning_package / "test" / "test_active_observation.py",
        spot_cleaning_package / "test" / "test_observation_pose_planner.py",
        spot_cleaning_package / "test" / "test_stage5br6w_engineering.py",
        spot_cleaning_package / "test" / "test_auto03_contract.py",
        spot_cleaning_package / "test" / "test_auto03_replay_audit.py",
        spot_cleaning_package / "test" / "test_cleaning_task_scheduler.py",
        spot_cleaning_package / "test" / "test_post_clean_verification.py",
        spot_cleaning_package / "test" / "test_product_orchestrator.py",
        spot_cleaning_package / "test" / "test_product_spot_node_helpers.py",
        spot_cleaning_package / "test" / "test_reobservation_orchestrator.py",
        ROOT / "reference_vision" / "test" / "test_reference_adapter_contract.py",
        ROOT / "reference_vision" / "test" / "test_third_party_registry.py",
        ROOT / "reference_vision" / "test" / "test_product_reference_isolation.py",
        ROOT / "reference_vision" / "test" / "test_gt_topic_denylist.py",
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
        learning_package / "test" / "test_native_to_model_scale_contract.py",
        learning_package / "test" / "test_small_object_bucket_scale.py",
        learning_package / "test" / "test_machine_evaluable_scale.py",
        learning_package / "test" / "test_factorized_split_contract.py",
        ROOT / "scripts" / "test_prepare_auto05r_factorized_capture.py",
        ROOT / "scripts" / "test_build_auto05r_screening_dataset.py",
        ROOT / "scripts" / "test_finalize_auto05r_factorized_capture.py",
        ROOT / "scripts" / "test_perception_oprv3_area_gate.py",
        ROOT / "scripts" / "test_perception_oprv3_moving_dev_gate.py",
        ROOT / "scripts" / "test_perception_oprv3_moving_product_map.py",
        ROOT / "scripts" / "test_perception_oprv3_product_map_gate.py",
        ROOT / "scripts" / "test_perception_oprv3_product_performance.py",
        ROOT / "scripts" / "test_perception_oprv3_product_dev_gate.py",
        ROOT / "scripts" / "test_perception_oprv3_freeze.py",
        ROOT / "scripts" / "test_perception_oprv3_sealed_final.py",
        ROOT / "scripts" / "test_g6_dataset_contract.py",
        ROOT / "scripts" / "test_small_specialist_dataset.py",
        ROOT / "scripts" / "test_small_specialist_fusion.py",
        ROOT / "scripts" / "test_fasterrcnn_small_anchors.py",
        ROOT / "scripts" / "test_opr_c_rtmdet_contract.py",
        ROOT / "scripts" / "test_g6_area_recovery.py",
        ROOT / "scripts" / "test_g6_area_screen_contract.py",
        ROOT / "scripts" / "test_ddrv4_data_boundary.py",
        ROOT / "scripts" / "test_g5_consumed_cannot_reopen.py",
        ROOT / "scripts" / "test_g5v2_denied_before_freeze.py",
        ROOT / "scripts" / "test_g6_not_used_for_ddrv4_selection.py",
        ROOT / "scripts" / "test_g7_dataset_contract.py",
        ROOT / "scripts" / "test_g7_domain_matrix.py",
        ROOT / "scripts" / "test_g7_negative_taxonomy.py",
        ROOT / "scripts" / "test_g7_no_g6_or_g5_leakage.py",
        ROOT / "scripts" / "test_detector_failure_taxonomy.py",
        ROOT / "scripts" / "test_d1_sampling_policy.py",
        ROOT / "scripts" / "test_d1_training_contract.py",
        ROOT / "scripts" / "test_d1_threshold_selection_holdout_only.py",
        ROOT / "scripts" / "test_ddrv4_online_gate.py",
        ROOT / "scripts" / "test_ddrv4_runtime_contract.py",
        ROOT / "scripts" / "test_perception_ddrv4_finalize.py",
        ROOT / "scripts" / "test_odcv5_attrition_ladder.py",
        ROOT / "scripts" / "test_odcv5_golden_frame_parity.py",
        ROOT / "scripts" / "test_odcv5_g7_moving.py",
        ROOT / "scripts" / "test_crv6_checkpoint_recovery.py",
        ROOT / "scripts" / "test_crv6_static_val_non_gating.py",
        ROOT / "scripts" / "test_crv6_golden_parity.py",
        ROOT / "scripts" / "test_gocv7_real_gazebo_trace.py",
        ROOT / "scripts" / "test_gocv7_ga1_data.py",
        ROOT / "scripts" / "test_gocv7_detector_gazebo_gate.py",
        ROOT / "scripts" / "test_finalize_gocv7.py",
        ROOT / "scripts" / "test_crv6_reconstitution_provenance.py",
        ROOT / "scripts" / "test_crv6_native_moving_gate.py",
        ROOT / "scripts" / "test_crv6_moving_adaptation_holdout_only.py",
        ROOT / "scripts" / "test_crv6_projection_attrition.py",
        ROOT / "scripts" / "test_crv6_real_moving_evaluator.py",
        ROOT / "scripts" / "test_audit_crv6_online_dev.py",
        ROOT / "scripts" / "test_finalize_crv6.py",
        learning_package / "test" / "test_auto13_real_domain.py",
        learning_package / "test" / "test_g4_assets.py",
        learning_package / "test" / "test_g4_scene_negative_prior.py",
        learning_package / "test" / "test_g4_qa.py",
        learning_package / "test" / "test_g4_models.py",
        learning_package / "test" / "test_g4_losses.py",
        learning_package / "test" / "test_g4_calibration.py",
        learning_package / "test" / "test_g4_data.py",
        learning_package / "test" / "test_g4_split_policy.py",
        learning_package / "test" / "test_g4_selection.py",
        learning_package / "test" / "test_g4_gates.py",
        learning_package / "test" / "test_g4_manifest.py",
        learning_package / "test" / "test_g4_pretrained.py",
        learning_package / "test" / "test_g4_sealed_final.py",
        learning_package / "test" / "test_g5_dataset.py",
        learning_package / "test" / "test_auto05r_freeze_and_g5_evaluator.py",
        learning_package / "test" / "test_g4_onnx_parity.py",
        learning_package / "test" / "test_g4_evaluation_metrics.py",
        learning_package / "test" / "test_ground_geometry.py",
        learning_package / "test" / "test_g4_training_protocol.py",
        hmi_package / "test" / "test_dsl.py",
        hmi_package / "test" / "test_gateway.py",
        hmi_package / "test" / "test_live_state.py",
        hmi_package / "test" / "test_state.py",
        hmi_package / "test" / "test_reference.py",
        hmi_package / "test" / "test_ros_adapter.py",
        hmi_package / "test" / "test_server.py",
        manipulation_package / "test" / "test_core.py",
        debug_visualization_package / "test" / "test_debug_visualization_model.py",
        gazebo_visualization_package / "test" / "test_coverage_telemetry_v2.py",
        spot_cleaning_package / "test" / "test_auto01_geometry.py",
        safety_package / "test" / "test_authority.py",
        safety_package / "test" / "test_actuator_timeout_guard.py",
        safety_package / "test" / "test_supervisor.py",
        safety_package / "test" / "test_velocity_gate.py",
        journey6_hil_package / "test" / "test_contract.py",
        journey6_hil_package / "test" / "test_emulation.py",
        journey6_hil_package / "test" / "test_journey6_hil_gateway_core.py",
        journey6_hil_package / "test" / "test_network_faults.py",
        journey6_hil_package / "test" / "test_placement.py",
        ROOT / "scripts" / "test_autonomous_runner.py",
        ROOT / "scripts" / "test_auto02_tools.py",
        ROOT / "scripts" / "test_auto03_matrix.py",
        ROOT / "scripts" / "test_auto10_formal.py",
        ROOT / "scripts" / "test_auto10_speech.py",
        ROOT / "scripts" / "test_auto15_competition_matrix.py",
        ROOT / "scripts" / "test_auto16_release.py",
        ROOT / "scripts" / "test_package_perception_release.py",
        ROOT / "scripts" / "test_perception_prod_resource_inventory.py",
        ROOT / "scripts" / "test_x1_full_pipeline.py",
        ROOT / "scripts" / "test_generate_perception_product_status.py",
        ROOT / "scripts" / "test_j6_product_contract.py",
        ROOT / "scripts" / "test_j6_pc_status.py",
        ROOT / "scripts" / "test_j6_calibration_manifest.py",
        ROOT / "scripts" / "test_j6_no_sealed_calibration.py",
        ROOT / "scripts" / "test_j6_source_bundle.py",
        ROOT / "scripts" / "test_audit_j6f2_area_dev.py",
        ROOT / "scripts" / "test_j6_tensor_parity.py",
        ROOT / "scripts" / "test_pretrained_model_tooling.py",
        ROOT / "scripts" / "test_existing_model_inventory.py",
        ROOT / "scripts" / "test_model_semantic_contract.py",
        ROOT / "scripts" / "test_screen_emf_yolox_reference.py",
        ROOT / "scripts" / "test_screen_emf_c1_gt_smoke.py",
        ROOT / "scripts" / "test_c4_native_worker.py",
        ROOT / "scripts" / "test_evaluate_emf_classifier_nontraining.py",
        ROOT / "scripts" / "test_prepare_trcrv10_g10_coco.py",
        ROOT / "scripts" / "test_build_emf_classifier_holdout_gt.py",
        ROOT / "scripts" / "test_c1_holdout_native_worker.py",
        ROOT / "scripts" / "test_c3_holdout_native_worker.py",
        ROOT / "scripts" / "test_evaluate_emf_classifier_holdout.py",
        ROOT / "scripts" / "test_build_emf_area_dataset.py",
        ROOT / "scripts" / "test_emf_g2_area_capture_contract.py",
        ROOT / "scripts" / "test_screen_emf_ewasr_negative.py",
        ROOT / "scripts" / "test_vit_native_worker.py",
        ROOT / "scripts" / "test_c3_native_worker.py",
        ROOT / "scripts" / "test_candidate_cap.py",
        ROOT / "scripts" / "test_no_training_before_exhaustion.py",
        ROOT / "scripts" / "test_native_vs_onnx.py",
        ROOT / "scripts" / "test_d1_golden_attribution.py",
        ROOT / "scripts" / "test_real_rgbd_capture.py",
        ROOT / "scripts" / "test_merge_auto05r_capture_shards.py",
        ROOT / "scripts" / "test_overlay_auto05r_capture_scenes.py",
        ROOT / "scripts" / "test_visual_demo_summary.py",
        ROOT / "scripts" / "test_dashboard_telemetry_frames.py",
        ROOT / "scripts" / "test_run_visual_demo_contract.py",
        ROOT / "scripts" / "test_gazebo_cleaning_demo_contract.py",
        ROOT / "scripts" / "test_coverage_dynamic_matrix_report.py",
        ROOT / "scripts" / "test_gazebo_viewport_probe.py",
        ROOT / "scripts" / "test_human_visualization_gate.py",
        ROOT / "scripts" / "test_gazebo_scene_contract.py",
        ROOT / "scripts" / "test_ackermann_xacro_profiles.py",
        ROOT / "scripts" / "test_ackermann_nav2_contract.py",
        ROOT / "scripts" / "test_ackermann_evidence_tooling.py",
        ROOT / "scripts" / "test_product_acceptance.py",
        ROOT / "scripts" / "test_product_mapping_acceptance.py",
        ROOT / "scripts" / "test_product_mapping_runner.py",
        ROOT / "scripts" / "test_product_cleaning_launch.py",
        ROOT / "scripts" / "test_crcrv11_blocker_package.py",
    )
    test_paths = tuple(path for path in test_paths if path.is_file())
    if not test_paths:
        raise RuntimeError("no ROS-independent tests were discovered")
    result = pytest.main(["-q", *(str(path) for path in test_paths)])
    if result != pytest.ExitCode.OK:
        raise RuntimeError(f"ROS-independent pytest gate failed with exit code {int(result)}")


def main() -> int:
    require_project_files()
    validate_repository_hygiene()
    validate_python()
    validate_structured_files()
    validate_product_acceptance_contract()
    validate_stage4w_runtime_contract()
    run_ros_independent_tests()
    print("development workflow fast validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
