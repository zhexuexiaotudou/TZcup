from j6_pc_status import evaluate


def test_j6_status_is_fail_closed_when_evidence_is_missing():
    status, blockers = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
    )
    assert not any(status["statuses"].values())
    assert {item["id"] for item in blockers["blockers"]} >= {
        "DEVELOPMENT_MODEL",
        "PC_AREA_FUNCTIONAL",
        "OFFICIAL_JOURNEY6_SDK",
        "MODEL_LICENSE_RELEASE",
        "DEPLOYABLE_JOURNEY6_BUNDLE",
    }
    assert status["board_metrics"]["FPS"] is None
    assert status["board_metrics"]["board_30_seed"] == "not_run"


def test_pc_onnx_candidate_evidence_stays_blocked_without_trusted_attestor():
    model_inventory = {
        "model_id": "d1_littercam_yolov9c",
        "artifact_sha256_verified": True,
        "artifact_sha256": "a" * 64,
        "artifact_size_bytes": 123,
        "class_names_verified": True,
        "class_names": ["plastic_bottle"],
        "pt_onnx_parity_pass": True,
        "pc_inference_pass": True,
        "mock_model": False,
    }
    report = {
        "duration_s": 1800,
        "runtime_backend": "PC_ONNX",
        "not_journey6_runtime": True,
        "official_journey6_runtime_evidence": False,
        "actual_ros2_processes": True,
        "sensor_source": "gazebo",
        "algorithm_host_full_stack_pass": True,
        "sensor_provenance": {
            "audited_launch": True,
            "gazebo_process_verified": True,
            "publisher_endpoints_verified": True,
            "pc_sensor_and_plant_only": True,
            "evidence_sha256": "b" * 64,
        },
        "transport": {
            "qos_contract_pass": True,
            "sensor_timestamps_monotonic": True,
            "clock_monotonic": True,
            "tf_received": True,
            "tf_static_received": True,
            "image_depth_sync_pass": True,
        },
        "algorithm": {
            "required_model_id_match": True,
            "model_contract_qualified": True,
            "model_loaded": True,
            "model_id": "d1_littercam_yolov9c",
            "model_sha256": "a" * 64,
            "inference_count": 1,
            "model_qualification": {
                "manifest_sha256": "c" * 64,
                "model_id": "d1_littercam_yolov9c",
                "model_sha256": "a" * 64,
                "pt_onnx_parity_pass": True,
                "pc_inference_pass": True,
                "full_stack_evidence_sha256": "d" * 64,
                "full_stack_pass": True,
            },
            "platform": {
                "os_id": "ubuntu",
                "os_version_id": "22.04",
                "ros_distro": "humble",
            },
        },
        "safety": {
            "ground_truth_control_violation_count": 0,
            "steady_state_pc_duplicate_algorithm_nodes": 0,
            "nonzero_authority_pass": True,
            "command_timeout_safe_stop": True,
            "actual_network_loss_safe_stop": True,
            "no_stale_command_replay": True,
            "network_reconnect_requires_manual_resume": True,
            "pc_blacklist_injection_detected": True,
            "pc_blacklist_safe_stop": True,
            "estop_safe_stop": True,
        },
    }
    status, _ = evaluate(
        model_inventory=model_inventory,
        model_selection={},
        sdk_inventory={},
        loopback_report=report,
        bundle_manifest={},
    )
    assert status["statuses"]["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert status["statuses"]["J6_LOOPBACK_ALGORITHM_READY"] is False
    assert status["statuses"]["J6_LOOPBACK_HIL_EMULATION_READY"] is False
    assert status["statuses"]["J6_LOOPBACK_HIL_READY"] is False
    report["safety"].pop("actual_network_loss_safe_stop")
    status, _ = evaluate(
        model_inventory=model_inventory,
        model_selection={},
        sdk_inventory={},
        loopback_report=report,
        bundle_manifest={},
    )
    assert status["statuses"]["J6_LOOPBACK_TRANSPORT_READY"] is False


def test_loopback_rejects_self_reported_model_safety_or_gazebo_claims():
    model_inventory = {
        "model_id": "d1_littercam_yolov9c",
        "artifact_sha256_verified": True,
        "artifact_sha256": "a" * 64,
        "artifact_size_bytes": 123,
        "class_names_verified": True,
        "class_names": ["plastic_bottle"],
        "pt_onnx_parity_pass": True,
        "pc_inference_pass": True,
        "mock_model": False,
    }
    report = {
        "duration_s": 1800,
        "runtime_backend": "PC_ONNX",
        "not_journey6_runtime": True,
        "actual_ros2_processes": True,
        "sensor_source": "gazebo",
        "algorithm_host_full_stack_pass": True,
        "sensor_provenance": {
            "audited_launch": True,
            "gazebo_process_verified": True,
            "publisher_endpoints_verified": True,
            "pc_sensor_and_plant_only": True,
            "evidence_sha256": "b" * 64,
        },
        "transport": {
            "qos_contract_pass": True,
            "sensor_timestamps_monotonic": True,
            "clock_monotonic": True,
            "tf_received": True,
            "tf_static_received": True,
            "image_depth_sync_pass": True,
        },
        "algorithm": {
            "required_model_id_match": True,
            "model_contract_qualified": True,
            "model_loaded": True,
            "model_id": "d1_littercam_yolov9c",
            "model_sha256": "a" * 64,
            "inference_count": 1,
            "platform": {
                "os_id": "ubuntu",
                "os_version_id": "22.04",
                "ros_distro": "humble",
            },
            "model_qualification": {
                "manifest_sha256": "c" * 64,
                "model_id": "d1_littercam_yolov9c",
                "model_sha256": "a" * 64,
                "pt_onnx_parity_pass": True,
                "pc_inference_pass": True,
                "full_stack_evidence_sha256": "d" * 64,
                "full_stack_pass": True,
            },
        },
        "safety": {
            "ground_truth_control_violation_count": 0,
            "steady_state_pc_duplicate_algorithm_nodes": 0,
            "nonzero_authority_pass": True,
            "command_timeout_safe_stop": True,
            "actual_network_loss_safe_stop": True,
            "no_stale_command_replay": True,
            "network_reconnect_requires_manual_resume": True,
            "pc_blacklist_injection_detected": True,
            "pc_blacklist_safe_stop": True,
            "estop_safe_stop": True,
        },
    }
    for section, field in (
        ("safety", "pc_blacklist_injection_detected"),
        ("safety", "pc_blacklist_safe_stop"),
        ("safety", "estop_safe_stop"),
        ("sensor_provenance", "gazebo_process_verified"),
        ("sensor_provenance", "publisher_endpoints_verified"),
        ("algorithm", "model_qualification"),
    ):
        saved = report[section].pop(field)
        status, _ = evaluate(
            model_inventory=model_inventory,
            model_selection={},
            sdk_inventory={},
            loopback_report=report,
            bundle_manifest={},
        )
        assert status["statuses"]["J6_LOOPBACK_ALGORITHM_READY"] is False
        report[section][field] = saved


def test_short_or_synthetic_transport_probe_cannot_pass():
    report = {
        "duration_s": 30,
        "runtime_backend": "PC_ONNX",
        "not_journey6_runtime": True,
        "actual_ros2_processes": True,
        "sensor_source": "synthetic_transport_probe",
        "transport": {},
        "algorithm": {},
        "safety": {},
    }
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report=report,
        bundle_manifest={},
    )
    assert status["statuses"]["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert status["statuses"]["J6_LOOPBACK_ALGORITHM_READY"] is False


def test_development_model_is_independent_of_release_license():
    status, _ = evaluate(
        model_inventory={
            "artifact_sha256_verified": True,
            "artifact_sha256": "a" * 64,
            "artifact_size_bytes": 123,
            "class_names_verified": True,
            "class_names": ["plastic_bottle"],
            "pt_onnx_parity_pass": True,
            "pc_inference_pass": True,
            "mock_model": False,
        },
        model_selection={
            "pc_discrete_functional_pass": True,
            "gt_control_violation_count": 0,
            "preknown_coordinates": False,
            "pre_fov_creation_count": 0,
            "pc_area_functional_pass": False,
        },
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
        license_report={"release_allowed": False, "unresolved_count": 1},
    )
    assert status["statuses"]["J6_DEV_MODEL_AVAILABLE"] is True
    assert status["statuses"]["J6_PC_DISCRETE_FUNCTIONAL_PASS"] is True
    assert status["statuses"]["J6_PC_FUNCTIONAL_PASS"] is False
    assert status["statuses"]["J6_LICENSE_RELEASE_READY"] is False


def test_development_model_summary_flag_cannot_bypass_artifact_evidence():
    status, _ = evaluate(
        model_inventory={"J6_DEV_MODEL_AVAILABLE": True},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
    )
    assert status["statuses"]["J6_DEV_MODEL_AVAILABLE"] is False


def test_source_bundle_and_compiled_bundle_are_separate_states():
    component_ids = {
        "detector_canonical_onnx", "classifier_canonical_onnx",
        "area_canonical_onnx", "model_lock", "model_license_audit",
        "calibration_manifest", "calibration_distribution",
        "calibration_sha256sums", "nv12_contract", "python_postprocess",
        "cpp_postprocess", "golden_tensor_lock", "nash_profiles",
        "toolchain_lock", "board_runtime_source", "install_source",
        "healthcheck_source", "rollback_source", "hil_config",
    }
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
        calibration_manifest={},
        source_bundle_manifest={
            "schema_version": 1,
            "status": "ready",
            "source_bundle_ready": True,
            "target_family": "journey6",
            "source_only": True,
            "blockers": [],
            "components": [
                {
                    "id": name,
                    "path": name,
                    "observed_sha256": "a" * 64,
                    "files": [{"path": name, "sha256": "a" * 64}],
                }
                for name in component_ids
            ],
        },
    )
    assert status["statuses"]["J6_SOURCE_DEPLOYMENT_BUNDLE_READY"] is True
    assert status["statuses"]["J6_COMPILED_HBM_BUNDLE_READY"] is False
    assert status["statuses"]["J6_DEPLOYMENT_BUNDLE_READY"] is False


def test_source_bundle_summary_cannot_bypass_component_inventory():
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
        source_bundle_manifest={
            "schema_version": 1,
            "status": "ready",
            "source_bundle_ready": True,
            "target_family": "journey6",
            "source_only": True,
            "blockers": [],
            "components": [],
        },
    )
    assert status["statuses"]["J6_SOURCE_DEPLOYMENT_BUNDLE_READY"] is False


def test_calibration_state_requires_manifest_counts_and_no_blockers():
    manifest = {
        "schema_version": 1,
        "target_family": "journey6",
        "calibration_ready": True,
        "sealed_access_allowed": False,
        "counts": {"detector_frame": 1000, "second_pass_roi": 1000},
        "records": [
            {
                "role": role,
                "relative_path": f"{role}/{index}.png",
                "sha256": "a" * 64,
                "strata": {"scene": "development"},
            }
            for role in ("detector_frame", "second_pass_roi")
            for index in range(1000)
        ],
        "source": {
            "source_id": "train-development",
            "record_inventory_sha256": "b" * 64,
        },
        "blockers": [],
    }
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
        calibration_manifest=manifest,
    )
    assert status["statuses"]["J6_CALIBRATION_PACK_READY"] is True
    manifest["blockers"] = [{"code": "stratification_metadata_missing"}]
    status, _ = evaluate(
        model_inventory={},
        model_selection={},
        sdk_inventory={},
        loopback_report={},
        bundle_manifest={},
        calibration_manifest=manifest,
    )
    assert status["statuses"]["J6_CALIBRATION_PACK_READY"] is False
