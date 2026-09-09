import hashlib
import json

import pytest

from journey6_hil_gateway.emulation import (
    RuntimeIdentity,
    SensorContractAudit,
    audit_gazebo_sensor_provenance,
    audit_model_qualification_manifest,
    evaluate_loopback_report,
    synthetic_sensor_publishers_allowed,
    validate_qos_evidence,
)

RUN_ID = "12345678-1234-4123-8123-123456789abc"


def complete_report(*, duration=1800.0, sensor_source="gazebo"):
    return {
        "runtime_backend": "PC_ONNX",
        "run_id": RUN_ID,
        "not_journey6_runtime": True,
        "actual_ros2_processes": True,
        "duration_s": duration,
        "sensor_source": sensor_source,
        "official_journey6_runtime_evidence": False,
        "sensor_provenance": {
            "audited_launch": True,
            "gazebo_process_verified": True,
            "publisher_endpoints_verified": True,
            "pc_sensor_and_plant_only": True,
            "evidence_sha256": "a" * 64,
        },
        "transport": {
            "qos_contract_pass": True,
            "sensor_timestamps_monotonic": True,
            "clock_monotonic": True,
            "tf_received": True,
            "tf_static_received": True,
            "image_depth_sync_pass": True,
            "synchronized_pair_count": 100,
        },
        "algorithm": {
            "platform": {
                "os_id": "ubuntu",
                "os_version_id": "22.04",
                "ros_distro": "humble",
            },
            "model_loaded": True,
            "provider": "CPUExecutionProvider",
            "fallback_used": False,
            "inference_count": 100,
            "required_model_id_match": True,
            "model_contract_qualified": True,
            "model_id": "d1_littercam_yolov9c",
            "model_sha256": "b" * 64,
            "model_qualification": {
                "manifest_sha256": "c" * 64,
                "model_id": "d1_littercam_yolov9c",
                "model_sha256": "b" * 64,
                "pt_onnx_parity_pass": True,
                "pc_inference_pass": True,
                "full_stack_evidence_sha256": "d" * 64,
                "full_stack_pass": True,
            },
        },
        "safety": {
            "steady_state_pc_duplicate_algorithm_nodes": 0,
            "pc_blacklist_injection_detected": True,
            "pc_blacklist_safe_stop": True,
            "ground_truth_control_violation_count": 0,
            "nonzero_authority_pass": True,
            "command_timeout_safe_stop": True,
            "actual_network_loss_safe_stop": True,
            "network_reconnect_requires_manual_resume": True,
            "no_stale_command_replay": True,
            "estop_safe_stop": True,
        },
    }


def test_pc_onnx_identity_must_explicitly_disclaim_journey6_runtime():
    RuntimeIdentity("PC_ONNX", True).validate()
    with pytest.raises(ValueError, match="identity mismatch"):
        RuntimeIdentity("PC_ONNX", False).validate()
    with pytest.raises(ValueError, match="unsupported"):
        RuntimeIdentity("CPU_FALLBACK", True).validate()


def test_sensor_audit_checks_monotonicity_and_image_depth_sync():
    audit = SensorContractAudit(sync_tolerance_s=0.005)
    audit.observe_clock(1.0)
    audit.observe_camera_info(1.0)
    audit.observe_tf(1.0, static=True)
    audit.observe_tf(1.0, static=False)
    audit.observe_color(1.0)
    audit.observe_depth(1.003)
    audit.observe_color(2.0)
    audit.observe_depth(2.020)
    audit.observe_clock(0.5)
    snapshot = audit.snapshot()
    assert snapshot["synchronized_pair_count"] == 1
    assert snapshot["rejected_unsynchronized_pair_count"] == 1
    assert snapshot["clock_monotonic"] is False


def test_v2_states_do_not_promote_legacy_journey6_readiness():
    statuses = evaluate_loopback_report(complete_report())
    assert statuses == {
        "J6_LOOPBACK_TRANSPORT_READY": False,
        "J6_LOOPBACK_ALGORITHM_READY": False,
        "J6_LOOPBACK_HIL_EMULATION_READY": False,
        "J6_LOOPBACK_HIL_READY": False,
    }


def test_short_or_synthetic_run_cannot_pass_30_min_emulation():
    report = complete_report(duration=30.0, sensor_source="synthetic_transport_probe")
    statuses = evaluate_loopback_report(report)
    assert statuses["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert statuses["J6_LOOPBACK_ALGORITHM_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_EMULATION_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_READY"] is False


def test_1800_second_synthetic_endurance_is_diagnostic_only():
    statuses = evaluate_loopback_report(
        complete_report(duration=1800.0, sensor_source="synthetic_transport_probe")
    )
    assert not any(statuses.values())


def test_missing_required_model_blocks_algorithm_but_not_transport():
    report = complete_report()
    report["algorithm"]["required_model_id_match"] = False
    statuses = evaluate_loopback_report(report)
    assert statuses["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert statuses["J6_LOOPBACK_ALGORITHM_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_EMULATION_READY"] is False


def test_self_reported_model_contract_without_manifest_is_rejected():
    report = complete_report()
    report["algorithm"]["model_contract_qualified"] = True
    report["algorithm"]["model_qualification"] = {}
    statuses = evaluate_loopback_report(report)
    assert statuses["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert statuses["J6_LOOPBACK_ALGORITHM_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_EMULATION_READY"] is False


def test_gazebo_string_without_machine_provenance_blocks_transport():
    report = complete_report(sensor_source="gazebo")
    report["sensor_provenance"] = {}
    assert not any(evaluate_loopback_report(report).values())


def test_gazebo_mode_never_enables_harness_sensor_publishers():
    assert synthetic_sensor_publishers_allowed("synthetic_transport_probe") is True
    assert synthetic_sensor_publishers_allowed("gazebo") is False
    with pytest.raises(ValueError, match="unsupported"):
        synthetic_sensor_publishers_allowed("spoofed_gazebo")


def test_missing_fault_or_authority_evidence_blocks_transport_readiness():
    for key in (
        "nonzero_authority_pass",
        "command_timeout_safe_stop",
        "actual_network_loss_safe_stop",
        "network_reconnect_requires_manual_resume",
        "no_stale_command_replay",
        "estop_safe_stop",
        "pc_blacklist_safe_stop",
    ):
        report = complete_report()
        report["safety"][key] = False
        assert evaluate_loopback_report(report)["J6_LOOPBACK_TRANSPORT_READY"] is False


def test_non_humble_algorithm_host_blocks_v2_transport_readiness():
    report = complete_report()
    report["algorithm"]["platform"]["ros_distro"] = "jazzy"
    assert not any(evaluate_loopback_report(report).values())


def test_legacy_journey6_state_does_not_require_humble_runtime():
    report = complete_report()
    report["runtime_backend"] = "JOURNEY6_OE"
    report["not_journey6_runtime"] = False
    report["official_journey6_runtime_evidence"] = True
    report["algorithm"]["platform"] = {
        "os_id": "proprietary",
        "os_version_id": "unknown",
        "ros_distro": "vendor",
    }
    statuses = evaluate_loopback_report(report)
    assert statuses["J6_LOOPBACK_TRANSPORT_READY"] is False
    assert statuses["J6_LOOPBACK_ALGORITHM_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_EMULATION_READY"] is False
    assert statuses["J6_LOOPBACK_HIL_READY"] is False


def test_live_qos_evidence_must_contain_all_matching_endpoints():
    def block(topic, publishers, reliability, durability, lifespan="Infinite", deadline="Infinite"):
        endpoints = publishers + 1
        qos = (
            f"  Reliability: {reliability}\n"
            f"  Durability: {durability}\n"
            f"  Lifespan: {lifespan}\n"
            f"  Deadline: {deadline}\n"
        )
        return (
            f"TOPIC={topic}\nPublisher count: {publishers}\n"
            f"Subscription count: 1\n" + qos * endpoints
        )

    evidence = "".join(
        (
            block("/hil/camera/color", 1, "BEST_EFFORT", "VOLATILE"),
            block("/hil/camera/depth", 1, "BEST_EFFORT", "VOLATILE"),
            block("/hil/camera/camera_info", 1, "BEST_EFFORT", "VOLATILE"),
            block("/hil/tf", 1, "BEST_EFFORT", "VOLATILE"),
            block("/hil/tf_static", 1, "RELIABLE", "TRANSIENT_LOCAL"),
            block(
                "/hil/vehicle/ackermann_command", 2, "RELIABLE", "VOLATILE",
                "120000000 nanoseconds", "80000000 nanoseconds",
            ),
            block(
                "/hil/vehicle/validated_ackermann_command", 1, "RELIABLE", "VOLATILE",
                "120000000 nanoseconds", "80000000 nanoseconds",
            ),
            block("/hil/health", 1, "RELIABLE", "VOLATILE"),
        )
    )
    assert validate_qos_evidence(evidence) is True
    assert validate_qos_evidence(evidence.replace("Deadline: 80000000 nanoseconds", "Deadline: Infinite", 1)) is False


def _write_json(path, payload):
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_model_qualification_requires_hashed_machine_evidence(tmp_path):
    model_id = "d1_littercam_yolov9c"
    model_sha = "b" * 64
    parity = tmp_path / "parity.json"
    inference = tmp_path / "pc_inference.json"
    full_stack = tmp_path / "full_stack.json"
    parity_sha = _write_json(
        parity,
        {
            "run_id": RUN_ID,
            "pt_onnx_parity_pass": True,
            "model_id": model_id,
            "model_sha256": model_sha,
        },
    )
    inference_sha = _write_json(
        inference,
        {
            "run_id": RUN_ID,
            "pc_inference_pass": True,
            "real_execution": True,
            "inference_count": 100,
            "model_id": model_id,
            "model_sha256": model_sha,
        },
    )
    full_stack_sha = _write_json(
        full_stack,
        {
            "run_id": RUN_ID,
            "full_stack_pass": True,
            "real_execution": True,
            "duration_s": 1800.0,
            "model_id": model_id,
            "model_sha256": model_sha,
        },
    )
    manifest = tmp_path / "model_qualification_manifest.json"
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "run_id": RUN_ID,
            "model_id": model_id,
            "model_sha256": model_sha,
            "pt_onnx_parity": {
                "pass": True,
                "evidence_file": parity.name,
                "evidence_sha256": parity_sha,
            },
            "pc_inference": {
                "pass": True,
                "evidence_file": inference.name,
                "evidence_sha256": inference_sha,
            },
            "full_stack": {
                "pass": True,
                "evidence_file": full_stack.name,
                "evidence_sha256": full_stack_sha,
            },
        },
    )
    result = audit_model_qualification_manifest(
        manifest, model_id=model_id, model_sha256=model_sha, run_id=RUN_ID
    )
    assert result["manifest_sha256"] == hashlib.sha256(manifest.read_bytes()).hexdigest()
    assert result["pt_onnx_parity_pass"] is True
    assert result["pc_inference_pass"] is True
    assert result["full_stack_evidence_sha256"] == full_stack_sha
    assert result["full_stack_pass"] is True

    inference.write_text("tampered\n", encoding="utf-8")
    tampered = audit_model_qualification_manifest(
        manifest, model_id=model_id, model_sha256=model_sha, run_id=RUN_ID
    )
    assert tampered["pc_inference_pass"] is False


def test_gazebo_provenance_requires_hashed_launch_process_and_endpoints(tmp_path):
    launch_source = tmp_path / "sensor_plant.launch.py"
    launch_source.write_text("# audited sensor/plant-only launch\n", encoding="utf-8")
    launch_source_sha = hashlib.sha256(launch_source.read_bytes()).hexdigest()
    launch = tmp_path / "launch.json"
    processes = tmp_path / "processes.json"
    endpoints = tmp_path / "endpoints.json"
    launch_sha = _write_json(
        launch,
        {
            "run_id": RUN_ID,
            "audited_launch": True,
            "pc_sensor_and_plant_only": True,
            "forbidden_algorithm_nodes": [],
            "launch_file": launch_source.name,
            "launch_file_sha256": launch_source_sha,
        },
    )
    process_sha = _write_json(
        processes,
        {
            "run_id": RUN_ID,
            "gazebo_process_verified": True,
            "gazebo_processes": [{"pid": 42, "executable": "gz sim"}],
            "algorithm_processes": [],
        },
    )
    endpoint_sha = _write_json(
        endpoints,
        {
            "run_id": RUN_ID,
            "publisher_endpoints_verified": True,
            "publisher_process_links_verified": True,
            "harness_sensor_publishers_present": False,
            "unexpected_publishers": [],
            "publisher_topics": [
                "/hil/clock",
                "/hil/camera/color",
                "/hil/camera/depth",
                "/hil/camera/camera_info",
                "/hil/tf",
                "/hil/tf_static",
            ],
        },
    )
    manifest = tmp_path / "HIL_GAZEBO_SENSOR_PROVENANCE.json"
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "run_id": RUN_ID,
            "audited_launch_evidence": {
                "pass": True,
                "evidence_file": launch.name,
                "evidence_sha256": launch_sha,
            },
            "gazebo_process_evidence": {
                "pass": True,
                "evidence_file": processes.name,
                "evidence_sha256": process_sha,
            },
            "publisher_endpoint_evidence": {
                "pass": True,
                "evidence_file": endpoints.name,
                "evidence_sha256": endpoint_sha,
            },
        },
    )
    result = audit_gazebo_sensor_provenance(manifest, run_id=RUN_ID)
    assert result == {
        "audited_launch": True,
        "gazebo_process_verified": True,
        "publisher_endpoints_verified": True,
        "pc_sensor_and_plant_only": True,
        "evidence_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
        "run_id": RUN_ID,
    }

    launch_source.write_text("tampered\n", encoding="utf-8")
    assert audit_gazebo_sensor_provenance(manifest, run_id=RUN_ID)["audited_launch"] is False

    endpoints.write_text("tampered\n", encoding="utf-8")
    assert audit_gazebo_sensor_provenance(manifest, run_id=RUN_ID)[
        "publisher_endpoints_verified"
    ] is False


@pytest.mark.parametrize(
    ("field", "bad_value"),
    (
        ("publisher_process_links_verified", False),
        ("harness_sensor_publishers_present", True),
        ("unexpected_publishers", ["/pc/journey6_loopback_harness"]),
    ),
)
def test_gazebo_provenance_rejects_unlinked_or_harness_publishers(
    tmp_path, field, bad_value
):
    endpoints = tmp_path / "endpoints.json"
    payload = {
        "run_id": RUN_ID,
        "publisher_endpoints_verified": True,
        "publisher_process_links_verified": True,
        "harness_sensor_publishers_present": False,
        "unexpected_publishers": [],
        "publisher_topics": [
            "/hil/clock",
            "/hil/camera/color",
            "/hil/camera/depth",
            "/hil/camera/camera_info",
            "/hil/tf",
            "/hil/tf_static",
        ],
    }
    payload[field] = bad_value
    endpoint_sha = _write_json(endpoints, payload)
    launch = tmp_path / "launch.json"
    launch_sha = _write_json(
        launch,
        {
            "run_id": RUN_ID,
            "audited_launch": True,
            "pc_sensor_and_plant_only": True,
            "forbidden_algorithm_nodes": [],
            "launch_file_sha256": "1" * 64,
        },
    )
    processes = tmp_path / "processes.json"
    process_sha = _write_json(
        processes,
        {
            "run_id": RUN_ID,
            "gazebo_process_verified": True,
            "gazebo_processes": ["gz sim"],
            "algorithm_processes": [],
        },
    )
    manifest = tmp_path / "HIL_GAZEBO_SENSOR_PROVENANCE.json"
    _write_json(
        manifest,
        {
            "schema_version": 1,
            "run_id": RUN_ID,
            "audited_launch_evidence": {
                "pass": True,
                "evidence_file": launch.name,
                "evidence_sha256": launch_sha,
            },
            "gazebo_process_evidence": {
                "pass": True,
                "evidence_file": processes.name,
                "evidence_sha256": process_sha,
            },
            "publisher_endpoint_evidence": {
                "pass": True,
                "evidence_file": endpoints.name,
                "evidence_sha256": endpoint_sha,
            },
        },
    )
    assert audit_gazebo_sensor_provenance(manifest, run_id=RUN_ID)[
        "publisher_endpoints_verified"
    ] is False
