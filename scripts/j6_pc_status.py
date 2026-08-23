#!/usr/bin/env python3
"""Generate fail-closed Journey 6 PC-first status and blocker evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


# No run-bound collector is trusted yet. Keep every loopback readiness state
# false until a future implementation is reviewed and its immutable evaluator
# identifier is explicitly added here.
TRUSTED_HIL_ATTESTATION_EVALUATORS: frozenset[str] = frozenset()


def _load(path: Path | None) -> dict:
    if path is None or not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def evaluate(
    *,
    model_inventory: dict,
    model_selection: dict,
    sdk_inventory: dict,
    loopback_report: dict,
    bundle_manifest: dict,
    calibration_manifest: dict | None = None,
    source_bundle_manifest: dict | None = None,
    license_report: dict | None = None,
) -> tuple[dict, dict]:
    calibration_manifest = calibration_manifest or {}
    source_bundle_manifest = source_bundle_manifest or {}
    license_report = license_report or {}
    dev_model_available = bool(
        model_inventory.get("artifact_sha256_verified") is True
        and isinstance(model_inventory.get("artifact_sha256"), str)
        and re.fullmatch(r"[0-9a-f]{64}", model_inventory["artifact_sha256"])
        and model_inventory.get("artifact_size_bytes", 0) > 0
        and model_inventory.get("class_names_verified") is True
        and isinstance(model_inventory.get("class_names"), list)
        and bool(model_inventory["class_names"])
        and model_inventory.get("pt_onnx_parity_pass") is True
        and model_inventory.get("pc_inference_pass") is True
        and model_inventory.get("mock_model") is False
    )
    discrete_ready = bool(
        dev_model_available
        and model_selection.get("pc_discrete_functional_pass") is True
        and model_selection.get("gt_control_violation_count") == 0
        and model_selection.get("preknown_coordinates") is False
        and model_selection.get("pre_fov_creation_count") == 0
    )
    area_ready = bool(
        model_selection.get("pc_area_functional_pass") is True
        and model_selection.get("area_model_sha256_verified") is True
        and model_selection.get("area_live_gazebo_gate_pass") is True
        and model_selection.get("area_per_class_metrics_present") is True
        and model_selection.get("area_gt_control_violation_count") == 0
    )
    model_ready = bool(discrete_ready and area_ready)
    calibration_counts = calibration_manifest.get("counts", {})
    calibration_records = calibration_manifest.get("records", [])
    calibration_source = calibration_manifest.get("source", {})
    calibration_record_contract_pass = bool(
        isinstance(calibration_records, list)
        and all(
            isinstance(record, dict)
            and record.get("role") in {"detector_frame", "second_pass_roi"}
            and isinstance(record.get("relative_path"), str)
            and bool(record.get("relative_path"))
            and isinstance(record.get("sha256"), str)
            and re.fullmatch(r"[0-9a-f]{64}", record["sha256"])
            and isinstance(record.get("strata"), dict)
            and bool(record.get("strata"))
            for record in calibration_records
        )
        and sum(record["role"] == "detector_frame" for record in calibration_records)
        == calibration_counts.get("detector_frame", -1)
        and sum(record["role"] == "second_pass_roi" for record in calibration_records)
        == calibration_counts.get("second_pass_roi", -1)
    )
    calibration_ready = bool(
        calibration_manifest.get("schema_version") == 1
        and calibration_manifest.get("target_family") == "journey6"
        and calibration_manifest.get("calibration_ready") is True
        and calibration_manifest.get("sealed_access_allowed") is False
        and calibration_counts.get("detector_frame", 0) >= 1000
        and calibration_counts.get("second_pass_roi", 0) >= 1000
        and calibration_record_contract_pass
        and isinstance(calibration_source, dict)
        and isinstance(calibration_source.get("record_inventory_sha256"), str)
        and re.fullmatch(
            r"[0-9a-f]{64}", calibration_source["record_inventory_sha256"]
        )
        and not calibration_manifest.get("blockers")
    )
    sdk_ready = bool(
        (
            sdk_inventory.get("J6_SDK_AVAILABLE") is True
            or (
                sdk_inventory.get("status") == "ready"
                and bool(sdk_inventory.get("accepted_sdk_roots"))
            )
        )
        and sdk_inventory.get("s100_or_rdk_substitution_detected") is not True
    )
    x86_ready = bool(
        sdk_ready
        and sdk_inventory.get("x86_simulation_runtime_available") is True
        and sdk_inventory.get("x86_simulation_sanity_pass") is True
        and sdk_inventory.get("selected_model_parity_pass") is True
    )
    loopback_safety = loopback_report.get("safety", {})
    loopback_transport = loopback_report.get("transport", {})
    loopback_algorithm = loopback_report.get("algorithm", {})
    loopback_platform = loopback_algorithm.get("platform", {})
    loopback_sensor_provenance = loopback_report.get("sensor_provenance", {})
    loopback_model_qualification = loopback_algorithm.get(
        "model_qualification", {}
    )
    trusted_loopback_attestation = bool(
        loopback_report.get("formal_attestation_evaluator_available") is True
        and loopback_report.get("attestation_evaluator_id")
        in TRUSTED_HIL_ATTESTATION_EVALUATORS
    )
    loopback_sensor_provenance_ready = bool(
        isinstance(loopback_sensor_provenance, dict)
        and loopback_sensor_provenance.get("audited_launch") is True
        and loopback_sensor_provenance.get("gazebo_process_verified") is True
        and loopback_sensor_provenance.get("publisher_endpoints_verified") is True
        and loopback_sensor_provenance.get("pc_sensor_and_plant_only") is True
        and isinstance(loopback_sensor_provenance.get("evidence_sha256"), str)
        and re.fullmatch(
            r"[0-9a-f]{64}", loopback_sensor_provenance["evidence_sha256"]
        )
    )
    loopback_safety_ready = bool(
        trusted_loopback_attestation
        and loopback_report.get("duration_s", 0) >= 1800
        and loopback_safety.get("ground_truth_control_violation_count") == 0
        and loopback_safety.get("steady_state_pc_duplicate_algorithm_nodes") == 0
        and loopback_safety.get("nonzero_authority_pass") is True
        and loopback_safety.get("command_timeout_safe_stop") is True
        and loopback_safety.get("actual_network_loss_safe_stop") is True
        and loopback_safety.get("no_stale_command_replay") is True
        and loopback_safety.get("network_reconnect_requires_manual_resume") is True
        and loopback_safety.get("pc_blacklist_injection_detected") is True
        and loopback_safety.get("pc_blacklist_safe_stop") is True
        and loopback_safety.get("estop_safe_stop") is True
    )
    loopback_transport_ready = bool(
        loopback_safety_ready
        and loopback_report.get("runtime_backend") in {"PC_ONNX", "pc_onnx"}
        and loopback_report.get("not_journey6_runtime") is True
        and loopback_report.get("actual_ros2_processes") is True
        and loopback_report.get("sensor_source") == "gazebo"
        and loopback_sensor_provenance_ready
        and loopback_platform.get("os_id") == "ubuntu"
        and loopback_platform.get("os_version_id") == "22.04"
        and loopback_platform.get("ros_distro") == "humble"
        and loopback_transport.get("qos_contract_pass") is True
        and loopback_transport.get("sensor_timestamps_monotonic") is True
        and loopback_transport.get("clock_monotonic") is True
        and loopback_transport.get("tf_received") is True
        and loopback_transport.get("tf_static_received") is True
        and loopback_transport.get("image_depth_sync_pass") is True
    )
    loopback_model_qualification_ready = bool(
        dev_model_available
        and loopback_algorithm.get("required_model_id_match") is True
        and loopback_algorithm.get("model_contract_qualified") is True
        and loopback_algorithm.get("model_loaded") is True
        and loopback_algorithm.get("inference_count", 0) > 0
        and loopback_algorithm.get("model_id") == model_inventory.get("model_id")
        and loopback_algorithm.get("model_sha256")
        == model_inventory.get("artifact_sha256")
        and isinstance(loopback_model_qualification, dict)
        and loopback_model_qualification.get("model_id")
        == model_inventory.get("model_id")
        and loopback_model_qualification.get("model_sha256")
        == model_inventory.get("artifact_sha256")
        and loopback_model_qualification.get("pt_onnx_parity_pass") is True
        and loopback_model_qualification.get("pc_inference_pass") is True
        and loopback_model_qualification.get("full_stack_pass") is True
        and isinstance(loopback_model_qualification.get("manifest_sha256"), str)
        and re.fullmatch(
            r"[0-9a-f]{64}", loopback_model_qualification["manifest_sha256"]
        )
        and isinstance(
            loopback_model_qualification.get("full_stack_evidence_sha256"), str
        )
        and re.fullmatch(
            r"[0-9a-f]{64}",
            loopback_model_qualification["full_stack_evidence_sha256"],
        )
        and loopback_report.get("algorithm_host_full_stack_pass") is True
    )
    loopback_algorithm_ready = bool(
        loopback_transport_ready and loopback_model_qualification_ready
    )
    loopback_ready = bool(
        loopback_safety_ready
        and sdk_ready
        and loopback_report.get("runtime_backend") == "JOURNEY6_OE"
        and loopback_report.get("not_journey6_runtime") is False
        and loopback_report.get("official_journey6_runtime_evidence") is True
        and loopback_report.get("actual_ros2_processes") is True
        and loopback_report.get("sensor_source") == "gazebo"
        and loopback_sensor_provenance_ready
        and loopback_transport.get("qos_contract_pass") is True
        and loopback_transport.get("image_depth_sync_pass") is True
        and loopback_algorithm.get("model_loaded") is True
        and loopback_model_qualification_ready
        and loopback_algorithm.get("inference_count", 0) > 0
        and loopback_report.get("algorithm_host_full_stack_pass") is True
    )
    source_components = {
        item.get("id"): item
        for item in source_bundle_manifest.get("components", [])
        if isinstance(item, dict) and isinstance(item.get("id"), str)
    }
    required_source_components = {
        "detector_canonical_onnx",
        "classifier_canonical_onnx",
        "area_canonical_onnx",
        "model_lock",
        "model_license_audit",
        "calibration_manifest",
        "calibration_distribution",
        "calibration_sha256sums",
        "nv12_contract",
        "python_postprocess",
        "cpp_postprocess",
        "golden_tensor_lock",
        "nash_profiles",
        "toolchain_lock",
        "board_runtime_source",
        "install_source",
        "healthcheck_source",
        "rollback_source",
        "hil_config",
    }
    source_bundle_ready = bool(
        source_bundle_manifest.get("schema_version") == 1
        and source_bundle_manifest.get("status") == "ready"
        and source_bundle_manifest.get("source_bundle_ready") is True
        and source_bundle_manifest.get("target_family") == "journey6"
        and source_bundle_manifest.get("source_only") is True
        and required_source_components <= source_components.keys()
        and all(
            source_components[name].get("path")
            and isinstance(source_components[name].get("observed_sha256"), str)
            and re.fullmatch(
                r"[0-9a-f]{64}", source_components[name]["observed_sha256"]
            )
            and isinstance(source_components[name].get("files"), list)
            and bool(source_components[name]["files"])
            and all(
                isinstance(item, dict)
                and isinstance(item.get("sha256"), str)
                and re.fullmatch(r"[0-9a-f]{64}", item["sha256"])
                for item in source_components[name]["files"]
            )
            for name in required_source_components
        )
        and not source_bundle_manifest.get("blockers")
    )
    license_release_ready = bool(
        license_report.get("release_allowed") is True
        and license_report.get("unresolved_count") == 0
    )
    compiled_hbm_ready = bool(
        bundle_manifest.get("compiled_hbm_bundle_ready") is True
        and sdk_ready
        and dev_model_available
        and calibration_ready
        and source_bundle_ready
        and license_release_ready
        and bundle_manifest.get("actual_march_locked") is True
        and bundle_manifest.get("hb_model_info_pass") is True
        and bundle_manifest.get("hb_verifier_pass") is True
        and bundle_manifest.get("runtime_load_pass") is True
    )
    bundle_ready = bool(
        compiled_hbm_ready
        and source_bundle_ready
        and bundle_manifest.get("bundle_ready") is True
        and bundle_manifest.get("target_family") == "journey6"
        and not bundle_manifest.get("external_blockers")
    )
    competition_model_ready = bool(
        model_selection.get("competition_model_ready") is True
        and model_selection.get("competition_claim_allowed") is True
    )
    board_metrics = {
        "FPS": None,
        "BPU_utilization": None,
        "CPU_utilization": None,
        "DDR_utilization": None,
        "temperature_c": None,
        "power_w": None,
        "HBM_latency_ms": None,
        "network_HIL_latency_ms": None,
        "board_30_seed": "not_run",
    }
    statuses = {
        "J6_DEV_MODEL_AVAILABLE": dev_model_available,
        "J6_PC_DISCRETE_FUNCTIONAL_PASS": discrete_ready,
        "J6_PC_AREA_FUNCTIONAL_PASS": area_ready,
        "J6_PC_FUNCTIONAL_PASS": model_ready,
        "J6_X86_SIMULATION_READY": x86_ready,
        "J6_LOOPBACK_TRANSPORT_READY": loopback_transport_ready,
        "J6_LOOPBACK_ALGORITHM_READY": loopback_algorithm_ready,
        "J6_LOOPBACK_HIL_EMULATION_READY": loopback_algorithm_ready,
        "J6_LOOPBACK_HIL_READY": loopback_ready,
        "J6_CALIBRATION_PACK_READY": calibration_ready,
        "J6_SOURCE_DEPLOYMENT_BUNDLE_READY": source_bundle_ready,
        "J6_COMPILED_HBM_BUNDLE_READY": compiled_hbm_ready,
        "J6_DEPLOYMENT_BUNDLE_READY": bundle_ready,
        "J6_COMPETITION_MODEL_READY": competition_model_ready,
        "J6_LICENSE_RELEASE_READY": license_release_ready,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "PRODUCT_INTEGRATION_READY": False,
        "PRODUCT_FIELD_READY": False,
    }
    blockers = []
    if not dev_model_available:
        if model_inventory.get("pt_onnx_parity_pass") is False:
            development_model_detail = (
                "the real D1 checkpoint and ONNX run on PC, but strict PT/ONNX "
                "parity failed"
            )
        elif model_inventory.get("pc_inference_pass") is False:
            development_model_detail = "the SHA-locked model failed real PC inference"
        else:
            development_model_detail = (
                "a SHA-locked real model with verified classes, parity, and PC inference "
                "is incomplete"
            )
        blockers.append(
            {
                "id": "DEVELOPMENT_MODEL",
                "type": "external_or_evidence",
                "detail": development_model_detail,
            }
        )
    if not discrete_ready:
        if model_inventory.get("pt_onnx_parity_pass") is False:
            discrete_detail = (
                "D1 failed strict parity and fixed-development semantics, so the "
                "three-class no-GT Gazebo loop was not started"
            )
        else:
            discrete_detail = (
                "the three-class no-GT Gazebo development loop has not passed"
            )
        blockers.append(
            {
                "id": "PC_DISCRETE_FUNCTIONAL",
                "type": "evidence_not_run",
                "detail": discrete_detail,
            }
        )
    if not area_ready:
        blockers.append(
            {
                "id": "PC_AREA_FUNCTIONAL",
                "type": "evidence_not_run",
                "detail": "the recovered development Area model has not passed the live Gazebo Area gate",
            }
        )
    if not calibration_ready:
        candidate_rgb_count = calibration_source.get("candidate_rgb_png_count", 0)
        candidate_roi_count = calibration_source.get(
            "candidate_roi_or_crop_file_count", 0
        )
        blockers.append(
            {
                "id": "CALIBRATION_PACK",
                "type": "evidence_not_ready",
                "detail": (
                    f"only {candidate_rgb_count} TRAIN RGB candidates and "
                    f"{candidate_roi_count} ROI candidates were inventoried; "
                    f"{calibration_counts.get('detector_frame', 0)}/1000 detector "
                    f"frames and {calibration_counts.get('second_pass_roi', 0)}/1000 "
                    "second-pass ROIs are audited"
                ),
            }
        )
    if not sdk_ready:
        blockers.append(
            {
                "id": "OFFICIAL_JOURNEY6_SDK",
                "type": "blocked_external",
                "detail": "official Journey 6 OpenExplorer/HUCP package is not verified",
            }
        )
    elif not x86_ready:
        blockers.append(
            {
                "id": "OFFICIAL_JOURNEY6_X86_SIMULATION",
                "type": "evidence_not_run",
                "detail": "official SDK x86 simulation and selected-model parity have not passed",
            }
        )
    if not loopback_transport_ready:
        blockers.append(
            {
                "id": "LOOPBACK_TRANSPORT_30_MIN",
                "type": "evidence_not_run",
                "detail": "30 minute Ubuntu 22.04/Humble PC_ONNX Gazebo evidence and a trusted run-bound attestation collector are not complete",
            }
        )
    if not source_bundle_ready:
        blockers.append(
            {
                "id": "SOURCE_DEPLOYMENT_BUNDLE",
                "type": "evidence_not_ready",
                "detail": "D1/Area references and source runtime exist, but model freeze, release license, calibration, nash profiles, or official toolchain locks remain incomplete",
            }
        )
    if not loopback_algorithm_ready:
        blockers.append(
            {
                "id": "LOOPBACK_ALGORITHM_HOST",
                "type": "evidence_not_run",
                "detail": "the D1 full algorithm-host stack has not passed 30 minutes on Gazebo transport",
            }
        )
    if not loopback_ready:
        blockers.append(
            {
                "id": "OFFICIAL_JOURNEY6_HIL",
                "type": "blocked_external",
                "detail": "official Journey 6 runtime and physical-board HIL evidence are unavailable",
            }
        )
    if not compiled_hbm_ready:
        blockers.append(
            {
                "id": "COMPILED_HBM_BUNDLE",
                "type": "blocked_external",
                "detail": "SDK, calibration, source bundle, release license, HBM verification, or runtime load evidence is incomplete",
            }
        )
    if not bundle_ready:
        blockers.append(
            {
                "id": "DEPLOYABLE_JOURNEY6_BUNDLE",
                "type": "blocked_external",
                "detail": "no checksum-locked installable Journey 6 bundle has passed all prerequisite gates",
            }
        )
    if not competition_model_ready:
        blockers.append(
            {
                "id": "COMPETITION_MODEL",
                "type": "model_blocked_internal",
                "detail": "development evidence is not competition model acceptance",
            }
        )
    if not license_release_ready:
        blockers.append(
            {
                "id": "MODEL_LICENSE_RELEASE",
                "type": "license",
                "detail": "model and exporter license layers are not release-clear",
            }
        )
    return (
        {
            "schema_version": 2,
            "target_family": "journey6",
            "target_sku": "auto",
            "target_march": "auto",
            "statuses": statuses,
            "board_metrics": board_metrics,
            "truth_boundary": (
                "PC contracts and bundle tooling do not prove official J6 x86 simulation, "
                "physical-board performance, V1 simulation acceptance, or field readiness."
            ),
        },
        {"schema_version": 1, "blockers": blockers},
    )


def _write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-inventory", type=Path)
    parser.add_argument("--model-selection", type=Path)
    parser.add_argument("--sdk-inventory", type=Path)
    parser.add_argument("--loopback-report", type=Path)
    parser.add_argument("--bundle-manifest", type=Path)
    parser.add_argument("--calibration-manifest", type=Path)
    parser.add_argument("--source-bundle-manifest", type=Path)
    parser.add_argument("--license-report", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    status, blockers = evaluate(
        model_inventory=_load(args.model_inventory),
        model_selection=_load(args.model_selection),
        sdk_inventory=_load(args.sdk_inventory),
        loopback_report=_load(args.loopback_report),
        bundle_manifest=_load(args.bundle_manifest),
        calibration_manifest=_load(args.calibration_manifest),
        source_bundle_manifest=_load(args.source_bundle_manifest),
        license_report=_load(args.license_report),
    )
    _write(args.output_dir / "J6_PC_FINAL_STATUS.json", status)
    _write(args.output_dir / "J6_PC_FINAL_BLOCKERS.json", blockers)
    index_path = args.output_dir / "J6_PC_EVIDENCE_INDEX.md"
    sums_path = args.output_dir / "J6_SHA256SUMS"
    files = sorted(
        path
        for path in args.output_dir.iterdir()
        if path.is_file()
        and path not in {index_path, sums_path}
        and path.suffix.lower() in {".json", ".md"}
    )
    index_lines = ["# Journey 6 PC-first evidence index", ""]
    for path in files:
        index_lines.append(f"- `{path.name}`: `{_sha256(path)}`")
    index_path.write_text(
        "\n".join(index_lines) + "\n", encoding="utf-8"
    )
    checksum_files = [*files, index_path]
    sums_path.write_text(
        "".join(f"{_sha256(path)}  {path.name}\n" for path in checksum_files),
        encoding="utf-8",
    )
    print(json.dumps(status, indent=2, ensure_ascii=False))
    return 0 if all(
        status["statuses"][name]
        for name in (
            "J6_PC_FUNCTIONAL_PASS",
            "J6_LOOPBACK_HIL_READY",
            "J6_DEPLOYMENT_BUNDLE_READY",
        )
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
