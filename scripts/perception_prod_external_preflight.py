#!/usr/bin/env python3
"""Generate fail-closed J6, board, and real-field product preflight evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def software_inventory() -> dict:
    roles = {
        "real_rgbd_capture_with_intrinsics_extrinsics_sync_privacy": "scripts/real_rgbd_capture.py",
        "known_placement_and_independent_gt": "scripts/real_rgbd_capture.py",
        "intrinsic_calibration": "scripts/auto13_real_domain.py",
        "dataset_ingestion": "scripts/auto13_real_domain.py",
        "annotation_review_protocol": "docs/real-domain-annotation-protocol.md",
        "unified_field_evaluator": "scripts/auto13_real_domain.py",
        "j6_onnx_contract": "scripts/auto14_onnx_preflight.py",
        "j6_operator_audit": "scripts/j6_operator_audit.py",
    }
    files = {}
    for role, relative in roles.items():
        path = ROOT / relative
        files[role] = {
            "path": relative,
            "present": path.is_file(),
            "sha256": sha256(path) if path.is_file() else None,
        }
    return {
        "roles": files,
        "all_required_software_present": all(item["present"] for item in files.values()),
    }


def build_toolchain_lock(discovery: dict, *, official_docs_version: str) -> dict:
    official = discovery["official_source"]
    required = discovery["required_versions"]
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-09",
        "OpenExplorer_version": official["oe_version"],
        "HBDK_compiler_version": required["hbdk4_compiler"],
        "HMCT_version": required["hmct"],
        "target_processor_family": "J6E/M",
        "compile_march": "nash-e_or_nash-m_must_match_physical_target",
        "package_sha256": official["archive_sha256"],
        "package_bytes": official["archive_bytes"],
        "package_integrity_pass_when_audited": official["archive_integrity_pass"],
        "installation_root": None,
        "current_archive_or_install_retained": False,
        "official_docs_url": "https://doc.oe.horizon.auto/",
        "official_docs_version": official_docs_version,
        "version_difference": f"local audited OE {official['oe_version']} vs official docs {official_docs_version}",
        "version_compatibility_resolved": official["oe_version"] == official_docs_version,
        "frozen_j6_student_available": False,
        "operator_audit_executed": False,
        "PTQ_executed": False,
        "compile_executed": False,
        "PRODUCT_J6_TOOLCHAIN_READY": False,
        "J6_MODEL_BLOCKED_INTERNAL": True,
        "truth_boundary": (
            "X86 product model did not qualify, so the dependent J6 student cannot be "
            "trained. A historical package audit is not an installed current toolchain, "
            "and static tooling does not substitute for PTQ or hb_compile."
        ),
    }


def scan_board() -> dict:
    command = [
        "powershell.exe", "-NoProfile", "-Command",
        "Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | "
        "Where-Object { $_.FriendlyName -match 'Horizon|D-Robotics|J6|RDK' } | "
        "Select-Object FriendlyName,InstanceId,Status | ConvertTo-Json -Compress",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    stdout = completed.stdout.strip()
    devices = []
    if stdout:
        parsed = json.loads(stdout)
        devices = parsed if isinstance(parsed, list) else [parsed]
    endpoint_configured = bool(os.environ.get("TZCUP_J6_ENDPOINT"))
    present = bool(devices) or endpoint_configured
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-10",
        "local_probe_exit_code": completed.returncode,
        "local_matching_devices": devices,
        "configured_remote_endpoint_present": endpoint_configured,
        "board_device_count": len(devices),
        "PRODUCT_J6_BOARD_READY": False,
        "BLOCKED_EXTERNAL_J6_BOARD": not present,
        "FPS": None,
        "latency_p95_ms": None,
        "temperature_c": None,
        "power_w": None,
        "memory_mb": None,
        "BPU_utilization": None,
        "CPU_utilization": None,
        "truth_boundary": (
            "No physical runtime metrics may be populated without an identified board, "
            "a compiled student, and a 30-minute board run."
        ),
    }


def build_field_status(sensor: dict, software: dict) -> dict:
    resources_present = all(
        sensor.get(name) is True
        for name in (
            "rgbd_device_present",
            "auditable_rgbd_recording_present",
            "independent_map_gt_present",
        )
    )
    return {
        "schema_version": 1,
        "stage": "PERCEPTION-PROD-11",
        "resource_scan": sensor,
        "software_preparation": software,
        "software_preparation_complete": software["all_required_software_present"],
        "minimum_real_scenes": 20,
        "minimum_real_frames": 1000,
        "actual_qualifying_scenes": 0,
        "actual_qualifying_frames": 0,
        "pseudo_labels_allowed_as_final_GT": False,
        "Integrated_Camera_accepted_as_RGBD": False,
        "PRODUCT_FIELD_READY": False,
        "REAL_DOMAIN_BLOCKED_EXTERNAL": not resources_present,
        "metrics": {
            "discrete_macro_f1": None,
            "each_class_recall": None,
            "paper_precision": None,
            "area_mIoU": None,
            "negative_specificity": None,
            "map_RMSE_m": None,
            "pre_FOV_false_discovery": None,
        },
        "truth_boundary": (
            "The Integrated Camera is not RGB-D. No qualifying moving RGB-D recording, "
            "calibration, human-reviewed labels, or independent map ground truth exists."
        ),
    }


def write(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--j6-discovery", type=Path, required=True)
    parser.add_argument("--sensor-inventory", type=Path, required=True)
    parser.add_argument("--official-docs-version", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    discovery = json.loads(args.j6_discovery.read_text(encoding="utf-8"))
    sensor = json.loads(args.sensor_inventory.read_text(encoding="utf-8"))
    software = software_inventory()
    write(
        args.output_root / "j6_toolchain/J6_TOOLCHAIN_LOCK.json",
        build_toolchain_lock(discovery, official_docs_version=args.official_docs_version),
    )
    write(args.output_root / "j6_board/J6_BOARD_STATUS.json", scan_board())
    write(
        args.output_root / "field/FIELD_RESOURCE_AND_SOFTWARE_STATUS.json",
        build_field_status(sensor, software),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
