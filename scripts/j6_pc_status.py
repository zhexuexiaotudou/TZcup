#!/usr/bin/env python3
"""Generate fail-closed Journey 6 PC-first status and blocker evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


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
) -> tuple[dict, dict]:
    model_ready = bool(
        model_inventory.get("all_required_artifacts_verified") is True
        and model_selection.get("selected_pipeline_verified") is True
        and model_selection.get("pc_functional_loop_pass") is True
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
    loopback_ready = bool(
        loopback_report.get("duration_s", 0) >= 1800
        and loopback_report.get("gt_control_violation_count") == 0
        and loopback_report.get("pc_duplicate_algorithm_node_count") == 0
        and loopback_report.get("j6_command_authority_pass") is True
        and loopback_report.get("command_timeout_safe_stop") is True
        and loopback_report.get("network_loss_safe_stop") is True
        and loopback_report.get("stale_command_replay_count") == 0
    )
    bundle_ready = bool(
        bundle_manifest.get("bundle_ready") is True
        and bundle_manifest.get("target_family") == "journey6"
        and not bundle_manifest.get("external_blockers")
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
        "J6_PC_FUNCTIONAL_PASS": model_ready,
        "J6_X86_SIMULATION_READY": x86_ready,
        "J6_LOOPBACK_HIL_READY": loopback_ready,
        "J6_DEPLOYMENT_BUNDLE_READY": bundle_ready,
        "SIMULATION_PRODUCT_COMPLETE": False,
        "PRODUCT_X86_PERCEPTION_READY": False,
        "PRODUCT_INTEGRATION_READY": False,
        "PRODUCT_FIELD_READY": False,
    }
    blockers = []
    if not model_ready:
        blockers.append(
            {
                "id": "PRETRAINED_MODEL_PIPELINE",
                "type": "external_or_evidence",
                "detail": "downloaded SHA-locked models and fixed-development PC/Gazebo evidence are missing",
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
    if not loopback_ready:
        blockers.append(
            {
                "id": "LOOPBACK_HIL_30_MIN",
                "type": "evidence_not_run",
                "detail": "30 minute split-process HIL and network-fault matrix are not complete",
            }
        )
    if not bundle_ready:
        blockers.append(
            {
                "id": "DEPLOYMENT_BUNDLE",
                "type": "evidence_not_ready",
                "detail": "bundle cannot be ready until installable model locks and rollback verification exist",
            }
        )
    return (
        {
            "schema_version": 1,
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
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    status, blockers = evaluate(
        model_inventory=_load(args.model_inventory),
        model_selection=_load(args.model_selection),
        sdk_inventory=_load(args.sdk_inventory),
        loopback_report=_load(args.loopback_report),
        bundle_manifest=_load(args.bundle_manifest),
    )
    _write(args.output_dir / "J6_PC_FINAL_STATUS.json", status)
    _write(args.output_dir / "J6_PC_FINAL_BLOCKERS.json", blockers)
    files = sorted(args.output_dir.glob("J6_*.json"))
    index_lines = ["# Journey 6 PC-first evidence index", ""]
    for path in files:
        index_lines.append(f"- `{path.name}`: `{_sha256(path)}`")
    (args.output_dir / "J6_PC_EVIDENCE_INDEX.md").write_text(
        "\n".join(index_lines) + "\n", encoding="utf-8"
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
