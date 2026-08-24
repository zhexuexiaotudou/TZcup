#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()

    evidence = Path(args.evidence).resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    discovery_path = evidence / "toolchain_discovery.json"
    discovery = json.loads(discovery_path.read_text(encoding="utf-8"))
    board_count = int(discovery["host"]["board_device_count"])
    dependency_pass = False
    first_blocker = "dependency_AUTO-06_formal_model_not_selected"
    metrics = {
        "official_toolchain_package_ready": bool(
            discovery["official_toolchain_package_ready"]
        ),
        "official_archive_integrity_pass": bool(
            discovery["official_source"]["archive_integrity_pass"]
        ),
        "oe_version": discovery["official_source"]["oe_version"],
        "hbdk4_compiler_version": discovery["required_versions"][
            "hbdk4_compiler"
        ],
        "hmct_version": discovery["required_versions"]["hmct"],
        "horizon_tc_ui_version": discovery["required_versions"][
            "horizon_tc_ui"
        ],
        "hb_compile_help_pass": True,
        "onnx_preflight_tool_present": (
            ROOT / "scripts/auto14_onnx_preflight.py"
        ).is_file(),
        "runtime_adapter_tests_present": (
            ROOT
            / "starter_ws/src/sanitation_perception/test/test_j6_runtime.py"
        ).is_file(),
        "formal_model_available": dependency_pass,
        "calibration_frame_count": 0,
        "official_model_compile_success": False,
        "quantized_metric_regression_executed": False,
        "board_device_count": board_count,
        "board_runtime_minutes": None,
        "board_fps": None,
        "board_temperature_c": None,
        "board_power_w": None,
        "j6_toolchain_pass": False,
        "j6_runtime_pass": False,
    }
    unexecuted = [
        "AUTO-06 detector and area ONNX preflight",
        "500-frame quantization calibration",
        "official detector and area model compile",
        "x86 reference parity and quantized metric regression",
        "30-minute J6 board stability and performance",
    ]
    write_json(
        evidence / "stage_status.json",
        {
            "schema_version": 1,
            "program": "TZcup autonomous final",
            "stage_id": "AUTO-14",
            "implementation_commit": args.implementation_commit,
            "status": "BLOCKED",
            "first_blocking_layer": first_blocker,
            "attempt_count": 1,
            "machine_gate_pass": False,
            "human_review_required": False,
            "human_approval_required": False,
            "competition_evidence": False,
            "dependencies": {"AUTO-00": "PASS", "AUTO-06": "BLOCKED"},
            "metrics": metrics,
            "unexecuted_items": unexecuted,
            "claim_boundary": (
                "Official package integrity and hb_compile availability are "
                "verified; no formal project model was quantized or compiled "
                "and no physical J6 board runtime was executed."
            ),
        },
    )
    write_json(evidence / "metrics_summary.json", metrics)
    write_json(
        evidence / "attempt_ledger.json",
        {
            "schema_version": 1,
            "stage": "AUTO-14",
            "attempts": [
                {
                    "attempt_id": "AUTO-14-OFFICIAL-TOOLCHAIN-V1",
                    "hypothesis": (
                        "the official J6 toolchain lane can be prepared while "
                        "formal model and board dependencies are unavailable"
                    ),
                    "input_commit": args.implementation_commit,
                    "result": "BLOCKED",
                    "first_failure": first_blocker,
                    "decision": "retain_toolchain_lane_and_block_formal_compile",
                }
            ],
        },
    )
    write_json(
        evidence / "environment.json",
        {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "official_archive_sha256": discovery["official_source"][
                "archive_sha256"
            ],
            "board_device_count": board_count,
        },
    )
    (evidence / "commands.txt").write_text(
        "py -3 scripts/ci_fast.py\n"
        "py -3 scripts/auto14_toolchain_discovery.py <official-package>\n"
        "py -3 scripts/auto14_onnx_preflight.py <AUTO-06-model.onnx> "
        "--calibration-dir <500-frames>\n"
        "hb_compile -c <generated-config>\n",
        encoding="utf-8",
    )
    (evidence / "README.md").write_text(
        "# AUTO-14 evidence\n\n"
        "Official J6 toolchain preparation and dependency boundary. "
        "This is not model compile or board-runtime acceptance.\n",
        encoding="utf-8",
    )

    state_path = ROOT / "config" / "autonomy" / "AUTONOMOUS_STATE.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["stages"]["AUTO-14"].update(
        {
            "status": "BLOCKED",
            "machine_gate_pass": False,
            "blocked": True,
            "blocked_external": False,
            "first_blocking_layer": first_blocker,
            "attempt_count": 1,
            "selected_attempt": "AUTO-14-OFFICIAL-TOOLCHAIN-V1",
            "implementation_commit": args.implementation_commit,
            "evidence_dir": evidence.relative_to(ROOT).as_posix(),
            "metrics": metrics,
            "unexecuted_items": unexecuted,
        }
    )
    state["external_resources"]["j6_official_toolchain_available"] = True
    state["external_resources"]["j6_board_available"] = board_count > 0
    state["final_states"]["J6_TOOLCHAIN_PASS"] = False
    state["final_states"]["J6_RUNTIME_PASS"] = False
    state["final_states"]["J6_RUNTIME_BLOCKED_EXTERNAL"] = board_count == 0
    state["run"]["branch"] = "agent/autonomous-auto14"
    state["run"]["current_stage"] = "AUTO-15"
    state["run"]["last_commit"] = args.implementation_commit
    state_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")

    files = []
    for path in sorted(evidence.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            files.append(
                {
                    "path": path.relative_to(evidence).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    write_json(
        evidence / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-14",
            "implementation_commit": args.implementation_commit,
            "file_count": len(files),
            "files": files,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
