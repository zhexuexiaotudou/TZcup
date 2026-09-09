#!/usr/bin/env python3
"""Run at most three fail-closed D1 canonical ONNX export routes in Docker."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
D1_PT_SHA256 = "1cf60873661811f51cd84fb6aafb403646b67d2add57c4851b0be48ebdff2873"
DEFAULT_IMAGE = "tzcup/perception-product:v12-functional"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def docker_mount(path: Path, target: str, mode: str = "rw") -> str:
    return f"{path.resolve()}:{target}:{mode}"


def run_recorded(
    command: list[str], log_dir: Path, route: str
) -> tuple[int, str, str]:
    result = subprocess.run(
        command,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / f"{route}.stdout.log").write_text(result.stdout, encoding="utf-8")
    (log_dir / f"{route}.stderr.log").write_text(result.stderr, encoding="utf-8")
    return result.returncode, result.stdout, result.stderr


def image_digest(image: str) -> str:
    result = subprocess.run(
        ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    if result.returncode != 0 or not result.stdout.strip().startswith("sha256:"):
        raise RuntimeError(f"cannot resolve Docker image digest: {result.stderr.strip()}")
    return result.stdout.strip()


def git_revision(source: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"cannot resolve exporter revision: {result.stderr.strip()}")
    revision = result.stdout.strip()
    if len(revision) != 40:
        raise RuntimeError("exporter revision is not a full commit SHA")
    return revision


def base_docker(
    image: str,
    checkpoint: Path,
    source: Path,
    evidence: Path,
    site_packages: Path,
) -> list[str]:
    return [
        "docker",
        "run",
        "--rm",
        "--entrypoint",
        "/bin/bash",
        "-e",
        "PYTHONPATH=/opt/d1site",
        "-e",
        "YOLOv5_AUTOINSTALL=false",
        "-v",
        docker_mount(checkpoint.parent, "/models"),
        "-v",
        docker_mount(source, "/source", "ro"),
        "-v",
        docker_mount(evidence, "/evidence"),
        "-v",
        docker_mount(ROOT / "scripts", "/tools", "ro"),
        "-v",
        docker_mount(site_packages, "/opt/d1site", "ro"),
        image,
    ]


def worker_command(base: list[str], arguments: list[str]) -> list[str]:
    quoted = " ".join(
        "'" + value.replace("'", "'\"'\"'") + "'" for value in arguments
    )
    return [*base, "-lc", f"python3 /tools/d1_export_worker.py {quoted}"]


def audit_candidate(
    base: list[str], route: str, evidence: Path, logs: Path
) -> tuple[bool, dict[str, Any] | None, str | None]:
    output = evidence / "routes" / f"{route}_audit.json"
    command = worker_command(
        base,
        [
            "audit",
            "--model",
            f"/evidence/routes/{route}.onnx",
            "--output",
            f"/evidence/routes/{route}_audit.json",
        ],
    )
    returncode, _stdout, stderr = run_recorded(command, logs, f"{route}_audit")
    if returncode != 0 or not output.is_file():
        return False, None, stderr.strip() or f"audit exited {returncode}"
    payload = json.loads(output.read_text(encoding="utf-8"))
    return bool(payload.get("canonical_contract_pass")), payload, None


def d1_output_contract(
    inspection: dict[str, Any], audit: dict[str, Any]
) -> dict[str, Any]:
    if inspection.get("detection_head_from_model_yaml") != "DualDDetect":
        raise RuntimeError("checkpoint is not the inspected DualDDetect architecture")
    expected = [
        {"name": "output0", "shape": [1, 14, 8400]},
        {"name": "1774", "shape": [1, 14, 8400]},
    ]
    if audit.get("outputs") != expected:
        raise RuntimeError(f"unexpected DualDDetect ONNX outputs: {audit.get('outputs')}")
    return {
        "detection_head": "DualDDetect",
        "primary_output_index": 1,
        "primary_output_name": "1774",
        "primary_output_shape": [1, 14, 8400],
        "auxiliary_output_index": 0,
        "auxiliary_output_name": "output0",
        "auxiliary_output_shape": [1, 14, 8400],
        "official_runtime_reference": "detect_dual.py: pred = pred[0][1]",
        "embedded_nms": False,
    }


def route_commands(base: list[str]) -> list[tuple[str, list[str], Path | None]]:
    checkpoint_onnx = Path("best.onnx")
    return [
        (
            "E1",
            [
                *base,
                "-lc",
                "cd /source && python3 export.py --weights /models/best.pt "
                "--include onnx --imgsz 640 640 --batch-size 1 --opset 17 "
                "--device cpu",
            ],
            checkpoint_onnx,
        ),
        (
            "E2",
            worker_command(
                base,
                [
                    "export",
                    "--checkpoint",
                    "/models/best.pt",
                    "--source",
                    "/source",
                    "--output",
                    "/evidence/routes/E2.onnx",
                ],
            ),
            None,
        ),
        (
            "E3",
            worker_command(
                base,
                [
                    "export",
                    "--checkpoint",
                    "/models/best.pt",
                    "--source",
                    "/source",
                    "--output",
                    "/evidence/routes/E3.onnx",
                    "--reconstruct",
                ],
            ),
            None,
        ),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--yolov9-source", type=Path, required=True)
    parser.add_argument("--site-packages", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--image", default=DEFAULT_IMAGE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = args.checkpoint.resolve()
    source = args.yolov9_source.resolve()
    site_packages = args.site_packages.resolve()
    evidence = args.evidence_root.resolve()
    logs = evidence / "routes" / "logs"
    evidence.mkdir(parents=True, exist_ok=True)
    if not checkpoint.is_file() or sha256(checkpoint) != D1_PT_SHA256:
        print("BLOCKED: pinned D1 checkpoint is missing or has wrong SHA", file=sys.stderr)
        return 2
    if not site_packages.is_dir():
        print("BLOCKED: isolated site-packages directory is missing", file=sys.stderr)
        return 2
    environment = {
        "image": args.image,
        "image_digest": image_digest(args.image),
        "yolov9_revision": git_revision(source),
        "site_packages_lock": str(site_packages.parent / "environment-target-freeze.txt"),
        "site_packages_lock_sha256": sha256(
            site_packages.parent / "environment-target-freeze.txt"
        ),
    }
    base = base_docker(args.image, checkpoint, source, evidence, site_packages)
    inspect_path = evidence / "D1_CHECKPOINT_INSPECTION.json"
    inspect_command = worker_command(
        base,
        [
            "inspect",
            "--checkpoint",
            "/models/best.pt",
            "--source",
            "/source",
            "--output",
            "/evidence/D1_CHECKPOINT_INSPECTION.json",
        ],
    )
    inspect_exit, _stdout, inspect_error = run_recorded(
        inspect_command, logs, "checkpoint_inspection"
    )
    if inspect_exit != 0 or not inspect_path.is_file():
        report = {
            "schema_version": 1,
            "status": "blocked_checkpoint_inspection",
            "environment": environment,
            "error": inspect_error.strip(),
        }
        (evidence / "D1_EXPORT_REPORT.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return 2
    inspection = json.loads(inspect_path.read_text(encoding="utf-8"))
    attempts: list[dict[str, Any]] = []
    selected_route: str | None = None
    selected_audit: dict[str, Any] | None = None
    selected_output_contract: dict[str, Any] | None = None
    routes_dir = evidence / "routes"
    routes_dir.mkdir(parents=True, exist_ok=True)
    for route, command, checkpoint_output in route_commands(base):
        candidate = routes_dir / f"{route}.onnx"
        candidate.unlink(missing_ok=True)
        if checkpoint_output:
            host_output = checkpoint.parent / checkpoint_output
            host_output.unlink(missing_ok=True)
        returncode, _stdout, stderr = run_recorded(command, logs, route)
        if returncode == 0 and checkpoint_output:
            host_output = checkpoint.parent / checkpoint_output
            if host_output.is_file():
                os.replace(host_output, candidate)
        attempt: dict[str, Any] = {
            "route": route,
            "exit_code": returncode,
            "stdout_log": str(logs / f"{route}.stdout.log"),
            "stderr_log": str(logs / f"{route}.stderr.log"),
            "raw_error_tail": stderr[-4000:],
            "artifact_created": candidate.is_file(),
        }
        if returncode == 0 and candidate.is_file():
            audit_pass, audit, audit_error = audit_candidate(base, route, evidence, logs)
            attempt["audit_pass"] = audit_pass
            attempt["audit_error"] = audit_error
            if audit is not None:
                attempt["artifact_sha256"] = audit["sha256"]
                attempt["artifact_size_bytes"] = audit["size_bytes"]
            if audit_pass:
                try:
                    output_contract = d1_output_contract(inspection, audit)
                except RuntimeError as error:
                    attempt["audit_pass"] = False
                    attempt["audit_error"] = str(error)
                else:
                    selected_route = route
                    selected_audit = audit
                    selected_output_contract = output_contract
                    attempts.append(attempt)
                    break
        attempts.append(attempt)
    status = "exported" if selected_route else "blocked_all_routes_failed"
    report = {
        "schema_version": 1,
        "model_id": "d1_littercam_yolov9c_development_export",
        "development_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "status": status,
        "source_checkpoint_sha256": D1_PT_SHA256,
        "checkpoint_inspection": inspection,
        "environment": environment,
        "attempts": attempts,
        "selected_route": selected_route,
        "canonical_onnx_audit": selected_audit,
        "output_contract": selected_output_contract,
        "parity_status": "not_run",
    }
    report_path = evidence / "D1_EXPORT_REPORT.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = {
        "schema_version": 1,
        "model_id": report["model_id"],
        "development_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "source": {
            "repo_id": "aryanshh/littercamv3",
            "revision": "861363597e109f9f0840f537f48d890cef5b5461",
            "filename": "best.pt",
            "sha256": D1_PT_SHA256,
        },
        "class_names": inspection.get("class_names"),
        "output_contract": selected_output_contract,
        "export_contract": {
            "input_shape": [1, 3, 640, 640],
            "input_dtype": "float32",
            "opset": 17,
            "dynamic_axes": False,
            "embedded_nms": False,
        },
        "artifact": {
            "path": str(routes_dir / f"{selected_route}.onnx")
            if selected_route
            else None,
            "sha256": selected_audit.get("sha256") if selected_audit else None,
            "size_bytes": selected_audit.get("size_bytes") if selected_audit else None,
            "export_route": selected_route,
            "onnx_checker_pass": bool(selected_audit),
            "canonical_contract_pass": bool(
                selected_audit and selected_audit.get("canonical_contract_pass")
            ),
        },
        "environment": environment,
    }
    (evidence / "D1_CANONICAL_ONNX_MANIFEST.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if selected_route else 2


if __name__ == "__main__":
    raise SystemExit(main())
