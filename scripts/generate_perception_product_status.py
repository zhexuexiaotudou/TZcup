#!/usr/bin/env python3
"""Generate fail-closed P14 status and evidence-index artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


EVIDENCE_KEYS = {
    "sealed_final": ("metrics", "P5_FINAL_PASS"),
    "ros_live": ("ROS_LIVE_PASS",),
    "spot_clean": ("LEARNED_SPOT_CLEAN_PASS",),
    "soak": ("soak_gate_pass",),
    "replay": ("REPLAY_PASS",),
    "container_release": ("CONTAINER_RELEASE_PASS",),
    "health_watchdog": ("HEALTH_WATCHDOG_PASS",),
    "cuda_runtime": ("CUDA_RUNTIME_PERFORMANCE_PASS",),
    "rollback": ("ROLLBACK_PASS",),
    "cold_start": ("COLD_START_10_OF_10_ACTIVE",),
    "j6_operator_audit": ("J6_OPERATOR_AUDIT_PASS",),
    "j6_ptq": ("J6_PTQ_PARITY_PASS",),
    "j6_compile": ("J6_COMPILE_PASS",),
    "j6_board": ("J6_BOARD_RUNTIME_PASS",),
    "field": ("PRODUCT_FIELD_READY",),
    "competition": ("COMPETITION_PERCEPTION_PASS",),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"evidence must be a JSON object: {path}")
    return payload


def _nested_true(payload: dict, keys: tuple[str, ...]) -> bool:
    value: object = payload
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return False
        value = value[key]
    return value is True


def _git_commit() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=Path(__file__).resolve().parents[1],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def generate(
    output_dir: Path,
    evidence: dict[str, Path | None],
    *,
    source_commit: str,
    external_blockers: set[str] | None = None,
    model_registry_path: Path | None = None,
    release_manifest_path: Path | None = None,
) -> dict:
    unknown = set(evidence) - set(EVIDENCE_KEYS)
    if unknown:
        raise ValueError(f"unknown evidence keys: {sorted(unknown)}")
    output_dir.mkdir(parents=True, exist_ok=True)
    external_blockers = external_blockers or set()
    checks = {}
    index = {}
    for name, field_path in EVIDENCE_KEYS.items():
        path = evidence.get(name)
        if path is None:
            checks[name] = False
            index[name] = {"status": "missing", "path": None, "sha256": None}
            continue
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        payload = _load(path)
        checks[name] = _nested_true(payload, field_path)
        index[name] = {
            "status": "passed" if checks[name] else "failed",
            "path": str(path),
            "sha256": sha256(path),
            "required_true_field": ".".join(field_path),
        }

    sim_requirements = ("sealed_final", "ros_live", "spot_clean", "soak", "replay")
    x86_requirements = (
        "container_release",
        "health_watchdog",
        "cuda_runtime",
        "rollback",
        "cold_start",
    )
    j6_toolchain_requirements = (
        "j6_operator_audit",
        "j6_ptq",
        "j6_compile",
    )
    sim_ready = all(checks[name] for name in sim_requirements)
    statuses = {
        "PRODUCT_SIM_PERCEPTION_READY": sim_ready,
        "PRODUCT_X86_RUNTIME_READY": sim_ready
        and all(checks[name] for name in x86_requirements),
        "PRODUCT_J6_TOOLCHAIN_READY": all(
            checks[name] for name in j6_toolchain_requirements
        ),
        "PRODUCT_J6_BOARD_READY": all(
            checks[name] for name in j6_toolchain_requirements
        )
        and checks["j6_board"],
        "PRODUCT_FIELD_READY": checks["field"],
        "COMPETITION_PERCEPTION_PASS": sim_ready and checks["competition"],
    }
    status_payload = {
        "schema_version": 1,
        "source_commit": source_commit,
        "statuses": statuses,
        "evidence_checks": checks,
        "all_current_machine_gates_pass": all(statuses.values()),
    }
    blockers = []
    for name, passed in checks.items():
        if passed:
            continue
        blockers.append(
            {
                "gate": name,
                "classification": (
                    "external_resource" if name in external_blockers else "incomplete_gate"
                ),
                "evidence_status": index[name]["status"],
            }
        )
    blocker_payload = {
        "schema_version": 1,
        "source_commit": source_commit,
        "blockers": blockers,
        "external_only": bool(blockers)
        and all(item["classification"] == "external_resource" for item in blockers),
    }
    if release_manifest_path is None:
        release_payload = {"schema_version": 1, "source_commit": source_commit}
    else:
        release_manifest_path = release_manifest_path.resolve()
        release_payload = _load(release_manifest_path)
        if release_payload.get("source_commit") != source_commit:
            raise ValueError("release manifest source_commit mismatch")
    release_payload.update({
        "status_sha256": None,
        "evidence": index,
        "release_ready": statuses["PRODUCT_X86_RUNTIME_READY"],
    })

    status_path = output_dir / "PERCEPTION_PRODUCT_STATUS.json"
    blockers_path = output_dir / "PERCEPTION_PRODUCT_BLOCKERS.json"
    registry_path = output_dir / "PERCEPTION_MODEL_REGISTRY.json"
    release_path = output_dir / "PERCEPTION_RELEASE_MANIFEST.json"
    status_path.write_text(
        json.dumps(status_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    blockers_path.write_text(
        json.dumps(blocker_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if model_registry_path is None:
        registry_payload = {
            "schema_version": 1,
            "source_commit": source_commit,
            "status": "not_bound_until_formal_release_packaging",
            "models": {},
        }
    else:
        model_registry_path = model_registry_path.resolve()
        registry_payload = _load(model_registry_path)
        if not registry_payload.get("models"):
            raise ValueError("formal model registry has no models")
        registry_payload = {
            **registry_payload,
            "source_path": str(model_registry_path),
            "source_sha256": sha256(model_registry_path),
        }
    registry_path.write_text(
        json.dumps(registry_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    release_payload["status_sha256"] = sha256(status_path)
    release_path.write_text(
        json.dumps(release_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# PERCEPTION PRODUCT EVIDENCE INDEX",
        "",
        f"Source commit: `{source_commit}`",
        "",
        "| Gate | Result | SHA-256 | Path |",
        "|---|---:|---|---|",
    ]
    for name, record in index.items():
        lines.append(
            f"| {name} | {record['status']} | {record['sha256'] or '-'} | "
            f"{record['path'] or '-'} |"
        )
    (output_dir / "PERCEPTION_PRODUCT_EVIDENCE_INDEX.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    return status_payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--source-commit", default=None)
    parser.add_argument("--model-registry", type=Path)
    parser.add_argument("--release-manifest", type=Path)
    parser.add_argument(
        "--external-blocker",
        action="append",
        default=[],
        choices=tuple(EVIDENCE_KEYS),
    )
    for name in EVIDENCE_KEYS:
        parser.add_argument(f"--{name.replace('_', '-')}", type=Path)
    args = parser.parse_args()
    evidence = {name: getattr(args, name) for name in EVIDENCE_KEYS}
    result = generate(
        args.output_dir,
        evidence,
        source_commit=args.source_commit or _git_commit(),
        external_blockers=set(args.external_blocker),
        model_registry_path=args.model_registry,
        release_manifest_path=args.release_manifest,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
