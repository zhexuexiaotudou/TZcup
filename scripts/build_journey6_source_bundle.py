#!/usr/bin/env python3
"""Create a reference-only, checksum-locked Journey 6 source deployment bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Iterable

import yaml


REQUIRED_COMPONENTS = {
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
FORBIDDEN_PATH_TOKENS = ("g5_v2", "sealed_final", "dev_val", "rdk", "s100", "s100p", "s600")
FORBIDDEN_PAYLOAD_SUFFIXES = {".hbm", ".bc", ".hbo", ".tgz", ".tar", ".zip"}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalized(value: object) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value).casefold()).strip("_")


def load_document(path: Path) -> dict:
    if path.suffix.lower() == ".json":
        value = json.loads(path.read_text(encoding="utf-8"))
    else:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected object document: {path}")
    return value


def path_digest(path: Path) -> tuple[str, list[dict]]:
    if path.is_symlink():
        raise ValueError("symlink components are not allowed")
    if path.is_file():
        digest = file_sha256(path)
        return digest, [{"path": path.name, "sha256": digest, "bytes": path.stat().st_size}]
    if not path.is_dir():
        raise FileNotFoundError(path)
    rows = []
    for candidate in sorted(path.rglob("*")):
        if candidate.is_symlink():
            raise ValueError(f"symlink component is not allowed: {candidate}")
        if candidate.is_file():
            relative = candidate.relative_to(path).as_posix()
            rows.append({"path": relative, "sha256": file_sha256(candidate), "bytes": candidate.stat().st_size})
    canonical = "".join(f"{row['sha256']}  {row['path']}\n" for row in rows).encode()
    return hashlib.sha256(canonical).hexdigest(), rows


def _semantic_failures(component_id: str, path: Path) -> list[str]:
    failures: list[str] = []
    try:
        if component_id.endswith("_canonical_onnx"):
            if path.suffix.lower() != ".onnx":
                failures.append("canonical_onnx_suffix_invalid")
            elif path.stat().st_size <= 0:
                failures.append("canonical_onnx_empty")
        elif component_id == "model_lock":
            value = load_document(path)
            if value.get("selection_frozen") is not True or not all(value.get("selected", {}).get(role) for role in ("detector", "close_range_classifier")):
                failures.append("model_selection_not_frozen")
        elif component_id == "model_license_audit":
            value = load_document(path)
            if value.get("release_license_pass") is not True:
                failures.append("model_license_not_release_clear")
        elif component_id == "calibration_manifest":
            value = load_document(path)
            if value.get("calibration_ready") is not True or value.get("sealed_access_allowed") is not False:
                failures.append("calibration_manifest_not_ready")
        elif component_id == "calibration_distribution":
            value = load_document(path)
            if value.get("stratification_pass") is not True:
                failures.append("calibration_distribution_not_ready")
        elif component_id == "calibration_sha256sums":
            rows = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
            parsed = {}
            for line in rows:
                match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
                if match:
                    parsed[match.group(2)] = match.group(1)
            for name in ("J6_CALIBRATION_MANIFEST.json", "J6_CALIBRATION_DISTRIBUTION.json"):
                matches = [(relative, digest) for relative, digest in parsed.items() if relative.endswith(name)]
                if len(matches) != 1:
                    failures.append(f"calibration_checksums_missing_{name}")
                    continue
                relative, expected = matches[0]
                candidates = (path.parent / relative, path.parent.parent / relative)
                target = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
                if target is None or file_sha256(target) != expected:
                    failures.append(f"calibration_checksums_mismatch_{name}")
        elif component_id == "golden_tensor_lock":
            value = load_document(path)
            if value.get("golden_tensor_ready") is not True or not value.get("tensors"):
                failures.append("golden_tensor_lock_not_ready")
            else:
                for tensor in value["tensors"]:
                    reference = tensor.get("path")
                    expected = tensor.get("sha256")
                    target = Path(reference) if isinstance(reference, str) else Path("")
                    if reference and not target.is_absolute():
                        target = path.parent / target
                    if (
                        not reference
                        or not isinstance(expected, str)
                        or not re.fullmatch(r"[0-9a-f]{64}", expected)
                        or not target.is_file()
                        or file_sha256(target) != expected
                    ):
                        failures.append("golden_tensor_reference_invalid")
                        break
        elif component_id == "toolchain_lock":
            value = load_document(path)
            if value.get("target_family") != "journey6" or value.get("status") != "validated":
                failures.append("journey6_toolchain_not_validated")
        elif component_id == "nash_profiles":
            profiles = [load_document(item) for item in sorted(path.glob("*.yaml"))]
            marches = {item.get("target_march") for item in profiles}
            if not {"nash-e", "nash-m", "nash-p"}.issubset(marches) or any(item.get("status") != "validated" for item in profiles if item.get("target_march") != "auto"):
                failures.append("nash_profile_matrix_not_validated")
    except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as error:
        failures.append(f"semantic_document_error:{type(error).__name__}")
    return failures


def build(template_path: Path, output_dir: Path, repo_root: Path, *, replace: bool = False) -> dict:
    template_path = template_path.resolve()
    output_dir = output_dir.resolve()
    repo_root = repo_root.resolve()
    template = load_document(template_path)
    blockers: list[dict] = []

    def block(code: str, **details: object) -> None:
        blockers.append({"code": code, **details})

    if template.get("target_family") != "journey6" or template.get("target_sku") != "auto" or template.get("target_march") != "auto":
        block("source_bundle_target_contract_invalid")
    if template.get("source_only") is not True:
        block("source_bundle_must_be_source_only")
    components = template.get("components", [])
    if not isinstance(components, list):
        components = []
        block("source_bundle_components_invalid")
    ids = [item.get("id") for item in components if isinstance(item, dict)]
    missing_ids = sorted(REQUIRED_COMPONENTS - set(ids))
    duplicate_ids = sorted(item for item in set(ids) if ids.count(item) > 1)
    if missing_ids:
        block("required_source_components_missing", components=missing_ids)
    if duplicate_ids:
        block("duplicate_source_component_ids", components=duplicate_ids)

    resolved_components: list[dict] = []
    checksum_rows: list[str] = []
    for item in components:
        if not isinstance(item, dict):
            block("invalid_source_component")
            continue
        component_id = item.get("id")
        raw_path = item.get("path")
        resolved = {**item, "observed_sha256": None, "files": []}
        resolved_components.append(resolved)
        if item.get("copy_policy") != "reference_only":
            block("component_copy_policy_must_be_reference_only", component=component_id)
        if not isinstance(raw_path, str) or not raw_path:
            block("source_component_path_unresolved", component=component_id)
            continue
        expanded = os.path.expandvars(raw_path)
        path = Path(expanded)
        if not path.is_absolute():
            path = repo_root / path
        path = path.resolve()
        resolved["resolved_path"] = str(path)
        normalized_path = normalized(path)
        forbidden = [token for token in FORBIDDEN_PATH_TOKENS if token in normalized_path]
        if forbidden:
            block("forbidden_source_component_path", component=component_id, tokens=forbidden)
            continue
        if path.suffix.lower() in FORBIDDEN_PAYLOAD_SUFFIXES:
            block("compiled_or_archive_payload_forbidden", component=component_id, suffix=path.suffix.lower())
            continue
        if not path.exists():
            block("source_component_missing", component=component_id, path=str(path))
            continue
        try:
            observed, rows = path_digest(path)
        except (OSError, ValueError) as error:
            block("source_component_digest_failed", component=component_id, error=str(error))
            continue
        resolved["observed_sha256"] = observed
        resolved["files"] = rows
        forbidden_payloads = sorted(
            row["path"] for row in rows if Path(row["path"]).suffix.lower() in FORBIDDEN_PAYLOAD_SUFFIXES
        )
        if forbidden_payloads:
            block(
                "compiled_or_archive_payload_nested_in_source_component",
                component=component_id,
                files=forbidden_payloads,
            )
        expected = item.get("expected_sha256")
        if expected is not None and expected != observed:
            block("source_component_sha256_mismatch", component=component_id, expected=expected, observed=observed)
        for failure in _semantic_failures(str(component_id), path):
            block(failure, component=component_id)
        checksum_rows.append(f"{observed}  reference/{component_id}")

    ready = not blockers
    manifest = {
        "schema_version": 1,
        "bundle_id": template.get("bundle_id"),
        "target_family": "journey6",
        "target_sku": "auto",
        "target_march": "auto",
        "source_only": True,
        "status": "ready" if ready else "blocked_external",
        "source_bundle_ready": ready,
        "components": resolved_components,
        "blockers": blockers,
        "prohibited_payloads": ["SDK archives", "HBM", "BC", "HBO", "large weights"],
        "truth_boundary": "The bundle locks source references only; it does not copy or claim SDK, HBM, board, x86-runtime, or HIL execution evidence.",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "J6_SOURCE_BUNDLE_MANIFEST.json"
    sums_path = output_dir / "J6_SOURCE_BUNDLE_SHA256SUMS"
    status_path = output_dir / "J6_SOURCE_BUNDLE_STATUS.json"
    if not replace and any(path.exists() for path in (manifest_path, sums_path, status_path)):
        raise FileExistsError("source bundle output exists; use --replace explicitly")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    checksum_rows.extend((
        f"{file_sha256(template_path)}  input/source_bundle.template.yaml",
        f"{file_sha256(manifest_path)}  evidence/J6_SOURCE_BUNDLE_MANIFEST.json",
    ))
    sums_path.write_text("\n".join(sorted(checksum_rows)) + "\n", encoding="utf-8")
    status = {
        "schema_version": 1,
        "status": manifest["status"],
        "source_bundle_ready": ready,
        "manifest_sha256": file_sha256(manifest_path),
        "checksums_sha256": file_sha256(sums_path),
        "blockers": blockers,
    }
    status_path.write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    return status


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", type=Path, default=repo_root / "deploy" / "journey6" / "source_bundle" / "source_bundle.template.yaml")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    status = build(args.template, args.output_dir, args.repo_root, replace=args.replace)
    print(json.dumps(status, indent=2))
    return 0 if status["source_bundle_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
