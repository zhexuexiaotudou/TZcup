"""Validate an S100P product-artifact preparation bundle without deployment.

This checker is intentionally local and low-memory: it reads only manifest/config
JSON plus each candidate file in 1 MiB SHA-256 blocks.  It never copies files,
opens a network connection, starts ROS, installs packages, or invokes a shell.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "config" / "s100p_product_artifact_bundle.json"
DEFAULT_ARTIFACT_ROOT = ROOT / ".work" / "formal_perception_assets"

EXPECTED_RUNTIME = {
    "platform": "rdk_s100",
    "board": "RDK S100P",
    "soc": "Journey 6P",
    "march": "nash-m",
}
EXPECTED_ASSETS = {
    "board_runtime_manifest",
    "dosod_hbm",
    "dosod_vocabulary",
    "edgesam_encoder_hbm",
    "edgesam_decoder_hbm",
    "class_mapping_config",
    "overlay_package_inventory",
    "board_launch_parameter_record",
}
MODEL_ASSETS = {
    "dosod_hbm": (
        "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
        "project_four_class_dosod_s100p_detector",
        "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
    ),
    "dosod_vocabulary": (
        "dosod/tzcup_offline_vocabulary.json",
        "frozen_project_prompt_vocabulary",
        "c50129b5badf6ed7bb85e692ab493d8bdb58da6a",
    ),
    "edgesam_encoder_hbm": (
        "edgesam/edgesam_encoder_512.hbm",
        "edgesam_512_s100p_image_encoder",
        "d24d99671f41a9c0003061248bded64a481e9059",
    ),
    "edgesam_decoder_hbm": (
        "edgesam/edgesam_decoder_512.hbm",
        "edgesam_512_s100p_box_prompt_decoder",
        "d24d99671f41a9c0003061248bded64a481e9059",
    ),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, "missing"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"invalid:{type(exc).__name__}"
    if not isinstance(payload, dict):
        return None, "not_mapping"
    return payload, None


def _relative(value: Any) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    normalized = value.replace("\\", "/")
    path = PurePosixPath(normalized)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return path.as_posix()


def _under_root(root: Path, candidate: Path) -> bool:
    try:
        candidate.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _board_path(root: str, relative: str) -> str:
    return str(PurePosixPath(root) / PurePosixPath(relative))


def _append(blockers: list[str], value: str) -> None:
    if value not in blockers:
        blockers.append(value)


def _validate_runtime_manifest(
    path: Path,
    assets: Mapping[str, Any],
    blockers: list[str],
    checks: dict[str, bool],
) -> None:
    runtime, error = _load_json(path)
    checks["board_runtime_manifest_parseable"] = error is None
    if error:
        _append(blockers, f"board_runtime_manifest_{error}")
        checks["board_runtime_manifest_matches_nash_m"] = False
        checks["board_runtime_manifest_models_complete"] = False
        return
    assert runtime is not None
    checks["board_runtime_manifest_matches_nash_m"] = (
        runtime.get("schema_version") == 1
        and runtime.get("board_runtime_contract") == EXPECTED_RUNTIME
    )
    if not checks["board_runtime_manifest_matches_nash_m"]:
        _append(blockers, "board_runtime_manifest_target_mismatch")
    rows = runtime.get("artifacts")
    complete = isinstance(rows, Mapping)
    if isinstance(rows, Mapping):
        for name, (relative, model_role, source_revision) in MODEL_ASSETS.items():
            row = rows.get(relative)
            asset = assets.get(name)
            if not isinstance(row, Mapping):
                complete = False
                _append(blockers, f"runtime_manifest_model_row_missing:{name}")
                continue
            if not isinstance(asset, Mapping) or any(
                row.get(key) != asset.get(key)
                for key in ("sha256", "byte_size")
            ):
                complete = False
                _append(blockers, f"runtime_manifest_digest_mismatch:{name}")
            if row.get("model_role") != model_role or row.get("source_revision") != source_revision:
                complete = False
                _append(blockers, f"runtime_manifest_provenance_mismatch:{name}")
    else:
        _append(blockers, "runtime_manifest_artifacts_missing")
    checks["board_runtime_manifest_models_complete"] = complete


def validate_bundle(
    manifest_path: str | Path = DEFAULT_MANIFEST,
    *,
    artifact_root: str | Path = DEFAULT_ARTIFACT_ROOT,
    repository_root: str | Path = ROOT,
) -> dict[str, Any]:
    """Return a local preparation report; no deployment action is performed."""

    manifest_path = Path(manifest_path).resolve()
    artifact_root = Path(artifact_root).resolve()
    repository_root = Path(repository_root).resolve()
    payload, manifest_error = _load_json(manifest_path)
    blockers: list[str] = []
    checks: dict[str, bool] = {}
    if manifest_error:
        return {
            "schema_version": 1,
            "report_id": "tzcup_s100p_product_artifact_bundle_validation_v1",
            "operation": "prepare_only_no_deployment",
            "status": "BLOCKED",
            "ready": False,
            "blockers": [f"bundle_manifest_{manifest_error}"],
            "checks": {"bundle_manifest_parseable": False},
            "asset_results": {},
        }
    assert payload is not None
    checks["bundle_manifest_parseable"] = True
    checks["bundle_identity_valid"] = (
        payload.get("schema_version") == 1
        and payload.get("bundle_id") == "tzcup_s100p_product_artifact_bundle_v1"
        and payload.get("operation_boundary")
        == "prepare_only_no_board_copy_ssh_dependency_install_node_start_or_data_collection"
    )
    if not checks["bundle_identity_valid"]:
        _append(blockers, "bundle_identity_or_operation_boundary_invalid")
    checks["board_runtime_contract_valid"] = payload.get("board_runtime_contract") == EXPECTED_RUNTIME
    if not checks["board_runtime_contract_valid"]:
        _append(blockers, "board_runtime_contract_not_rdk_s100_nash_m")

    policy = payload.get("path_policy")
    policy_valid = isinstance(policy, Mapping) and (
        policy.get("source_paths_are_relative") is True
        and policy.get("source_path_escape_forbidden") is True
        and policy.get("board_target_paths_are_derived_not_executed") is True
        and isinstance(policy.get("board_artifact_root"), str)
        and policy["board_artifact_root"].startswith("/")
        and isinstance(policy.get("board_overlay_root"), str)
        and policy["board_overlay_root"].startswith("/")
    )
    checks["path_policy_valid"] = policy_valid
    if not policy_valid:
        _append(blockers, "path_policy_invalid")
        policy = {}
    assets = payload.get("assets")
    asset_results: dict[str, Any] = {}
    if not isinstance(assets, Mapping) or set(assets) != EXPECTED_ASSETS:
        checks["asset_role_set_exact"] = False
        _append(blockers, "asset_role_set_invalid")
        assets = {}
    else:
        checks["asset_role_set_exact"] = True

    for name in sorted(EXPECTED_ASSETS):
        asset = assets.get(name)
        result: dict[str, Any] = {"valid": False}
        asset_results[name] = result
        if not isinstance(asset, Mapping):
            _append(blockers, f"asset_definition_missing:{name}")
            continue
        source_scope = asset.get("source_scope")
        relative = _relative(asset.get("relative_path"))
        target_relative = _relative(asset.get("target_relative_path"))
        target_root = asset.get("target_root")
        if source_scope not in {"artifact_root", "repository_root"} or not relative:
            _append(blockers, f"source_path_invalid:{name}")
            continue
        if target_root not in {"artifact_root", "overlay_root"} or not target_relative:
            _append(blockers, f"board_target_path_invalid:{name}")
            continue
        source_root = artifact_root if source_scope == "artifact_root" else repository_root
        source_path = source_root / relative
        result["source_path"] = str(source_path)
        target_base = str(policy.get("board_artifact_root", "")) if target_root == "artifact_root" else str(policy.get("board_overlay_root", ""))
        result["board_target_path"] = _board_path(target_base, target_relative)
        if not _under_root(source_root, source_path):
            _append(blockers, f"source_path_escape:{name}")
            continue
        declared_sha = asset.get("sha256")
        declared_size = asset.get("byte_size")
        if not isinstance(declared_sha, str) or len(declared_sha) != 64:
            _append(blockers, f"declared_sha256_missing_or_invalid:{name}")
        if isinstance(declared_size, bool) or not isinstance(declared_size, int) or declared_size <= 0:
            _append(blockers, f"declared_byte_size_missing_or_invalid:{name}")
        if not source_path.is_file():
            _append(blockers, f"asset_missing:{name}")
            continue
        actual_size = source_path.stat().st_size
        actual_sha = _sha256(source_path)
        result.update({"actual_sha256": actual_sha, "actual_byte_size": actual_size})
        digest_ok = actual_sha == declared_sha and actual_size == declared_size
        result["digest_matches"] = digest_ok
        if not digest_ok:
            _append(blockers, f"asset_digest_or_size_mismatch:{name}")
        result["valid"] = digest_ok

    for name, (relative, role, revision) in MODEL_ASSETS.items():
        asset = assets.get(name)
        row_ok = isinstance(asset, Mapping) and (
            asset.get("relative_path") == relative
            and asset.get("target_root") == "artifact_root"
            and asset.get("target_relative_path") == relative
            and asset.get("model_role") == role
            and asset.get("source_revision") == revision
        )
        checks[f"{name}_provenance_valid"] = row_ok
        if not row_ok:
            _append(blockers, f"model_provenance_invalid:{name}")

    runtime_asset = assets.get("board_runtime_manifest")
    if isinstance(runtime_asset, Mapping) and _relative(runtime_asset.get("relative_path")):
        _validate_runtime_manifest(
            artifact_root / str(_relative(runtime_asset["relative_path"])),
            assets,
            blockers,
            checks,
        )
    else:
        checks["board_runtime_manifest_parseable"] = False
        checks["board_runtime_manifest_matches_nash_m"] = False
        checks["board_runtime_manifest_models_complete"] = False
        _append(blockers, "board_runtime_manifest_definition_missing")

    status = payload.get("status")
    checks["declared_status_valid"] = status in {"PREPARATION_ONLY_BLOCKED", "PREPARED_NOT_DEPLOYED"}
    if not checks["declared_status_valid"]:
        _append(blockers, "bundle_declared_status_invalid")
    local_assets_valid = all(result.get("valid") for result in asset_results.values())
    runtime_valid = bool(
        checks.get("board_runtime_manifest_matches_nash_m")
        and checks.get("board_runtime_manifest_models_complete")
    )
    if status == "PREPARED_NOT_DEPLOYED" and not (local_assets_valid and runtime_valid):
        _append(blockers, "bundle_claims_prepared_without_complete_verified_assets")
    ready = bool(
        not blockers
        and status == "PREPARED_NOT_DEPLOYED"
        and local_assets_valid
        and runtime_valid
    )
    checks["all_formal_assets_verified"] = ready
    return {
        "schema_version": 1,
        "report_id": "tzcup_s100p_product_artifact_bundle_validation_v1",
        "operation": "prepare_only_no_deployment",
        "status": "PREPARED_NOT_DEPLOYED" if ready else "BLOCKED",
        "ready": ready,
        "manifest_path": str(manifest_path),
        "artifact_root": str(artifact_root),
        "repository_root": str(repository_root),
        "checks": checks,
        "blockers": blockers,
        "asset_results": asset_results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--report", type=Path, help="Optional local JSON report path.")
    parser.add_argument(
        "--allow-blocked-exit-zero",
        action="store_true",
        help="Keep the report fail-closed but return zero for inventory tooling.",
    )
    args = parser.parse_args()
    report = validate_bundle(
        args.manifest,
        artifact_root=args.artifact_root,
        repository_root=args.repository_root,
    )
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["ready"] or args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
