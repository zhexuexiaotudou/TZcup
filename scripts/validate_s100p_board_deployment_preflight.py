#!/usr/bin/env python3
"""Read-only admission for an already-authorized S100P staging operation."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import stat
from pathlib import Path
from typing import Any, Callable, Mapping

from formal_s100_live_acceptance_core import runtime_closure_binding


SAFETY_MARGIN_BYTES = 1024 * 1024 * 1024
EXPECTED_MODEL_TOKEN = "rdk s100p"
EXPECTED_COMPATIBLE_TOKEN = "drobot,s100-rdk"
EXPECTED_ARCHITECTURE = "aarch64"
EXPECTED_BPU_DEVICE = "/dev/bpu_core0"
TROS_SETUPS = ("/opt/tros/humble/setup.bash", "/opt/tros/jazzy/setup.bash")
FINAL_REPORT_ID = "tzcup_s100p_final_predeploy_audit_v1"
FINAL_BOUNDARY = "local_read_only_audit_no_board_copy_ssh_install_node_start_data_collection_or_receipt_generation"
MODEL_RECEIPT_ID = "tzcup_s100p_model_payload_receipt_v1"
EXPECTED_PAYLOAD_PATHS = {
    "dosod_hbm": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
    "dosod_vocabulary": "dosod/tzcup_offline_vocabulary.json",
    "edgesam_encoder_hbm": "edgesam/edgesam_encoder_512.hbm",
    "edgesam_decoder_hbm": "edgesam/edgesam_decoder_512.hbm",
}
DEFAULT_ACTIVE = "/opt/tzcup/s100p"


def _append(blockers: list[str], value: str) -> None:
    if value not in blockers:
        blockers.append(value)


def _json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _is_digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _inside(root: Path, absolute: str) -> Path:
    if not isinstance(absolute, str) or not absolute.startswith("/") or "\\" in absolute:
        raise ValueError("board path must be absolute")
    if any(part in ("", ".", "..") for part in absolute.split("/")[1:]):
        raise ValueError("board path must be normalized")
    return root / absolute.lstrip("/")


def _relative_entry(root: Path, relative: Any) -> Path | None:
    if not isinstance(relative, str) or not relative:
        return None
    candidate = Path(relative)
    if candidate.is_absolute() or "\\" in relative or any(part in ("", ".", "..") for part in relative.split("/")):
        return None
    return root / candidate


def _regular_nonlink(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _directory_nonlink(path: Path) -> bool:
    return path.is_dir() and not path.is_symlink()


def _nonlink_ancestors(root: Path, path: Path) -> bool:
    """Reject a target that leaves the board root through any symlink ancestor."""
    try:
        relative = path.relative_to(root)
    except ValueError:
        return False
    cursor = root
    if cursor.is_symlink():
        return False
    for part in relative.parts:
        cursor = cursor / part
        if cursor.is_symlink():
            return False
    return True


def _character_nonlink(path: Path, stat_path: Callable[[Path], os.stat_result] = os.lstat) -> bool:
    try:
        return not path.is_symlink() and stat.S_ISCHR(stat_path(path).st_mode)
    except OSError:
        return False


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").lower()
    except OSError:
        return ""


def _verified_entry(root: Path, entry: Any) -> tuple[Path | None, str | None]:
    if not isinstance(entry, Mapping):
        return None, "invalid_entry"
    path = _relative_entry(root, entry.get("relative_path"))
    size, digest = entry.get("byte_size"), entry.get("sha256")
    if path is None or not isinstance(size, int) or size <= 0 or not _is_digest(digest):
        return None, "invalid_entry"
    if not _nonlink_ancestors(root, path) or not _regular_nonlink(path):
        return None, "missing_or_linked"
    try:
        if path.stat().st_size != size or _sha256(path) != digest:
            return None, "digest_or_size_mismatch"
    except OSError:
        return None, "unreadable"
    return path, None


def _validate_final_report(report: Any) -> tuple[bool, Mapping[str, Any] | None, Mapping[str, Any] | None]:
    if not isinstance(report, Mapping):
        return False, None, None
    checks, blockers = report.get("checks"), report.get("blockers")
    receipts = report.get("receipt_requirements", {}).get("receipts")
    model = receipts.get("model_payload") if isinstance(receipts, Mapping) else None
    identity = report.get("pc_session_runtime_identity")
    handoff = report.get("board_handoff_binding")
    closure = identity.get("runtime_closure_binding") if isinstance(identity, Mapping) else None
    binding_consistent = (
        isinstance(identity, Mapping) and isinstance(handoff, Mapping)
        and handoff.get("session_sha256") == identity.get("session_sha256")
        and handoff.get("session_byte_size") == identity.get("session_byte_size")
        and handoff.get("runtime_closure_binding") == closure
    )
    valid = (
        report.get("schema_version") == 1 and report.get("report_id") == FINAL_REPORT_ID
        and report.get("operation_boundary") == FINAL_BOUNDARY
        and report.get("status") == "PREDEPLOY_READY_NOT_DEPLOYED" and report.get("ready_to_deploy") is True
        and isinstance(checks, Mapping) and bool(checks) and all(value is True for value in checks.values())
        and isinstance(blockers, list) and not blockers and isinstance(model, Mapping)
        and model.get("present") is True and _is_digest(model.get("sha256"))
        and isinstance(model.get("byte_size"), int) and model["byte_size"] > 0
        and isinstance(identity, Mapping) and _is_digest(identity.get("session_sha256"))
        and isinstance(identity.get("session_byte_size"), int) and identity["session_byte_size"] > 0
        and isinstance(closure, Mapping) and _is_digest(closure.get("runtime_closure_manifest_sha256"))
        and _is_digest(closure.get("runtime_closure_sha256"))
        and binding_consistent
    )
    return valid, model if isinstance(model, Mapping) else None, identity if isinstance(identity, Mapping) else None


def _direct_child(path: str, parent: str) -> bool:
    prefix = parent + "/"
    return path.startswith(prefix) and bool(path[len(prefix):]) and "/" not in path[len(prefix):]


def _validate_model_receipt(receipt: Any) -> tuple[bool, Mapping[str, Any] | None, int]:
    if not isinstance(receipt, Mapping) or receipt.get("schema_version") != 1:
        return False, None, 0
    payloads = receipt.get("payloads")
    if receipt.get("receipt_id") != MODEL_RECEIPT_ID or receipt.get("status") != "VERIFIED" or not isinstance(payloads, Mapping):
        return False, None, 0
    if set(payloads) != set(EXPECTED_PAYLOAD_PATHS):
        return False, None, 0
    total = 0
    for name, expected in EXPECTED_PAYLOAD_PATHS.items():
        row = payloads.get(name)
        if not isinstance(row, Mapping) or row.get("target_relative_path") != expected:
            return False, None, 0
        size = row.get("byte_size")
        if not isinstance(size, int) or size <= 0 or not _is_digest(row.get("sha256")):
            return False, None, 0
        total += size
    return True, payloads, total


def _validate_handoff(root: Path, manifest_path: Path) -> tuple[dict[str, Path], Mapping[str, Any] | None, list[str]]:
    blockers: list[str] = []
    if not _nonlink_ancestors(root, manifest_path) or not _regular_nonlink(manifest_path):
        return {}, None, ["board_handoff_manifest_missing_or_linked"]
    manifest = _json(manifest_path)
    entries = manifest.get("entries") if isinstance(manifest, Mapping) else None
    if not isinstance(manifest, Mapping) or manifest.get("schema_version") != 1 or not isinstance(entries, Mapping):
        return {}, None, ["board_handoff_manifest_invalid"]
    required = {"final_predeploy", "model_payload_receipt", "acceptance_session", "runtime_closure", "payloads"}
    if set(entries) != required or not isinstance(entries.get("payloads"), Mapping):
        return {}, None, ["board_handoff_manifest_roles_invalid"]
    verified: dict[str, Path] = {}
    for role in ("final_predeploy", "model_payload_receipt", "acceptance_session", "runtime_closure"):
        path, error = _verified_entry(root, entries[role])
        if error:
            _append(blockers, f"board_handoff_{role}_{error}")
        elif path is not None:
            verified[role] = path
    payloads = entries["payloads"]
    if set(payloads) != set(EXPECTED_PAYLOAD_PATHS):
        _append(blockers, "board_handoff_payload_roles_invalid")
    else:
        for role in EXPECTED_PAYLOAD_PATHS:
            path, error = _verified_entry(root, payloads[role])
            if error:
                _append(blockers, f"board_handoff_payload_{role}_{error}")
            elif path is not None:
                verified[f"payload:{role}"] = path
    return verified, entries, blockers


def validate(*, handoff_manifest: Path, board_root: Path, candidate: str, retained_old: str,
             active: str = DEFAULT_ACTIVE,
             stat_path: Callable[[Path], os.stat_result] = os.lstat,
             disk_usage: Callable[[Path], Any] = shutil.disk_usage,
             platform_machine: Callable[[], str] = platform.machine,
             safety_margin_bytes: int = SAFETY_MARGIN_BYTES) -> dict[str, Any]:
    """Return a fail-closed, non-mutating S100P staging decision."""
    blockers: list[str] = []
    root = board_root.resolve()
    try:
        manifest_path = (_inside(root, handoff_manifest.as_posix()) if handoff_manifest.is_absolute()
                         else _relative_entry(root, handoff_manifest.as_posix()))
        if manifest_path is None:
            raise ValueError("handoff manifest path must be normalized")
    except ValueError:
        manifest_path = root / "__invalid_handoff_manifest__"
        _append(blockers, "board_handoff_manifest_path_invalid")
    verified, entries, handoff_blockers = _validate_handoff(root, manifest_path)
    blockers.extend(handoff_blockers)

    report = _json(verified["final_predeploy"]) if "final_predeploy" in verified else None
    final_ready, embedded_model, final_handoff = _validate_final_report(report)
    if not final_ready:
        _append(blockers, "final_predeploy_not_ready_not_deployed")
    session_path, closure_path = verified.get("acceptance_session"), verified.get("runtime_closure")
    closure_binding: Mapping[str, Any] | None = None
    try:
        closure_binding = runtime_closure_binding(closure_path) if closure_path else None
    except (OSError, ValueError, json.JSONDecodeError):
        _append(blockers, "board_handoff_runtime_closure_invalid")
    session_ok = False
    if final_ready and session_path is not None and closure_binding is not None and isinstance(final_handoff, Mapping):
        session = _json(session_path)
        session_closure = session.get("runtime_closure_binding") if isinstance(session, Mapping) else None
        session_ok = (
            final_handoff.get("session_sha256") == _sha256(session_path)
            and final_handoff.get("session_byte_size") == session_path.stat().st_size
            and isinstance(session_closure, Mapping)
            and session_closure.get("manifest_sha256") == closure_binding.get("runtime_closure_manifest_sha256")
            and session_closure.get("closure_sha256") == closure_binding.get("runtime_closure_sha256")
            and final_handoff.get("runtime_closure_binding") == closure_binding
        )
    if not session_ok:
        _append(blockers, "board_handoff_session_or_runtime_closure_binding_mismatch")
    model_receipt_path = verified.get("model_payload_receipt")
    receipt_valid, receipt_payloads, payload_bytes = _validate_model_receipt(_json(model_receipt_path) if model_receipt_path else None)
    if not receipt_valid:
        _append(blockers, "board_local_model_payload_receipt_invalid")
        payload_bytes = 0
    matching_embedded_receipt = False
    if final_ready and model_receipt_path is not None and embedded_model is not None:
        matching_embedded_receipt = (embedded_model.get("sha256") == _sha256(model_receipt_path)
                                    and embedded_model.get("byte_size") == model_receipt_path.stat().st_size)
        if not matching_embedded_receipt:
            _append(blockers, "final_predeploy_model_payload_receipt_binding_mismatch")
    if receipt_valid and isinstance(entries, Mapping) and isinstance(receipt_payloads, Mapping):
        manifest_payloads = entries["payloads"]
        for role in EXPECTED_PAYLOAD_PATHS:
            handoff, receipt_row = manifest_payloads.get(role), receipt_payloads[role]
            if not isinstance(handoff, Mapping) or (handoff.get("sha256") != receipt_row.get("sha256")
                    or handoff.get("byte_size") != receipt_row.get("byte_size")):
                _append(blockers, f"board_handoff_payload_{role}_does_not_match_model_receipt")

    model, compatible = _read(_inside(root, "/proc/device-tree/model")), _read(_inside(root, "/proc/device-tree/compatible"))
    identity_ok = (EXPECTED_MODEL_TOKEN in model and EXPECTED_COMPATIBLE_TOKEN in compatible
                   and platform_machine().strip().lower() == EXPECTED_ARCHITECTURE)
    if not identity_ok:
        _append(blockers, "board_identity_not_exact_s100p_aarch64")
    bpu_ok = _character_nonlink(_inside(root, EXPECTED_BPU_DEVICE), stat_path)
    if not bpu_ok:
        _append(blockers, "bpu_core0_not_nonlink_character_device")
    setup_ok = any(_regular_nonlink(_inside(root, item)) for item in TROS_SETUPS)
    if not setup_ok:
        _append(blockers, "allowlisted_tros_setup_missing_or_linked")
    modules = _read(_inside(root, "/proc/modules"))
    runtime_ok = "bpu_cores " in modules and "bpu_framework " in modules
    if not runtime_ok:
        _append(blockers, "required_bpu_modules_not_observed")

    try:
        active_path = _inside(root, active)
        candidate_path = _inside(root, candidate)
        retained_old_path = _inside(root, retained_old)
        active_parent, candidate_parent, retained_parent = active_path.parent, candidate_path.parent, retained_old_path.parent
        role_paths_exact = (
            active == DEFAULT_ACTIVE
            and _direct_child(candidate, "/opt/tzcup/stages")
            and _direct_child(retained_old, "/opt/tzcup/rollback")
            and len({active, candidate, retained_old}) == 3
        )
        parents = (active_parent, candidate_parent, retained_parent)
        parents_ok = all(_nonlink_ancestors(root, path) and _directory_nonlink(path) for path in parents)
        active_ok = _nonlink_ancestors(root, active_path) and _directory_nonlink(active_path)
        candidate_absent = _nonlink_ancestors(root, candidate_parent) and not candidate_path.exists() and not candidate_path.is_symlink()
        retained_old_absent = _nonlink_ancestors(root, retained_parent) and not retained_old_path.exists() and not retained_old_path.is_symlink()
        same_fs = parents_ok and active_ok and len({stat_path(active_path).st_dev, *(stat_path(path).st_dev for path in parents)}) == 1
    except (OSError, ValueError):
        active_path = candidate_path = retained_old_path = candidate_parent = root
        active_ok = candidate_absent = retained_old_absent = parents_ok = role_paths_exact = same_fs = False
        _append(blockers, "active_candidate_or_retained_old_path_invalid")
    if not role_paths_exact:
        _append(blockers, "active_candidate_retained_old_roles_not_exact")
    if not active_ok:
        _append(blockers, "active_not_existing_nonlink_directory")
    if not candidate_absent:
        _append(blockers, "candidate_target_not_fresh_absent")
    if not retained_old_absent:
        _append(blockers, "retained_old_target_not_fresh_absent")
    if not parents_ok:
        _append(blockers, "active_candidate_retained_old_parents_not_nonlink_directories")
    if parents_ok and not same_fs:
        _append(blockers, "active_candidate_retained_old_parents_not_same_filesystem")
    margin_ok = type(safety_margin_bytes) is int and safety_margin_bytes > 0
    if not margin_ok:
        _append(blockers, "safety_margin_bytes_not_positive_integer")
    effective_margin = safety_margin_bytes if margin_ok else 0
    required_free = payload_bytes + effective_margin
    try:
        free = disk_usage(candidate_parent).free if parents_ok else 0
    except OSError:
        free = 0
    space_ok = role_paths_exact and parents_ok and active_ok and candidate_absent and retained_old_absent and same_fs and free >= required_free
    if not space_ok:
        _append(blockers, "same_filesystem_free_space_below_payload_plus_margin")

    checks = {
        "board_handoff_manifest_verified": not handoff_blockers,
        "final_predeploy_ready_not_deployed": final_ready,
        "handoff_session_and_runtime_closure_bound": session_ok,
        "model_payload_receipt_bound_to_final_predeploy": matching_embedded_receipt,
        "model_payload_receipt_schema_valid": receipt_valid,
        "s100p_aarch64": identity_ok, "bpu_core0_nonlink_character_device": bpu_ok,
        "tros_setup_regular_nonlink": setup_ok, "bpu_modules_observed": runtime_ok,
        "active_nonlink_directory": active_ok, "candidate_target_fresh_absent": candidate_absent,
        "retained_old_target_fresh_absent": retained_old_absent,
        "role_paths_exact_and_distinct": role_paths_exact,
        "role_parents_nonlink_directories": parents_ok,
        "role_parents_same_filesystem": bool(same_fs), "free_space_payload_plus_margin": space_ok,
        "safety_margin_positive_integer": margin_ok,
    }
    ready = not blockers and all(checks.values())
    return {"schema_version": 1, "report_id": "tzcup_s100p_board_deployment_preflight_v1",
            "operation_boundary": "read_only_no_copy_rename_delete_install_node_start_or_actuator_command",
            "status": "READY_FOR_CONTROLLED_STAGE" if ready else "BLOCKED", "ready_for_controlled_stage": ready,
            "board_interaction_performed": False, "payload_copy_performed": False, "node_started": False,
            "checks": checks, "blockers": blockers, "active": active, "candidate": candidate,
            "retained_old": retained_old,
            "payload_bytes": payload_bytes, "safety_margin_bytes": safety_margin_bytes if margin_ok else None,
            "required_free_bytes": required_free, "observed_free_bytes": free}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff-manifest", type=Path, required=True)
    parser.add_argument("--board-root", type=Path, default=Path("/"))
    parser.add_argument("--active", default=DEFAULT_ACTIVE)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--retained-old", required=True)
    parser.add_argument("--safety-margin-bytes", type=int, default=SAFETY_MARGIN_BYTES)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(handoff_manifest=args.handoff_manifest, board_root=args.board_root, active=args.active,
                      candidate=args.candidate, retained_old=args.retained_old,
                      safety_margin_bytes=args.safety_margin_bytes)
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        if args.output.exists():
            raise SystemExit("refusing to overwrite retained preflight")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if result["ready_for_controlled_stage"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
