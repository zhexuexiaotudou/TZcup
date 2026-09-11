"""Build the one board-bridge payload receipt from already staged files only."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import stat
import tempfile
from typing import Any, Callable, Mapping

from formal_s100_live_acceptance_core import acceptance_session_binding, runtime_closure_binding
import validate_s100p_final_predeploy as final_predeploy
from validate_dosod_s100p_hbm_compile_contract import validate_contract_shape


PAYLOADS = {
    "dosod_hbm": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
    "dosod_vocabulary": "dosod/tzcup_offline_vocabulary.json",
    "edgesam_encoder_hbm": "edgesam/edgesam_encoder_512.hbm",
    "edgesam_decoder_hbm": "edgesam/edgesam_decoder_512.hbm",
}
EXPECTED_STAGE_PARENT = "/opt/tzcup/stages"
EXPECTED_MODEL_TOKEN = "rdk s100p"
EXPECTED_COMPATIBLE_TOKEN = "drobot,s100-rdk"
EXPECTED_ARCHITECTURE = "aarch64"
EXPECTED_BPU_DEVICE = "/dev/bpu_core0"
SESSION_STATUS = "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
OUTPUT_RELATIVE_PATH = "evidence/model_payload_receipt.json"
DEFAULT_HBM_CONTRACT = Path(__file__).resolve().parents[1] / "config" / "dosod_s100p_hbm_compile_contract.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _nonlink_file(path: Path) -> bool:
    absolute = Path(os.path.abspath(path))
    cursor = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        cursor = cursor / part
        if cursor.is_symlink():
            return False
    return absolute.is_file() and not absolute.is_symlink()


def _load(path: Path) -> Mapping[str, Any]:
    if not _nonlink_file(path):
        raise ValueError(f"receipt is not a regular non-link file: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"receipt is not a JSON mapping: {path}")
    return value


def _absolute_board_path(board_root: Path, value: str) -> Path:
    if not isinstance(value, str) or not value.startswith("/") or "\\" in value:
        raise ValueError("board path must be absolute")
    parts = value.split("/")[1:]
    if any(part in ("", ".", "..") for part in parts):
        raise ValueError("board path must be normalized")
    return board_root / value.lstrip("/")


def _direct_stage(candidate_stage: str) -> bool:
    prefix = EXPECTED_STAGE_PARENT + "/"
    return candidate_stage.startswith(prefix) and bool(candidate_stage[len(prefix):]) and "/" not in candidate_stage[len(prefix):]


def _nonlink_ancestors(root: Path, path: Path) -> bool:
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


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").lower()
    except OSError:
        return ""


def _device_tree_fact(path: Path, board_root: Path) -> tuple[str, str]:
    # /proc/device-tree is the stable Linux ABI and is normally a kernel
    # symlink. Permit that single link into sysfs, not arbitrary fact links.
    tree = path.parent
    if tree.is_symlink():
        if not _nonlink_ancestors(board_root, tree.parent):
            raise ValueError("board device-tree parent is a link")
        link = tree.readlink()
        target = Path(os.path.abspath(link if link.is_absolute() else tree.parent / link))
        sysfs = _absolute_board_path(board_root, "/sys/firmware/devicetree/base")
        if target != sysfs:
            raise ValueError("board device-tree link is outside kernel sysfs")
        path = target / path.name
    if not _nonlink_file(path):
        raise ValueError(f"board device-tree fact is not a regular non-link file: {path}")
    raw = path.read_bytes()
    return raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip(), hashlib.sha256(raw).hexdigest()


def _device_major_minor(value: int) -> tuple[int, int]:
    """Use the native device decoder on Linux; preserve portable fake-root tests."""
    major = getattr(os, "major", None)
    minor = getattr(os, "minor", None)
    if callable(major) and callable(minor):
        return int(major(value)), int(minor(value))
    return 0, 0


def _board_identity(
    *, board_root: Path, platform_machine: Callable[[], str], stat_path: Callable[[Path], os.stat_result]
) -> dict[str, Any]:
    model, model_sha256 = _device_tree_fact(_absolute_board_path(board_root, "/proc/device-tree/model"), board_root)
    compatible, compatible_sha256 = _device_tree_fact(_absolute_board_path(board_root, "/proc/device-tree/compatible"), board_root)
    device = _absolute_board_path(board_root, EXPECTED_BPU_DEVICE)
    try:
        device_stat = stat_path(device)
        device_mode = device_stat.st_mode
        device_major, device_minor = _device_major_minor(device_stat.st_rdev)
        bpu_ok = not device.is_symlink() and stat.S_ISCHR(device_mode)
    except OSError:
        bpu_ok = False
    modules = _read(_absolute_board_path(board_root, "/proc/modules"))
    identity_ok = (
        EXPECTED_MODEL_TOKEN in model.lower()
        and EXPECTED_COMPATIBLE_TOKEN in compatible.lower()
        and platform_machine().strip().lower() == EXPECTED_ARCHITECTURE
    )
    runtime_ok = "bpu_cores " in modules and "bpu_framework " in modules
    if not identity_ok or not bpu_ok or not runtime_ok:
        raise ValueError("producer requires a real S100P/aarch64 BPU runtime identity")
    return {
        "model": model, "model_sha256": model_sha256,
        "compatible": compatible, "compatible_sha256": compatible_sha256,
        "architecture": EXPECTED_ARCHITECTURE,
        "bpu_device": {
            "path": EXPECTED_BPU_DEVICE, "st_mode": device_mode,
            "st_rdev_major": device_major, "st_rdev_minor": device_minor,
            "st_ino": device_stat.st_ino,
            "is_character_device": True, "is_symlink": False,
        },
        "required_modules": ["bpu_cores", "bpu_framework"],
    }


def _session_binding(session: Mapping[str, Any], closure_path: Path, session_path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    """Reuse the acceptance core's frozen-closure/session identity semantics."""
    closure = runtime_closure_binding(closure_path)
    snapshot = session.get("snapshot")
    if not isinstance(snapshot, dict):
        raise ValueError("acceptance session lacks canonical snapshot identity")
    binding = acceptance_session_binding(session_path, snapshot, closure)
    return binding, closure


def _atomic_write_fresh(stage_root: Path, output: Path, receipt: Mapping[str, Any]) -> None:
    evidence_root = stage_root / "evidence"
    if output != stage_root / OUTPUT_RELATIVE_PATH:
        raise ValueError("output must be the fixed candidate-stage evidence receipt path")
    if output.exists() or output.is_symlink() or not _nonlink_ancestors(stage_root, evidence_root) or not evidence_root.is_dir():
        raise ValueError("receipt destination is not a fresh non-link candidate-stage evidence path")
    encoded = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(prefix=".model-payload-", dir=evidence_root)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)


def build_receipt(*, artifact_root: Path, candidate_stage: str, board_root: Path,
                  offline_compile_receipt: Path, acceptance_session: Path,
                  runtime_closure: Path, platform_machine: Callable[[], str] = platform.machine,
                  stat_path: Callable[[Path], os.stat_result] = os.lstat,
                  hbm_contract_path: Path = DEFAULT_HBM_CONTRACT,
                  compile_validator: Callable[..., bool] = final_predeploy._validate_offline_compile_receipt) -> dict[str, Any]:
    if not _direct_stage(candidate_stage):
        raise ValueError("candidate stage must be a direct /opt/tzcup/stages child")
    stage_root = _absolute_board_path(board_root, candidate_stage)
    if not _nonlink_ancestors(board_root, stage_root) or not stage_root.is_dir() or stage_root.is_symlink():
        raise ValueError("candidate stage is not an existing non-link directory")
    if not _nonlink_ancestors(stage_root, artifact_root) or artifact_root.is_symlink() or not artifact_root.is_dir():
        raise ValueError("artifact root must be a non-link directory inside candidate stage")
    board_identity = _board_identity(
        board_root=board_root, platform_machine=platform_machine, stat_path=stat_path
    )
    compile_receipt = _load(offline_compile_receipt)
    hbm_contract = _load(hbm_contract_path)
    contract_blockers: list[str] = []
    validate_contract_shape(hbm_contract, contract_blockers)
    if contract_blockers:
        raise ValueError("HBM compile contract is not frozen: " + ";".join(contract_blockers))
    session = _load(acceptance_session)
    _load(runtime_closure)
    session_binding, closure = _session_binding(session, runtime_closure, acceptance_session)
    if (compile_receipt.get("receipt_id"), compile_receipt.get("status"), compile_receipt.get("board_interaction_performed")) != (
        "tzcup_s100p_dosod_hbm_compile_receipt_v1", "COMPILED_NOT_BOARD_ACCEPTED", False,
    ):
        raise ValueError("offline compile receipt is not canonical non-board evidence")
    compile_blockers: list[str] = []
    if not compile_validator(compile_receipt, hbm_contract_path=hbm_contract_path, blockers=compile_blockers):
        raise ValueError("offline compile receipt fails canonical schema validation")
    payloads: dict[str, dict[str, Any]] = {}
    for role, relative in PAYLOADS.items():
        path = artifact_root / relative
        if not _nonlink_ancestors(stage_root, path) or path.is_symlink() or not path.is_file():
            raise ValueError(f"payload is not a regular non-link file: {relative}")
        payloads[role] = {"target_relative_path": relative, "sha256": _sha256(path), "byte_size": path.stat().st_size}
    dosod = payloads["dosod_hbm"]
    if dosod["sha256"] != compile_receipt.get("output_sha256") or dosod["byte_size"] != compile_receipt.get("output_byte_size"):
        raise ValueError("staged DOSOD HBM does not match the offline compile receipt")
    vocabulary_contract = hbm_contract.get("vocabulary")
    vocabulary = payloads["dosod_vocabulary"]
    if not isinstance(vocabulary_contract, Mapping) or any(
        vocabulary[field] != vocabulary_contract.get(contract_field)
        for field, contract_field in (
            ("target_relative_path", "relative_path"), ("sha256", "sha256"), ("byte_size", "byte_size")
        )
    ):
        raise ValueError("staged DOSOD vocabulary does not match the frozen compile contract")
    return {
        "schema_version": 1,
        "receipt_id": "tzcup_s100p_model_payload_receipt_v1",
        "status": "VERIFIED",
        "board_interaction_performed": True,
        "board_identity": board_identity,
        "candidate_stage": candidate_stage,
        "stage_root": candidate_stage,
        "offline_compile_receipt_sha256": _sha256(offline_compile_receipt),
        "payloads": payloads,
        "acceptance_session_binding": session_binding,
        "runtime_closure_binding": closure,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--candidate-stage", required=True)
    parser.add_argument("--board-root", type=Path, default=Path("/"))
    parser.add_argument("--offline-compile-receipt", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--hbm-contract", type=Path, default=DEFAULT_HBM_CONTRACT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    receipt = build_receipt(
        artifact_root=args.artifact_root, candidate_stage=args.candidate_stage, board_root=args.board_root,
        offline_compile_receipt=args.offline_compile_receipt,
        acceptance_session=args.acceptance_session,
        runtime_closure=args.runtime_closure,
        hbm_contract_path=args.hbm_contract,
    )
    stage_root = _absolute_board_path(args.board_root, args.candidate_stage)
    _atomic_write_fresh(stage_root, args.output, receipt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
