"""Trust-anchored, non-formal S100P development artifact validation only."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
from types import MappingProxyType
from typing import Any, Mapping

from .s100p_product_adapter_core import (
    BOARD_ARTIFACT_SPECS,
    BoardArtifactContract,
    DOSOD_VOCABULARY_RELATIVE_PATH,
    FROZEN_CLASS_ORDER,
    S100PProductAdapterError,
)


DEVELOPMENT_MANIFEST_ID = "tzcup_s100p_development_artifact_manifest_v1"
DEVELOPMENT_STATUS = "NON_FORMAL_DEVELOPMENT"
DEVELOPMENT_CLASSIFICATION = "NON_FORMAL_ABI_DEVELOPMENT"
DEVELOPMENT_CLAIM_SCOPE = "CHAIN_LIVENESS_ONLY_NON_FORMAL_NON_SEMANTIC"
DEVELOPMENT_DOSOD_SOURCE_REVISION = "55b5525e0df3dff35e9ba75fa601601dff057b81"
DEVELOPMENT_DOSOD_HBM_SHA256 = "5a17357c8dd52769a1ef0829a12ed03f1253139315f7b272441013aa50979f9b"
DEVELOPMENT_DOSOD_HBM_BYTE_SIZE = 13055760
DEVELOPMENT_VOCABULARY_SHA256 = "c5b10ba0e26ee28cdbf5192775e7d2ddb3f5852e515f59a074b38b7ed69d7ffd"
DEVELOPMENT_EDGESAM_ENCODER_SHA256 = "82557c8f270f227035aee0246237d743e555b1816c2d7eb2e5694a4fa17fbfef"
DEVELOPMENT_EDGESAM_ENCODER_BYTE_SIZE = 12383448
DEVELOPMENT_EDGESAM_DECODER_SHA256 = "c4f32f9340a9c4f6365599d62588fcce572791840c3b905c939d1d297cd40388"
DEVELOPMENT_EDGESAM_DECODER_BYTE_SIZE = 13844912
DEVELOPMENT_ABI_GATE_SHA256 = "7cc1877c42f3de88fdae53c949460a2d30812141a4b086c945ae67b1be03ca4b"
DEVELOPMENT_DISAS_SHA256 = "85d6048c1fb68ce53f238258df1aa83f8656fe9362d2e9942706267c5874f2b0"
DEVELOPMENT_DOSOD_ROLE = "development_a2_custom4_dosod_s100p_detector"
DEVELOPMENT_ARTIFACT_PATHS = frozenset(BOARD_ARTIFACT_SPECS)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise S100PProductAdapterError(f"{label} must be a mapping")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise S100PProductAdapterError(f"{label} must be a non-empty string")
    return value.strip()


def _digest(value: Any, label: str) -> str:
    digest = _string(value, label).lower()
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise S100PProductAdapterError(f"{label} must be a SHA-256 digest")
    return digest


def _positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise S100PProductAdapterError(f"{label} must be a positive integer")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_nonlink(path: Path, label: str) -> Path:
    absolute = Path(os.path.abspath(path))
    try:
        resolved = absolute.resolve(strict=True)
        mode = resolved.stat().st_mode
    except OSError as exc:
        raise S100PProductAdapterError(f"{label} is missing: {path}") from exc
    if os.path.normcase(str(absolute)) != os.path.normcase(str(resolved)) or not stat.S_ISREG(mode):
        raise S100PProductAdapterError(f"{label} must be a regular path with no symlink components: {path}")
    return resolved


def _json(path: Path, label: str) -> Mapping[str, Any]:
    path = _regular_nonlink(path, label)
    try:
        return _mapping(json.loads(path.read_text(encoding="utf-8")), label)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise S100PProductAdapterError(f"{label} is unreadable") from exc


def _bound_file(row: Any, *, label: str, supplied_path: str | Path | None = None, anchor_sha: str | None = None, anchor_size: int | None = None) -> tuple[Path, str]:
    value = _mapping(row, label)
    path = Path(_string(value.get("path"), f"{label}.path"))
    if not path.is_absolute():
        raise S100PProductAdapterError(f"{label}.path must be absolute")
    path = _regular_nonlink(path, label)
    if supplied_path is not None and path != _regular_nonlink(Path(supplied_path), f"{label} supplied path"):
        raise S100PProductAdapterError(f"{label}.path does not match the supplied artifact path")
    declared_sha = _digest(value.get("sha256"), f"{label}.sha256")
    declared_size = _positive_int(value.get("byte_size"), f"{label}.byte_size")
    actual_sha, actual_size = _sha256(path), path.stat().st_size
    if actual_sha != declared_sha or actual_size != declared_size:
        raise S100PProductAdapterError(f"{label} hash or byte size mismatch")
    if (anchor_sha is not None and actual_sha != anchor_sha) or (anchor_size is not None and actual_size != anchor_size):
        raise S100PProductAdapterError(f"{label} does not match the development trust anchor")
    return path, actual_sha


def _strings(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise S100PProductAdapterError(f"{label} must be a JSON array")
    return tuple(_string(item, f"{label}[{index}]") for index, item in enumerate(value))


def _validate_abi_gate(receipt: Mapping[str, Any], *, hbm_sha: str, disas_sha: str) -> None:
    if receipt.get("status") != "NON_FORMAL_ABI_GATE_PASSED" or receipt.get("classification") != DEVELOPMENT_CLASSIFICATION or receipt.get("pass") is not True:
        raise S100PProductAdapterError("development ABI gate receipt classification is invalid")
    hbm = _mapping(receipt.get("hbm"), "development ABI gate receipt.hbm")
    if hbm.get("sha256") != hbm_sha or hbm.get("byte_size") != DEVELOPMENT_DOSOD_HBM_BYTE_SIZE:
        raise S100PProductAdapterError("development ABI gate receipt HBM provenance mismatches")
    tool = _mapping(receipt.get("tool"), "development ABI gate receipt.tool")
    if not isinstance(tool.get("argv"), list) or not tool["argv"] or tool.get("returncode") != 0:
        raise S100PProductAdapterError("development ABI gate receipt capture producer is invalid")
    _string(tool.get("path"), "development ABI gate receipt.tool.path")
    _digest(tool.get("sha256"), "development ABI gate receipt.tool.sha256")
    _string(tool.get("version"), "development ABI gate receipt.tool.version")
    if _mapping(receipt.get("disas"), "development ABI gate receipt.disas").get("sha256") != disas_sha:
        raise S100PProductAdapterError("development ABI gate receipt disas provenance mismatches")
    expected = (("scores", [1, 8400, 4], [67200, 8, 2]), ("boxes", [1, 8400, 4], [67200, 8, 2]))
    outputs = receipt.get("outputs")
    if not isinstance(outputs, list) or len(outputs) != len(expected):
        raise S100PProductAdapterError("development ABI gate receipt output order is invalid")
    for output, (name, dims, strides) in zip(outputs, expected, strict=True):
        if not isinstance(output, Mapping) or output.get("name") != name or output.get("dtype") != "TYPE_TAG_SI16" or output.get("dims") != dims or output.get("strides_bytes") != strides:
            raise S100PProductAdapterError("development ABI gate receipt output ABI is invalid")


def load_verified_development_artifact_contract(*, artifact_manifest_path: str | Path, artifact_paths: Mapping[str, str | Path]) -> BoardArtifactContract:
    """Load the independent A2 trust-anchored development manifest only."""

    if not isinstance(artifact_paths, Mapping) or frozenset(artifact_paths) != DEVELOPMENT_ARTIFACT_PATHS:
        raise S100PProductAdapterError("development artifact path set is invalid")
    payload = _json(Path(artifact_manifest_path), "development artifact manifest")
    if payload.get("schema_version") != 1 or payload.get("manifest_id") != DEVELOPMENT_MANIFEST_ID or payload.get("status") != DEVELOPMENT_STATUS or payload.get("classification") != DEVELOPMENT_CLASSIFICATION or payload.get("formal") is not False or payload.get("board_acceptance") is not False or payload.get("claim_scope") != DEVELOPMENT_CLAIM_SCOPE:
        raise S100PProductAdapterError("development artifact manifest classification is invalid")
    artifacts = _mapping(payload.get("artifacts"), "development artifact manifest.artifacts")
    if set(artifacts) != DEVELOPMENT_ARTIFACT_PATHS:
        raise S100PProductAdapterError("development artifact manifest artifact set is invalid")
    anchors = {
        "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm": (DEVELOPMENT_DOSOD_HBM_SHA256, DEVELOPMENT_DOSOD_HBM_BYTE_SIZE, DEVELOPMENT_DOSOD_ROLE, DEVELOPMENT_DOSOD_SOURCE_REVISION),
        DOSOD_VOCABULARY_RELATIVE_PATH: (DEVELOPMENT_VOCABULARY_SHA256, None, *BOARD_ARTIFACT_SPECS[DOSOD_VOCABULARY_RELATIVE_PATH]),
        "edgesam/edgesam_encoder_512.hbm": (DEVELOPMENT_EDGESAM_ENCODER_SHA256, DEVELOPMENT_EDGESAM_ENCODER_BYTE_SIZE, *BOARD_ARTIFACT_SPECS["edgesam/edgesam_encoder_512.hbm"]),
        "edgesam/edgesam_decoder_512.hbm": (DEVELOPMENT_EDGESAM_DECODER_SHA256, DEVELOPMENT_EDGESAM_DECODER_BYTE_SIZE, *BOARD_ARTIFACT_SPECS["edgesam/edgesam_decoder_512.hbm"]),
    }
    hashes: dict[str, str] = {}; vocabulary_path: Path | None = None
    for relative, (sha, size, role, revision) in anchors.items():
        row = _mapping(artifacts.get(relative), f"development artifact {relative}")
        if row.get("model_role") != role or row.get("source_revision") != revision:
            raise S100PProductAdapterError(f"development artifact provenance mismatch: {relative}")
        path, digest = _bound_file(row, label=f"development artifact {relative}", supplied_path=artifact_paths[relative], anchor_sha=sha, anchor_size=size)
        hashes[relative] = digest
        if relative == DOSOD_VOCABULARY_RELATIVE_PATH:
            vocabulary_path = path
    assert vocabulary_path is not None
    vocabulary_declaration = _mapping(payload.get("custom_vocabulary"), "development custom vocabulary")
    if vocabulary_declaration.get("artifact") != DOSOD_VOCABULARY_RELATIVE_PATH or vocabulary_declaration.get("kind") != "CUSTOM4" or _strings(vocabulary_declaration.get("semantic_class_ids"), "development custom vocabulary.semantic_class_ids") != FROZEN_CLASS_ORDER:
        raise S100PProductAdapterError("development custom vocabulary declaration is invalid")
    labels = _strings(vocabulary_declaration.get("emitted_labels"), "development custom vocabulary.emitted_labels")
    try:
        vocabulary = json.loads(vocabulary_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise S100PProductAdapterError("development custom vocabulary is unreadable") from exc
    actual_labels = tuple(_string(group[0], f"development custom vocabulary group {index}[0]") if isinstance(group, list) and group else "" for index, group in enumerate(vocabulary))
    if len(vocabulary) != 4 or len(set(labels)) != 4 or labels != actual_labels:
        raise S100PProductAdapterError("development custom vocabulary labels do not match the manifest")
    abi_gate_path, _ = _bound_file(payload.get("abi_gate"), label="development ABI gate", anchor_sha=DEVELOPMENT_ABI_GATE_SHA256)
    disas_path, _ = _bound_file(payload.get("disas"), label="development disas receipt", anchor_sha=DEVELOPMENT_DISAS_SHA256)
    _validate_abi_gate(_json(abi_gate_path, "development ABI gate"), hbm_sha=hashes["dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"], disas_sha=_sha256(disas_path))
    return BoardArtifactContract(model_hashes=MappingProxyType(hashes), emitted_label_to_class_id=MappingProxyType(dict(zip(actual_labels, FROZEN_CLASS_ORDER, strict=True))))
