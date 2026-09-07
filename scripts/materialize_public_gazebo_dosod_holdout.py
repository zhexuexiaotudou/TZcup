#!/usr/bin/env python3
"""Materialize the isolated public-Gazebo 100-frame holdout bundle.

The caller supplies the already-pinned official adapter as a callable.  This
module never substitutes a project byte layout for that adapter.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Callable

import numpy as np

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, sha256_file


class HoldoutBlocked(ValueError):
    pass


def _regular(root: Path, relative: str, label: str) -> Path:
    candidate = (root / relative).resolve()
    if not candidate.is_relative_to(root.resolve()) or candidate.is_symlink() or not candidate.is_file():
        raise HoldoutBlocked(f"{label}_missing_or_unsafe")
    return candidate


def materialize(*, capture_manifest: Path, capture_root: Path, output: Path,
                adapter: Callable[[Path, Path, Path], None], adapter_identity: dict[str, Any]) -> Path:
    normal_file(capture_manifest, "capture_manifest")
    manifest = load_object(capture_manifest)
    rows = manifest.get("holdout_records")
    if manifest.get("status") != "FROZEN" or not isinstance(rows, list) or len(rows) != 100:
        raise HoldoutBlocked("exactly_100_frozen_holdout_records_required")
    if not isinstance(adapter_identity, dict) or set(adapter_identity) != {"status", "path", "sha256", "version", "command"}:
        raise HoldoutBlocked("official_adapter_identity_required")
    if adapter_identity["status"] != "VERIFIED" or not all(isinstance(adapter_identity[key], str) and adapter_identity[key] for key in ("path", "sha256", "version")) or not isinstance(adapter_identity["command"], list) or not adapter_identity["command"] or not all(isinstance(item, str) and item for item in adapter_identity["command"]):
        raise HoldoutBlocked("official_adapter_identity_required")
    fresh_directory(output, "holdout_output")
    records: list[dict[str, Any]] = []
    sources: set[str] = set()
    try:
        for index, row in enumerate(rows):
            if not isinstance(row, dict) or row.get("source_role") != "evaluation_holdout_only":
                raise HoldoutBlocked("holdout_source_role_invalid")
            source = row.get("source_sha256")
            if not isinstance(source, str) or len(source) != 64 or source in sources:
                raise HoldoutBlocked("holdout_source_not_unique")
            sources.add(source)
            tensor_source = _regular(capture_root, str(row.get("relative_path", "")), "holdout_tensor")
            if tensor_source.stat().st_size != row.get("byte_size") or sha256_file(tensor_source) != row.get("sha256"):
                raise HoldoutBlocked("holdout_tensor_drift")
            raw = row.get("raw_sensor")
            if not isinstance(raw, dict) or set(raw) != {"relative_path", "sha256", "byte_size", "width", "height", "step", "encoding", "frame_id", "stamp_ns"}:
                raise HoldoutBlocked("holdout_raw_sensor_missing")
            raw_path = _regular(capture_root, str(raw["relative_path"]), "holdout_raw_sensor")
            if raw_path.stat().st_size != raw["byte_size"] or sha256_file(raw_path) != raw["sha256"] or raw["sha256"] != source:
                raise HoldoutBlocked("holdout_raw_sensor_drift")
            tensor = np.load(tensor_source, allow_pickle=False)
            if tensor.dtype != np.float32 or tensor.shape != (1, 3, 640, 640) or not np.isfinite(tensor).all() or tensor.min() < 0 or tensor.max() > 1:
                raise HoldoutBlocked("holdout_nchw_contract_invalid")
            sidecar = row.get("gt_sidecar")
            if not isinstance(sidecar, dict):
                raise HoldoutBlocked("holdout_sidecar_missing")
            sidecar_path = _regular(capture_root, str(sidecar.get("relative_path", "")), "holdout_sidecar")
            if sidecar_path.stat().st_size != sidecar.get("byte_size") or sha256_file(sidecar_path) != sidecar.get("sha256"):
                raise HoldoutBlocked("holdout_sidecar_drift")
            name = f"{index:03d}"
            onnx = output / f"{name}.npy"
            np.save(onnx, tensor, allow_pickle=False)
            y_binary, uv_binary = output / f"{name}.images_y.bin", output / f"{name}.images_uv.bin"
            adapter(onnx, y_binary, uv_binary)
            binaries = (("images_y", y_binary), ("images_uv", uv_binary))
            if any(binary.is_symlink() or not binary.is_file() or binary.stat().st_size <= 0 for _, binary in binaries):
                raise HoldoutBlocked("official_adapter_output_missing")
            records.append({"sample_id": name, "source_sha256": source, "source_role": "evaluation_holdout_only",
                            "onnx_input_npy": onnx.name, "onnx_input_sha256": sha256_file(onnx), "onnx_input_byte_size": onnx.stat().st_size,
                            "hbm_input_files": [{"role": role, "relative_path": binary.name, "sha256": sha256_file(binary), "byte_size": binary.stat().st_size} for role, binary in binaries],
                            "gt_sidecar": sidecar, "raw_sensor": raw, "generation_nonce": row.get("generation_nonce"), "episode_manifest_sha256": row.get("episode_manifest_sha256")})
        result = {"schema_version": 1, "status": "FROZEN", "classification": "EVALUATOR_ONLY_PUBLIC_GAZEBO_HOLDOUT", "formal_passed": False,
                  "hbm_input_adapter": adapter_identity, "capture_manifest_sha256": sha256_file(capture_manifest), "records": records}
        result["records_sha256"] = __import__("hashlib").sha256(json.dumps(records, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        target = output / "holdout_manifest.json"; atomic_json(target, result); return target
    except Exception:
        # Retain evidence directory and never write a frozen manifest after a
        # partial adapter result.
        raise
