from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


class BackendUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class BackendSelection:
    requested: str
    active: str
    synthetic_only: bool
    ground_truth_control_allowed: bool
    detail: str


def select_backend(
    requested: str,
    *,
    model_path: str | Path | None = None,
    test_mode: bool = False,
    allow_ground_truth: bool = False,
    j6_toolchain_available: bool = False,
    j6_runtime_available: bool = False,
) -> BackendSelection:
    if requested == "ground_truth":
        if not allow_ground_truth:
            raise BackendUnavailable("ground_truth backend is evaluation-only")
        return BackendSelection(requested, requested, True, False, "evaluation_annotations_only")
    if requested == "mock":
        if not test_mode:
            raise BackendUnavailable("mock backend is test-only")
        return BackendSelection(requested, requested, True, False, "unit_test_only")
    if requested == "onnxruntime":
        if model_path is None or not Path(model_path).is_file():
            raise BackendUnavailable("ONNX model artifact is missing")
        return BackendSelection(requested, requested, True, False, "x86_onnxruntime")
    if requested == "horizon_j6":
        if not j6_toolchain_available:
            raise BackendUnavailable("Horizon J6 toolchain unavailable; fail-closed")
        if not j6_runtime_available:
            raise BackendUnavailable("Horizon J6 runtime unavailable; fail-closed")
        return BackendSelection(requested, requested, False, False, "horizon_j6_runtime")
    raise BackendUnavailable(f"unknown perception backend: {requested}")
