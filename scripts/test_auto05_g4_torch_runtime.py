"""Real G4 CPU forward/export/ONNX Runtime parity checks when deps are present."""

from __future__ import annotations

import importlib.util
from pathlib import Path

try:
    import cv2  # noqa: F401
    import onnx  # noqa: F401
    import onnxruntime  # noqa: F401
    import torch
except ImportError as exc:
    if __name__ == "__main__":
        raise SystemExit(f"G4 real parity test dependencies unavailable: {exc}") from exc
    import pytest
    pytest.skip(f"G4 real parity dependencies unavailable: {exc}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("auto05_screening", ROOT / "scripts" / "auto05_screening.py")
assert spec and spec.loader
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_g4_direct_detector_real_forward_and_onnx_parity(tmp_path: Path) -> None:
    model = module.G4DirectDetector(width=8, input_channels=6).eval()
    sample = torch.rand(1, 6, 32, 32)
    outputs = model(sample)
    assert len(outputs) == 4
    assert outputs[0].shape == (1, 3, 8, 8)
    report = module.export_and_compare(model, tuple(sample.shape), tmp_path / "detector.onnx")
    assert report["max_numeric_output_error"] <= 1e-4


def test_g4_area_heads_are_independent_and_onnx_parity(tmp_path: Path) -> None:
    model = module.G4IndependentAreaHeads(width=4, input_channels=7).eval()
    assert set(map(id, model.leaf.parameters())).isdisjoint(map(id, model.puddle.parameters()))
    sample = torch.rand(1, 7, 32, 32)
    assert model(sample).shape == (1, 2, 32, 32)
    report = module.export_and_compare(model, tuple(sample.shape), tmp_path / "area.onnx")
    assert report["max_numeric_output_error"] <= 1e-4


def run_runtime_smoke() -> None:
    """Dependency-required test entrypoint for the retained Stage5B image build."""
    import tempfile

    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        test_g4_direct_detector_real_forward_and_onnx_parity(directory)
        test_g4_area_heads_are_independent_and_onnx_parity(directory)


if __name__ == "__main__":
    run_runtime_smoke()
