import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rtmdet_decoder_preserves_native_coordinates_and_thresholds():
    module = _load(ROOT / "starter_ws/src/sanitation_perception/sanitation_perception/rtmdet_product_runtime.py")

    class TensorRows:
        def __init__(self, value): self.value = value
        def tolist(self): return self.value
    class Instances:
        bboxes = TensorRows([[1.25, 2.5, 30.75, 40.0], [4, 5, 6, 7]])
        scores = TensorRows([0.8, 0.04])
        labels = TensorRows([1, 0])
        def to(self, _): return self
    class Result: pred_instances = Instances()

    rows = module.decode_rtmdet_result(Result(), observation_threshold=0.05, action_threshold=0.53)
    assert len(rows) == 1
    assert rows[0]["class_name"] == "metal_can"
    assert rows[0]["bbox_xyxy"] == [1.25, 2.5, 30.75, 40.0]
    assert rows[0]["actionable"] is True


def test_crv6_auditor_accepts_hash_new_candidate(tmp_path):
    module = _load(ROOT / "scripts/audit_odcv5_golden_parity.py")
    candidate = tmp_path / "candidate.pth"
    candidate.write_bytes(b"new-r1")
    expected = module.sha256(candidate)
    assert module.checkpoint_preflight(candidate, expected)["pass"] is True
    assert expected != module.EXPECTED_CHECKPOINT
