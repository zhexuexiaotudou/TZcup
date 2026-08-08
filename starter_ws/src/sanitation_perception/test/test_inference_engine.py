import json
from pathlib import Path

import numpy as np
import pytest

from sanitation_perception.inference_engine import CudaIOBindingSession
from sanitation_perception.model_registry import ProductModel


class FakeOrtValue:
    def __init__(self, shape, dtype, device):
        self._shape = tuple(shape); self.dtype = dtype; self.device = device; self.updated = 0

    @classmethod
    def ortvalue_from_shape_and_type(cls, shape, dtype, device, device_id):
        return cls(shape, dtype, (device, device_id))

    def shape(self):
        return self._shape

    def update_inplace(self, value):
        assert tuple(value.shape) == self._shape
        self.updated += 1


class FakeBinding:
    def __init__(self):
        self.inputs = {}; self.outputs = {}; self.sync_count = 0

    def bind_ortvalue_input(self, name, value): self.inputs[name] = value
    def bind_ortvalue_output(self, name, value): self.outputs[name] = value
    def synchronize_outputs(self): self.sync_count += 1


class FakeOptions:
    pass


class FakeSession:
    provider = "CUDAExecutionProvider"

    def __init__(self, path, sess_options, providers):
        self.path = path; self.providers = providers; self.binding = FakeBinding(); self.runs = 0; self.disabled = False

    def disable_fallback(self): self.disabled = True
    def get_providers(self): return [self.provider]
    def io_binding(self): return self.binding
    def run_with_iobinding(self, binding): self.runs += 1

    def end_profiling(self):
        path = Path(self.path).with_suffix(".profile.json")
        path.write_text(json.dumps([{"cat": "Node", "args": {"provider": self.provider}}]), encoding="utf-8")
        return str(path)


class FakeOrt:
    SessionOptions = FakeOptions
    OrtValue = FakeOrtValue
    InferenceSession = FakeSession

    class GraphOptimizationLevel:
        ORT_ENABLE_ALL = "all"

    @staticmethod
    def get_available_providers(): return ["CUDAExecutionProvider", "CPUExecutionProvider"]


def model(tmp_path):
    artifact = tmp_path / "model.onnx"; artifact.write_bytes(b"model")
    manifest = {
        "input": {"names": ["images"], "shapes": [[1, 3, 8, 8]], "dtypes": ["float32"]},
        "output": {"names": ["scores"], "shapes": [[1, 4]], "dtypes": ["float32"]},
    }
    return ProductModel("detector", "model", "1.0.0", "0" * 64, artifact, artifact, manifest)


def test_cuda_session_preallocates_iobinding_and_never_calls_plain_run(tmp_path):
    session = CudaIOBindingSession(model(tmp_path), FakeOrt)
    assert session.session.disabled is True
    assert set(session.binding.inputs) == {"images"}
    assert set(session.binding.outputs) == {"scores"}
    outputs = session.run({"images": np.zeros((1, 3, 8, 8), np.float32)})
    assert set(outputs) == {"scores"}
    assert session.session.runs == 1
    assert session.input_values["images"].updated == 1
    audit = session.warm_up_and_audit()
    assert audit["all_nodes_cuda"] is True


def test_provider_profile_cpu_node_fails_closed(tmp_path):
    class CpuSession(FakeSession):
        provider = "CPUExecutionProvider"

        def get_providers(self):
            return ["CUDAExecutionProvider", "CPUExecutionProvider"]

    class CpuOrt(FakeOrt):
        InferenceSession = CpuSession

    session = CudaIOBindingSession(model(tmp_path), CpuOrt)
    with pytest.raises(RuntimeError, match="silent fallback forbidden"):
        session.warm_up_and_audit(1)


def test_missing_cuda_or_dynamic_shape_is_rejected(tmp_path):
    class CpuOnly(FakeOrt):
        @staticmethod
        def get_available_providers(): return ["CPUExecutionProvider"]

    with pytest.raises(RuntimeError, match="CPU fallback forbidden"):
        CudaIOBindingSession(model(tmp_path), CpuOnly)
    dynamic = model(tmp_path)
    dynamic.manifest["input"]["shapes"] = [[1, 3, -1, 8]]
    with pytest.raises(RuntimeError, match="fixed positive shapes"):
        CudaIOBindingSession(dynamic, FakeOrt)
