import json
from pathlib import Path

import numpy as np
import pytest

from sanitation_perception.inference_engine import (
    CudaIOBindingSession,
    ProductInferenceEngine,
)
from sanitation_perception.model_registry import ProductModel


class FakeOrtValue:
    def __init__(self, shape, dtype, device):
        self._shape = tuple(shape); self.dtype = dtype; self.device = device; self.updated = 0
        self.value = np.zeros(self._shape, dtype=dtype)

    @classmethod
    def ortvalue_from_shape_and_type(cls, shape, dtype, device, device_id):
        return cls(shape, dtype, (device, device_id))

    def shape(self):
        return self._shape

    def update_inplace(self, value):
        assert tuple(value.shape) == self._shape
        self.updated += 1
        self.value[...] = value

    def numpy(self):
        return self.value.copy()


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


def test_product_engine_runs_discovery_classifier_and_both_area_models(tmp_path):
    def product_model(role, input_name, input_shape, output_shape, thresholds, nms=None):
        artifact = tmp_path / f"{role}.onnx"
        artifact.write_bytes(role.encode())
        manifest = {
            "input": {"names": [input_name], "shapes": [input_shape], "dtypes": ["float32"]},
            "output": {"names": ["outputs"], "shapes": [output_shape], "dtypes": ["float32"]},
            "thresholds": thresholds,
            "NMS": nms or {"iou_threshold": None},
        }
        return ProductModel(role, role, "1", "0" * 64, artifact, artifact, manifest)

    models = {
        "detector": product_model(
            "detector", "image_rgb", [1, 3, 480, 640], [1, 15, 120, 160],
            {"score": 0.8}, {"iou_threshold": 0.5},
        ),
        "classifier": product_model(
            "classifier",
            "crop_rgb",
            [16, 3, 192, 192],
            [16, 4],
            {"score": 0.75},
        ),
        "leaf_segmenter": product_model(
            "leaf_segmenter", "area_features", [1, 10, 384, 512],
            [1, 2, 384, 512], {"mask": 0.8},
        ),
        "puddle_segmenter": product_model(
            "puddle_segmenter", "area_features", [1, 10, 384, 512],
            [1, 2, 384, 512], {"mask": 0.8},
        ),
    }

    class Output:
        def __init__(self, value): self.value = value
        def numpy(self): return self.value

    class Session:
        def __init__(self, value):
            self.value = value
            self.last_latency_ms = 1.0
            self.run_calls = 0

        def run(self, _inputs):
            self.run_calls += 1
            return {"outputs": Output(self.value.copy())}

    detector = np.full((1, 15, 120, 160), -20.0, np.float32)
    detector[0, 0, 60, 80] = 10.0
    detector[0, 3:5, 60, 80] = 0.5
    detector[0, 9:11, 60, 80] = 10.0
    detector[0, 0, 20, 30] = 9.0
    detector[0, 3:5, 20, 30] = 0.5
    detector[0, 9:11, 20, 30] = 8.0
    area = np.full((1, 2, 384, 512), -20.0, np.float32)
    engine = ProductInferenceEngine.__new__(ProductInferenceEngine)
    engine.registry = type("Registry", (), {"models": models})()
    engine.sessions = {
        "detector": Session(detector),
        "classifier": Session(
            np.tile(
                np.array([[0.0, 5.0, 0.0, 0.0]], np.float32),
                (16, 1),
            )
        ),
        "leaf_segmenter": Session(area),
        "puddle_segmenter": Session(area),
    }
    engine.last_metrics = None
    yy, xx = np.mgrid[0:480, 0:640]
    rgb = np.stack((xx % 255, yy % 255, (xx + yy) % 255), axis=-1).astype(np.uint8)
    depth = np.full((480, 640), 2.0, np.float32)
    result = engine.run_frame(
        rgb,
        depth,
        {"width": 640, "height": 480, "fx": 343.0, "fy": 343.0,
         "cx": 320.0, "cy": 240.0},
    )
    assert len(result["candidates"]) == 2
    assert len(result["discrete"]) == 2
    assert engine.sessions["classifier"].run_calls == 1
    assert result["discrete"][0]["class_id"] == "plastic_bottle"
    assert result["areas"]["leaf"]["mask"].sum() == 0
    assert result["metrics"]["candidate_count"] == 2
    with pytest.raises(ValueError, match="RGB contrast"):
        engine.run_frame(np.zeros_like(rgb), depth, {
            "width": 640, "height": 480, "fx": 343.0, "fy": 343.0,
            "cx": 320.0, "cy": 240.0,
        })
