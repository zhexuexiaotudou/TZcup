"""Strict CUDA ONNX Runtime sessions with preallocated device I/O binding."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time

import numpy as np

from sanitation_perception.model_registry import ProductModel, ProductModelRegistry


DTYPES = {
    "float32": np.float32,
    "float16": np.float16,
    "int64": np.int64,
    "int32": np.int32,
    "uint8": np.uint8,
}


def audit_profile_providers(profile_path: str | Path) -> dict:
    events = json.loads(Path(profile_path).read_text(encoding="utf-8"))
    node_events = [
        event
        for event in events
        if event.get("cat") == "Node" and event.get("args", {}).get("provider")
    ]
    providers = sorted({event["args"]["provider"] for event in node_events})
    non_cuda = [provider for provider in providers if provider != "CUDAExecutionProvider"]
    return {
        "node_event_count": len(node_events),
        "providers": providers,
        "non_cuda_providers": non_cuda,
        "all_nodes_cuda": bool(node_events) and not non_cuda,
    }


class CudaIOBindingSession:
    """One fixed-shape session; inputs and outputs remain allocated on CUDA."""

    def __init__(self, model: ProductModel, ort, *, device_id: int = 0):
        self.model = model
        self.ort = ort
        self.device_id = int(device_id)
        if "CUDAExecutionProvider" not in ort.get_available_providers():
            raise RuntimeError("CUDAExecutionProvider is unavailable; CPU fallback forbidden")
        options = ort.SessionOptions()
        options.enable_profiling = True
        options.profile_file_prefix = str(
            Path(tempfile.gettempdir()) / f"tzcup_ort_{model.role}"
        )
        if hasattr(ort, "GraphOptimizationLevel"):
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(
            str(model.artifact_path),
            sess_options=options,
            providers=[
                (
                    "CUDAExecutionProvider",
                    {"device_id": self.device_id, "do_copy_in_default_stream": "1"},
                )
            ],
        )
        self.session.disable_fallback()
        if not self.session.get_providers() or self.session.get_providers()[0] != "CUDAExecutionProvider":
            raise RuntimeError("CUDAExecutionProvider is not the active primary provider")
        self.binding = self.session.io_binding()
        self.input_values = self._allocate_and_bind("input")
        self.output_values = self._allocate_and_bind("output")
        self.run_count = 0
        self.last_latency_ms = None

    @staticmethod
    def _fixed_shape(shape) -> tuple[int, ...]:
        result = tuple(int(value) for value in shape)
        if not result or any(value <= 0 for value in result):
            raise RuntimeError(f"product ONNX tensors require fixed positive shapes: {shape}")
        return result

    def _allocate_and_bind(self, kind: str) -> dict:
        spec = self.model.manifest[kind]
        names, shapes, dtypes = spec["names"], spec["shapes"], spec["dtypes"]
        if not len(names) == len(shapes) == len(dtypes):
            raise RuntimeError(f"{self.model.role} {kind} tensor metadata length mismatch")
        values = {}
        for name, raw_shape, dtype_name in zip(names, shapes, dtypes):
            if dtype_name not in DTYPES:
                raise RuntimeError(f"unsupported product tensor dtype: {dtype_name}")
            value = self.ort.OrtValue.ortvalue_from_shape_and_type(
                self._fixed_shape(raw_shape),
                DTYPES[dtype_name],
                "cuda",
                self.device_id,
            )
            if kind == "input":
                self.binding.bind_ortvalue_input(name, value)
            else:
                self.binding.bind_ortvalue_output(name, value)
            values[name] = value
        return values

    def warm_up_and_audit(self, iterations: int = 2) -> dict:
        if iterations < 1:
            raise ValueError("warm-up iterations must be positive")
        zeros = {
            name: np.zeros(value.shape(), dtype=DTYPES[dtype])
            for name, value, dtype in zip(
                self.model.manifest["input"]["names"],
                self.input_values.values(),
                self.model.manifest["input"]["dtypes"],
            )
        }
        for _ in range(iterations):
            self.run(zeros)
        profile_path = self.session.end_profiling()
        audit = audit_profile_providers(profile_path)
        if not audit["all_nodes_cuda"]:
            raise RuntimeError(
                "ORT graph contains CPU/unassigned nodes; silent fallback forbidden: "
                f"{audit}"
            )
        return {**audit, "warm_up_iterations": iterations}

    def run(self, inputs: dict[str, np.ndarray]) -> dict:
        if set(inputs) != set(self.input_values):
            raise ValueError(
                f"input names mismatch: expected={sorted(self.input_values)} actual={sorted(inputs)}"
            )
        for name, array in inputs.items():
            expected = tuple(self.input_values[name].shape())
            value = np.asarray(array)
            if tuple(value.shape) != expected:
                raise ValueError(f"input {name} shape mismatch: {value.shape} != {expected}")
            self.input_values[name].update_inplace(value)
        started = time.perf_counter()
        self.session.run_with_iobinding(self.binding)
        if hasattr(self.binding, "synchronize_outputs"):
            self.binding.synchronize_outputs()
        self.last_latency_ms = (time.perf_counter() - started) * 1000.0
        self.run_count += 1
        return dict(self.output_values)


class ProductInferenceEngine:
    def __init__(self, registry: ProductModelRegistry, ort, *, device_id: int = 0):
        self.registry = registry
        self.sessions = {
            role: CudaIOBindingSession(model, ort, device_id=device_id)
            for role, model in registry.models.items()
        }
        self.provider_audits = {}

    def warm_up(self) -> dict:
        self.provider_audits = {
            role: session.warm_up_and_audit() for role, session in self.sessions.items()
        }
        return self.provider_audits
