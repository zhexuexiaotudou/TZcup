"""Strict CUDA ONNX Runtime sessions with preallocated device I/O binding."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time

import numpy as np

from sanitation_perception.model_registry import ProductModel, ProductModelRegistry
from sanitation_perception.area_runtime import decode_area, preprocess_area
from sanitation_perception.classifier_runtime import (
    classifier_batch_input,
    classify_candidates,
)
from sanitation_perception.detector_runtime import (
    decode_discovery,
    preprocess_discovery,
)


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
        self.last_metrics = None

    def warm_up(self) -> dict:
        self.provider_audits = {
            role: session.warm_up_and_audit() for role, session in self.sessions.items()
        }
        return self.provider_audits

    @staticmethod
    def _single_numpy(outputs: dict) -> np.ndarray:
        if len(outputs) != 1:
            raise RuntimeError(
                f"frozen task must expose exactly one flat output, got {list(outputs)}"
            )
        value = next(iter(outputs.values()))
        if not hasattr(value, "numpy"):
            raise RuntimeError("ORT device output cannot be copied for postprocessing")
        return np.asarray(value.numpy())

    def run_frame(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        camera: dict,
        *,
        maximum_candidates: int = 16,
        minimum_valid_depth_ratio: float = 0.05,
        minimum_rgb_stddev: float = 2.0,
        maximum_dark_or_saturated_fraction: float = 0.98,
    ) -> dict:
        """Run all four frozen models and return prediction-derived outputs."""
        started = time.perf_counter()
        rgb = np.asarray(rgb)
        depth_m = np.asarray(depth_m, dtype=np.float32)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("product RGB must be HxWx3")
        if depth_m.shape != rgb.shape[:2]:
            raise ValueError("RGB and depth dimensions differ")
        grayscale = np.asarray(rgb, dtype=np.float32).mean(axis=2)
        invalid_exposure_fraction = float(
            ((grayscale <= 2.0) | (grayscale >= 253.0)).mean()
        )
        if float(grayscale.std()) < float(minimum_rgb_stddev):
            raise ValueError("RGB contrast is below product minimum")
        if invalid_exposure_fraction > float(
            maximum_dark_or_saturated_fraction
        ):
            raise ValueError("RGB frame is predominantly dark or saturated")
        valid_depth_ratio = float(
            (np.isfinite(depth_m) & (depth_m > 0.0)).mean()
        )
        if valid_depth_ratio < float(minimum_valid_depth_ratio):
            raise ValueError("valid depth ratio is below product minimum")

        preprocess_started = time.perf_counter()
        discovery_input = preprocess_discovery(rgb)
        preprocess_ms = (time.perf_counter() - preprocess_started) * 1000.0
        detector_model = self.registry.models["detector"]
        detector_threshold = float(detector_model.manifest["thresholds"]["score"])
        nms_threshold = float(detector_model.manifest["NMS"]["iou_threshold"])
        discovery_output = self._single_numpy(
            self.sessions["detector"].run(
                {detector_model.manifest["input"]["names"][0]: discovery_input}
            )
        )
        candidates = decode_discovery(
            discovery_output,
            score_threshold=detector_threshold,
            nms_iou_threshold=nms_threshold,
            maximum_candidates=int(maximum_candidates),
        )

        classifier_model = self.registry.models["classifier"]
        classifier_threshold = float(
            classifier_model.manifest["thresholds"]["score"]
        )
        classifier_started = time.perf_counter()
        classifier_batch_size = int(
            classifier_model.manifest["input"]["shapes"][0][0]
        )
        if classifier_batch_size != int(maximum_candidates):
            raise RuntimeError(
                "classifier fixed batch must equal maximum_candidates"
            )
        if candidates:
            crops = classifier_batch_input(
                rgb,
                candidates,
                fixed_batch_size=classifier_batch_size,
            )
            classifier_logits = self._single_numpy(
                self.sessions["classifier"].run(
                    {
                        classifier_model.manifest["input"]["names"][0]: crops
                    }
                )
            )
            classified = classify_candidates(
                classifier_logits,
                candidates,
                score_threshold=classifier_threshold,
            )
        else:
            classified = []
        classifier_ms = (time.perf_counter() - classifier_started) * 1000.0

        area_outputs = {}
        geometry = None
        area_preprocess_ms = 0.0
        for role, task in (
            ("leaf_segmenter", "leaf"),
            ("puddle_segmenter", "puddle"),
        ):
            area_model = self.registry.models[role]
            area_started = time.perf_counter()
            tensor, task_geometry = preprocess_area(
                rgb, depth_m, camera, task=task, geometry=geometry
            )
            area_preprocess_ms += (time.perf_counter() - area_started) * 1000.0
            geometry = task_geometry if geometry is None else geometry
            flat = self._single_numpy(
                self.sessions[role].run(
                    {area_model.manifest["input"]["names"][0]: tensor}
                )
            )
            area_outputs[task] = decode_area(
                flat,
                mask_threshold=float(
                    area_model.manifest["thresholds"]["mask"]
                ),
                native_size=(rgb.shape[1], rgb.shape[0]),
            )

        total_ms = (time.perf_counter() - started) * 1000.0
        self.last_metrics = {
            "preprocess_ms": preprocess_ms + area_preprocess_ms,
            "discovery_ms": self.sessions["detector"].last_latency_ms,
            "classifier_batch_ms": classifier_ms,
            "leaf_ms": self.sessions["leaf_segmenter"].last_latency_ms,
            "puddle_ms": self.sessions["puddle_segmenter"].last_latency_ms,
            "inference_pipeline_ms": total_ms,
            "candidate_count": len(candidates),
            "accepted_discrete_count": sum(item["accepted"] for item in classified),
            "rejected_candidate_count": sum(not item["accepted"] for item in classified),
            "valid_depth_ratio": valid_depth_ratio,
            "rgb_stddev": float(grayscale.std()),
            "dark_or_saturated_fraction": invalid_exposure_fraction,
        }
        return {
            "candidates": candidates,
            "discrete": [item for item in classified if item["accepted"]],
            "rejected": [item for item in classified if not item["accepted"]],
            "areas": area_outputs,
            "geometry": {
                "valid_depth_ratio": geometry.get("valid_depth_ratio"),
                "ground_plane": geometry.get("ground_plane"),
            },
            "metrics": dict(self.last_metrics),
        }
