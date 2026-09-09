"""Strict ONNX Runtime provider used for explicit PC reference lanes."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Mapping

import numpy as np

from sanitation_perception.journey6_provider import TensorContract


class StrictOnnxProvider:
    def __init__(
        self,
        *,
        artifact: str | Path,
        artifact_sha256: str,
        inputs: tuple[TensorContract, ...],
        outputs: tuple[TensorContract, ...],
        provider: str,
        ort_module=None,
    ) -> None:
        if provider not in {"onnx_cpu", "onnx_cuda"}:
            raise ValueError("unsupported ONNX provider")
        self.provider_id = provider
        self.artifact = Path(artifact)
        self.artifact_sha256 = artifact_sha256.lower()
        self.inputs = inputs
        self.outputs = outputs
        self.ort = ort_module
        self.session = None
        self.inference_count = 0

    def load(self) -> None:
        if not self.artifact.is_file():
            raise FileNotFoundError(self.artifact)
        actual = hashlib.sha256(self.artifact.read_bytes()).hexdigest()
        if actual != self.artifact_sha256:
            raise ValueError("ONNX artifact SHA-256 mismatch")
        for contract in (*self.inputs, *self.outputs):
            contract.validate()
        if self.ort is None:
            try:
                import onnxruntime as ort
            except ImportError as exc:
                raise RuntimeError("onnxruntime is not installed") from exc
            self.ort = ort
        requested = (
            "CUDAExecutionProvider"
            if self.provider_id == "onnx_cuda"
            else "CPUExecutionProvider"
        )
        if requested not in self.ort.get_available_providers():
            raise RuntimeError(f"requested ONNX provider unavailable: {requested}")
        self.session = self.ort.InferenceSession(str(self.artifact), providers=[requested])
        if self.provider_id == "onnx_cuda" and hasattr(self.session, "disable_fallback"):
            self.session.disable_fallback()
        active = self.session.get_providers()
        if not active or active[0] != requested:
            raise RuntimeError("requested ONNX provider is not active")

    @staticmethod
    def _check(contract: TensorContract, value: np.ndarray) -> np.ndarray:
        array = np.asarray(value)
        if array.shape != contract.shape or array.dtype.name != contract.dtype:
            raise ValueError(f"ONNX tensor contract mismatch: {contract.name}")
        return np.ascontiguousarray(array)

    def infer(self, inputs: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        if self.session is None:
            raise RuntimeError("ONNX provider is not loaded")
        contracts = {item.name: item for item in self.inputs}
        if set(inputs) != set(contracts):
            raise ValueError("ONNX input names mismatch")
        feed = {name: self._check(contracts[name], value) for name, value in inputs.items()}
        names = [item.name for item in self.outputs]
        raw = self.session.run(names, feed)
        result = {}
        for contract, value in zip(self.outputs, raw):
            result[contract.name] = self._check(contract, value)
        self.inference_count += 1
        return result

    def warmup(self, iterations: int) -> None:
        if iterations < 1:
            raise ValueError("warmup iterations must be positive")
        zeros = {
            item.name: np.zeros(item.shape, dtype=np.dtype(item.dtype))
            for item in self.inputs
        }
        for _ in range(iterations):
            self.infer(zeros)

    def health(self) -> dict:
        return {
            "provider_id": self.provider_id,
            "loaded": self.session is not None,
            "inference_count": self.inference_count,
            "fallback_used": False,
        }

    def close(self) -> None:
        self.session = None


__all__ = ["StrictOnnxProvider"]
