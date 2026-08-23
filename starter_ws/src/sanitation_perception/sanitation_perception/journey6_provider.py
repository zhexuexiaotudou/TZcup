"""Provider-neutral tensor contracts and strict Journey 6 HBM execution."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Callable, Mapping, Protocol

import numpy as np

from sanitation_perception.journey6_contract import SUPPORTED_MARCHES


@dataclass(frozen=True)
class TensorContract:
    name: str
    shape: tuple[int, ...]
    dtype: str
    stride_bytes: int | None = None

    def validate(self) -> None:
        if not self.name or not self.shape or any(int(value) <= 0 for value in self.shape):
            raise ValueError("tensor name and static positive shape are required")
        if self.dtype not in {"float32", "float16", "int8", "uint8", "int32", "int64"}:
            raise ValueError(f"unsupported tensor dtype: {self.dtype}")
        if self.stride_bytes is not None and self.stride_bytes <= 0:
            raise ValueError("tensor stride must be positive")


@dataclass(frozen=True)
class Journey6HbmContract:
    model_id: str
    artifact: Path
    artifact_sha256: str
    march: str
    runtime_version: str
    input_format: str
    inputs: tuple[TensorContract, ...]
    outputs: tuple[TensorContract, ...]

    def validate(self) -> None:
        if self.march not in SUPPORTED_MARCHES:
            raise ValueError("HBM march is not a supported Journey 6 profile")
        if not self.runtime_version or self.runtime_version == "auto":
            raise ValueError("HBM contract requires an exact runtime version")
        if self.input_format not in {"nv12", "rgb"}:
            raise ValueError("unsupported Journey 6 runtime input format")
        if not self.artifact.is_file():
            raise FileNotFoundError(self.artifact)
        actual = hashlib.sha256(self.artifact.read_bytes()).hexdigest()
        if actual.lower() != self.artifact_sha256.lower():
            raise ValueError("HBM SHA-256 mismatch")
        if not self.inputs or not self.outputs:
            raise ValueError("HBM input and output contracts are required")
        for tensor in (*self.inputs, *self.outputs):
            tensor.validate()


class InferenceProvider(Protocol):
    provider_id: str

    def load(self) -> None: ...
    def warmup(self, iterations: int) -> None: ...
    def infer(self, inputs: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]: ...
    def health(self) -> dict: ...
    def close(self) -> None: ...


class Journey6HbmProvider:
    """Official-runtime injection point; it intentionally has no ONNX fallback."""

    provider_id = "journey6_hbm"

    def __init__(
        self,
        contract: Journey6HbmContract,
        *,
        installed_runtime_version: str,
        installed_march: str,
        loader: Callable[[Journey6HbmContract], object] | None,
        runner: Callable[[object, Mapping[str, np.ndarray]], Mapping[str, np.ndarray]] | None,
    ) -> None:
        self.contract = contract
        self.installed_runtime_version = installed_runtime_version
        self.installed_march = installed_march
        self.loader = loader
        self.runner = runner
        self.handle: object | None = None
        self.inference_count = 0
        self.last_error: str | None = None

    def load(self) -> None:
        self.contract.validate()
        if self.installed_march != self.contract.march:
            raise RuntimeError("installed BPU march does not match HBM")
        if self.installed_runtime_version != self.contract.runtime_version:
            raise RuntimeError("installed J6 runtime version does not match HBM")
        if self.loader is None or self.runner is None:
            raise RuntimeError("official Journey 6 HUCP/DNN runtime is unavailable")
        self.handle = self.loader(self.contract)
        if self.handle is None:
            raise RuntimeError("official Journey 6 runtime returned no model handle")

    @staticmethod
    def _validate_array(contract: TensorContract, value: np.ndarray) -> np.ndarray:
        array = np.asarray(value)
        if array.shape != contract.shape:
            raise ValueError(f"tensor {contract.name} shape mismatch")
        if array.dtype.name != contract.dtype:
            raise ValueError(f"tensor {contract.name} dtype mismatch")
        if not array.flags.c_contiguous:
            raise ValueError(f"tensor {contract.name} must be contiguous")
        row_stride = array.strides[-2] if array.ndim >= 2 else array.strides[0]
        if contract.stride_bytes is not None and row_stride != contract.stride_bytes:
            raise ValueError(f"tensor {contract.name} stride mismatch")
        return array

    def infer(self, inputs: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        if self.handle is None or self.runner is None:
            raise RuntimeError("Journey 6 HBM provider is not loaded")
        expected = {item.name: item for item in self.contract.inputs}
        if set(inputs) != set(expected):
            raise ValueError("Journey 6 input tensor names mismatch")
        checked = {name: self._validate_array(expected[name], value) for name, value in inputs.items()}
        try:
            raw_outputs = dict(self.runner(self.handle, checked))
            output_contracts = {item.name: item for item in self.contract.outputs}
            if set(raw_outputs) != set(output_contracts):
                raise RuntimeError("Journey 6 output tensor names mismatch")
            outputs = {
                name: self._validate_array(output_contracts[name], value)
                for name, value in raw_outputs.items()
            }
            self.inference_count += 1
            self.last_error = None
            return outputs
        except Exception as exc:
            self.last_error = str(exc)
            raise

    def warmup(self, iterations: int) -> None:
        if iterations < 1:
            raise ValueError("warmup iterations must be positive")
        zeros = {
            item.name: np.zeros(item.shape, dtype=np.dtype(item.dtype))
            for item in self.contract.inputs
        }
        for _ in range(iterations):
            self.infer(zeros)

    def health(self) -> dict:
        return {
            "provider_id": self.provider_id,
            "loaded": self.handle is not None,
            "march": self.installed_march,
            "runtime_version": self.installed_runtime_version,
            "inference_count": self.inference_count,
            "last_error": self.last_error,
            "fallback_used": False,
        }

    def close(self) -> None:
        self.handle = None


__all__ = [
    "InferenceProvider",
    "Journey6HbmContract",
    "Journey6HbmProvider",
    "TensorContract",
]
