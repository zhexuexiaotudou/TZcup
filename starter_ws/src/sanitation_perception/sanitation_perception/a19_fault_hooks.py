"""Small, fail-closed A19 hooks for the real PC inference product."""

from __future__ import annotations

import hashlib
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Mapping


PRODUCT_FAULTS = frozenset({
    "cuda_provider_failure", "model_hash_mismatch", "corrupt_model",
    "sustained_slow_inference",
})


class A19ProductFaultError(RuntimeError):
    """An injected A19 product fault must stop inference safely."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_product_fault(fault: str, parameters: Mapping[str, Any]) -> None:
    if fault not in PRODUCT_FAULTS or not isinstance(parameters, Mapping):
        raise A19ProductFaultError("unsupported or malformed A19 product fault")
    if fault == "cuda_provider_failure":
        if parameters.get("provider") != "CUDAExecutionProvider" or float(parameters.get("duration_s", 0)) <= 0:
            raise A19ProductFaultError("cuda_provider_failure requires CUDAExecutionProvider and duration_s")
    elif fault == "model_hash_mismatch":
        if parameters.get("model") != "dosod" or parameters.get("mismatch_count") != 1:
            raise A19ProductFaultError("model_hash_mismatch requires dosod and mismatch_count=1")
    elif fault == "corrupt_model":
        if parameters.get("model") != "edgesam" or int(parameters.get("corrupt_bytes", 0)) <= 0:
            raise A19ProductFaultError("corrupt_model requires edgesam and positive corrupt_bytes")
    elif fault == "sustained_slow_inference":
        if float(parameters.get("latency_ms", 0)) <= 0 or float(parameters.get("duration_s", 0)) <= 0:
            raise A19ProductFaultError("sustained_slow_inference requires positive latency_ms and duration_s")


class ProductFaultHooks:
    """Drive real model validation/load paths while leaving original artifacts intact."""

    def __init__(
        self, model_paths: Mapping[str, Path], *, provider_probe: Callable[[str], Mapping[str, Any]],
        model_probe: Callable[[Path], Mapping[str, Any]],
    ) -> None:
        self._paths = dict(model_paths)
        if set(self._paths) != {"dosod", "edgesam"} or not all(path.is_file() for path in self._paths.values()):
            raise A19ProductFaultError("real DOSOD and EdgeSAM model files are required")
        self._hashes = {name: sha256_file(path) for name, path in self._paths.items()}
        self._provider_probe, self._model_probe = provider_probe, model_probe
        self._active: tuple[str, dict[str, Any]] | None = None
        self._trigger_readback: dict[str, Any] | None = None

    def begin(self, fault: str, parameters: Mapping[str, Any]) -> dict[str, Any]:
        validate_product_fault(fault, parameters)
        if self._active is not None:
            raise A19ProductFaultError("another A19 product fault is already active")
        details: dict[str, Any]
        if fault == "cuda_provider_failure":
            details = dict(self._provider_probe(str(parameters["provider"])))
            details.update(requested_provider=parameters["provider"])
            provider = parameters["provider"]
            available = details.get("available_providers")
            session = details.get("session_providers")
            if (
                not isinstance(available, list) or provider not in available
                or details.get("selected_provider") != provider
                or not isinstance(session, list) or provider not in session
            ):
                raise A19ProductFaultError(
                    "UNSUPPORTED cuda_provider_failure: CUDAExecutionProvider was not actually selected"
                )
        elif fault == "model_hash_mismatch":
            actual = sha256_file(self._paths["dosod"])
            expected = "0" * 64 if actual != "0" * 64 else "f" * 64
            if actual == expected:
                raise A19ProductFaultError("model hash mismatch injection did not mismatch")
            details = {
                "model": "dosod", "actual_sha256": actual,
                "expected_sha256": expected,
                "loader_probe": dict(self._model_probe(self._paths["dosod"])),
            }
        elif fault == "corrupt_model":
            source = self._paths["edgesam"]
            with tempfile.TemporaryDirectory(prefix="tzcup-a19-corrupt-") as directory:
                shadow = Path(directory) / source.name
                shutil.copyfile(source, shadow)
                with shadow.open("r+b") as stream:
                    stream.write(b"\0" * min(int(parameters["corrupt_bytes"]), shadow.stat().st_size))
                corrupted_sha = sha256_file(shadow)
                if corrupted_sha == self._hashes["edgesam"]:
                    raise A19ProductFaultError("corrupt_model did not change private shadow bytes")
                try:
                    probe = dict(self._model_probe(shadow))
                except Exception as exc:  # The actual ONNX loader rejection is the desired readback.
                    probe = {"loader_error": str(exc)}
                else:
                    raise A19ProductFaultError("corrupt_model shadow was accepted by the real loader")
            details = {"model": "edgesam", "original_sha256": self._hashes["edgesam"], "shadow_sha256": corrupted_sha, "shadow_only": True, **probe}
        else:
            details = {"requested_latency_ms": float(parameters["latency_ms"]), "duration_s": float(parameters["duration_s"])}
        self._active = (fault, {**dict(parameters), **details})
        self._trigger_readback = None
        return {"fault": fault, "active": True, **details}

    def before_inference(self) -> dict[str, Any] | None:
        if self._active is None:
            return None
        fault, details = self._active
        if fault != "sustained_slow_inference":
            self._trigger_readback = {
                "fault": fault, "active": True, "observed": True,
                "inference_path_triggered": True, **details,
            }
            raise A19ProductFaultError(f"A19 injected {fault} is active")
        started = time.monotonic()
        time.sleep(float(details["latency_ms"]) / 1000.0)
        self._trigger_readback = {
            "fault": fault, "active": True, "observed": True,
            "inference_path_triggered": True,
            "observed_delay_ms": (time.monotonic() - started) * 1000.0,
            "requested_latency_ms": details["latency_ms"],
        }
        return dict(self._trigger_readback)

    def trigger_readback(self) -> dict[str, Any] | None:
        return None if self._trigger_readback is None else dict(self._trigger_readback)

    def clear(self) -> dict[str, Any]:
        if self._active is None:
            raise A19ProductFaultError("no A19 product fault is active")
        fault, _ = self._active
        restored = {
            name: sha256_file(path) == self._hashes[name]
            for name, path in self._paths.items()
        }
        if not all(restored.values()):
            raise A19ProductFaultError("original model hash changed during A19 fault injection")
        self._active = None
        triggered = self._trigger_readback is not None
        self._trigger_readback = None
        return {"fault": fault, "cleared": True, "trigger_was_observed": triggered, "original_model_hashes_restored": restored}
