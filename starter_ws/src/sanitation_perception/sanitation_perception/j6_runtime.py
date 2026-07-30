"""Fail-closed Horizon J6/Nash runtime adapter.

The adapter never substitutes ONNX Runtime when a J6 HBM execution was
requested.  Reference inference is an explicitly separate backend.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class J6Artifact:
    model_name: str
    hbm_path: Path
    sha256: str
    input_name: str
    input_shape: tuple[int, ...]
    output_names: tuple[str, ...]
    march: str

    @classmethod
    def from_manifest(cls, path: str | Path) -> "J6Artifact":
        manifest_path = Path(path)
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        hbm_path = (manifest_path.parent / payload["hbm_path"]).resolve()
        artifact = cls(
            model_name=payload["model_name"],
            hbm_path=hbm_path,
            sha256=payload["sha256"],
            input_name=payload["input"]["name"],
            input_shape=tuple(payload["input"]["shape"]),
            output_names=tuple(payload["output_names"]),
            march=payload["march"],
        )
        artifact.validate()
        return artifact

    def validate(self) -> None:
        if self.march not in {
            "nash-a2p",
            "nash-a3",
            "nash-b",
            "nash-b-lite",
            "nash-b-plus",
            "nash-e",
            "nash-h",
            "nash-m",
            "nash-p",
        }:
            raise ValueError(f"unsupported J6/Nash march: {self.march}")
        if not self.hbm_path.is_file():
            raise FileNotFoundError(self.hbm_path)
        actual = hashlib.sha256(self.hbm_path.read_bytes()).hexdigest()
        if actual.lower() != self.sha256.lower():
            raise ValueError("HBM SHA-256 mismatch")
        if not self.input_shape or self.input_shape[0] != 1:
            raise ValueError("J6 deployment requires fixed batch=1")
        if any(not isinstance(value, int) or value <= 0 for value in self.input_shape):
            raise ValueError("J6 deployment requires fully static positive input shape")
        if not self.output_names:
            raise ValueError("J6 manifest has no outputs")


class J6RuntimeAdapter:
    def __init__(
        self,
        artifact: J6Artifact,
        *,
        board_available: bool,
        runner: Callable[[J6Artifact, np.ndarray], dict[str, np.ndarray]]
        | None = None,
    ) -> None:
        artifact.validate()
        self.artifact = artifact
        self.board_available = board_available
        self.runner = runner

    def infer(self, tensor: np.ndarray) -> dict[str, np.ndarray]:
        if not self.board_available:
            raise RuntimeError(
                "J6 board runtime unavailable; silent CPU/ONNX fallback is forbidden"
            )
        normalized = np.ascontiguousarray(tensor, dtype=np.float32)
        if normalized.shape != self.artifact.input_shape:
            raise ValueError(
                f"input shape {normalized.shape} does not match "
                f"{self.artifact.input_shape}"
            )
        if self.runner is None:
            raise RuntimeError("no J6 HBM runner configured")
        outputs = self.runner(self.artifact, normalized)
        if set(outputs) != set(self.artifact.output_names):
            raise RuntimeError("J6 output contract mismatch")
        return outputs


def run_hbrt4_tool(
    executable: str,
    artifact: J6Artifact,
    input_binary: str | Path,
    output_dir: str | Path,
) -> subprocess.CompletedProcess:
    """Invoke the official x86 Nash runtime tool without a fallback lane."""
    artifact.validate()
    return subprocess.run(
        [
            executable,
            "--model",
            str(artifact.hbm_path),
            "--input",
            str(Path(input_binary)),
            "--output-path",
            str(Path(output_dir)),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
