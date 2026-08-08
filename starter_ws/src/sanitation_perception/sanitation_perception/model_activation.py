"""Atomic, fail-closed switching between immutable perception model bundles."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import time
from typing import Callable

from sanitation_perception.model_registry import ProductModelRegistry


@dataclass(frozen=True)
class ActivationResult:
    release_id: str
    previous_release_id: str | None
    registry: ProductModelRegistry


class AtomicModelActivator:
    """Switch a small JSON pointer only after registry validation and warm-up."""

    def __init__(self, releases_root: str | Path, state_path: str | Path):
        self.releases_root = Path(releases_root).resolve()
        self.state_path = Path(state_path).resolve()

    def _release_root(self, release_id: str) -> Path:
        if not release_id or Path(release_id).name != release_id:
            raise ValueError("release_id must be one safe path component")
        root = (self.releases_root / release_id).resolve()
        if self.releases_root not in root.parents:
            raise ValueError("release path escapes releases_root")
        return root

    def current(self) -> dict | None:
        if not self.state_path.is_file():
            return None
        payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        if payload.get("schema_version") != 1 or not payload.get("release_id"):
            raise RuntimeError("active model pointer is corrupt")
        return payload

    def _write_pointer(self, payload: dict) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_name(f".{self.state_path.name}.{os.getpid()}.tmp")
        temporary.write_text(
            json.dumps(payload, sort_keys=True, indent=2) + "\n", encoding="utf-8"
        )
        os.replace(temporary, self.state_path)

    def activate(
        self,
        release_id: str,
        *,
        required_provider: str,
        warm_up: Callable[[ProductModelRegistry], None],
    ) -> ActivationResult:
        root = self._release_root(release_id)
        pipeline = root / "manifests" / "perception_pipeline_manifest.yaml"
        artifacts = root / "models"
        registry = ProductModelRegistry.load(
            pipeline,
            artifacts,
            required_provider=required_provider,
            required_claim="formal",
        )
        previous = self.current()
        # This callback must create and warm an inactive pipeline. Any exception
        # leaves the existing pointer untouched.
        warm_up(registry)
        self._write_pointer(
            {
                "schema_version": 1,
                "release_id": release_id,
                "pipeline_sha256": registry.pipeline_sha256,
                "activated_unix_s": time.time(),
                "previous_release_id": previous.get("release_id") if previous else None,
            }
        )
        return ActivationResult(
            release_id=release_id,
            previous_release_id=previous.get("release_id") if previous else None,
            registry=registry,
        )

    def rollback(self) -> str:
        current = self.current()
        if current is None or not current.get("previous_release_id"):
            raise RuntimeError("no rollback release is registered")
        previous_id = str(current["previous_release_id"])
        previous_root = self._release_root(previous_id)
        if not previous_root.is_dir():
            raise RuntimeError(f"rollback release is missing: {previous_root}")
        self._write_pointer(
            {
                "schema_version": 1,
                "release_id": previous_id,
                "pipeline_sha256": None,
                "activated_unix_s": time.time(),
                "previous_release_id": current["release_id"],
                "rollback": True,
            }
        )
        return previous_id
