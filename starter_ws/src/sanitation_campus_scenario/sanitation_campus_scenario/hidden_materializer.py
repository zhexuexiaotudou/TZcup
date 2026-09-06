"""Session-bound, one-time materialization for formal hidden scenarios.

The public scenario CLI must never make a hidden task inspectable merely by
selecting ``--split hidden``.  Formal runners consume a hidden task through
this module instead.  Its receipt is committed *before* generation starts and
is never overwritten: an interrupted or failed attempt remains consumed.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

from .generator import GenerationError, generate_episode, generate_stage_a_episode, load_config
from .io import write_episode


class HiddenMaterializationError(GenerationError):
    """A hidden task could not be safely consumed exactly once."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular(path: Path, label: str) -> Path:
    resolved = path.resolve()
    if path.is_symlink() or not path.is_file():
        raise HiddenMaterializationError(f"{label} must be a regular file: {path}")
    return resolved


def _json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise HiddenMaterializationError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise HiddenMaterializationError(f"{label} must be a JSON object")
    return value


def _source_session_binding(
    *, snapshot_path: Path, session_path: Path, scenario_config: Path
) -> dict[str, Any]:
    snapshot_path = _regular(snapshot_path, "snapshot manifest")
    session_path = _regular(session_path, "acceptance session")
    scenario_config = _regular(scenario_config, "scenario config")
    snapshot = _json_object(snapshot_path, "snapshot manifest")
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or not source_hash or not isinstance(urdf, dict):
        raise HiddenMaterializationError("snapshot manifest lacks frozen source identity")
    urdf_hash = urdf.get("sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise HiddenMaterializationError("snapshot manifest lacks expanded URDF identity")
    source_binding = {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }
    session = _json_object(session_path, "acceptance session")
    started = session.get("started_epoch_ns")
    if (
        session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or not isinstance(started, int)
        or started <= 0
        or session.get("snapshot") != source_binding
    ):
        raise HiddenMaterializationError(
            "hidden materialization requires the active source-bound RUNNING acceptance session"
        )
    return {
        "source_binding": source_binding,
        "acceptance_session_binding": {
            "session_manifest_sha256": _sha256(session_path),
            "session_started_epoch_ns": started,
            "session_status_at_consumption": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "scenario_config_sha256": _sha256(scenario_config),
    }


def _commit_receipt(path: Path, payload: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise HiddenMaterializationError(
            f"hidden task was already consumed; retained receipt cannot be overwritten: {path}"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise HiddenMaterializationError(
            f"hidden task was already consumed; retained receipt cannot be overwritten: {path}"
        ) from exc
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        # A partially written receipt is still a consumed lock.  Do not remove
        # it: retrying after uncertainty would invalidate one-time semantics.
        raise


def _consume(
    *, receipt_path: Path, snapshot_path: Path, session_path: Path,
    scenario_config: Path, producer: str, request: dict[str, Any], output: Path,
    freeze_receipt_path: Path | None = None,
) -> None:
    if output.exists() or output.is_symlink():
        raise HiddenMaterializationError(f"hidden materialization output must be fresh: {output}")
    binding = _source_session_binding(
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
    )
    if freeze_receipt_path is None:
        raise HiddenMaterializationError("hidden materialization requires an immutable configuration freeze receipt")
    freeze_binding: dict[str, str] = {}
    if freeze_receipt_path is not None:
        freeze_receipt_path = _regular(freeze_receipt_path, "configuration freeze receipt")
        freeze = _json_object(freeze_receipt_path, "configuration freeze receipt")
        if (
            freeze.get("receipt_id") != "tzcup_hidden_materialization_freeze_receipt_v1"
            or freeze.get("status") != "HIDDEN_CONFIGURATION_FROZEN"
            or freeze.get("hidden_materialization_allowed") is not True
            or freeze.get("source_binding") != binding["source_binding"]
            or freeze.get("acceptance_session_binding") != binding["acceptance_session_binding"]
            or freeze.get("scenario_config_sha256") != binding["scenario_config_sha256"]
        ):
            raise HiddenMaterializationError(
                "configuration freeze receipt is not bound to the active source/session"
            )
        freeze_binding = {"configuration_freeze_receipt_sha256": _sha256(freeze_receipt_path)}
    _commit_receipt(receipt_path, {
        "schema_version": 1,
        "receipt_id": "tzcup_hidden_materialization_consumed_receipt_v1",
        "status": "HIDDEN_MATERIALIZATION_CONSUMED",
        "consumed_before_generation": True,
        "retry_permitted": False,
        "producer": producer,
        "request": request,
        "output_root": str(output.resolve()),
        **freeze_binding,
        **binding,
    })


def commit_hidden_configuration_freeze(
    *, receipt_path: Path, snapshot_path: Path, session_path: Path,
    scenario_config: Path, producer: str, frozen_configuration: dict[str, Any],
) -> Path:
    """Durably freeze a runner's train/validation decision before hidden use."""

    binding = _source_session_binding(
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
    )
    _commit_receipt(receipt_path, {
        "schema_version": 1,
        "receipt_id": "tzcup_hidden_materialization_freeze_receipt_v1",
        "status": "HIDDEN_CONFIGURATION_FROZEN",
        "hidden_materialization_allowed": True,
        "retry_permitted": False,
        "producer": producer,
        "frozen_configuration": frozen_configuration,
        **binding,
    })
    return receipt_path


def materialize_hidden_episode(
    *, scenario_config: Path, snapshot_path: Path, session_path: Path,
    receipt_path: Path, output: Path, map_index: int, mission_index: int,
    freeze_receipt_path: Path | None = None,
) -> Path:
    """Consume and generate one standard formal hidden episode exactly once."""

    _consume(
        receipt_path=receipt_path,
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
        producer="formal_hidden_episode",
        request={
            "profile": "formal", "split": "hidden", "map_index": map_index,
            "mission_index": mission_index,
        },
        output=output,
        freeze_receipt_path=freeze_receipt_path,
    )
    files = generate_episode(
        load_config(scenario_config), "formal", "hidden", map_index, mission_index,
        include_proxy=False,
    )
    return write_episode(output, files)


def materialize_hidden_stage_a_episode(
    *, scenario_config: Path, snapshot_path: Path, session_path: Path,
    receipt_path: Path, output: Path, task_index: int,
    freeze_receipt_path: Path | None = None,
) -> Path:
    """Consume and generate one fixed-map Stage-A hidden task exactly once."""

    _consume(
        receipt_path=receipt_path,
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
        producer="formal_rl_stage_a_hidden_task",
        request={"profile": "formal", "phase": "stage_a_hidden", "task_index": task_index},
        output=output,
        freeze_receipt_path=freeze_receipt_path,
    )
    files = generate_stage_a_episode(
        load_config(scenario_config), "formal", "hidden", task_index, include_proxy=False,
    )
    return write_episode(output, files)
