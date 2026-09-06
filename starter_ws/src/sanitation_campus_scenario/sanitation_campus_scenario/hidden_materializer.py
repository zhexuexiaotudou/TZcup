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


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
CANONICAL_SNAPSHOT = REPOSITORY_ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
CANONICAL_SESSION = REPOSITORY_ROOT / "artifacts/formal_final_acceptance_session.json"
CANONICAL_SCENARIO_CONFIG = (
    REPOSITORY_ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
)


class HiddenMaterializationError(GenerationError):
    """A hidden task could not be safely consumed exactly once."""


def require_canonical_formal_inputs(
    *, snapshot_path: Path, session_path: Path, scenario_config: Path,
) -> None:
    """Keep the public hidden entrypoint tied to the formal trust roots."""

    actual = (snapshot_path.resolve(), session_path.resolve(), scenario_config.resolve())
    expected = (CANONICAL_SNAPSHOT.resolve(), CANONICAL_SESSION.resolve(), CANONICAL_SCENARIO_CONFIG.resolve())
    if actual != expected:
        raise HiddenMaterializationError(
            "public hidden materialization requires canonical formal snapshot, session and scenario config"
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_symlink_ancestors(path: Path, label: str) -> None:
    """Reject a leaf *and every existing ancestor* that is a symlink."""

    candidate = path.absolute()
    while True:
        if candidate.is_symlink():
            raise HiddenMaterializationError(f"{label} must not traverse a symlink: {candidate}")
        parent = candidate.parent
        if parent == candidate:
            return
        candidate = parent


def _regular(path: Path, label: str) -> Path:
    _reject_symlink_ancestors(path, label)
    resolved = path.resolve()
    if path.is_symlink() or not path.is_file():
        raise HiddenMaterializationError(f"{label} must be a regular file: {path}")
    return resolved


def _run_root(path: Path) -> Path:
    _reject_symlink_ancestors(path, "formal run root")
    if not path.exists():
        try:
            path.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            raise HiddenMaterializationError(f"cannot create canonical formal run root: {path}: {exc}") from exc
    if not path.is_dir() or path.is_symlink():
        raise HiddenMaterializationError(f"formal run root must be a regular directory: {path}")
    return path.resolve()


def _within_run_root(path: Path, run_root: Path, label: str) -> Path:
    _reject_symlink_ancestors(path, label)
    resolved = path.resolve()
    try:
        resolved.relative_to(run_root)
    except ValueError as exc:
        raise HiddenMaterializationError(f"{label} must be under the canonical formal run root") from exc
    return resolved


def _ledger_path(
    *, run_root: Path, binding: dict[str, Any], producer: str,
    request: dict[str, Any], kind: str,
) -> Path:
    """Derive, rather than accept, the durable formal consumption path."""

    identity = json.dumps(
        {"binding": binding, "producer": producer, "request": request},
        sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    request_id = hashlib.sha256(identity).hexdigest()
    session_id = binding["acceptance_session_binding"]["session_manifest_sha256"]
    return run_root / "hidden-consumption-ledger" / session_id / f"{kind}-{request_id}.json"


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
    _reject_symlink_ancestors(path, "hidden consumption receipt")
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
    *, run_root: Path, snapshot_path: Path, session_path: Path,
    scenario_config: Path, producer: str, request: dict[str, Any], output: Path,
    freeze_producer: str,
) -> None:
    run_root = _run_root(run_root)
    output = _within_run_root(output, run_root, "hidden materialization output")
    if output.exists() or output.is_symlink():
        raise HiddenMaterializationError(f"hidden materialization output must be fresh: {output}")
    binding = _source_session_binding(
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
    )
    freeze_request = {"phase": "configuration_freeze"}
    freeze_receipt_path = _ledger_path(
        run_root=run_root, binding=binding, producer=freeze_producer,
        request=freeze_request, kind="freeze",
    )
    freeze_receipt_path = _regular(freeze_receipt_path, "configuration freeze receipt")
    freeze = _json_object(freeze_receipt_path, "configuration freeze receipt")
    if (
        freeze.get("receipt_id") != "tzcup_hidden_materialization_freeze_receipt_v1"
        or freeze.get("status") != "HIDDEN_CONFIGURATION_FROZEN"
        or freeze.get("hidden_materialization_allowed") is not True
        or freeze.get("producer") != freeze_producer
        or freeze.get("source_binding") != binding["source_binding"]
        or freeze.get("acceptance_session_binding") != binding["acceptance_session_binding"]
        or freeze.get("scenario_config_sha256") != binding["scenario_config_sha256"]
    ):
        raise HiddenMaterializationError(
            "configuration freeze receipt is not bound to the active source/session/configuration"
        )
    receipt_path = _ledger_path(
        run_root=run_root, binding=binding, producer=producer,
        request=request, kind="consumed",
    )
    _commit_receipt(receipt_path, {
        "schema_version": 1,
        "receipt_id": "tzcup_hidden_materialization_consumed_receipt_v1",
        "status": "HIDDEN_MATERIALIZATION_CONSUMED",
        "consumed_before_generation": True,
        "retry_permitted": False,
        "producer": producer,
        "request": request,
        "output_root": str(output.resolve()),
        "configuration_freeze_producer": freeze_producer,
        "configuration_freeze_receipt_sha256": _sha256(freeze_receipt_path),
        **binding,
    })


def commit_hidden_configuration_freeze(
    *, run_root: Path, snapshot_path: Path, session_path: Path,
    scenario_config: Path, producer: str, frozen_configuration: dict[str, Any],
) -> Path:
    """Durably freeze a runner's train/validation decision before hidden use."""

    binding = _source_session_binding(
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
    )
    run_root = _run_root(run_root)
    receipt_path = _ledger_path(
        run_root=run_root, binding=binding, producer=producer,
        request={"phase": "configuration_freeze"}, kind="freeze",
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
    run_root: Path, output: Path, map_index: int, mission_index: int,
    freeze_producer: str,
) -> Path:
    """Consume and generate one standard formal hidden episode exactly once."""

    _consume(
        run_root=run_root,
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
        producer="formal_hidden_episode",
        request={
            "profile": "formal", "split": "hidden", "map_index": map_index,
            "mission_index": mission_index,
        },
        output=output,
        freeze_producer=freeze_producer,
    )
    files = generate_episode(
        load_config(scenario_config), "formal", "hidden", map_index, mission_index,
        include_proxy=False,
    )
    return write_episode(output, files)


def materialize_hidden_stage_a_episode(
    *, scenario_config: Path, snapshot_path: Path, session_path: Path,
    run_root: Path, output: Path, task_index: int, freeze_producer: str,
) -> Path:
    """Consume and generate one fixed-map Stage-A hidden task exactly once."""

    _consume(
        run_root=run_root,
        snapshot_path=snapshot_path,
        session_path=session_path,
        scenario_config=scenario_config,
        producer="formal_rl_stage_a_hidden_task",
        request={"profile": "formal", "phase": "stage_a_hidden", "task_index": task_index},
        output=output,
        freeze_producer=freeze_producer,
    )
    files = generate_stage_a_episode(
        load_config(scenario_config), "formal", "hidden", task_index, include_proxy=False,
    )
    return write_episode(output, files)


def verify_hidden_consumption_records(
    *, run_root: Path, snapshot_path: Path, session_path: Path,
    scenario_config: Path, records: list[dict[str, Any]],
) -> list[dict[str, str]]:
    """Re-read the retained locks and their generated outputs before PASS.

    A final aggregator uses this rather than trusting a runner's remembered
    paths.  Missing receipts, a different session/configuration, a changed
    output location, or a removed manifest are all terminal failures.
    """

    run_root = _run_root(run_root)
    binding = _source_session_binding(
        snapshot_path=snapshot_path, session_path=session_path,
        scenario_config=scenario_config,
    )
    summaries: list[dict[str, str]] = []
    for record in records:
        producer = record.get("producer")
        request = record.get("request")
        output_value = record.get("output")
        if not isinstance(producer, str) or not isinstance(request, dict) or not isinstance(output_value, Path):
            raise HiddenMaterializationError("hidden aggregate record is malformed")
        output = _within_run_root(output_value, run_root, "hidden aggregate output")
        if not output.is_dir() or output.is_symlink():
            raise HiddenMaterializationError(f"hidden aggregate output is absent or non-regular: {output}")
        manifest = _regular(output / "public" / "episode_manifest.json", "hidden aggregate episode manifest")
        receipt_path = _ledger_path(
            run_root=run_root, binding=binding, producer=producer,
            request=request, kind="consumed",
        )
        receipt_path = _regular(receipt_path, "hidden consumption receipt")
        receipt = _json_object(receipt_path, "hidden consumption receipt")
        if (
            receipt.get("receipt_id") != "tzcup_hidden_materialization_consumed_receipt_v1"
            or receipt.get("status") != "HIDDEN_MATERIALIZATION_CONSUMED"
            or receipt.get("consumed_before_generation") is not True
            or receipt.get("retry_permitted") is not False
            or receipt.get("producer") != producer
            or receipt.get("request") != request
            or receipt.get("output_root") != str(output)
            or receipt.get("source_binding") != binding["source_binding"]
            or receipt.get("acceptance_session_binding") != binding["acceptance_session_binding"]
            or receipt.get("scenario_config_sha256") != binding["scenario_config_sha256"]
        ):
            raise HiddenMaterializationError("hidden consumption receipt binding or output summary drifted")
        freeze_producer = receipt.get("configuration_freeze_producer")
        if not isinstance(freeze_producer, str) or not freeze_producer:
            raise HiddenMaterializationError("hidden consumption receipt lacks its freeze producer")
        freeze_path = _ledger_path(
            run_root=run_root, binding=binding, producer=freeze_producer,
            request={"phase": "configuration_freeze"}, kind="freeze",
        )
        freeze_path = _regular(freeze_path, "retained configuration freeze receipt")
        freeze = _json_object(freeze_path, "retained configuration freeze receipt")
        if (
            receipt.get("configuration_freeze_receipt_sha256") != _sha256(freeze_path)
            or freeze.get("receipt_id") != "tzcup_hidden_materialization_freeze_receipt_v1"
            or freeze.get("status") != "HIDDEN_CONFIGURATION_FROZEN"
            or freeze.get("producer") != freeze_producer
            or freeze.get("source_binding") != binding["source_binding"]
            or freeze.get("acceptance_session_binding") != binding["acceptance_session_binding"]
            or freeze.get("scenario_config_sha256") != binding["scenario_config_sha256"]
        ):
            raise HiddenMaterializationError("hidden consumption freeze receipt drifted or was replaced")
        summaries.append({
            "receipt_sha256": _sha256(receipt_path),
            "episode_manifest_sha256": _sha256(manifest),
            "output_root": str(output),
        })
    return summaries
