from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from sanitation_campus_scenario import cli, hidden_materializer
from sanitation_campus_scenario.generator import GenerationError
from sanitation_campus_scenario.hidden_materializer import (
    HiddenMaterializationError,
    commit_hidden_configuration_freeze,
    materialize_hidden_episode,
    require_canonical_formal_inputs,
    verify_hidden_consumption_records,
)


ROOT = Path(__file__).parents[4]
CONFIG = ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"


def _bound_inputs(tmp_path: Path) -> tuple[Path, Path, Path]:
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text(json.dumps({
        "source_inventory_sha256": "a" * 64,
        "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {"sha256": "b" * 64}},
    }), encoding="utf-8")
    identity = {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        "source_inventory_sha256": "a" * 64,
        "expanded_urdf_sha256": "b" * 64,
    }
    session = tmp_path / "session.json"
    session.write_text(json.dumps({
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": 1,
        "snapshot": identity,
    }), encoding="utf-8")
    return snapshot, session, tmp_path


def test_public_generate_cli_cannot_select_hidden_split() -> None:
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args([
            "generate", "--config", str(CONFIG), "--profile", "formal",
            "--split", "hidden", "--map-index", "0", "--mission-index", "0",
            "--output", "unused",
        ])


def test_hidden_failure_still_leaves_an_immutable_consumed_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    snapshot, session, run_root = _bound_inputs(tmp_path)
    output = tmp_path / "episode"
    freeze = commit_hidden_configuration_freeze(
        run_root=run_root,
        snapshot_path=snapshot, session_path=session, scenario_config=CONFIG,
        producer="test", frozen_configuration={"validation_complete": True},
    )

    def fail_after_consumption(*args, **kwargs):
        raise GenerationError("synthetic post-lock failure")

    monkeypatch.setattr(hidden_materializer, "generate_episode", fail_after_consumption)
    with pytest.raises(GenerationError, match="post-lock"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            run_root=run_root, output=output, map_index=0, mission_index=0,
            freeze_producer="test",
        )
    receipts = list((run_root / "hidden-consumption-ledger").rglob("consumed-*.json"))
    assert len(receipts) == 1
    payload = json.loads(receipts[0].read_text(encoding="utf-8"))
    assert payload["status"] == "HIDDEN_MATERIALIZATION_CONSUMED"
    assert payload["consumed_before_generation"] is True
    assert payload["retry_permitted"] is False
    assert payload["configuration_freeze_receipt_sha256"] == hashlib.sha256(freeze.read_bytes()).hexdigest()
    assert not output.exists()
    with pytest.raises(HiddenMaterializationError, match="already consumed"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            run_root=run_root, output=output, map_index=0, mission_index=0,
            freeze_producer="test",
        )


def test_hidden_materializer_rejects_unbound_session_before_consuming(tmp_path: Path) -> None:
    snapshot, session, run_root = _bound_inputs(tmp_path)
    session.write_text(json.dumps({"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"}), encoding="utf-8")
    with pytest.raises(HiddenMaterializationError, match="RUNNING"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            run_root=run_root, output=tmp_path / "episode", map_index=0, mission_index=0,
            freeze_producer="test",
        )
    assert not list((run_root / "hidden-consumption-ledger").rglob("consumed-*.json"))


def test_hidden_materializer_rejects_output_outside_canonical_run_root(tmp_path: Path) -> None:
    snapshot, session, run_root = _bound_inputs(tmp_path)
    commit_hidden_configuration_freeze(
        run_root=run_root, snapshot_path=snapshot, session_path=session,
        scenario_config=CONFIG, producer="test", frozen_configuration={"validation_complete": True},
    )
    with pytest.raises(HiddenMaterializationError, match="canonical formal run root"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            run_root=run_root, output=tmp_path.parent / "escape", map_index=0, mission_index=0,
            freeze_producer="test",
        )


def test_hidden_materializer_rejects_changed_freeze_configuration(tmp_path: Path) -> None:
    snapshot, session, run_root = _bound_inputs(tmp_path)
    freeze = commit_hidden_configuration_freeze(
        run_root=run_root, snapshot_path=snapshot, session_path=session,
        scenario_config=CONFIG, producer="test", frozen_configuration={"validation_complete": True},
    )
    payload = json.loads(freeze.read_text(encoding="utf-8"))
    payload["scenario_config_sha256"] = "0" * 64
    freeze.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(HiddenMaterializationError, match="source/session/configuration"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            run_root=run_root, output=run_root / "episode", map_index=0, mission_index=0,
            freeze_producer="test",
        )


def test_hidden_materializer_rejects_symlink_ancestor(tmp_path: Path) -> None:
    if not hasattr(Path, "symlink_to"):
        pytest.skip("symlink unavailable")
    real_root = tmp_path / "real"
    real_root.mkdir()
    linked_root = tmp_path / "linked"
    try:
        linked_root.symlink_to(real_root, target_is_directory=True)
    except OSError:
        pytest.skip("symlink privilege unavailable")
    snapshot, session, _ = _bound_inputs(real_root)
    with pytest.raises(HiddenMaterializationError, match="must not traverse a symlink"):
        commit_hidden_configuration_freeze(
            run_root=linked_root, snapshot_path=snapshot, session_path=session,
            scenario_config=CONFIG, producer="test", frozen_configuration={},
        )


def test_public_hidden_entrypoint_rejects_self_consistent_untrusted_inputs(tmp_path: Path) -> None:
    snapshot, session, _ = _bound_inputs(tmp_path)
    with pytest.raises(HiddenMaterializationError, match="canonical formal snapshot"):
        require_canonical_formal_inputs(
            snapshot_path=snapshot, session_path=session, scenario_config=CONFIG,
        )


def test_aggregate_verifier_fails_closed_when_retained_output_is_deleted(tmp_path: Path) -> None:
    snapshot, session, run_root = _bound_inputs(tmp_path)
    commit_hidden_configuration_freeze(
        run_root=run_root, snapshot_path=snapshot, session_path=session,
        scenario_config=CONFIG, producer="test", frozen_configuration={},
    )
    output = run_root / "episode"
    materialize_hidden_episode(
        scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
        run_root=run_root, output=output, map_index=0, mission_index=0,
        freeze_producer="test",
    )
    record = {
        "producer": "formal_hidden_episode",
        "request": {"profile": "formal", "split": "hidden", "map_index": 0, "mission_index": 0},
        "output": output,
    }
    assert len(verify_hidden_consumption_records(
        run_root=run_root, snapshot_path=snapshot, session_path=session,
        scenario_config=CONFIG, records=[record],
    )) == 1
    for child in output.iterdir():
        if child.is_dir():
            import shutil
            shutil.rmtree(child)
        else:
            child.unlink()
    output.rmdir()
    with pytest.raises(HiddenMaterializationError, match="output is absent"):
        verify_hidden_consumption_records(
            run_root=run_root, snapshot_path=snapshot, session_path=session,
            scenario_config=CONFIG, records=[record],
        )
