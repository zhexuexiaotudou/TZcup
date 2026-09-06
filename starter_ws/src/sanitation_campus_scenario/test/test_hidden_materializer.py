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
)


ROOT = Path(__file__).parents[4]
CONFIG = ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"


def _bound_inputs(tmp_path: Path) -> tuple[Path, Path]:
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
    return snapshot, session


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
    snapshot, session = _bound_inputs(tmp_path)
    receipt = tmp_path / "receipts" / "hidden.json"
    output = tmp_path / "episode"
    freeze = commit_hidden_configuration_freeze(
        receipt_path=tmp_path / "receipts" / "freeze.json",
        snapshot_path=snapshot, session_path=session, scenario_config=CONFIG,
        producer="test", frozen_configuration={"validation_complete": True},
    )

    def fail_after_consumption(*args, **kwargs):
        raise GenerationError("synthetic post-lock failure")

    monkeypatch.setattr(hidden_materializer, "generate_episode", fail_after_consumption)
    with pytest.raises(GenerationError, match="post-lock"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            receipt_path=receipt, output=output, map_index=0, mission_index=0,
            freeze_receipt_path=freeze,
        )
    payload = json.loads(receipt.read_text(encoding="utf-8"))
    assert payload["status"] == "HIDDEN_MATERIALIZATION_CONSUMED"
    assert payload["consumed_before_generation"] is True
    assert payload["retry_permitted"] is False
    assert payload["configuration_freeze_receipt_sha256"] == hashlib.sha256(freeze.read_bytes()).hexdigest()
    assert not output.exists()
    with pytest.raises(HiddenMaterializationError, match="already consumed"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            receipt_path=receipt, output=output, map_index=0, mission_index=0,
            freeze_receipt_path=freeze,
        )


def test_hidden_materializer_rejects_unbound_session_before_consuming(tmp_path: Path) -> None:
    snapshot, session = _bound_inputs(tmp_path)
    receipt = tmp_path / "hidden.json"
    session.write_text(json.dumps({"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"}), encoding="utf-8")
    with pytest.raises(HiddenMaterializationError, match="RUNNING"):
        materialize_hidden_episode(
            scenario_config=CONFIG, snapshot_path=snapshot, session_path=session,
            receipt_path=receipt, output=tmp_path / "episode", map_index=0, mission_index=0,
        )
    assert not receipt.exists()
