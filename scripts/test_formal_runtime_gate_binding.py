import json
import os
from pathlib import Path

import pytest

import formal_runtime_gate_binding as gate


IDENTITY = {
    "snapshot_manifest_sha256": "unused",
    "source_inventory_sha256": "a" * 64,
    "expanded_urdf_sha256": "b" * 64,
}


def _inputs(tmp_path: Path, monkeypatch):
    root = tmp_path / "repo"
    snapshot = root / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    snapshot.parent.mkdir(parents=True)
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": IDENTITY["source_inventory_sha256"],
                "output_inventory_sha256": "d" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": IDENTITY["expanded_urdf_sha256"]
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    identity = gate._snapshot_identity(snapshot)
    closure = root / ".work/final_frozen_runtime/final_runtime_closure_manifest.json"
    closure.parent.mkdir(parents=True)
    closure.write_text(json.dumps({"recorded_epoch_ns": 100}), encoding="utf-8")
    session = root / "artifacts/formal_final_acceptance_session.json"
    session.parent.mkdir(parents=True)
    install = closure.parent / "install"
    install.mkdir()
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": 300,
                "snapshot": identity,
                "runtime_closure_binding": {
                    "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
                    "manifest_sha256": "c" * 64,
                    "runtime_install_root": str(install.resolve()),
                },
            }
        ),
        encoding="utf-8",
    )
    for path in (snapshot, closure):
        os.utime(path, ns=(200, 200))
    monkeypatch.setattr(
        gate,
        "verify_recorded_manifest",
        lambda *_: {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "manifest_sha256": "c" * 64,
        },
    )
    monkeypatch.setattr(
        gate,
        "verify_snapshot",
        lambda repository_root, snapshot_path: json.loads(
            snapshot_path.read_text(encoding="utf-8")
        ),
    )
    monkeypatch.setattr(gate.time, "time_ns", lambda: 400)
    return root, snapshot, closure, session, install


def test_build_binding_requires_one_running_session_and_frozen_closure(tmp_path, monkeypatch):
    root, snapshot, closure, session, install = _inputs(tmp_path, monkeypatch)
    binding = gate.build_binding(
        repository_root=root,
        install_root=install,
        closure_manifest=closure,
        session_path=session,
        snapshot_path=snapshot,
    )
    assert binding["status"] == "FORMAL_RUNTIME_GATE_BOUND"
    assert binding["acceptance_session_binding"]["snapshot"] == gate._snapshot_identity(snapshot)
    assert binding["acceptance_session_binding"]["snapshot_output_inventory_sha256"] == "d" * 64
    assert binding["acceptance_session_binding"]["snapshot_current_source_verified"] is True
    assert (
        binding["runtime_closure_binding"]["status"]
        == "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
    )
    assert binding["runtime_closure_binding"]["runtime_install_root"] == str(
        install.resolve()
    )


def test_build_binding_rejects_session_started_before_closure(tmp_path, monkeypatch):
    root, snapshot, closure, session, install = _inputs(tmp_path, monkeypatch)
    payload = json.loads(session.read_text(encoding="utf-8"))
    payload["started_epoch_ns"] = 50
    session.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(gate.RuntimeGateError, match="predates"):
        gate.build_binding(
            repository_root=root,
            install_root=install,
            closure_manifest=closure,
            session_path=session,
            snapshot_path=snapshot,
        )


def test_build_binding_rejects_noncanonical_snapshot(tmp_path, monkeypatch):
    root, snapshot, closure, session, install = _inputs(tmp_path, monkeypatch)
    alternate = root / "alternate.json"
    alternate.write_bytes(snapshot.read_bytes())
    with pytest.raises(gate.RuntimeGateError, match="canonical"):
        gate.build_binding(
            repository_root=root,
            install_root=install,
            closure_manifest=closure,
            session_path=session,
            snapshot_path=alternate,
        )


def test_build_binding_rejects_session_bound_to_another_closure(
    tmp_path, monkeypatch
):
    root, snapshot, closure, session, install = _inputs(tmp_path, monkeypatch)
    payload = json.loads(session.read_text(encoding="utf-8"))
    payload["runtime_closure_binding"]["manifest_sha256"] = "d" * 64
    session.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(gate.RuntimeGateError, match="does not match"):
        gate.build_binding(
            repository_root=root,
            install_root=install,
            closure_manifest=closure,
            session_path=session,
            snapshot_path=snapshot,
        )


def test_build_binding_rejects_snapshot_that_no_longer_matches_current_sources(tmp_path, monkeypatch):
    root, snapshot, closure, session, install = _inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(
        gate,
        "verify_snapshot",
        lambda *_: (_ for _ in ()).throw(
            gate.SnapshotError("authoritative source inventory differs from committed manifest")
        ),
    )
    with pytest.raises(gate.RuntimeGateError, match="current vehicle snapshot verification failed"):
        gate.build_binding(
            repository_root=root,
            install_root=install,
            closure_manifest=closure,
            session_path=session,
            snapshot_path=snapshot,
        )


def test_load_binding_rejects_unverified_closure(tmp_path):
    path = tmp_path / "binding.json"
    path.write_text(
        json.dumps(
            {
                "status": "FORMAL_RUNTIME_GATE_BOUND",
                "acceptance_session_binding": {
                    "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
                },
                "runtime_closure_binding": {"status": "BLOCKED"},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(gate.RuntimeGateError, match="verified final closure"):
        gate.load_binding(path)


@pytest.mark.parametrize(
    "runtime_install_root,error",
    [
        (None, "no frozen runtime install root"),
        ("relative/install", "must be absolute"),
        ("noncanonical", "not canonical"),
    ],
)
def test_load_binding_requires_a_canonical_absolute_runtime_install_root(
    tmp_path, runtime_install_root, error
):
    if runtime_install_root == "noncanonical":
        runtime_install_root = str(tmp_path / "frozen" / ".." / "install")
    path = tmp_path / "binding.json"
    path.write_text(
        json.dumps(
            {
                "status": "FORMAL_RUNTIME_GATE_BOUND",
                "acceptance_session_binding": {
                    "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
                },
                "runtime_closure_binding": {
                    "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
                    "runtime_install_root": runtime_install_root,
                },
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(gate.RuntimeGateError, match=error):
        gate.load_binding(path)
