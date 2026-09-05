from __future__ import annotations

import json
import hashlib
import os
import time
from pathlib import Path

import pytest
import yaml

import formal_acceptance_session as session_module
from formal_acceptance_session import (
    AcceptanceSessionError,
    _strict_json_equal,
    finalize,
    start,
)


def _snapshot(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "source",
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "urdf"
                    }
                },
            }
        ),
        encoding="utf-8",
    )


def _mark_current_fixture(path: Path, session_path: Path) -> None:
    """Make fixture evidence unambiguously newer than its session.

    PRoot-backed filesystems can coalesce a write immediately after ``start``
    to the same (or an earlier-looking) mtime.  This helper is intentionally
    test-only: production freshness checks keep reading filesystem mtimes.
    """
    session = json.loads(session_path.read_text(encoding="utf-8"))
    started_ns = session["started_epoch_ns"]
    assert isinstance(started_ns, int)
    mtime_ns = max(time.time_ns(), started_ns) + 1_000_000_000
    os.utime(path, ns=(mtime_ns, mtime_ns))
    assert path.stat().st_mtime_ns >= started_ns


def test_session_only_binds_fresh_passing_evidence(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(json.dumps({"status": "PASS"}), encoding="utf-8")
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    assert result["evidence"]["runtime"]["status"] == "PASS"


def test_snapshot_drift_is_rejected(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    snapshot.write_text("{}", encoding="utf-8")
    contract.write_text("evidence_gates: {}\n", encoding="utf-8")
    try:
        finalize(contract, snapshot, output, tmp_path)
    except AcceptanceSessionError:
        pass
    else:
        raise AssertionError("snapshot drift must fail closed")


def test_start_refuses_to_overwrite_retained_session(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    _snapshot(snapshot)
    start(snapshot, output)
    retained = output.read_bytes()
    with pytest.raises(AcceptanceSessionError, match="refusing to overwrite"):
        start(snapshot, output)
    assert output.read_bytes() == retained


def test_formal_start_binds_verified_runtime_closure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    closure = tmp_path / "runtime/final_runtime_closure_manifest.json"
    install = tmp_path / "runtime/install"
    closure.parent.mkdir(parents=True)
    closure.write_text("{}", encoding="utf-8")
    install.mkdir()
    _snapshot(snapshot)
    expected = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest": str(closure.resolve()),
        "manifest_sha256": "manifest",
        "closure_sha256": "closure",
        "symbolic_link_count": 0,
    }
    monkeypatch.setattr(
        session_module,
        "verify_recorded_manifest",
        lambda manifest, repository_root, install_root: dict(expected),
    )

    result = start(
        snapshot,
        output,
        runtime_closure_manifest=closure,
        runtime_install_root=install,
        repository_root=tmp_path,
    )

    assert result["runtime_closure_binding"] == {
        **expected,
        "runtime_install_root": str(install.resolve()),
    }


def test_start_rejects_partial_runtime_closure_arguments(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    _snapshot(snapshot)

    with pytest.raises(AcceptanceSessionError, match="provided together"):
        start(
            snapshot,
            tmp_path / "session.json",
            runtime_closure_manifest=tmp_path / "closure.json",
        )


def test_session_rejects_passing_engineering_report_for_another_urdf(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "inputs": {"expanded_urdf_sha256": "old"}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "engineering": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "snapshot_urdf_hash_field": "inputs.expanded_urdf_sha256",
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "does not match" in result["failures"]["engineering"]


def test_session_rejects_passing_report_for_another_source_inventory(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps(
            {"status": "PASS", "source_binding": {"source_inventory_sha256": "old"}}
        ),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "external_runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "snapshot_source_hash_field": (
                            "source_binding.source_inventory_sha256"
                        ),
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "source inventory hash" in result["failures"]["external_runtime"]


def test_session_rejects_replaced_visual_frame(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence_dir = tmp_path / "visual"
    evidence_dir.mkdir()
    frame = evidence_dir / "shot.png"
    frame.write_bytes(b"original-png")
    import hashlib
    manifest = evidence_dir / "manifest.json"
    manifest.write_text(
        json.dumps({
            "status": "PASS",
            "frames": {"shot": {
                "path": "shot.png",
                "png_sha256": hashlib.sha256(frame.read_bytes()).hexdigest(),
                "png_size_bytes": frame.stat().st_size,
            }},
        }),
        encoding="utf-8",
    )
    contract = tmp_path / "contract.yaml"
    contract.write_text(yaml.safe_dump({"evidence_gates": {"visual": {
        "path": "visual/manifest.json",
        "success_statuses": ["PASS"],
        "session_bound": True,
        "bound_file_mapping": "frames",
    }}}), encoding="utf-8")
    _snapshot(snapshot)
    start(snapshot, output)
    # Refresh the manifest after the session started, then substitute its PNG.
    manifest.touch()
    frame.write_bytes(b"swapped-png")
    _mark_current_fixture(manifest, output)
    _mark_current_fixture(frame, output)
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "hash or size mismatch" in result["failures"]["visual"]


def test_session_rejects_visual_frame_from_before_session(tmp_path: Path) -> None:
    import hashlib
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    visual = tmp_path / "visual"
    visual.mkdir()
    frame = visual / "shot.png"
    frame.write_bytes(b"old-png")
    _snapshot(snapshot)
    start(snapshot, output)
    manifest = visual / "manifest.json"
    manifest.write_text(json.dumps({"status": "PASS", "frames": {"shot": {
        "path": "shot.png",
        "png_sha256": hashlib.sha256(frame.read_bytes()).hexdigest(),
        "png_size_bytes": frame.stat().st_size,
    }}}), encoding="utf-8")
    _mark_current_fixture(manifest, output)
    contract = tmp_path / "contract.yaml"
    contract.write_text(yaml.safe_dump({"evidence_gates": {"visual": {
        "path": "visual/manifest.json", "success_statuses": ["PASS"],
        "session_bound": True, "bound_file_mapping": "frames",
    }}}), encoding="utf-8")
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "predates" in result["failures"]["visual"]


def test_session_resumes_only_the_exact_missing_s100_pending_state(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(json.dumps({"status": "PASS"}), encoding="utf-8")
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                    },
                    "s100_live_runtime": {
                        "path": "s100.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                    },
                }
            }
        ),
        encoding="utf-8",
    )
    pending = finalize(contract, snapshot, output, tmp_path)
    assert pending["failures"] == {"s100_live_runtime": "missing"}

    s100 = tmp_path / "s100.json"
    s100.write_text(json.dumps({"status": "PASS"}), encoding="utf-8")
    _mark_current_fixture(s100, output)
    resumed = finalize(contract, snapshot, output, tmp_path)
    assert resumed["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    assert resumed["resumed_epoch_ns"] >= pending["finished_epoch_ns"]
    assert set(resumed["evidence"]) == {"runtime", "s100_live_runtime"}


def test_session_refuses_pending_failures_other_than_missing_s100(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    session = json.loads(output.read_text(encoding="utf-8"))
    session.update(
        status="FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
        failures={"runtime": "missing"},
    )
    output.write_text(json.dumps(session), encoding="utf-8")
    contract.write_text("evidence_gates: {}\n", encoding="utf-8")
    with pytest.raises(AcceptanceSessionError, match="not resumable"):
        finalize(contract, snapshot, output, tmp_path)


def test_session_mapping_values_reject_bool_integer_coercion(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "items": {"one": {"passed": 1}}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "required_mapping_item_values": {
                            "items": {"passed": True}
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    result = finalize(contract, snapshot, output, tmp_path)
    assert result["failures"] == {"runtime": "mapping item value mismatch: items.one"}
    assert _strict_json_equal({"passed": True}, {"passed": True})
    assert not _strict_json_equal({"passed": 1}, {"passed": True})


def test_session_enforces_complete_session_bound_gate_contract(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    manifest_hash = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    start(snapshot, output)
    evidence.write_text(
        json.dumps(
            {
                "status": "PASS",
                "report_id": "formal_runtime_v1",
                "summary": {"passed": True, "details": {"count": 2}},
                "items": {
                    "one": {"passed": True},
                    "two": {"passed": True},
                },
                "snapshot_binding": {
                    "manifest_sha256": manifest_hash,
                    "expanded_urdf_sha256": "urdf",
                    "source_inventory_sha256": "source",
                },
            }
        ),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "report_id": "formal_runtime_v1",
                        "required_values": {
                            "summary.passed": True,
                            "summary.details": {"count": 2},
                        },
                        "required_mapping_keys": {"items": ["one", "two"]},
                        "required_mapping_item_values": {
                            "items": {"passed": True}
                        },
                        "snapshot_manifest_hash_field": (
                            "snapshot_binding.manifest_sha256"
                        ),
                        "snapshot_urdf_hash_field": (
                            "snapshot_binding.expanded_urdf_sha256"
                        ),
                        "snapshot_source_hash_field": (
                            "snapshot_binding.source_inventory_sha256"
                        ),
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    assert set(result["evidence"]) == {"runtime"}


def test_session_required_values_reject_bool_integer_coercion(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "summary": {"passed": 1}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "required_values": {"summary.passed": True},
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert result["failures"] == {"runtime": "required value mismatch: summary.passed"}
    assert result["evidence"] == {}


def test_session_rejects_missing_required_mapping_key(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "items": {"one": {}}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "required_mapping_keys": {"items": ["one", "two"]},
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert result["failures"] == {"runtime": "mapping keys do not match: items"}
    assert result["evidence"] == {}


def test_session_rejects_invalid_required_mapping_keys_contract(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "items": {"one": {}}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "required_mapping_keys": {"items": "one"},
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert result["failures"] == {
        "runtime": "invalid required_mapping_keys entries: items"
    }
    assert result["evidence"] == {}


def test_session_rejects_wrong_snapshot_manifest_hash(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "binding": {"manifest_sha256": "old"}}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "snapshot_manifest_hash_field": "binding.manifest_sha256",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert "snapshot manifest hash" in result["failures"]["runtime"]
    assert result["evidence"] == {}


def test_session_rejects_wrong_contract_report_id(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    output = tmp_path / "session.json"
    evidence = tmp_path / "evidence.json"
    contract = tmp_path / "contract.yaml"
    _snapshot(snapshot)
    start(snapshot, output)
    evidence.write_text(
        json.dumps({"status": "PASS", "report_id": "wrong"}),
        encoding="utf-8",
    )
    _mark_current_fixture(evidence, output)
    contract.write_text(
        yaml.safe_dump(
            {
                "evidence_gates": {
                    "runtime": {
                        "path": "evidence.json",
                        "success_statuses": ["PASS"],
                        "session_bound": True,
                        "report_id": "expected",
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    result = finalize(contract, snapshot, output, tmp_path)

    assert result["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    assert result["failures"] == {
        "runtime": "evidence report_id does not match the gate contract"
    }
    assert result["evidence"] == {}
