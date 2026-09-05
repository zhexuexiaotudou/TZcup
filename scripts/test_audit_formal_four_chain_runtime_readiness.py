from __future__ import annotations

import importlib.util
import json
import os
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "scripts/audit_formal_four_chain_runtime_readiness.py"
SPEC = importlib.util.spec_from_file_location("four_chain_readiness", PATH)
assert SPEC and SPEC.loader
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def _snapshot(path: Path) -> dict[str, str]:
    path.write_text(json.dumps({"source_inventory_sha256": "source", "output_inventory_sha256": "outputs", "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {"sha256": "urdf"}}}), encoding="utf-8")
    return AUDIT.snapshot_identity(path)


def _mark_current_fixture(started_ns: int, *paths: Path) -> None:
    """Set deterministic post-session mtimes for PRoot-only test fixtures."""
    mtime_ns = max(time.time_ns(), started_ns) + 1_000_000_000
    for path in paths:
        os.utime(path, ns=(mtime_ns, mtime_ns))
        assert path.stat().st_mtime_ns >= started_ns


def test_scheduled_runner_parser_ignores_comments_and_unrelated_strings(
    tmp_path: Path,
) -> None:
    source = tmp_path / "orchestrator.py"
    source.write_text(
        "# run_formal_vehicle_mobility_runtime.sh\n"
        "DECOY = 'run_formal_water_recovery_runtime.sh'\n"
        "STEP_SPECS = (\n"
        "    StepSpec('chassis', 'gazebo', 'motion', "
        "'run_formal_vehicle_mobility_runtime.sh'),\n"
        ")\n",
        encoding="utf-8",
    )

    runners, error = AUDIT._scheduled_runner_names(source)

    assert error is None
    assert runners == {"run_formal_vehicle_mobility_runtime.sh"}


def test_scheduled_runner_parser_fails_closed_without_step_specs(tmp_path: Path) -> None:
    source = tmp_path / "orchestrator.py"
    source.write_text(
        "RUNNER = 'run_formal_vehicle_mobility_runtime.sh'\n", encoding="utf-8"
    )

    runners, error = AUDIT._scheduled_runner_names(source)

    assert runners == set()
    assert error == "STEP_SPECS assignment is missing"


def test_scheduled_runner_parser_rejects_dead_conditional_entries(
    tmp_path: Path,
) -> None:
    source = tmp_path / "orchestrator.py"
    source.write_text(
        "STEP_SPECS = (StepSpec('chassis', 'gazebo', 'motion', "
        "'run_formal_vehicle_mobility_runtime.sh'),) if False else ()\n",
        encoding="utf-8",
    )

    runners, error = AUDIT._scheduled_runner_names(source)

    assert runners == set()
    assert error == "STEP_SPECS must be a direct tuple or list literal"


def _runner_source(*, guard: str = "", order: str = "snapshot_first") -> str:
    snapshot = (
        'python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" \\\n'
        '  --check --output "${snapshot}"\n'
    )
    binding = (
        'python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \\\n'
        '  --repository-root "${repo_root}" --install-root "${runtime_ws}" \\\n'
        '  --closure-manifest "${closure_manifest}" --session "${session}" \\\n'
        '  --snapshot "${snapshot}" --output "${runtime_binding}"\n'
    )
    commands = snapshot + binding if order == "snapshot_first" else binding + snapshot
    return "set -eo pipefail\n" + (guard + commands + "fi\n" if guard else commands)


def test_shell_wiring_requires_direct_ordered_python_commands() -> None:
    assert AUDIT._runner_wiring_errors(_runner_source(), "unconditional") == []
    errors = AUDIT._runner_wiring_errors(
        _runner_source(order="binding_first"), "unconditional"
    )
    assert "runner binds runtime before the snapshot --check" in errors


def test_shell_wiring_rejects_echo_and_dead_conditional_token_lookalikes() -> None:
    echoed = (
        "set -eo pipefail\n"
        'echo "generate_formal_vehicle_snapshot.py formal_runtime_gate_binding.py '
        'runtime_binding --check --session ${session} --snapshot ${snapshot}"\n'
    )
    assert "runner lacks a direct snapshot --check command" in AUDIT._runner_wiring_errors(
        echoed, "unconditional"
    )
    dead = "set -eo pipefail\nif false; then\n" + _runner_source().split(
        "\n", 1
    )[1] + "fi\n"
    assert "runner lacks a direct runtime-binding command" in AUDIT._runner_wiring_errors(
        dead, "unconditional"
    )
    hidden_set_e = (
        "not_called() {\n"
        "  set -eo pipefail\n"
        "}\n"
        + _runner_source().replace("set -eo pipefail\n", "", 1)
    )
    assert "runner lacks fail-fast set -e before binding commands" in AUDIT._runner_wiring_errors(
        hidden_set_e, "unconditional"
    )


def test_shell_wiring_allows_only_canonical_water_all_guard() -> None:
    canonical = _runner_source(guard='if [[ "${scenario}" == "all" ]]; then\n')
    assert AUDIT._runner_wiring_errors(canonical, "canonical_all") == []
    noncanonical = _runner_source(guard='if [[ "${scenario}" == "diagnostic" ]]; then\n')
    assert "runner lacks a direct snapshot --check command" in AUDIT._runner_wiring_errors(
        noncanonical, "canonical_all"
    )


def _live_validator_source(main_body: str) -> str:
    return (
        "def _bound_runtime_evidence():\n"
        "    binding = load_binding('runtime.json')\n"
        "    return binding\n\n"
        "def main():\n"
        + main_body
    )


def test_live_validator_wiring_requires_main_preflight_before_runtime() -> None:
    valid = _live_validator_source(
        "    evidence = _bound_runtime_evidence()\n    rclpy.init()\n"
    )
    assert AUDIT._validator_wiring_errors(valid, Path("validator.py"), "live") == []
    dead = _live_validator_source(
        "    if False:\n        _bound_runtime_evidence()\n    rclpy.init()\n"
    )
    assert "validator main does not directly call _bound_runtime_evidence" in AUDIT._validator_wiring_errors(
        dead, Path("validator.py"), "live"
    )
    wrong_order = _live_validator_source(
        "    rclpy.init()\n    _bound_runtime_evidence()\n"
    )
    assert "validator binds runtime evidence after runtime execution begins" in AUDIT._validator_wiring_errors(
        wrong_order, Path("validator.py"), "live"
    )


def test_live_validator_wiring_rejects_docstring_and_unreachable_helper() -> None:
    source = (
        '"""_bound_runtime_evidence load_binding runtime_gate_binding"""\n'
        "def unused():\n"
        "    return _bound_runtime_evidence()\n\n"
        "def main():\n"
        "    rclpy.init()\n"
    )
    errors = AUDIT._validator_wiring_errors(source, Path("validator.py"), "live")
    assert "validator main does not directly call _bound_runtime_evidence" in errors
    assert "_bound_runtime_evidence does not directly load runtime binding" in errors


def test_water_finalizer_wiring_requires_required_runtime_argument_and_loader() -> None:
    valid = (
        "def combine(runtime_binding_path):\n"
        "    if runtime_binding_path is not None:\n"
        "        return load_binding(runtime_binding_path)\n"
        "    return None\n\n"
        "def main():\n"
        "    parser.add_argument('--runtime-binding', required=True)\n"
        "    return combine(args.runtime_binding)\n"
    )
    assert AUDIT._validator_wiring_errors(valid, Path("finalizer.py"), "water_finalizer") == []
    dead_loader = valid.replace(
        "if runtime_binding_path is not None:", "if 0:"
    )
    assert "water finalizer combine does not execute load_binding" in AUDIT._validator_wiring_errors(
        dead_loader, Path("finalizer.py"), "water_finalizer"
    )
    omitted_argument = valid.replace("args.runtime_binding", "args.other")
    assert "water finalizer main does not pass args.runtime_binding to combine" in AUDIT._validator_wiring_errors(
        omitted_argument, Path("finalizer.py"), "water_finalizer"
    )


def _report(identity: dict[str, str], gate: dict[str, object], session: Path, started_ns: int) -> dict[str, object]:
    install_root = session.parent / "runtime" / "install"
    install_root.mkdir(parents=True, exist_ok=True)
    closure_manifest = session.parent / "runtime" / "final_runtime_closure_manifest.json"
    closure_manifest.parent.mkdir(parents=True, exist_ok=True)
    closure = {
        "runtime_ws": str(install_root.parent.resolve()),
        "runtime_packages": list(AUDIT.FINAL_RUNTIME_PACKAGES),
        "symbolic_link_count": 0,
    }
    closure_manifest.write_text(
        json.dumps(
            {
                "schema_version": 6,
                "kind": "tzcup_formal_final_runtime_closure",
                "runtime_contract_revision": AUDIT.FORMAL_RUNTIME_CONTRACT_REVISION,
                "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
                "closure_sha256": AUDIT._json_digest(closure),
                "closure": closure,
            }
        ),
        encoding="utf-8",
    )
    closure_binding = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "manifest": str(closure_manifest.resolve()),
        "manifest_sha256": AUDIT._sha256(closure_manifest),
        "closure_sha256": AUDIT._json_digest(closure),
        "runtime_install_root": str(install_root.resolve()),
        "symbolic_link_count": 0,
    }
    session_payload = json.loads(session.read_text(encoding="utf-8"))
    session_payload["runtime_closure_binding"] = closure_binding
    session.write_text(json.dumps(session_payload), encoding="utf-8")
    session_hash = AUDIT._sha256(session)
    binding = {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "verified_epoch_ns": started_ns,
        "acceptance_session_binding": {"snapshot": {key: value for key, value in identity.items() if key != "output_inventory_sha256"}, "snapshot_output_inventory_sha256": identity["output_inventory_sha256"], "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "snapshot_current_source_verified": True, "session_manifest_sha256": session_hash, "session_started_epoch_ns": started_ns},
        "runtime_closure_binding": closure_binding,
    }
    report = {"report_id": gate["report_id"], "status": gate["success_statuses"][0], "source_binding": identity, "acceptance_session_binding": binding["acceptance_session_binding"], "runtime_gate_binding": binding}
    return report


def test_missing_current_formal_reports_are_blocked() -> None:
    report = AUDIT.audit(ROOT)
    assert report["status"] == "FORMAL_FOUR_CHAIN_RUNTIME_BLOCKED"
    assert not report["ready_for_formal_runtime_acceptance"]
    assert all(not chain["formal_report"]["fresh"] for chain in report["chains"].values())
    assert all(chain["source_wiring_ready"] for chain in report["chains"].values())


def test_main_writes_a_static_readiness_json_without_starting_runtime(tmp_path: Path) -> None:
    output = tmp_path / "readiness.json"
    assert AUDIT.main(["--root", str(ROOT), "--output", str(output)]) == 2
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["status"] == "FORMAL_FOUR_CHAIN_RUNTIME_BLOCKED"
    assert report["execution_scope"]["generated_runtime_report"] is False


def test_pass_json_without_active_session_is_historical_not_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    report_path = tmp_path / "mobility.json"
    gate = {"report_id": "mobility", "success_statuses": ["PASSED"], "runtime_binding": {"report_field": "runtime_gate_binding", "sidecar_suffix": ".runtime_binding.json"}, "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256", "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256", "snapshot_source_hash_field": "source_binding.source_inventory_sha256", "required_values": {}}
    report_path.write_text(json.dumps({"report_id": "mobility", "status": "PASSED", "source_binding": snapshot}), encoding="utf-8")
    result = AUDIT.evaluate_fresh_report(report_path=report_path, gate=gate, expected_snapshot=snapshot, session_path=tmp_path / "missing-session.json")
    assert not result["fresh"]
    assert result["classification"] == "HISTORICAL_OR_INVALID"
    assert "ACTIVE_ACCEPTANCE_SESSION_MISSING" in result["blockers"]


def test_completed_session_is_reported_as_finalized_not_active(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {},
    }
    report_path = tmp_path / "mobility.json"
    report_path.write_text(
        json.dumps({"report_id": "mobility", "status": "PASSED"}),
        encoding="utf-8",
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "SESSION_FINALIZED_NOT_ACTIVE" in result["blockers"]


def test_session_bound_report_with_current_hashes_and_sidecar_is_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {"report_id": "mobility", "success_statuses": ["PASSED"], "runtime_binding": {"report_field": "runtime_gate_binding", "sidecar_suffix": ".runtime_binding.json"}, "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256", "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256", "snapshot_source_hash_field": "source_binding.source_inventory_sha256", "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"}}
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    report_path.write_text(json.dumps(report), encoding="utf-8")
    sidecar = report_path.with_name("mobility.json.runtime_binding.json")
    sidecar.write_text(json.dumps(report["runtime_gate_binding"]), encoding="utf-8")
    _mark_current_fixture(started_ns, sidecar, report_path)
    result = AUDIT.evaluate_fresh_report(report_path=report_path, gate=gate, expected_snapshot=snapshot, session_path=session)
    assert result["fresh"], result["blockers"]
    assert result["classification"] == "FRESH"


def test_touched_sidecar_from_another_session_is_not_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"},
    }
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    report["runtime_gate_binding"]["acceptance_session_binding"]["session_manifest_sha256"] = "stale-session"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    report_path.with_name("mobility.json.runtime_binding.json").write_text(
        json.dumps(report["runtime_gate_binding"]), encoding="utf-8"
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "RUNTIME_BINDING_SESSION_MANIFEST_MISMATCH" in result["blockers"]


def test_session_without_current_snapshot_is_not_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {"source_inventory_sha256": "historical"},
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"},
    }
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    report_path.write_text(json.dumps(report), encoding="utf-8")
    report_path.with_name("mobility.json.runtime_binding.json").write_text(
        json.dumps(report["runtime_gate_binding"]), encoding="utf-8"
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "ACTIVE_ACCEPTANCE_SESSION_SNAPSHOT_MISMATCH" in result["blockers"]


def test_runtime_closure_manifest_hash_drift_is_not_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"},
    }
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    closure_path = Path(report["runtime_gate_binding"]["runtime_closure_binding"]["manifest"])
    closure_path.write_text(
        json.dumps({"closure_sha256": "changed", "symbolic_link_count": 0}),
        encoding="utf-8",
    )
    report_path.write_text(json.dumps(report), encoding="utf-8")
    report_path.with_name("mobility.json.runtime_binding.json").write_text(
        json.dumps(report["runtime_gate_binding"]), encoding="utf-8"
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "RUNTIME_CLOSURE_MANIFEST_HASH_MISMATCH" in result["blockers"]


def test_minimal_closure_lookalike_is_not_fresh(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"},
    }
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    closure_binding = report["runtime_gate_binding"]["runtime_closure_binding"]
    closure_path = Path(closure_binding["manifest"])
    lookalike = {"closure_sha256": "lookalike", "symbolic_link_count": 0}
    closure_path.write_text(json.dumps(lookalike), encoding="utf-8")
    closure_binding["manifest_sha256"] = AUDIT._sha256(closure_path)
    closure_binding["closure_sha256"] = "lookalike"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    report_path.with_name("mobility.json.runtime_binding.json").write_text(
        json.dumps(report["runtime_gate_binding"]), encoding="utf-8"
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "RUNTIME_CLOSURE_MANIFEST_SCHEMA_INVALID" in result["blockers"]


def test_runtime_install_root_must_exist_and_be_canonical(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "snapshot.json")
    started_ns = time.time_ns() - 1_000_000
    session = tmp_path / "session.json"
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_ns,
                "snapshot": {
                    key: value
                    for key, value in snapshot.items()
                    if key != "output_inventory_sha256"
                },
            }
        ),
        encoding="utf-8",
    )
    gate = {
        "report_id": "mobility",
        "success_statuses": ["PASSED"],
        "runtime_binding": {
            "report_field": "runtime_gate_binding",
            "sidecar_suffix": ".runtime_binding.json",
        },
        "snapshot_manifest_hash_field": "source_binding.snapshot_manifest_sha256",
        "snapshot_urdf_hash_field": "source_binding.expanded_urdf_sha256",
        "snapshot_source_hash_field": "source_binding.source_inventory_sha256",
        "required_values": {"runtime_gate_binding.status": "FORMAL_RUNTIME_GATE_BOUND"},
    }
    report_path = tmp_path / "mobility.json"
    report = _report(snapshot, gate, session, started_ns)
    report["runtime_gate_binding"]["runtime_closure_binding"][
        "runtime_install_root"
    ] = str((tmp_path / "missing-install").resolve())
    report_path.write_text(json.dumps(report), encoding="utf-8")
    report_path.with_name("mobility.json.runtime_binding.json").write_text(
        json.dumps(report["runtime_gate_binding"]), encoding="utf-8"
    )

    result = AUDIT.evaluate_fresh_report(
        report_path=report_path,
        gate=gate,
        expected_snapshot=snapshot,
        session_path=session,
    )

    assert not result["fresh"]
    assert "RUNTIME_INSTALL_ROOT_MISSING_OR_SYMLINK" in result["blockers"]
