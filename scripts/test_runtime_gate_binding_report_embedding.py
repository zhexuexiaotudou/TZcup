"""Static and pure-Python coverage for final runtime-gate report provenance."""

from __future__ import annotations

import json
from pathlib import Path

import validate_formal_service_interface_acceptance as service_interface


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


GATES = {
    "sensor_runtime": (
        "run_formal_vehicle_sensor_runtime.sh",
        "collect_formal_vehicle_sensor_runtime.py",
        "runtime_binding",
    ),
    "whole_vehicle_interlock": (
        "run_whole_vehicle_actuator_interlock_runtime.sh",
        "validate_whole_vehicle_actuator_interlock.py",
        "runtime_binding",
    ),
    "auxiliary_power_lighting": (
        "run_formal_auxiliary_runtime.sh",
        "validate_formal_auxiliary_runtime.py",
        "binding",
    ),
    "service_door_runtime": (
        "run_formal_service_door_runtime.sh",
        "collect_formal_service_door_runtime.py",
        "binding",
    ),
    "service_interface_acceptance": (
        "run_formal_service_interface_acceptance.sh",
        "validate_formal_service_interface_acceptance.py",
        "binding",
    ),
}


PROPAGATED_EXISTING_BINDING_GATES = {
    "first_map_then_clean": (
        "run_formal_saved_map_cleaning_lifecycle.sh",
        "validate_formal_map_lifecycle_runtime.py",
    ),
    "random_scene_perception": (
        "run_formal_random_scene_perception.sh",
        "aggregate_formal_random_scene_perception.py",
    ),
}


def test_every_target_gate_runner_builds_sidecar_before_runtime() -> None:
    for gate, (runner_name, _, _) in GATES.items():
        source = (SCRIPTS / runner_name).read_text(encoding="utf-8")
        assert 'runtime_binding="${output}.runtime_binding.json"' in source or (
            'runtime_binding="${aggregate_output}.runtime_binding.json"' in source
        ), gate
        assert "formal_runtime_gate_binding.py" in source, gate
        assert "--runtime-binding" in source, gate
        assert source.index("formal_runtime_gate_binding.py") < source.index(
            "ros2 launch"
        ), gate


def test_every_target_final_report_embeds_the_complete_loaded_binding() -> None:
    for gate, (_, finalizer_name, binding_name) in GATES.items():
        source = (SCRIPTS / finalizer_name).read_text(encoding="utf-8")
        assert "from formal_runtime_gate_binding import load_binding" in source, gate
        assert f'"runtime_gate_binding": {binding_name}' in source or (
            f'report["runtime_gate_binding"] = {binding_name}' in source
        ) or (f'result["runtime_gate_binding"] = {binding_name}' in source), gate
        assert "acceptance_session_binding" in source, gate
        assert "runtime_closure_binding" in source, gate


def test_existing_preflight_bindings_propagate_to_the_canonical_final_reports() -> None:
    for gate, (runner_name, finalizer_name) in PROPAGATED_EXISTING_BINDING_GATES.items():
        runner = (SCRIPTS / runner_name).read_text(encoding="utf-8")
        finalizer = (SCRIPTS / finalizer_name).read_text(encoding="utf-8")
        assert "formal_source_bound_preflight" in runner, gate
        assert '--runtime-binding "${runtime_binding}"' in runner, gate
        assert "from formal_runtime_gate_binding import RuntimeGateError, load_binding" in finalizer, gate
        assert '"runtime_gate_binding"' in finalizer, gate
        assert '".runtime_binding.json"' in finalizer, gate
        assert "_atomic_write_json(sidecar, binding)" in finalizer, gate


def _write_passing_episode(root: Path, scenario: str) -> None:
    (root / f"{scenario}.json").write_text(
        json.dumps(
            {
                "schema": "tzcup.formal_service_interface_episode.v1",
                "scenario": scenario,
                "result": "PASS",
                "gates": {
                    name: True
                    for name in service_interface.required_gate_names(scenario)
                },
                "wastewater_capacity_kg": 8.30,
                "subscription_topics": ["/joint_states"],
            }
        ),
        encoding="utf-8",
    )


def test_service_interface_final_report_preserves_binding_without_projection(
    tmp_path: Path, monkeypatch
) -> None:
    for scenario in service_interface.EXPECTED_SCENARIOS:
        _write_passing_episode(tmp_path, scenario)
    complete_binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "verified_epoch_ns": 123,
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "snapshot": {"expanded_urdf_sha256": "a" * 64},
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "runtime_install_root": str(tmp_path.resolve()),
        },
    }
    monkeypatch.setattr(service_interface, "load_binding", lambda _: complete_binding)

    monkeypatch.setattr(
        service_interface,
        "_bound_runtime_evidence",
        lambda *_: (complete_binding, complete_binding["acceptance_session_binding"]),
    )
    report = service_interface.aggregate(
        tmp_path,
        tmp_path / "gate-binding.json",
        snapshot_path=tmp_path / "snapshot.json",
        session_path=tmp_path / "session.json",
    )

    assert report["runtime_gate_binding"] == complete_binding
    assert report["acceptance_session_binding"] == complete_binding[
        "acceptance_session_binding"
    ]
    assert report["runtime_closure_binding"] == complete_binding[
        "runtime_closure_binding"
    ]
