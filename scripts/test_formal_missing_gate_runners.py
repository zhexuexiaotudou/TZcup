from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _runner(name: str) -> str:
    return (ROOT / "scripts" / name).read_text(encoding="utf-8")


def test_manipulator_runner_is_frozen_self_contained_and_isolated() -> None:
    source = _runner("run_formal_manipulator_trajectory_runtime.sh")
    assert "FORMAL_VEHICLE_RUNTIME_WS:?" in source
    assert "FORMAL_VEHICLE_SNAPSHOT_MANIFEST" in source
    assert "formal_vehicle_sim.launch.py" in source
    assert "validate_formal_manipulator_runtime.py" in source
    assert "--snapshot-manifest" in source
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in source
    assert "GZ_PARTITION" in source
    assert "generate_formal_vehicle_snapshot.py" in source
    assert '--check --output "${snapshot_manifest}"' in source
    assert "formal_runtime_gate_binding.py" in source
    assert '--runtime-binding "${runtime_binding}"' in source


def test_function_position_runner_is_frozen_self_contained_and_isolated() -> None:
    source = _runner("run_formal_function_positions_runtime.sh")
    assert "FORMAL_VEHICLE_RUNTIME_WS:?" in source
    assert "FORMAL_VEHICLE_SNAPSHOT_MANIFEST" in source
    assert "formal_vehicle_sim.launch.py" in source
    assert "validate_formal_function_positions_runtime.py" in source
    assert "--snapshot-manifest" in source
    assert '"${FORMAL_RUNTIME_SESSION_PREFIX[@]}" ros2 launch' in source
    assert "GZ_PARTITION" in source
    assert "generate_formal_vehicle_snapshot.py" in source
    assert '--check --output "${snapshot_manifest}"' in source
    assert "formal_runtime_gate_binding.py" in source
    assert '--runtime-binding "${runtime_binding}"' in source


def test_runtime_validators_emit_bound_snapshot_session_and_closure_identity() -> None:
    for name in (
        "validate_formal_manipulator_runtime.py",
        "validate_formal_function_positions_runtime.py",
    ):
        source = _runner(name)
        assert "bound_runtime_evidence(" in source
        assert '"source_binding": source_binding' in source
        assert '"acceptance_session_binding": acceptance_session_binding' in source
        assert '"runtime_gate_binding": runtime_gate_binding' in source
        assert '"expanded_urdf_sha256": output["sha256"]' in source
