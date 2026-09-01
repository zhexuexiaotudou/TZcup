from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_formal_manipulator_trajectory_runtime.sh"


def test_runner_binds_fresh_manipulator_gate_before_launch() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert "FORMAL_MANIPULATOR_TRAJECTORY_RUNTIME_BINDING" in source
    assert 'runtime_binding="${FORMAL_MANIPULATOR_TRAJECTORY_RUNTIME_BINDING:-${output}.runtime_binding.json}"' in source
    assert "formal_runtime_gate_binding.py" in source
    assert 'source "${repo_root}/scripts/formal_source_bound_preflight.sh"' in source
    assert 'formal_source_bound_verify_overlay "${runtime_ws}/install"' in source
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in source
    assert "FORMAL_ACCEPTANCE_SESSION" in source
    assert "generate_formal_vehicle_snapshot.py" in source
    assert 'formal_runtime_register_evidence_paths "${output}" "${runtime_binding}"' in source
    assert source.index("formal_runtime_gate_binding.py") < source.index(
        "ros2 launch sanitation_vehicle_description formal_vehicle_sim.launch.py"
    )
    assert '"${runtime_binding}"' in source
    assert "--session \"${session}\" --runtime-binding \"${runtime_binding}\"" in source


def test_runner_supersedes_prior_output_binding_and_log_before_preflight() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert 'for retained in "${output}" "${runtime_binding}" "${launch_log}"; do' in source
    assert '[[ -e "${retained}" || -L "${retained}" ]]' in source
    assert 'superseded="${retained}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"' in source
    assert 'mv -- "${retained}" "${superseded}"' in source
    assert source.index('for retained in "${output}" "${runtime_binding}" "${launch_log}"; do') < source.index(
        "source /opt/ros/jazzy/setup.bash"
    )
