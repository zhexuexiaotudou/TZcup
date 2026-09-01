from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts/run_formal_preembedded_control_diagnostic.sh"


def test_runner_requires_an_explicit_frozen_runtime_and_has_no_r53_fallback() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'runtime_setup="${FORMAL_DIAGNOSTIC_RUNTIME_SETUP:-}"' in source
    assert "FORMAL_DIAGNOSTIC_RUNTIME_SETUP must explicitly name" in source
    assert "r53" not in source


def test_runner_resolves_the_package_share_inside_the_requested_runtime() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'runtime_setup="$(readlink -f -- "${runtime_setup}")"' in source
    assert 'package_share_raw="$(ros2 pkg prefix --share sanitation_vehicle_description)"' in source
    assert 'expected_package_share="$(readlink -f -- "${install_root}/share/sanitation_vehicle_description")"' in source
    assert '[[ "${package_share}" == "${expected_package_share}" ]]' in source
    assert "vehicle package resolves outside the requested frozen runtime" in source


def test_runner_binds_current_sources_to_installed_bytes_before_launch() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    for name in (
        "root_xacro",
        "manipulator_stack",
        "launch",
        "controller_config",
        "source_world",
    ):
        assert f"bind_source_install {name}" in source
    for relative in (
        "urdf/formal_competition_vehicle.urdf.xacro",
        "urdf/high_fidelity/manipulator_stack.xacro",
        "launch/formal_vehicle_sim.launch.py",
        "config/formal_vehicle_controllers.yaml",
        "worlds/formal_vehicle_validation.sdf",
    ):
        assert relative in source
    assert "prepare_formal_preembedded_sensor_world.py" in source
    assert "runtime_source_install_bindings.tsv" in source
    assert source.index("bind_source_install root_xacro") < source.index("ros2 launch")
    assert '[[ "${source_sha}" == "${installed_sha}" ]]' in source


def test_runner_is_diagnostic_only_and_fails_before_success_on_watchdog_error() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'printf \'diagnostic=true\\n\'' in source
    assert 'printf \'formal_eligible=false\\n\'' in source
    assert '"formal_eligible": False' in source
    assert "Diagnostic-only preembedded-control bisection" in source
    assert "PREEMBEDDED_CONTROL_DIAGNOSTIC_MEMORY_WATCHDOG_FAILED" in source
    assert '"passed": False' in source
    assert source.index("formal_runtime_stop_memory_watchdog") < source.index(
        "PREEMBEDDED_CONTROL_DIAGNOSTIC_SURVIVED"
    )
    assert source.index("if (( watchdog_result != 0 )); then") < source.index(
        "PREEMBEDDED_CONTROL_DIAGNOSTIC_SURVIVED"
    )
