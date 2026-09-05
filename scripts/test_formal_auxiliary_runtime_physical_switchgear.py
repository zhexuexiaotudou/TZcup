from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_auxiliary_runner_uses_the_physical_gazebo_switchgear() -> None:
    runner = (ROOT / "scripts/run_formal_auxiliary_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "formal_vehicle_sim.launch.py" in runner
    assert "enable_safety_manager:=true" in runner
    assert "start_simulation_safety_inputs:=false" in runner
    assert "start_power_system_simulators:=false" in runner
    assert "--in-process-product-node" in runner
    assert "--check --output \"${snapshot}\"" in runner
    assert "--snapshot \"${snapshot}\" --session \"${session}\"" in runner
    assert "GZ_PARTITION" in runner


def test_auxiliary_probe_requires_measured_isolator_and_contactor() -> None:
    probe = (ROOT / "scripts/validate_formal_auxiliary_runtime.py").read_text(
        encoding="utf-8"
    )
    assert '"enabled_physical_switchgear_measured_closed"' in probe
    assert '"main_isolator_feedback_fresh"' in probe
    assert '"main_contactor_feedback_fresh"' in probe
    assert '"emergency_stop_reset"' in probe
    assert '"/formal_vehicle/lighting/work_lights_applied"' in probe
    assert '"/formal_vehicle/lighting/tail_lights_applied"' in probe
    assert '"/formal_vehicle/lighting/warning_lights_applied"' in probe
    assert "from sanitation_power_system.a300_bms_node import A300BmsNode" in probe
    assert "def _bound_runtime_evidence(" in probe
    assert "session_manifest_sha256" in probe


def test_lighting_plugin_handles_sdformat_fixed_joint_lump_names() -> None:
    plugin = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_auxiliary/src/FormalAuxiliaryVisualSystem.cc"
    ).read_text(encoding="utf-8")
    assert '"__" + candidate->first + "_visual_"' in plugin
    assert "matchedNames.insert(spec->first)" in plugin
    assert "matched != this->visualGroups.size()" in plugin
