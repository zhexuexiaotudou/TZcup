import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _module():
    path = ROOT / "scripts/validate_cleaning_actuator_motor_contract.py"
    spec = importlib.util.spec_from_file_location("motor_contract", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_cleaning_actuator_motor_contract_is_complete_and_truthful():
    report = _module().validate()
    assert report["passed"], report["failed_checks"]
    assert len(report["checks"]) >= 10


def test_runtime_contract_does_not_claim_unrun_gazebo_acceptance():
    contract = (ROOT / "config/high_fidelity_vehicle/cleaning_actuator_motor_realism_contract.yaml").read_text(encoding="utf-8")
    assert "pending_no_gazebo_run_in_this_change" in contract
    assert "source_integrated_runtime_runner_ready_acceptance_pending" in contract
    assert "live_overtemperature_required: false" in contract


def test_safety_fault_heartbeat_is_not_throttled_by_low_simulation_rtf():
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc"
    ).read_text(encoding="utf-8")
    assert "std::chrono::steady_clock::now()" in source
    assert "telemetryGate.Update(output, physicsUpdateWallTimeS)" in source
    assert "physics_update_stale" in source
    assert "TelemetryLoop" in source
    assert "telemetryThread.join()" in source
    assert "std::chrono::milliseconds(50)" in source
    assert "lastPublishSimTimeS" not in source
    assert "input.step_s = std::chrono::duration<double>(info.dt).count()" in source
    assert "simTimeS - this->lastCommandSimTimeS" in source
    assert "const auto output = this->core.Step(input)" in source


def test_telemetry_snapshot_clock_is_sampled_after_lock_acquisition():
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc"
    ).read_text(encoding="utf-8")
    loop = source[source.index("void TelemetryLoop()") : source.index(
        "template<typename MessageT>", source.index("void TelemetryLoop()")
    )]
    lock = "std::lock_guard<std::mutex> lock(this->telemetryMutex);"
    snapshot_clock = "const double snapshotWallNowS = std::chrono::duration<double>("
    snapshot_call = "snapshot = this->telemetryGate.Snapshot(snapshotWallNowS);"

    assert "const auto wallNow = std::chrono::steady_clock::now();" in loop
    assert loop.index(lock) < loop.index(snapshot_clock) < loop.index(snapshot_call)
    snapshot_section = loop[loop.index(snapshot_clock) : loop.index(snapshot_call)]
    assert "std::chrono::steady_clock::now()" in snapshot_section
    assert "Snapshot(wallNowS)" not in loop
    assert "wallNow >= nextStatusPublish" in loop
    assert "nextPublish <= wallNow" in loop


def test_gazebo_publish_failures_are_topic_tagged_and_json_is_isolatable():
    source = (
        ROOT
        / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc"
    ).read_text(encoding="utf-8")
    xacro = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert "kPublishTopicTags" in source
    assert "gz_publish_failed topic=" in source
    assert "publishFailureCounts" in source
    assert "status_json_publish_rate_hz" in source
    assert "realtime_telemetry_enabled" in source
    assert "status_json_enabled" in source
    assert "PublishRealtime(snapshot)" in source
    assert "PublishStatusJson(snapshot)" in source
    for name in (
        "cleaning_realtime_telemetry_enabled",
        "cleaning_status_json_enabled",
        "cleaning_status_json_publish_rate_hz",
    ):
        assert name in xacro
        assert name in launch
