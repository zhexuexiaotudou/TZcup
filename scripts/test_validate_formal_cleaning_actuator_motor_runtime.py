import importlib.util
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAMES = [
    "left_side_brush",
    "right_side_brush",
    "central_roller",
    "cleaning_lift",
    "recovery_pump",
]


def _module():
    path = ROOT / "scripts/validate_formal_cleaning_actuator_motor_runtime.py"
    spec = importlib.util.spec_from_file_location("motor_runtime", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _motors(*, fault="none", lift_above_rating=False):
    return [
        {
            "name": name,
            "fault": fault if index == 3 else "none",
            "current_above_rating": lift_above_rating if index == 3 else False,
        }
        for index, name in enumerate(NAMES)
    ]


def _sample(phase, currents, temperatures, **extra):
    lift_fault = extra.pop("lift_fault", "none")
    return {
        "phase": phase,
        "current_a": currents,
        "temperature_c": temperatures,
        "output_load": [0.1] * 5,
        "fault": lift_fault != "none",
        "safety_enabled": lift_fault == "none",
        "status": {
            "motors": _motors(
                fault=lift_fault,
                lift_above_rating=lift_fault == "stall",
            )
        },
        **extra,
    }


def _passing_artifact():
    safe = [0.2, 0.2, 0.3, 0.1, 2.0]
    vector_root = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors"
    bridge_graph = {
        vector_root + suffix: {
            "publisher_count": 1,
            "publishers": [
                {
                    "node_name": "cleaning_actuator_motor_bridge",
                    "node_namespace": "/",
                    "topic_type": "std_msgs/msg/Float64MultiArray",
                }
            ],
            "ros_subscription_count": 1,
        }
        for suffix in (
            "/motor_current_a",
            "/motor_temperature_c",
            "/estimated_output_load",
        )
    }
    return {
        "schema_version": 2,
        "evidence_authority": "GAZEBO_PHYSICAL_JOINT_AND_POST_SAFETY_CONTROLLER_OBSERVATION",
        "joint_state_mutation_used": False,
        "production_motor_parameters_modified": False,
        "lift_trajectory_published": True,
        "reset_publish_count": 2,
        "live_overtemperature_claimed": False,
        "thermal_protection_evidence": {"kind": "separate_core_unit_test"},
        "cleaning_vector_bridge_graph": bridge_graph,
        "samples": [
            _sample("normal_load", safe, [30.0] * 5),
            _sample(
                "physical_travel_stop_stall",
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [30.0, 30.0, 30.0, 36.0, 30.0],
                lift_fault="stall",
                safety_enabled=False,
                lift_reference_m=0.125,
                lift_position_m=0.100,
                lift_velocity_m_s=0.0,
                whole_vehicle_safety_state="INHIBITED",
                whole_vehicle_safety_values={
                    "cleaning_motor_fault_active": "true",
                    "actuators_enabled": "false",
                },
            ),
            _sample(
                "idle_cooling",
                [0.0] * 5,
                [30.0, 30.0, 30.0, 35.9, 30.0],
                lift_fault="stall",
                safety_enabled=False,
            ),
            _sample(
                "idle_cooling",
                [0.0] * 5,
                [30.0, 30.0, 30.0, 35.5, 30.0],
                lift_fault="stall",
                safety_enabled=False,
            ),
            _sample("explicit_reset", [0.0] * 5, [30.0] * 5),
        ],
    }


def _runtime_inputs(tmp_path: Path, monkeypatch, module):
    snapshot = tmp_path / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    snapshot.parent.mkdir(parents=True)
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "b" * 64
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    source_binding = module._source_binding(snapshot)
    session = tmp_path / "artifacts/formal_final_acceptance_session.json"
    session.parent.mkdir(parents=True)
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": 123,
                "snapshot": source_binding,
            }
        ),
        encoding="utf-8",
    )
    install = tmp_path / "frozen/install"
    marker = install / "share/ament_index/resource_index/packages/sanitation_vehicle_description"
    marker.parent.mkdir(parents=True)
    marker.write_text("", encoding="utf-8")
    (install / "setup.bash").write_text("# frozen\n", encoding="utf-8")
    binding = tmp_path / "runtime_binding.json"
    binding.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": "FORMAL_RUNTIME_GATE_BOUND",
                "verified_epoch_ns": 456,
                "acceptance_session_binding": {
                    "session_manifest": str(session.resolve()),
                    "session_manifest_sha256": hashlib.sha256(session.read_bytes()).hexdigest(),
                    "session_started_epoch_ns": 123,
                    "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                    "snapshot": source_binding,
                    "snapshot_current_source_verified": True,
                },
                "runtime_closure_binding": {
                    "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
                    "runtime_install_root": str(install.resolve()),
                    "manifest_sha256": "c" * 64,
                    "closure_sha256": "d" * 64,
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(install.resolve()))
    return snapshot, session, binding, source_binding


def _validate_passing_artifact(tmp_path: Path, monkeypatch):
    module = _module()
    snapshot, session, binding, source_binding = _runtime_inputs(tmp_path, monkeypatch, module)
    artifact = tmp_path / "runtime.json"
    payload = _passing_artifact()
    payload["source_binding"] = source_binding
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    return module, artifact, snapshot, session, binding


def test_runtime_validator_accepts_physical_stall_global_inhibit_and_reset(tmp_path):
    from pytest import MonkeyPatch

    monkeypatch = MonkeyPatch()
    try:
        module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
        report = module.validate(
            artifact,
            snapshot_path=snapshot,
            session_path=session,
            runtime_binding_path=binding,
        )
    finally:
        monkeypatch.undo()
    assert report["passed"], report["failed_checks"]
    assert report["status"] == "FORMAL_CLEANING_ACTUATOR_MOTOR_RUNTIME_PASSED"
    assert report["thermal_evidence_boundary"]["live_overtemperature_tested"] is False
    assert report["runtime_gate_binding"]["status"] == "FORMAL_RUNTIME_GATE_BOUND"
    assert report["active_frozen_overlay"]["runtime_install_root"] == report[
        "runtime_gate_binding"
    ]["runtime_closure_binding"]["runtime_install_root"]


def test_runtime_validator_rejects_uninhibited_fault(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    stalled = payload["samples"][1]
    stalled["safety_enabled"] = True
    stalled["whole_vehicle_safety_state"] = "ENABLED"
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    report = module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
    assert not report["passed"]
    assert "stall_fault_caused_whole_vehicle_global_inhibit" in report["failed_checks"]


def test_runtime_validator_rejects_metadata_only_stall_without_physical_joint_evidence(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    payload["samples"][1].pop("lift_position_m")
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    report = module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
    assert not report["passed"]
    assert "physical_lift_travel_stop_reached_stall_boundary" in report["failed_checks"]


def test_runtime_validator_rejects_live_overtemperature_claim(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    payload["live_overtemperature_claimed"] = True
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    report = module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
    assert not report["passed"]
    assert "live_overtemperature_not_claimed" in report["failed_checks"]


def test_runtime_validator_rejects_second_ros_writer(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    endpoint = next(iter(payload["cleaning_vector_bridge_graph"].values()))
    endpoint["publisher_count"] = 2
    endpoint["publishers"].append(
        {
            "node_name": "rogue_writer",
            "node_namespace": "/",
            "topic_type": "std_msgs/msg/Float64MultiArray",
        }
    )
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    report = module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
    assert not report["passed"]
    assert (
        "double_v_bridge_has_exact_type_direction_and_single_ros_writer"
        in report["failed_checks"]
    )


def test_runtime_validator_rejects_missing_output_load_vector(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    for sample in payload["samples"]:
        sample.pop("output_load")
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    report = module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
    assert not report["passed"]
    assert "five_named_motor_vectors_finite" in report["failed_checks"]


def test_runtime_validator_rejects_capture_from_a_different_snapshot(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    payload["source_binding"]["source_inventory_sha256"] = "wrong"
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    import pytest

    with pytest.raises(ValueError, match="capture source binding"):
        module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)


def test_runtime_validator_rejects_when_active_overlay_resolves_another_vehicle_package(tmp_path, monkeypatch):
    module, artifact, snapshot, session, binding = _validate_passing_artifact(tmp_path, monkeypatch)
    other = tmp_path / "other_overlay"
    marker = other / "share/ament_index/resource_index/packages/sanitation_vehicle_description"
    marker.parent.mkdir(parents=True)
    marker.write_text("", encoding="utf-8")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(other))
    import pytest

    with pytest.raises(ValueError, match="omits the frozen runtime install root"):
        module.validate(artifact, snapshot_path=snapshot, session_path=session, runtime_binding_path=binding)
