from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


RUNNERS = (
    "run_formal_service_door_runtime.sh",
    "run_formal_service_interface_acceptance.sh",
    "run_formal_vehicle_sensor_runtime.sh",
    "run_formal_auxiliary_runtime.sh",
    "run_whole_vehicle_actuator_interlock_runtime.sh",
)


def test_all_service_sensor_and_interlock_runners_use_one_frozen_runtime_gate():
    for name in RUNNERS:
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert ".work/final_frozen_runtime" in source, name
        assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in source, name
        assert "FORMAL_ACCEPTANCE_SESSION" in source, name
        assert "FORMAL_VEHICLE_SNAPSHOT_MANIFEST" in source, name
        assert "generate_formal_vehicle_snapshot.py" in source, name
        assert "--check --output \"${snapshot}\"" in source, name
        assert "formal_runtime_gate_binding.py" in source, name
        assert "--runtime-binding" in source, name
        assert source.index("formal_runtime_gate_binding.py") < source.index(
            "ros2 launch"
        ), name
        assert source.index("generate_formal_vehicle_snapshot.py") < source.index(
            "formal_runtime_gate_binding.py"
        ), name


def test_finalizers_recheck_current_snapshot_and_session_against_sidecar():
    for name in (
        "validate_whole_vehicle_actuator_interlock.py",
        "validate_formal_auxiliary_runtime.py",
        "collect_formal_service_door_runtime.py",
        "validate_formal_service_interface_acceptance.py",
    ):
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        assert "def _bound_runtime_evidence(" in source, name
        assert "session_manifest_sha256" in source, name
        assert 'bound_session.get("snapshot")' in source, name


def test_interlock_uses_the_real_ros_gz_contact_element_type():
    source = (SCRIPTS / "validate_whole_vehicle_actuator_interlock.py").read_text(
        encoding="utf-8"
    )
    assert "from ros_gz_interfaces.msg import Contact, Contacts" in source
    assert "Contacts(contacts=[Contact()] if collision else [])" in source
