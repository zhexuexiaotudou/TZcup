from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]


def test_fixed_topic_and_qos_contract_is_complete():
    contract = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hil_topic_qos_contract.yaml").read_text(
            encoding="utf-8"
        )
    )
    topics = {row["name"]: row for row in contract["topics"]}
    required = {
        "/hil/camera/color",
        "/hil/camera/depth",
        "/hil/camera/camera_info",
        "/hil/scan",
        "/hil/imu",
        "/hil/wheel_odom",
        "/hil/gnss/fix",
        "/hil/gnss/heading",
        "/hil/clock",
        "/hil/tf",
        "/hil/tf_static",
        "/hil/cleanable_boundary",
        "/hil/keepout",
        "/hil/vehicle/ackermann_command",
        "/hil/vehicle/brake_command",
        "/hil/brush/command",
        "/hil/safety/estop_request",
        "/hil/task/state",
        "/hil/health",
    }
    assert required <= topics.keys()
    assert topics["/hil/camera/color"]["qos"] == "sensor_data"
    assert topics["/hil/vehicle/ackermann_command"]["qos"] == "control_deadline"
    assert contract["qos_profiles"]["control_deadline"] == {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 1,
        "deadline_ms": 80,
        "lifespan_ms": 120,
    }


def test_algorithm_container_has_only_allowlisted_mounts():
    compose = yaml.safe_load(
        (REPO_ROOT / "docker" / "compose.journey6-loopback.yaml").read_text(
            encoding="utf-8"
        )
    )
    volumes = compose["services"]["j6-algorithm"]["volumes"]
    rendered = "\n".join(volumes).lower()
    for forbidden in ("ground_truth", "/world", "sealed", "evaluator"):
        assert forbidden not in rendered
    assert "/opt/tzcup/runtime:ro" in rendered
    assert "/opt/tzcup/models:ro" in rendered
    assert compose["services"]["j6-algorithm"]["cap_add"] == ["NET_ADMIN"]


def test_pc_onnx_profile_is_explicit_and_cannot_claim_journey6_runtime():
    compose = yaml.safe_load(
        (REPO_ROOT / "docker" / "compose.journey6-loopback.yaml").read_text(
            encoding="utf-8"
        )
    )
    service = compose["services"]["pc-onnx-algorithm"]
    assert service["profiles"] == ["pc-onnx"]
    assert service["build"]["args"]["RUNTIME_BACKEND"] == "PC_ONNX"
    assert "ros:humble-ros-base" in service["build"]["args"][
        "J6_ALGORITHM_BASE_IMAGE"
    ]
    assert "/opt/ros/humble/setup.bash" in service["build"]["args"]["ROS_SETUP"]
    assert service["environment"]["TZCUP_RUNTIME_BACKEND"] == "PC_ONNX"
    assert service["environment"]["TZCUP_NOT_JOURNEY6_RUNTIME"] == "true"
    mounts = "\n".join(service["volumes"]).lower()
    for forbidden in ("ground_truth", "/world", "sealed", "evaluator"):
        assert forbidden not in mounts


def test_contract_keeps_emulation_and_legacy_j6_readiness_separate():
    contract = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hil_topic_qos_contract.yaml").read_text(
            encoding="utf-8"
        )
    )
    identity = contract["runtime_identity"]["pc_onnx"]
    assert identity == {
        "runtime_backend": "PC_ONNX",
        "not_journey6_runtime": True,
        "algorithm_host_os": "ubuntu-22.04",
        "algorithm_host_ros_distro": "humble",
        "may_satisfy_legacy_J6_LOOPBACK_HIL_READY": False,
    }
    assert contract["formal_transport"] == {
        "minimum_duration_s": 1800,
        "sensor_source": "gazebo",
        "pc_ros_distro": "jazzy",
        "require_full_fault_and_authority_matrix": True,
        "sensor_provenance_manifest": "HIL_GAZEBO_SENSOR_PROVENANCE.json",
        "sensor_provenance_required": [
            "audited_launch",
            "gazebo_process_verified",
            "publisher_endpoints_verified",
            "pc_sensor_and_plant_only",
            "evidence_sha256",
        ],
        "endpoint_provenance_required": {
            "publisher_process_links_verified": True,
            "harness_sensor_publishers_present": False,
            "unexpected_publishers": [],
        },
    }
    assert contract["model_qualification"]["self_reported_boolean_allowed"] is False
    assert contract["model_qualification"]["bind_to_fields"] == [
        "model_id",
        "model_sha256",
    ]
    assert "J6_LOOPBACK_HIL_EMULATION_READY" in contract["readiness_states"]
    assert "official Journey6 runtime only" in contract["readiness_states"][
        "J6_LOOPBACK_HIL_READY"
    ]
