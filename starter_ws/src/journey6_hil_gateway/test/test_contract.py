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
