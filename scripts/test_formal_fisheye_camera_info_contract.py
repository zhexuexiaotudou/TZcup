import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_fisheye_camera_info_is_single_writer_and_matches_nominal_sdf_lens() -> None:
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    publisher = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/scripts/formal_fisheye_camera_info_publisher.py"
    ).read_text(encoding="utf-8")
    assert "formal_fisheye_camera_info_publisher.py" in launch
    assert "rear_left_fisheye/camera_info@sensor_msgs" not in launch
    assert "rear_right_fisheye/camera_info@sensor_msgs" not in launch
    assert 'message.distortion_model = "equidistant"' in publisher
    assert "message.d = DISTORTION_COEFFICIENTS.copy()" in publisher
    assert "-1.0 / 24.0" in publisher
    assert "1.0 / 1920.0" in publisher
    assert "-1.0 / 322560.0" in publisher
    assert "1.0 / 92897280.0" in publisher
    expected_focal = 960.0 / (2.0 * math.sin(math.radians(150.0) / 4.0))
    assert abs(expected_focal - 788.4862232182) < 1e-9


def test_fisheye_image_bridge_rewrites_scoped_gazebo_frames_to_optical_frames() -> None:
    rows = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_vehicle_description/config/formal_high_bandwidth_sensor_bridge.yaml"
        ).read_text(encoding="utf-8")
    )
    by_topic = {row["ros_topic_name"]: row for row in rows}
    for side in ("left", "right"):
        topic = f"/sensors/rear_{side}_fisheye/image_raw"
        assert by_topic[topic]["frame_id"] == (
            f"rear_{side}_fisheye_optical_frame"
        )


def test_equisolid_taylor_contract_is_below_5e_10_at_fov_edge() -> None:
    theta = math.radians(150.0) / 2.0
    coefficients = (-1 / 24, 1 / 1920, -1 / 322560, 1 / 92897280)
    ros_radius = theta * (
        1.0 + sum(k * theta ** (2 * (index + 1)) for index, k in enumerate(coefficients))
    )
    sdf_radius = 2.0 * math.sin(theta / 2.0)
    assert abs(ros_radius - sdf_radius) < 5.0e-10


def test_nominal_profile_is_registered_without_claiming_hardware_calibration() -> None:
    contract = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/pre_urdf_contract.yaml").read_text(
            encoding="utf-8"
        )
    )
    register = yaml.safe_load(
        (
            ROOT
            / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
        ).read_text(encoding="utf-8")
    )
    layouts = {
        item["id"]: item for item in contract["sensor_contracts"]
    }
    installations = {
        item["id"]: item for item in register["sensor_installations"]
    }
    camera_info_contracts = register["topic_contracts"]
    expected_d = [-1 / 24, 1 / 1920, -1 / 322560, 1 / 92897280]
    for sensor_id in ("rear_left_fisheye", "rear_right_fisheye"):
        layout = layouts[sensor_id]
        assert layout["simulation_lens_projection"] == "equisolid_angle"
        assert layout["simulation_calibration_profile"] == (
            "sdf_equisolid_to_ros_kannala_brandt_taylor_v1"
        )
        assert layout["real_camera_calibration_required"] is True
        assert layout["calibration_source_boundary"].endswith(
            "not_measured_hardware"
        )
        assert all(
            abs(observed - expected) < 5.0e-14
            for observed, expected in zip(
                layout["nominal_distortion_coefficients"], expected_d
            )
        )
        installation = installations[sensor_id]
        assert installation["model"] == (
            "Arducam B0202 IMX291 UVC plus M27195H15 lens"
        )
        assert installation["hardware_calibration_status"] == (
            "required_before_real_camera_deployment"
        )
    for contract_id in ("rear_left_camera_info", "rear_right_camera_info"):
        topic_contract = camera_info_contracts[contract_id]
        assert topic_contract["transport"] == "ros_native"
        assert topic_contract["direction"] == "publisher"
        assert topic_contract["single_writer"] is True
        assert topic_contract["writer_node"] == (
            "formal_fisheye_camera_info_publisher"
        )
        assert topic_contract["source_path"].endswith(
            "formal_fisheye_camera_info_publisher.py"
        )
        assert "gz_type" not in topic_contract
