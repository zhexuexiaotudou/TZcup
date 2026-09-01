from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_formal_robot_description_has_one_bounded_transient_product_writer() -> None:
    publisher = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/scripts/formal_robot_description_publisher.py"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'String, "/robot_description", qos' in publisher
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in publisher
    assert "ReliabilityPolicy.RELIABLE" in publisher
    assert "self.create_timer(0.25, self._publish_until_matched)" in publisher
    assert "self._timer.cancel()" in publisher
    assert "TRANSIENT_LOCAL sample serves every later subscriber" in publisher
    assert "self._description_volatile_publisher" not in publisher
    assert "formal_robot_description_publisher.py" in launch
    assert "/formal_vehicle/internal/robot_description_from_state_publisher" in launch


def test_camera_info_publisher_does_not_shadow_rclpy_internal_publishers() -> None:
    publisher = (
        ROOT
        / "starter_ws/src/sanitation_vehicle_description/scripts/formal_fisheye_camera_info_publisher.py"
    ).read_text(encoding="utf-8")
    assert "self._camera_info_publishers" in publisher
    assert "self._publishers =" not in publisher
