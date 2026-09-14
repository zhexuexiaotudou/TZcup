from sanitation_tasks.smoke_check import REQUIRED_TOPICS


def test_required_topics_cover_motion_sensing_and_transforms():
    assert {"/clock", "/cmd_vel", "/odom"} <= REQUIRED_TOPICS
    assert {"/imu/data", "/scan"} <= REQUIRED_TOPICS
    assert {"/tf", "/tf_static"} <= REQUIRED_TOPICS


def test_required_camera_topics_are_namespaced_consistently():
    camera_topics = {topic for topic in REQUIRED_TOPICS if topic.startswith("/camera/")}
    assert camera_topics == {
        "/camera/color/camera_info",
        "/camera/color/image_raw",
        "/camera/depth/color/points",
        "/camera/depth/image_rect_raw",
    }
