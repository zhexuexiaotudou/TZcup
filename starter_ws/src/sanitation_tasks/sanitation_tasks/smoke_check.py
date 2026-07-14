import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node


REQUIRED_TOPICS = {
    "/clock",
    "/cmd_vel",
    "/odom",
    "/imu/data",
    "/scan",
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/depth/image_rect_raw",
    "/camera/depth/color/points",
    "/tf",
    "/tf_static",
}


class SmokeCheck(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_smoke_check")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("output_path", "smoke_check.json")

    def run(self) -> int:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        start = time.monotonic()
        found = set()
        topic_types = {}

        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            pairs = self.get_topic_names_and_types()
            topic_types = {name: types for name, types in pairs}
            found = REQUIRED_TOPICS.intersection(topic_types)
            if found == REQUIRED_TOPICS:
                break

        missing = sorted(REQUIRED_TOPICS - found)
        report = {
            "success": not missing,
            "timeout_sec": timeout_sec,
            "required_topics": sorted(REQUIRED_TOPICS),
            "found_required_topics": sorted(found),
            "missing_topics": missing,
            "all_topics": topic_types,
        }
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if not missing else 2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmokeCheck()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
