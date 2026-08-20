"""ROS 2 telemetry bridge and read-only HTTP dashboard for AUTO-17."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import yaml

from .live_state import LiveMissionState
from .snapshot_io import write_text_snapshot


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def build_live_handler(state: LiveMissionState, web_root: Path):
    class Handler(BaseHTTPRequestHandler):
        server_version = "TZcupLiveDemo/1"

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            route = self.path.split("?", 1)[0]
            if route == "/healthz":
                snapshot = state.snapshot()
                self._send_json(
                    200,
                    {
                        "status": "ok",
                        "mission_status": snapshot["status"],
                        "last_update_age_sec": snapshot["last_update_age_sec"],
                    },
                )
                return
            if route == "/api/v1/telemetry":
                self._send_json(200, state.snapshot())
                return
            if route in {"/", "/demo.html"}:
                body = (web_root / "demo.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            self._send_json(404, {"status": "not_found"})

        def log_message(self, _format, *_args):
            return

    return Handler


class LiveDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_live_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("mission_config", "")
        self.declare_parameter("expected_components", 17)

        mission_config = str(self.get_parameter("mission_config").value)
        mission_id = "demo_coverage_001"
        geometry: dict = {}
        if mission_config and Path(mission_config).is_file():
            config = yaml.safe_load(
                Path(mission_config).read_text(encoding="utf-8")
            )
            mission_id = str(config.get("mission_id", mission_id))
            geometry = {
                "outer_polygon": config.get("outer_polygon", []),
                "keepout_polygons": config.get("keepout_polygons", []),
                "exclusion_polygons": config.get("exclusion_polygons", []),
            }

        self.state = LiveMissionState(
            expected_components=int(
                self.get_parameter("expected_components").value
            ),
            mission_id=mission_id,
            geometry=geometry,
        )
        output_value = str(self.get_parameter("output_dir").value).strip()
        self.output_dir = Path(output_value) if output_value else None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(
            String, "/coverage/state", self._on_state, 20
        )
        self.create_subscription(
            String,
            "/coverage/component_state",
            self._on_component_state,
            20,
        )
        self.create_subscription(
            String,
            "/coverage/evaluation_sample",
            self._on_evaluation_sample,
            50,
        )
        self.create_subscription(
            NavPath, "/coverage/current_path", self._on_path, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/fused_pose",
            self._on_estimated_pose,
            20,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 20)
        self.create_subscription(
            Bool, "/brush_enabled", self._on_brush, 20
        )
        self.create_subscription(
            Bool, "/emergency_stop", self._on_emergency_stop, 20
        )
        self.create_timer(1.0, self._write_snapshot)

        web_root = (
            Path(get_package_share_directory("sanitation_hmi")) / "web"
        )
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.server = ThreadingHTTPServer(
            (host, port), build_live_handler(self.state, web_root)
        )
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            name="tzcup-live-dashboard-http",
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            json.dumps(
                {
                    "url": f"http://{host}:{port}",
                    "mode": "read_only_live_demo",
                    "claim_boundary": self.state.snapshot()[
                        "claim_boundary"
                    ],
                },
                ensure_ascii=False,
            )
        )

    def _on_state(self, message: String) -> None:
        self.state.update_state(message.data)

    def _on_component_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"state": message.data}
        self.state.update_component(payload)

    def _on_evaluation_sample(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.state.update_evaluation_sample(
                payload["base_x_m"],
                payload["base_y_m"],
                payload["yaw_rad"],
                brush_enabled=bool(payload.get("brush_enabled")),
                coverage_state=payload.get("coverage_state"),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                "ignored malformed /coverage/evaluation_sample"
            )

    def _on_path(self, message: NavPath) -> None:
        self.state.update_planned_path(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in message.poses
            ]
        )

    def _on_estimated_pose(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        pose = message.pose.pose
        self.state.update_estimated_pose(
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
        )

    def _on_velocity(self, message: Twist) -> None:
        self.state.update_velocity(message.linear.x, message.angular.z)

    def _on_brush(self, message: Bool) -> None:
        self.state.update_brush(message.data)

    def _on_emergency_stop(self, message: Bool) -> None:
        self.state.update_emergency_stop(message.data)

    def _write_snapshot(self) -> None:
        if self.output_dir is None:
            return
        target = self.output_dir / "dashboard_telemetry.json"
        written = write_text_snapshot(
            target,
            json.dumps(
                self.state.snapshot(), ensure_ascii=False, indent=2
            )
            + "\n",
        )
        if not written:
            self.get_logger().warning(
                "dashboard snapshot replace blocked by a transient file lock; "
                "the live server remains active and will retry on the next tick"
            )

    def destroy_node(self):
        self._write_snapshot()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
