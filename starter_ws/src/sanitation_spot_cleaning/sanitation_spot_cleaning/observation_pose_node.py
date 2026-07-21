from __future__ import annotations

import json
import math
import threading

from .observation_pose_planner import (
    CandidateRegion,
    ObservationPosePlanner,
    PlannerConstraints,
    Pose2D,
    VerificationCameraModel,
)


def main() -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class ObservationPoseNode(Node):
        def __init__(self):
            super().__init__("stage5br5_observation_pose_planner")
            self.declare_parameter("cleanable_polygon_json", "[[-9,-3],[9,-3],[9,3],[-9,3]]")
            self.declare_parameter("keepout_polygons_json", "[]")
            self.declare_parameter("camera_xyz_m", [0.67, 0.0, 0.48])
            self.declare_parameter("camera_pitch_deg", -50.0)
            self.declare_parameter("camera_mount_rpy_deg", [0.0, -50.0, 0.0])
            self.declare_parameter("camera_self_pixel_fraction", 1.0)
            self.declare_parameter("camera_target_self_overlap", 1.0)
            self.declare_parameter("camera_info_topic", "/verification_camera/color/camera_info")
            self.declare_parameter("candidate_footprint_json", "[]")
            self.declare_parameter("self_mask_roi_xyxy_json", "[]")
            self.declare_parameter("engineering_mode", False)
            self.declare_parameter("compute_path_timeout_s", 4.0)
            self.declare_parameter("global_frame", "map")
            self.declare_parameter("base_frame", "base_footprint")
            engineering_mode = bool(self.get_parameter("engineering_mode").value)
            self.planner = ObservationPosePlanner(PlannerConstraints(
                require_polygon_checks=engineering_mode,
                require_costmap_footprint_cost=engineering_mode,
                require_pose_dependent_self_overlap=engineering_mode,
            ))
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
            self.pose_publisher = self.create_publisher(PoseStamped, "/active_observation/selected_pose", 10)
            self.status_publisher = self.create_publisher(String, "/active_observation/pose_plan", 10)
            self.create_subscription(String, "/active_observation/candidate", self.on_candidate, 10)
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                10,
            )
            self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, 10)
            self.camera_info = None
            self.global_costmap = None
            self._busy = threading.Lock()

        def _on_camera_info(self, message):
            self.camera_info = message

        def _on_costmap(self, message):
            self.global_costmap = message

        @staticmethod
        def _point_in_polygon(point, polygon):
            x, y = point
            inside = False
            for index, first in enumerate(polygon):
                second = polygon[(index + 1) % len(polygon)]
                if (first[1] > y) != (second[1] > y):
                    crossing = (second[0] - first[0]) * (y - first[1]) / (second[1] - first[1]) + first[0]
                    if x < crossing:
                        inside = not inside
            return inside

        def _costmap_footprint_cost(self, _pose, polygon):
            message = self.global_costmap
            if message is None or len(polygon) < 3:
                return None
            resolution = float(message.info.resolution)
            origin_x = float(message.info.origin.position.x)
            origin_y = float(message.info.origin.position.y)
            min_x, max_x = min(p[0] for p in polygon), max(p[0] for p in polygon)
            min_y, max_y = min(p[1] for p in polygon), max(p[1] for p in polygon)
            x0 = max(0, math.floor((min_x - origin_x) / resolution))
            x1 = min(message.info.width - 1, math.ceil((max_x - origin_x) / resolution))
            y0 = max(0, math.floor((min_y - origin_y) / resolution))
            y1 = min(message.info.height - 1, math.ceil((max_y - origin_y) / resolution))
            if x0 > x1 or y0 > y1:
                return None
            costs = []
            for row in range(y0, y1 + 1):
                for column in range(x0, x1 + 1):
                    point = (origin_x + (column + 0.5) * resolution, origin_y + (row + 0.5) * resolution)
                    if not self._point_in_polygon(point, polygon):
                        continue
                    raw = int(message.data[row * message.info.width + column])
                    costs.append(255.0 if raw < 0 else 2.54 * raw)
            return max(costs) if costs else None

        def _self_overlap(self, _pose, target_roi):
            self_fraction = float(self.get_parameter("camera_self_pixel_fraction").value)
            self_roi = json.loads(str(self.get_parameter("self_mask_roi_xyxy_json").value))
            if len(self_roi) != 4:
                return self_fraction, 0.0
            left, top = max(target_roi[0], self_roi[0]), max(target_roi[1], self_roi[1])
            right, bottom = min(target_roi[2], self_roi[2]), min(target_roi[3], self_roi[3])
            intersection = max(0.0, right - left) * max(0.0, bottom - top)
            target_area = max(1e-9, (target_roi[2] - target_roi[0]) * (target_roi[3] - target_roi[1]))
            return self_fraction, intersection / target_area

        def on_candidate(self, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError) as error:
                self.status_publisher.publish(String(data=json.dumps({"accepted": False, "reason": f"invalid_candidate_json:{error}"})))
                return
            if not self._busy.acquire(blocking=False):
                self.status_publisher.publish(String(data=json.dumps({"accepted": False, "reason": "planner_busy"})))
                return
            threading.Thread(target=self.plan_candidate, args=(payload,), daemon=True).start()

        def _current_pose(self) -> Pose2D:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("global_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(),
            ).transform
            yaw = 2.0 * math.atan2(transform.rotation.z, transform.rotation.w)
            return Pose2D(transform.translation.x, transform.translation.y, yaw)

        def _compute_path(self, pose: Pose2D):
            if not self.path_client.wait_for_server(timeout_sec=1.0):
                return None
            frame = str(self.get_parameter("global_frame").value)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = frame
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose.x
            goal_pose.pose.position.y = pose.y
            goal_pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
            goal = ComputePathToPose.Goal()
            goal.goal = goal_pose
            goal.use_start = False
            completed = threading.Event()
            result_holder = {}

            def result_done(future):
                try:
                    response = future.result()
                    path_message = response.result.path
                    result_holder["path"] = tuple(
                        Pose2D(
                            item.pose.position.x,
                            item.pose.position.y,
                            2.0 * math.atan2(item.pose.orientation.z, item.pose.orientation.w),
                        )
                        for item in path_message.poses
                    )
                except Exception as error:  # ROS action failures are fail-closed.
                    result_holder["error"] = str(error)
                completed.set()

            def goal_done(future):
                try:
                    handle = future.result()
                    if not handle.accepted:
                        completed.set()
                        return
                    handle.get_result_async().add_done_callback(result_done)
                except Exception as error:
                    result_holder["error"] = str(error)
                    completed.set()

            self.path_client.send_goal_async(goal).add_done_callback(goal_done)
            completed.wait(float(self.get_parameter("compute_path_timeout_s").value))
            path = result_holder.get("path")
            return path if path and len(path) >= 2 else None

        def plan_candidate(self, payload: dict):
            try:
                camera_xyz = tuple(float(value) for value in self.get_parameter("camera_xyz_m").value)
                engineering_mode = bool(self.get_parameter("engineering_mode").value)
                if engineering_mode and self.camera_info is None:
                    raise RuntimeError("actual_camera_info_unavailable")
                info = self.camera_info
                mount_rpy = tuple(math.radians(float(value)) for value in self.get_parameter("camera_mount_rpy_deg").value)
                camera = VerificationCameraModel(
                    width_px=int(info.width) if info is not None else 640,
                    height_px=int(info.height) if info is not None else 480,
                    horizontal_fov_rad=1.50098,
                    mount_xyz_m=camera_xyz,
                    pitch_rad=math.radians(float(self.get_parameter("camera_pitch_deg").value)),
                    predicted_self_pixel_fraction=float(self.get_parameter("camera_self_pixel_fraction").value),
                    predicted_target_self_overlap=float(self.get_parameter("camera_target_self_overlap").value),
                    mount_rpy_rad=mount_rpy,
                    fx_px=float(info.k[0]) if info is not None else None,
                    fy_px=float(info.k[4]) if info is not None else None,
                    cx_px=float(info.k[2]) if info is not None else None,
                    cy_px=float(info.k[5]) if info is not None else None,
                )
                region = CandidateRegion(
                    candidate_id=str(payload["candidate_id"]),
                    center_xy_m=(float(payload["x_m"]), float(payload["y_m"])),
                    target_size_m=float(payload["target_size_m"]),
                    class_id=str(payload["class_id"]),
                )
                result = self.planner.plan(
                    region=region,
                    covariance_trace=float(payload["covariance_trace"]),
                    camera=camera,
                    cleanable_polygon=json.loads(str(self.get_parameter("cleanable_polygon_json").value)),
                    keepout_polygons=json.loads(str(self.get_parameter("keepout_polygons_json").value)),
                    current_pose=self._current_pose(),
                    compute_path=self._compute_path,
                    candidate_footprint=json.loads(str(self.get_parameter("candidate_footprint_json").value)),
                    footprint_cost=self._costmap_footprint_cost if engineering_mode else None,
                    self_overlap_estimator=self._self_overlap if engineering_mode else None,
                )
                if result is None:
                    self.status_publisher.publish(String(data=json.dumps({
                        "candidate_id": region.candidate_id,
                        "accepted": False,
                        "reason": "UNREACHABLE:no_reachable_visible_observation_pose",
                        "ground_truth_pose_used": False,
                    }, sort_keys=True)))
                    return
                pose = PoseStamped()
                pose.header.frame_id = str(self.get_parameter("global_frame").value)
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = result.pose.x
                pose.pose.position.y = result.pose.y
                pose.pose.orientation.z = math.sin(result.pose.yaw / 2.0)
                pose.pose.orientation.w = math.cos(result.pose.yaw / 2.0)
                self.pose_publisher.publish(pose)
                record = result.to_record()
                record.update({"candidate_id": region.candidate_id, "accepted": True, "ground_truth_pose_used": False})
                self.status_publisher.publish(String(data=json.dumps(record, sort_keys=True)))
            except Exception as error:
                self.status_publisher.publish(String(data=json.dumps({"accepted": False, "reason": f"planner_exception:{error}"})))
            finally:
                self._busy.release()

    rclpy.init()
    node = ObservationPoseNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
