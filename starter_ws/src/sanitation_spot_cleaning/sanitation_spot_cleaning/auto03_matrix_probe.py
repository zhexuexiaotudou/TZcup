from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import threading
import time

import numpy as np

from .active_observation import (
    ActiveObservationCoordinator,
    ObservationPreflight,
    ObservationState,
)
from .auto03_contract import projection_measurement, validate_oracle_candidate
from .observation_pose_planner import (
    CandidateRegion,
    ObservationPosePlanner,
    Pose2D,
    VerificationCameraModel,
)


def _stamp_s(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _angular_distance(first: float, second: float) -> float:
    return abs((float(first) - float(second) + math.pi) % (2.0 * math.pi) - math.pi)


def _image_rgb(message) -> np.ndarray:
    array = np.frombuffer(bytes(message.data), dtype=np.uint8)
    return array.reshape((int(message.height), int(message.step)))[:, : int(message.width) * 3].reshape(
        (int(message.height), int(message.width), 3)
    )


def _set_pose_vector(world_id: str, poses: list[dict]) -> None:
    if any(item["name"] == "sanitation_vehicle" for item in poses):
        raise RuntimeError("AUTO-03 oracle is forbidden from setting the robot pose")
    request = " ".join(
        "pose { "
        f'name: "{item["name"]}" position {{ x: {item["xyz"][0]:.6f} y: {item["xyz"][1]:.6f} z: {item["xyz"][2]:.6f} }} '
        f'orientation {{ z: {math.sin(item["yaw"] / 2):.8f} w: {math.cos(item["yaw"] / 2):.8f} }} }}'
        for item in poses
    )
    failures = []
    for attempt in range(1, 5):
        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", f"/world/{world_id}/set_pose_vector",
                    "--reqtype", "gz.msgs.Pose_V", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "10000", "--req", request,
                ],
                capture_output=True,
                text=True,
                timeout=20,
            )
            if result.returncode == 0 and "data: true" in result.stdout:
                return
            failures.append(
                f"attempt={attempt} rc={result.returncode} "
                f"stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
            )
        except subprocess.TimeoutExpired as error:
            failures.append(f"attempt={attempt} subprocess_timeout={error.timeout}")
        if attempt < 4:
            time.sleep(float(attempt))
    raise RuntimeError("set_pose_vector failed after retries: " + " | ".join(failures))


def main() -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose, Spin
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    class OracleSceneSource(Node):
        """The only node allowed to read scene truth; publishes the narrow Oracle contract."""

        def __init__(self):
            super().__init__("auto03_oracle_scene_source")
            self.declare_parameter("matrix_path", "")
            self.declare_parameter("world_id", "")
            self.declare_parameter("output_path", "/tmp/auto03_runtime_trials.json")
            self.declare_parameter("max_trials", 0)
            self.declare_parameter("trial_offset", 0)
            self.declare_parameter("source_start_delay_s", 1.0)
            matrix_path = Path(str(self.get_parameter("matrix_path").value))
            self.matrix = json.loads(matrix_path.read_text(encoding="utf-8"))
            self.world_id = str(self.get_parameter("world_id").value)
            self.trials = [
                item for item in self.matrix["trials"] if item["world_id"] == self.world_id
            ]
            trial_offset = int(self.get_parameter("trial_offset").value)
            if trial_offset < 0:
                raise ValueError("trial_offset must be non-negative")
            self.trials = self.trials[trial_offset:]
            max_trials = int(self.get_parameter("max_trials").value)
            if max_trials > 0:
                self.trials = self.trials[:max_trials]
            if not self.trials:
                raise RuntimeError(f"matrix has no trials for {self.world_id}")
            self.candidate_pub = self.create_publisher(String, "/auto03/oracle_candidate", 10)
            self.done_pub = self.create_publisher(Bool, "/auto03/done", 1)
            self.create_subscription(String, "/auto03/trial_result", self._on_result, 20)
            self.results = []
            self.waiting_id = None
            self.failed = None
            self._condition = threading.Condition()
            self._worker = threading.Thread(target=self._run, daemon=True)
            self._start_not_before = (
                time.monotonic()
                + float(self.get_parameter("source_start_delay_s").value)
            )
            self.create_timer(1.0, self._start_once)

        def _start_once(self):
            if (
                time.monotonic() >= self._start_not_before
                and not self._worker.is_alive()
                and not self.results
                and self.failed is None
            ):
                self._worker.start()

        def _on_result(self, message):
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            with self._condition:
                if payload.get("candidate_id") != self.waiting_id:
                    return
                self.results.append(payload)
                self.waiting_id = None
                self._write()
                self._condition.notify_all()

        def _publish_candidate(self, trial):
            candidate = dict(trial["oracle_candidate"])
            now_s = self.get_clock().now().nanoseconds * 1e-9
            candidate["timestamp_s"] = (
                now_s - 5.0 if trial["case_type"] == "stale_dropout" else now_s
            )
            candidate = validate_oracle_candidate(candidate)
            self.waiting_id = candidate["candidate_id"]
            self.candidate_pub.publish(String(data=json.dumps(candidate, separators=(",", ":"))))

        def _run(self):
            try:
                hidden = [
                    {"name": name, "xyz": [-200.0 - index * 0.25, 200.0, -5.0], "yaw": 0.0}
                    for index, name in enumerate(self.matrix["all_model_names"])
                ]
                _set_pose_vector(self.world_id, hidden)
                previous = None
                for trial in self.trials:
                    updates = []
                    if previous:
                        updates.append({"name": previous, "xyz": [-250.0, 250.0, -5.0], "yaw": 0.0})
                    if trial["active_model_name"]:
                        updates.append({
                            "name": trial["active_model_name"],
                            "xyz": trial["active_model_world_xyz_m"],
                            "yaw": trial["active_model_yaw_rad"],
                        })
                    if updates:
                        _set_pose_vector(self.world_id, updates)
                    previous = trial["active_model_name"]
                    time.sleep(0.20)
                    with self._condition:
                        self._publish_candidate(trial)
                        deadline = time.monotonic() + 180.0
                        while self.waiting_id is not None and time.monotonic() < deadline:
                            self._condition.wait(timeout=1.0)
                        if self.waiting_id is not None:
                            raise RuntimeError(f"trial timeout: {self.waiting_id}")
                self._write()
                self.done_pub.publish(Bool(data=True))
            except Exception as error:
                self.failed = str(error)
                self._write()
                self.done_pub.publish(Bool(data=False))

        def _write(self):
            output = Path(str(self.get_parameter("output_path").value))
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps({
                "schema_version": 1,
                "stage": "AUTO-03",
                "world_id": self.world_id,
                "oracle_source_node": self.get_name(),
                "oracle_published_fields": sorted(validate_oracle_candidate(
                    {**self.trials[0]["oracle_candidate"], "timestamp_s": 0.0}
                )),
                "robot_pose_set_by_oracle": False,
                "result_count": len(self.results),
                "expected_count": len(self.trials),
                "error": self.failed,
                "trials": self.results,
                "runtime_complete": self.failed is None and len(self.results) == len(self.trials),
            }, indent=2) + "\n", encoding="utf-8")

    class MachineReadyEvaluator(Node):
        """Evaluation-only GT subscriber. Its outputs never select a pose or goal."""

        def __init__(self):
            super().__init__("auto03_machine_ready_evaluator")
            self.declare_parameter("matrix_path", "")
            self.declare_parameter("world_id", "")
            matrix = json.loads(
                Path(str(self.get_parameter("matrix_path").value)).read_text(encoding="utf-8")
            )
            world_id = str(self.get_parameter("world_id").value)
            self.truth = {
                item["candidate_id"]: item for item in matrix["trials"] if item["world_id"] == world_id
            }
            self.rgb = {}
            self.semantic = {}
            self.pending = {}
            self.result_pub = self.create_publisher(String, "/auto03/machine_ready_result", 20)
            self.create_subscription(
                Image, "/verification_camera/color/image_raw", self._on_rgb, 10
            )
            self.create_subscription(
                Image, "/ground_truth/verification_semantic/image", self._on_semantic, 10
            )
            self.create_subscription(String, "/auto03/capture_request", self._on_request, 20)

        @staticmethod
        def _trim(bucket):
            while len(bucket) > 30:
                bucket.pop(next(iter(bucket)))

        def _on_rgb(self, message):
            self.rgb[_stamp_s(message)] = message
            self._trim(self.rgb)
            self._try_evaluate()

        def _on_semantic(self, message):
            self.semantic[_stamp_s(message)] = message
            self._trim(self.semantic)
            self._try_evaluate()

        def _on_request(self, message):
            payload = json.loads(message.data)
            self.pending[str(payload["candidate_id"])] = payload
            self._try_evaluate()

        def _try_evaluate(self):
            common = sorted(set(self.rgb) & set(self.semantic))
            if not common:
                return
            for candidate_id, request in list(self.pending.items()):
                stamps = [stamp for stamp in common if stamp >= float(request["requested_at_s"])]
                if not stamps:
                    continue
                stamp = stamps[0]
                semantic = _image_rgb(self.semantic[stamp])[:, :, 0]
                truth = self.truth[candidate_id]
                target_mask = semantic == int(truth["semantic_label"])
                self_mask = semantic == 250
                self_bbox = None
                if self_mask.any():
                    self_ys, self_xs = np.nonzero(self_mask)
                    self_bbox = [
                        int(self_xs.min()), int(self_ys.min()),
                        int(self_xs.max() - self_xs.min() + 1),
                        int(self_ys.max() - self_ys.min() + 1),
                    ]
                bbox = None
                if target_mask.any():
                    ys, xs = np.nonzero(target_mask)
                    bbox = [
                        int(xs.min()), int(ys.min()),
                        int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1),
                    ]
                roi = request["expected_roi_xyxy"]
                if bbox is not None:
                    projection = projection_measurement(
                        roi,
                        bbox,
                        predicted_target_short_side_px=request[
                            "expected_target_short_side_px"
                        ],
                    )
                    projection["search_roi_padding_px"] = float(
                        request["search_roi_padding_px"]
                    )
                    x, y, width, height = bbox
                    overlap = float(self_mask[y:y + height, x:x + width].mean())
                    actual_ready = (
                        min(width, height) >= 12
                        and float(self_mask.mean()) <= 0.05
                        and overlap <= 0.05
                    )
                else:
                    projection = None
                    overlap = 0.0
                    actual_ready = bool(truth["case_type"] == "false_candidate")
                confirmed = bool(target_mask.any() and truth["case_type"] == "reachable")
                result = {
                    "candidate_id": candidate_id,
                    "capture_stamp_s": stamp,
                    "rgb_frame_received": bool(self.rgb[stamp].data),
                    "semantic_frame_received": bool(self.semantic[stamp].data),
                    "actual_bbox_xywh": bbox,
                    "projection": projection,
                    "self_pixel_fraction": float(self_mask.mean()),
                    "self_bbox_xywh": self_bbox,
                    "target_self_overlap": overlap,
                    "actual_ready": actual_ready,
                    "confirmed": confirmed,
                    "gt_used_for_pose_or_navigation": False,
                    "evaluation_only_node": self.get_name(),
                }
                self.result_pub.publish(String(data=json.dumps(result, separators=(",", ":"))))
                del self.pending[candidate_id]

    class ObservationExecutive(Node):
        """Consumes only the sanitized candidate and actual navigation/sensor interfaces."""

        def __init__(self):
            super().__init__("auto03_observation_executive")
            self.declare_parameter("sensor_stale_s", 2.0)
            self.declare_parameter("navigation_timeout_s", 45.0)
            self.declare_parameter("capture_timeout_s", 5.0)
            self.coordinator = ActiveObservationCoordinator(
                maximum_approaches=1,
                sensor_stale_s=float(self.get_parameter("sensor_stale_s").value),
                queue_timeout_s=120.0,
                approach_timeout_s=20.0,
                minimum_approach_speed_mps=0.10,
                minimum_clearance_m=0.15,
                maximum_covariance_trace=0.03,
            )
            self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.spin_client = ActionClient(self, Spin, "/spin")
            self.candidate_pub = self.create_publisher(String, "/active_observation/candidate", 10)
            self.capture_pub = self.create_publisher(String, "/auto03/capture_request", 20)
            self.result_pub = self.create_publisher(String, "/auto03/trial_result", 20)
            self.coverage_pub = self.create_publisher(String, "/coverage/state", 20)
            self.brush_pub = self.create_publisher(Bool, "/brush_enabled", 20)
            self.create_subscription(String, "/auto03/oracle_candidate", self._on_candidate, 20)
            self.create_subscription(String, "/active_observation/pose_plan", self._on_pose_plan, 20)
            self.create_subscription(String, "/auto03/machine_ready_result", self._on_evaluation, 20)
            self.create_subscription(PoseWithCovarianceStamped, "/localization/fused_pose", self._on_pose, 20)
            self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self.create_subscription(LaserScan, "/scan", self._on_scan, 20)
            self.create_subscription(
                CameraInfo,
                "/verification_camera/color/camera_info",
                lambda message: setattr(self, "_camera_info", message),
                10,
            )
            self._camera_info = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._condition = threading.Condition()
            self._pose_plans = {}
            self._evaluations = {}
            self._busy = threading.Lock()
            self._active_id = None
            self._distance_m = 0.0
            self._last_odom = None
            self._collision_count = 0
            self._collision_active = False
            self._keepout_violations = 0

        def _on_pose(self, message):
            if self._active_id is None:
                return
            x = float(message.pose.pose.position.x)
            y = float(message.pose.pose.position.y)
            if 2.0 <= x <= 4.0 and 1.0 <= y <= 3.0:
                self._keepout_violations += 1

        def _on_odom(self, message):
            if self._active_id is None:
                self._last_odom = None
                return
            point = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))
            if self._last_odom is not None:
                self._distance_m += math.dist(point, self._last_odom)
            self._last_odom = point

        def _on_scan(self, message):
            finite = [float(value) for value in message.ranges if math.isfinite(value)]
            if not finite:
                return
            active = min(finite) < 0.12
            if self._active_id is not None and active and not self._collision_active:
                self._collision_count += 1
            self._collision_active = active

        def _on_pose_plan(self, message):
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            candidate_id = payload.get("candidate_id")
            if candidate_id:
                with self._condition:
                    self._pose_plans[str(candidate_id)] = payload
                    self._condition.notify_all()

        def _on_evaluation(self, message):
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            with self._condition:
                self._evaluations[str(payload["candidate_id"])] = payload
                self._condition.notify_all()

        def _on_candidate(self, message):
            try:
                payload = validate_oracle_candidate(json.loads(message.data))
            except (ValueError, json.JSONDecodeError) as error:
                self.get_logger().error(f"candidate rejected: {error}")
                return
            if not self._busy.acquire(blocking=False):
                self.get_logger().error("candidate arrived while executive busy")
                return
            threading.Thread(target=self._execute, args=(payload,), daemon=True).start()

        def _current_pose(self):
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            ).transform
            yaw = 2.0 * math.atan2(transform.rotation.z, transform.rotation.w)
            return (
                float(transform.translation.x),
                float(transform.translation.y),
                yaw,
            )

        @staticmethod
        def _pose_message(pose):
            message = PoseStamped()
            message.header.frame_id = "map"
            message.pose.position.x = float(pose[0])
            message.pose.position.y = float(pose[1])
            message.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
            message.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
            return message

        @staticmethod
        def _cancel_goal(handle, result_event):
            if handle is None:
                return
            cancel_event = threading.Event()
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(lambda _future: cancel_event.set())
                cancel_event.wait(3.0)
                result_event.wait(3.0)
            except Exception:
                pass

        def _sim_now_s(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _navigate(
            self,
            pose,
            proximity_success_m=None,
            near_goal_yaw_handoff_m=None,
            timeout_s=None,
        ):
            started_wall = time.monotonic()
            started_sim = self._sim_now_s()
            if not self.navigate_client.wait_for_server(timeout_sec=10.0):
                return (
                    False,
                    "navigate_server_unavailable",
                    self._sim_now_s() - started_sim,
                )
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_message(pose)
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            sent = self.navigate_client.send_goal_async(goal)
            event = threading.Event()
            result = {}

            def result_done(future):
                try:
                    wrapped = future.result()
                    result["success"] = (
                        wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and int(getattr(wrapped.result, "error_code", 0)) == 0
                    )
                    result["error_code"] = int(getattr(wrapped.result, "error_code", 0))
                except Exception as error:
                    result["error"] = str(error)
                event.set()

            def goal_done(future):
                try:
                    handle = future.result()
                    if not handle.accepted:
                        result["error"] = "goal_rejected"
                        event.set()
                        return
                    result["handle"] = handle
                    handle.get_result_async().add_done_callback(result_done)
                except Exception as error:
                    result["error"] = str(error)
                    event.set()

            sent.add_done_callback(goal_done)
            timeout = (
                float(self.get_parameter("navigation_timeout_s").value)
                if timeout_s is None
                else float(timeout_s)
            )
            sim_deadline = started_sim + timeout
            wall_deadline = started_wall + max(60.0, timeout * 10.0)
            while not event.wait(0.10):
                if (
                    self._sim_now_s() >= sim_deadline
                    or time.monotonic() >= wall_deadline
                ):
                    break
                if (
                    near_goal_yaw_handoff_m is not None
                    and result.get("handle") is not None
                    and self._sim_now_s() - started_sim >= 3.0
                ):
                    try:
                        current = self._current_pose()
                        if (
                            math.dist(current[:2], pose[:2])
                            <= near_goal_yaw_handoff_m
                            and _angular_distance(current[2], pose[2]) > 0.50
                        ):
                            self._cancel_goal(result.get("handle"), event)
                            return (
                                False,
                                "near_goal_yaw_handoff",
                                self._sim_now_s() - started_sim,
                            )
                    except Exception:
                        pass
                if proximity_success_m is not None:
                    try:
                        if math.dist(self._current_pose()[:2], pose[:2]) <= proximity_success_m:
                            self._cancel_goal(result.get("handle"), event)
                            return (
                                True,
                                "boundary_proximity_success",
                                self._sim_now_s() - started_sim,
                            )
                    except Exception:
                        pass
            if not event.is_set():
                self._cancel_goal(result.get("handle"), event)
                return (
                    False,
                    "navigation_timeout",
                    self._sim_now_s() - started_sim,
                )
            return (
                bool(result.get("success")),
                str(result.get("error", result.get("error_code", 0))),
                self._sim_now_s() - started_sim,
            )

        def _spin_to_yaw(self, target_yaw, timeout_s=20.0):
            started_wall = time.monotonic()
            started_sim = self._sim_now_s()
            if not self.spin_client.wait_for_server(timeout_sec=10.0):
                return (
                    False,
                    "spin_server_unavailable",
                    self._sim_now_s() - started_sim,
                )
            current = self._current_pose()
            relative_yaw = (
                float(target_yaw) - float(current[2]) + math.pi
            ) % (2.0 * math.pi) - math.pi
            if abs(relative_yaw) <= 0.30:
                return (
                    True,
                    "already_aligned",
                    self._sim_now_s() - started_sim,
                )

            goal = Spin.Goal()
            goal.target_yaw = float(relative_yaw)
            goal.time_allowance.sec = int(math.ceil(float(timeout_s)))
            sent = self.spin_client.send_goal_async(goal)
            event = threading.Event()
            result = {}

            def result_done(future):
                try:
                    wrapped = future.result()
                    result["success"] = (
                        wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and int(getattr(wrapped.result, "error_code", 0)) == 0
                    )
                    result["error_code"] = int(
                        getattr(wrapped.result, "error_code", 0)
                    )
                except Exception as error:
                    result["error"] = str(error)
                event.set()

            def goal_done(future):
                try:
                    handle = future.result()
                    if not handle.accepted:
                        result["error"] = "goal_rejected"
                        event.set()
                        return
                    result["handle"] = handle
                    handle.get_result_async().add_done_callback(result_done)
                except Exception as error:
                    result["error"] = str(error)
                    event.set()

            sent.add_done_callback(goal_done)
            sim_deadline = started_sim + float(timeout_s)
            wall_deadline = started_wall + max(
                60.0, float(timeout_s) * 10.0
            )
            while not event.wait(0.10):
                if (
                    self._sim_now_s() >= sim_deadline
                    or time.monotonic() >= wall_deadline
                ):
                    break
            if not event.is_set():
                self._cancel_goal(result.get("handle"), event)
                return (
                    False,
                    "spin_timeout",
                    self._sim_now_s() - started_sim,
                )
            return (
                bool(result.get("success")),
                str(result.get("error", result.get("error_code", 0))),
                self._sim_now_s() - started_sim,
            )

        def _wait_for(self, bucket, key, timeout):
            with self._condition:
                deadline = time.monotonic() + timeout
                while key not in bucket and time.monotonic() < deadline:
                    self._condition.wait(timeout=0.25)
                return bucket.pop(key, None)

        def _capture_projection(self, candidate):
            info = self._camera_info
            if info is None:
                return None, None, None
            current = self._current_pose()
            camera = VerificationCameraModel(
                width_px=int(info.width),
                height_px=int(info.height),
                horizontal_fov_rad=1.50098,
                mount_xyz_m=(0.32, 0.28, 0.66),
                pitch_rad=math.radians(-35.0),
                predicted_self_pixel_fraction=0.0,
                predicted_target_self_overlap=0.0,
                mount_rpy_rad=(0.0, math.radians(35.0), math.radians(45.0)),
                fx_px=float(info.k[0]),
                fy_px=float(info.k[4]),
                cx_px=float(info.k[2]),
                cy_px=float(info.k[5]),
                projection_center_affine=(
                    0.9200577497930318,
                    0.19807390392387791,
                    -6.743277535198672,
                    -0.009249842484302759,
                    1.2094590560977079,
                    -1.5324689495967263,
                ),
                class_projection_calibration=(
                    (
                        "plastic_bottle",
                        3.3742821866035477,
                        -22.005209523783325,
                        1.297,
                    ),
                    (
                        "metal_can",
                        0.6257472835344164,
                        -16.946553085057214,
                        1.285,
                    ),
                    (
                        "paper_litter",
                        -4.061201371314579,
                        -4.3156677050183445,
                        0.920,
                    ),
                    ("leaf_pile", 0.0, 0.0, 0.695),
                    (
                        "puddle",
                        1.4460236390133452,
                        4.448860887459424,
                        0.751,
                    ),
                ),
                class_short_side_correction=(
                    (
                        "plastic_bottle",
                        0.520954678843004,
                        0.5125023267986658,
                        -0.09219435860629135,
                        -0.1326499293653301,
                        1.3135403850862895,
                    ),
                    (
                        "metal_can",
                        0.6475405866470701,
                        0.30678977197920954,
                        -1.2688165105403737,
                        4.400331288832601,
                        -7.904909757041143,
                    ),
                    (
                        "paper_litter",
                        1.1724995991027891,
                        -0.06145769893108791,
                        0.4392835224074717,
                        -0.002902544199251083,
                        -0.19576236688704346,
                    ),
                    (
                        "leaf_pile",
                        1.4655016083716128,
                        0.0017350097960302125,
                        0.7066249553725945,
                        -0.3887994611022482,
                        0.12984499130263635,
                    ),
                    (
                        "puddle",
                        1.818205185685996,
                        -0.04755093852330815,
                        0.7917667072685282,
                        -0.45137859584505613,
                        0.0,
                    ),
                ),
                projection_roi_margin_px=15.0,
            )
            region = CandidateRegion(
                candidate_id=candidate["candidate_id"],
                center_xy_m=(candidate["x_m"], candidate["y_m"]),
                target_size_m=candidate["target_size_m"],
                class_id=candidate["class_id"],
            )
            short, roi, _angle, visible = ObservationPosePlanner._camera_projection(
                region,
                Pose2D(*current),
                camera,
            )
            return roi if visible else None, short, current

        def _execute(self, candidate):
            candidate_id = candidate["candidate_id"]
            started = self._sim_now_s()
            self._active_id = candidate_id
            self._distance_m = 0.0
            self._last_odom = None
            self._collision_count = 0
            self._keepout_violations = 0
            result = {
                "candidate_id": candidate_id,
                "coverage_boundary_pause_safe": True,
                "preflight_path_success": False,
                "navigation_attempted": False,
                "navigate_success": False,
                "capture_completed": False,
                "coverage_resumed": False,
                "cleaning_commanded": False,
                "gt_control_violation_count": 0,
            }
            try:
                self.brush_pub.publish(Bool(data=False))
                self.coverage_pub.publish(String(data=json.dumps({
                    "state": "PAUSED_AT_COMPONENT_BOUNDARY",
                    "candidate_id": candidate_id,
                    "brush_enabled": False,
                })))
                now_s = self.get_clock().now().nanoseconds * 1e-9
                task = self.coordinator.discover(
                    candidate_id,
                    candidate["timestamp_s"],
                    (candidate["x_m"], candidate["y_m"]),
                )
                if now_s - candidate["timestamp_s"] > float(self.get_parameter("sensor_stale_s").value):
                    task = self.coordinator.preflight(candidate_id, now_s, ObservationPreflight(
                        at_component_boundary=True,
                        path_available=False,
                        keepout_clear=False,
                        footprint_clearance_m=0.0,
                        visibility_expected=False,
                        covariance_trace=candidate["covariance_trace"],
                    ))
                    result.update({
                        "terminal_state": task.state.value,
                        "terminal_reason": task.terminal_reason,
                        "coverage_resumed": True,
                    })
                    return

                boundary_pose = self._current_pose()
                result["coverage_boundary_pose_map"] = list(boundary_pose)
                self.candidate_pub.publish(String(data=json.dumps(candidate, separators=(",", ":"))))
                plan = self._wait_for(self._pose_plans, candidate_id, 30.0)
                accepted = bool(plan and plan.get("accepted"))
                task = self.coordinator.preflight(candidate_id, now_s, ObservationPreflight(
                    at_component_boundary=True,
                    path_available=accepted,
                    keepout_clear=accepted,
                    footprint_clearance_m=float(plan.get("clearance_m", 0.0)) if plan else 0.0,
                    visibility_expected=bool(plan.get("visibility_expected", False)) if plan else False,
                    covariance_trace=candidate["covariance_trace"],
                    path_length_m=float(plan.get("path_length_m", 0.0)) if plan else 0.0,
                ))
                if not accepted:
                    result.update({
                        "terminal_state": task.state.value,
                        "terminal_reason": task.terminal_reason or (plan or {}).get("reason", "planner_timeout"),
                        "coverage_resumed": True,
                    })
                    return

                result["preflight_path_success"] = task.state == ObservationState.APPROACHING
                result["navigation_attempted"] = True
                goal_pose = plan["pose"]
                result["planned_observation_pose_map"] = [
                    goal_pose["x"], goal_pose["y"], goal_pose["yaw"]
                ]
                current_for_path = self._current_pose()
                path_heading = math.atan2(
                    goal_pose["y"] - current_for_path[1],
                    goal_pose["x"] - current_for_path[0],
                )
                if (
                    _angular_distance(current_for_path[2], path_heading)
                    > 0.80
                ):
                    prealign_success, prealign_detail, _ = (
                        self._spin_to_yaw(path_heading, timeout_s=20.0)
                    )
                    result["navigation_pre_alignment_attempted"] = True
                    result["navigation_pre_alignment_success"] = (
                        prealign_success
                    )
                    result["navigation_pre_alignment_detail"] = (
                        prealign_detail
                    )
                success, navigation_detail, _ = self._navigate(
                    (goal_pose["x"], goal_pose["y"], goal_pose["yaw"]),
                    near_goal_yaw_handoff_m=0.28,
                )
                if not success and navigation_detail == "near_goal_yaw_handoff":
                    spin_success, spin_detail, _ = self._spin_to_yaw(
                        goal_pose["yaw"],
                        timeout_s=20.0,
                    )
                    current_after_spin = self._current_pose()
                    retry_xy = (
                        current_after_spin[:2]
                        if spin_success
                        else (goal_pose["x"], goal_pose["y"])
                    )
                    success, retry_detail, _ = self._navigate(
                        (
                            retry_xy[0],
                            retry_xy[1],
                            goal_pose["yaw"],
                        ),
                        timeout_s=15.0,
                    )
                    result["navigation_yaw_handoff_attempted"] = True
                    result["navigation_spin_handoff_success"] = spin_success
                    navigation_detail = (
                        "near_goal_yaw_handoff:"
                        f"spin={spin_detail}:navigate={retry_detail}"
                    )
                result["after_approach_pose_map"] = list(self._current_pose())
                result["navigate_success"] = success
                result["navigate_detail"] = navigation_detail
                if not success:
                    task = self.coordinator.mark_approach_failed(
                        candidate_id,
                        self.get_clock().now().nanoseconds * 1e-9,
                        reason="navigate_to_pose_failed",
                        distance_m=self._distance_m,
                        elapsed_s=self._sim_now_s() - started,
                    )
                if success:
                    arrived_roi, arrived_short, arrived_pose = self._capture_projection(
                        candidate
                    )
                    capture = {
                        "candidate_id": candidate_id,
                        "requested_at_s": self.get_clock().now().nanoseconds * 1e-9,
                        "expected_roi_xyxy": arrived_roi or plan["expected_roi_xyxy"],
                        "expected_target_short_side_px": (
                            arrived_short
                            if arrived_short is not None
                            else plan["expected_target_short_side_px"]
                        ),
                        "search_roi_padding_px": 15.0,
                        "arrived_pose_map": list(arrived_pose) if arrived_pose else None,
                        "predicted_ready": bool(
                            (
                                arrived_short
                                if arrived_short is not None
                                else plan["expected_target_short_side_px"]
                            )
                            >= 12.0
                            and plan["expected_self_pixel_fraction"] <= 0.05
                            and plan["expected_target_self_overlap"] <= 0.05
                        ),
                    }
                    self.capture_pub.publish(String(data=json.dumps(capture, separators=(",", ":"))))
                    evaluation = self._wait_for(
                        self._evaluations,
                        candidate_id,
                        float(self.get_parameter("capture_timeout_s").value),
                    )
                    if evaluation:
                        result.update(evaluation)
                        result["predicted_ready"] = capture["predicted_ready"]
                        result["capture_completed"] = True
                        task = self.coordinator.observation_result(
                            candidate_id,
                            self.get_clock().now().nanoseconds * 1e-9,
                            ready=bool(evaluation["actual_ready"]),
                            confirmed=bool(evaluation["confirmed"]),
                            distance_m=self._distance_m,
                            elapsed_s=self._sim_now_s() - started,
                        )
                return_success, return_detail, _ = self._navigate(
                    boundary_pose,
                    proximity_success_m=0.30,
                )
                if not return_success:
                    current_after_return = self._current_pose()
                    if math.dist(
                        current_after_return[:2], boundary_pose[:2]
                    ) <= 0.35:
                        return_success = True
                        return_detail = (
                            f"{return_detail}:boundary_proximity_fallback"
                        )
                result["coverage_resumed"] = return_success
                result["return_detail"] = return_detail
                if task.coverage_resume_required:
                    task = self.coordinator.mark_coverage_resumed(
                        candidate_id,
                        self.get_clock().now().nanoseconds * 1e-9,
                        return_success,
                    )
                self.coverage_pub.publish(String(data=json.dumps({
                    "state": "RESUMED_AFTER_ACTIVE_OBSERVATION" if return_success else "RESUME_FAILED",
                    "candidate_id": candidate_id,
                })))
                result.update({
                    "terminal_state": task.state.value,
                    "terminal_reason": task.terminal_reason,
                })
            except Exception as error:
                result.update({
                    "terminal_state": "REJECTED",
                    "terminal_reason": f"executive_exception:{error}",
                })
            finally:
                result.update({
                    "extra_distance_m": self._distance_m,
                    "extra_time_s": self._sim_now_s() - started,
                    "coverage_interruption_s": self._sim_now_s() - started,
                    "collision_count": self._collision_count,
                    "keepout_violation_count": self._keepout_violations,
                })
                self._active_id = None
                self._last_odom = None
                self.result_pub.publish(String(data=json.dumps(result, separators=(",", ":"))))
                self._busy.release()

    rclpy.init()
    source = OracleSceneSource()
    evaluator = MachineReadyEvaluator()
    executive = ObservationExecutive()
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (source, evaluator, executive):
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in (source, evaluator, executive):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
