from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .g2_scene import set_poses


DEFAULT_TOPICS = ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/ground_truth/semantic/image", "/ground_truth/instance/image")


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def decode_label(rgb: np.ndarray) -> np.ndarray:
    values = np.asarray(rgb, dtype=np.uint32)
    return values[:, :, 0] + (values[:, :, 1] << 8) + (values[:, :, 2] << 16)


def outside_start_envelope(
    current_xy: tuple[float, float],
    start_xyz: list[float],
    tolerance_m: float = 0.50,
) -> bool:
    return (
        math.hypot(
            current_xy[0] - float(start_xyz[0]),
            current_xy[1] - float(start_xyz[1]),
        )
        > tolerance_m
    )


def should_reapply_start(
    saved_count: int,
    current_xy: tuple[float, float],
    start_xyz: list[float],
) -> bool:
    """Gate only frame zero; normal scene motion must not trigger a reset."""
    return saved_count == 0 and outside_start_envelope(current_xy, start_xyz)


def removal_trigger_frame(frame_count: int, trigger_fraction: float) -> int:
    if frame_count < 4:
        raise ValueError("dynamic removal capture requires at least four frames")
    if not 0.0 < trigger_fraction < 1.0:
        raise ValueError("dynamic removal trigger_fraction must be in (0, 1)")
    return max(2, min(frame_count - 2, int(round(frame_count * trigger_fraction))))


def insertion_trigger_frame(frame_count: int, trigger_fraction: float) -> int:
    if frame_count < 4:
        raise ValueError("dynamic insertion capture requires at least four frames")
    if not 0.0 < trigger_fraction < 1.0:
        raise ValueError("dynamic insertion trigger_fraction must be in (0, 1)")
    return max(2, min(frame_count - 2, int(round(frame_count * trigger_fraction))))


def adjacent_translation_gate(
    records: list[dict],
    requested_frames: int,
    minimum_translation_m: float = 0.25,
) -> bool:
    if len(records) != requested_frames:
        return False
    translations = [
        math.hypot(
            current["vehicle_xy_m"][0] - previous["vehicle_xy_m"][0],
            current["vehicle_xy_m"][1] - previous["vehicle_xy_m"][1],
        )
        for previous, current in zip(records, records[1:])
    ]
    return bool(translations) and min(translations) >= minimum_translation_m


def wrapped_angle_delta(first_rad: float, second_rad: float) -> float:
    """Return the absolute shortest planar rotation between two yaws."""
    return abs((second_rad - first_rad + math.pi) % (2.0 * math.pi) - math.pi)


def adjacent_motion_gate(
    records: list[dict],
    requested_frames: int,
    *,
    minimum_translation_m: float,
    minimum_rotation_rad: float,
) -> bool:
    """Accept every adjacent sample only when it contains commanded SE(2) motion."""
    if len(records) != requested_frames:
        return False
    checks = []
    for previous, current in zip(records, records[1:]):
        translation = math.hypot(
            current["vehicle_xy_m"][0] - previous["vehicle_xy_m"][0],
            current["vehicle_xy_m"][1] - previous["vehicle_xy_m"][1],
        )
        rotation = wrapped_angle_delta(
            float(previous.get("vehicle_yaw_rad", 0.0)),
            float(current.get("vehicle_yaw_rad", 0.0)),
        )
        checks.append(
            translation >= minimum_translation_m
            or minimum_rotation_rad > 0.0 and rotation >= minimum_rotation_rad
        )
    return bool(checks) and all(checks)


def frame_translation_ready(
    previous_xy: tuple[float, float],
    current_xy: tuple[float, float],
    minimum_translation_m: float,
) -> bool:
    """Apply the same frozen spacing to capture admission and final QA."""
    return math.hypot(
        current_xy[0] - previous_xy[0], current_xy[1] - previous_xy[1]
    ) >= minimum_translation_m


def frame_motion_ready(
    previous_pose: tuple[float, float, float],
    current_pose: tuple[float, float, float],
    *,
    minimum_translation_m: float,
    minimum_rotation_rad: float,
) -> bool:
    return frame_translation_ready(
        previous_pose[:2], current_pose[:2], minimum_translation_m
    ) or (
        minimum_rotation_rad > 0.0
        and wrapped_angle_delta(previous_pose[2], current_pose[2])
        >= minimum_rotation_rad
    )


def commanded_motion_ready(
    previous_pose: tuple[float, float, float],
    current_pose: tuple[float, float, float],
    *,
    linear_x_mps: float,
    angular_z_rad_s: float,
    minimum_translation_m: float,
    minimum_rotation_rad: float,
) -> bool:
    """Admit a frame only after odometry reflects its commanded phase."""
    translation = math.hypot(
        current_pose[0] - previous_pose[0], current_pose[1] - previous_pose[1]
    )
    rotation = wrapped_angle_delta(previous_pose[2], current_pose[2])
    if abs(linear_x_mps) <= 1e-9 and abs(angular_z_rad_s) > 1e-9:
        return minimum_rotation_rad > 0.0 and rotation >= minimum_rotation_rad
    if abs(linear_x_mps) > 1e-9 and abs(angular_z_rad_s) <= 1e-9:
        return translation >= minimum_translation_m
    return frame_motion_ready(
        previous_pose,
        current_pose,
        minimum_translation_m=minimum_translation_m,
        minimum_rotation_rad=minimum_rotation_rad,
    )


def motion_command_for_frame(
    profile: dict | None,
    frame_index: int,
    default_linear_speed_mps: float,
    *,
    world_switch_triggered: bool = False,
    world_switch_frame_index: int | None = None,
    maximum_linear_speed_mps: float | None = None,
) -> tuple[float, float, str]:
    """Resolve a deterministic frame-counted motion phase."""
    def bounded(linear: float, angular: float, phase: str) -> tuple[float, float, str]:
        if maximum_linear_speed_mps is not None and abs(linear) > maximum_linear_speed_mps:
            linear = math.copysign(maximum_linear_speed_mps, linear)
        return linear, angular, phase

    if not profile:
        return default_linear_speed_mps, 0.0, "straight_approach"
    if profile.get("control_mode") == "latched_world_x_switch":
        if world_switch_triggered:
            post_switch = profile.get("post_switch_phases")
            if post_switch:
                if world_switch_frame_index is None:
                    raise ValueError("latched post-switch phases require a switch frame index")
                relative_index = max(0, frame_index - world_switch_frame_index)
                offset = 0
                for phase in post_switch:
                    count = int(phase["frame_count"])
                    if count <= 0:
                        raise ValueError("post-switch phase frame_count must be positive")
                    if relative_index < offset + count:
                        return bounded(
                            float(phase.get("linear_x_mps", 0.0)),
                            float(phase.get("angular_z_rad_s", 0.0)),
                            str(phase["name"]),
                        )
                    offset += count
                raise ValueError("post-switch motion profile does not cover requested frames")
            return bounded(
                float(profile["orbit_linear_x_mps"]),
                float(profile["orbit_angular_z_rad_s"]),
                str(profile.get("post_switch_phase_name", "safe_orbit_after_candidate")),
            )
        return bounded(
            float(profile["straight_linear_x_mps"]),
            0.0,
            "straight_candidate_drive_by",
        )
    offset = 0
    for phase in profile.get("phases", []):
        count = int(phase["frame_count"])
        if count <= 0:
            raise ValueError("motion phase frame_count must be positive")
        if frame_index < offset + count:
            return bounded(
                float(phase.get("linear_x_mps", 0.0)),
                float(phase.get("angular_z_rad_s", 0.0)),
                str(phase.get("name", f"phase_{offset}")),
            )
        offset += count
    raise ValueError("motion profile does not cover requested capture frames")


def nearest_stamp_within(
    available_stamps: list[int], target_stamp: int, maximum_skew_ns: int
) -> int | None:
    if not available_stamps:
        return None
    nearest = min(available_stamps, key=lambda value: abs(value - target_stamp))
    return nearest if abs(nearest - target_stamp) <= maximum_skew_ns else None


def observed_speeds_from_records(records: list[dict]) -> list[float]:
    """Use synchronized odom stamps, not camera stamps, for odom velocity."""
    speeds = []
    for previous, current in zip(records, records[1:]):
        elapsed_s = (
            current["odom_timestamp_ns"] - previous["odom_timestamp_ns"]
        ) / 1e9
        if elapsed_s <= 0.0:
            continue
        speeds.append(
            math.hypot(
                current["vehicle_xy_m"][0] - previous["vehicle_xy_m"][0],
                current["vehicle_xy_m"][1] - previous["vehicle_xy_m"][1],
            ) / elapsed_s
        )
    return speeds


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene-manifest", required=True)
    parser.add_argument("--frame-count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--rgb-topic", default=DEFAULT_TOPICS[0])
    parser.add_argument("--depth-topic", default=DEFAULT_TOPICS[1])
    parser.add_argument("--semantic-topic", default=DEFAULT_TOPICS[2])
    parser.add_argument("--instance-topic", default=DEFAULT_TOPICS[3])
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/ground_truth/model_odom_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--camera-xyz", nargs=3, type=float, default=[0.53, 0.0, 0.22])
    parser.add_argument("--optical-frame", default="camera_depth_link")
    parser.add_argument("--node-name", default="stage5br3_g2_collector")
    parser.add_argument("--linear-speed-mps", type=float, default=0.35)
    parser.add_argument("--minimum-adjacent-translation-m", type=float, default=0.25)
    parser.add_argument("--minimum-adjacent-rotation-rad", type=float, default=0.0)
    args = parser.parse_args()
    if args.linear_speed_mps <= 0.0:
        parser.error("--linear-speed-mps must be positive")
    if args.minimum_adjacent_translation_m < 0.0:
        parser.error("--minimum-adjacent-translation-m must be non-negative")
    if args.minimum_adjacent_rotation_rad < 0.0:
        parser.error("--minimum-adjacent-rotation-rad must be non-negative")
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    output = Path(args.output)
    scene = json.loads(Path(args.scene_manifest).read_text(encoding="utf-8"))
    topics = (args.rgb_topic, args.depth_topic, args.semantic_topic, args.instance_topic)
    for name in ("rgb", "depth", "semantic", "instance", "camera", "tf", "capture"):
        (output / name).mkdir(parents=True, exist_ok=True)

    class Collector(Node):
        def __init__(self):
            super().__init__(args.node_name)
            self.bridge = CvBridge(); self.buffers = {topic: {} for topic in topics}
            self.camera = None; self.odom = None; self.odom_buffer = {}; self.last_saved_pose = None; self.saved = []
            self.pending_writes = []
            self.dynamic_positions = []
            self.dynamic_removal_events = []
            self.dynamic_insertion_events = []
            self.motion_enabled = False
            self.world_switch_triggered = False
            self.world_switch_frame_index = None
            self.vehicle_reset_applied = False
            self.vehicle_reset_started = None
            self.started = time.monotonic(); self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
            for topic in topics:
                self.create_subscription(Image, topic, lambda msg, key=topic: self.receive(key, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, args.camera_info_topic, lambda msg: setattr(self, "camera", msg), qos_profile_sensor_data)
            self.create_subscription(Odometry, args.odom_topic, self.receive_odom, 20)
            self.create_timer(0.05, self.tick)

        def receive_odom(self, message):
            self.odom = message
            self.odom_buffer[stamp_ns(message)] = message
            while len(self.odom_buffer) > 200:
                self.odom_buffer.pop(next(iter(self.odom_buffer)))

        def tick(self):
            command = Twist()
            if self.motion_enabled and len(self.saved) < args.frame_count:
                profile = scene.get("oprv3_motion_profile")
                if (
                    profile
                    and profile.get("control_mode") == "latched_world_x_switch"
                    and self.odom is not None
                    and self.odom.pose.pose.position.x
                    >= float(profile["switch_world_x_m"])
                    and not self.world_switch_triggered
                ):
                    self.world_switch_triggered = True
                    self.world_switch_frame_index = len(self.saved)
                linear, angular, _ = motion_command_for_frame(
                    profile,
                    len(self.saved),
                    args.linear_speed_mps,
                    world_switch_triggered=self.world_switch_triggered,
                    world_switch_frame_index=self.world_switch_frame_index,
                    maximum_linear_speed_mps=args.linear_speed_mps,
                )
                command.linear.x = linear
                command.angular.z = angular
            self.publisher.publish(command)

        def receive(self, topic, message):
            self.buffers[topic][stamp_ns(message)] = message
            for bucket in self.buffers.values():
                while len(bucket) > 30: bucket.pop(next(iter(bucket)))
            common = set.intersection(*(set(bucket) for bucket in self.buffers.values()))
            if not common or self.camera is None or not self.odom_buffer or len(self.saved) >= args.frame_count:
                return
            stamp = min(common)
            odom_stamp = nearest_stamp_within(
                list(self.odom_buffer), stamp, 50_000_000
            )
            if odom_stamp is None:
                return
            pose = self.odom_buffer[odom_stamp].pose.pose
            yaw = math.atan2(
                2.0 * pose.orientation.w * pose.orientation.z,
                1.0 - 2.0 * pose.orientation.z * pose.orientation.z,
            )
            current = (pose.position.x, pose.position.y, yaw)
            # A world-scoped DiffDrive controller can retain the prior scene's
            # velocity while RGB-D / segmentation sensors are warming up.
            # Re-apply the manifest start pose only after every required sensor
            # and odometry stream is live, then hold zero velocity for a full
            # second before accepting frame 0. This keeps the sensor entity
            # alive without allowing cross-scene motion state to leak.
            vehicle_start = scene.get("vehicle_start_xyz_m", [-8.0, 0.0, 0.18])
            if not self.vehicle_reset_applied:
                set_poses(
                    scene["world_id"],
                    [
                        {
                            "name": "sanitation_vehicle",
                            "xyz": vehicle_start,
                            "yaw": float(scene.get("vehicle_start_yaw_rad", 0.0)),
                        }
                    ],
                )
                self.vehicle_reset_applied = True
                self.vehicle_reset_started = time.monotonic()
                self.odom = None
                self.odom_buffer.clear()
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if time.monotonic() - self.vehicle_reset_started < 1.0:
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if should_reapply_start(len(self.saved), current[:2], vehicle_start):
                set_poses(
                    scene["world_id"],
                    [
                        {
                            "name": "sanitation_vehicle",
                            "xyz": vehicle_start,
                            "yaw": float(scene.get("vehicle_start_yaw_rad", 0.0)),
                        }
                    ],
                )
                self.vehicle_reset_started = time.monotonic()
                self.odom = None
                self.odom_buffer.clear()
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if self.last_saved_pose is not None:
                linear, angular, _ = motion_command_for_frame(
                    scene.get("oprv3_motion_profile"),
                    len(self.saved),
                    args.linear_speed_mps,
                    world_switch_triggered=self.world_switch_triggered,
                    world_switch_frame_index=self.world_switch_frame_index,
                    maximum_linear_speed_mps=args.linear_speed_mps,
                )
                if not commanded_motion_ready(
                    self.last_saved_pose,
                    current,
                    linear_x_mps=linear,
                    angular_z_rad_s=angular,
                    minimum_translation_m=args.minimum_adjacent_translation_m,
                    minimum_rotation_rad=args.minimum_adjacent_rotation_rad,
                ):
                    return
            messages = [self.buffers[topic].pop(stamp) for topic in topics]
            self.stage(stamp, messages, current, odom_stamp)
            self.apply_dynamic_step(len(self.saved))
            self.last_saved_pose = current
            self.motion_enabled = True

        def apply_dynamic_step(self, frame_count):
            plan = scene.get("dynamic_motion_plan")
            if plan:
                start = plan["start_xyz_m"]
                delta = plan["delta_per_frame_m"]
                xyz = [
                    float(start[index]) + float(delta[index]) * frame_count
                    for index in range(3)
                ]
                set_poses(
                    scene["world_id"],
                    [{"name": plan["model_name"], "xyz": xyz, "yaw": 0.0}],
                )
                self.dynamic_positions.append(
                    {"after_frame": frame_count - 1, "xyz_m": xyz}
                )
            insertion = scene.get("dynamic_insertion_plan")
            if insertion and not self.dynamic_insertion_events:
                trigger = insertion_trigger_frame(
                    args.frame_count, float(insertion["trigger_fraction"])
                )
                if frame_count >= trigger:
                    inserted = [float(value) for value in insertion["inserted_xyz_m"]]
                    set_poses(
                        scene["world_id"],
                        [{"name": insertion["model_name"], "xyz": inserted, "yaw": float(insertion.get("yaw_rad", 0.0))}],
                    )
                    for bucket in self.buffers.values():
                        bucket.clear()
                    self.dynamic_insertion_events.append(
                        {"after_frame": frame_count - 1, "first_post_insertion_frame": frame_count, "model_name": insertion["model_name"], "class_id": insertion["class_id"], "inserted_xyz_m": inserted}
                    )
            removal = scene.get("dynamic_removal_plan")
            if not removal or self.dynamic_removal_events:
                return
            trigger = removal_trigger_frame(
                args.frame_count, float(removal["trigger_fraction"])
            )
            if frame_count < trigger:
                return
            parked = [float(value) for value in removal["parked_xyz_m"]]
            set_poses(
                scene["world_id"],
                [
                    {
                        "name": removal["model_name"],
                        "xyz": parked,
                        "yaw": 0.0,
                    }
                ],
            )
            # Sensor callbacks may already have queued synchronized images
            # from immediately before the pose update.  Discard them so the
            # declared first post-removal frame is genuinely captured after
            # the target has left the world, rather than being a stale buffer.
            for bucket in self.buffers.values():
                bucket.clear()
            self.dynamic_removal_events.append(
                {
                    "after_frame": frame_count - 1,
                    "first_post_removal_frame": frame_count,
                    "model_name": removal["model_name"],
                    "class_id": removal["class_id"],
                    "parked_xyz_m": parked,
                }
            )

        def stage(self, stamp, messages, pose, odom_stamp):
            """Decode into a bounded in-memory batch; persist after motion stops.

            Writing four image products from the ROS callback drops sensor
            samples and makes the empirical observation cadence a disk-speed
            benchmark.  ``frame_count`` bounds this queue (90 for OPRV3), so
            collection remains finite while preserving the incoming cadence.
            """
            index = len(self.saved); stem = f"frame_{index:02d}"
            rgb = self.bridge.imgmsg_to_cv2(messages[0], "rgb8")
            depth = self.bridge.imgmsg_to_cv2(messages[1], "passthrough").astype(np.float32)
            semantic_rgb = self.bridge.imgmsg_to_cv2(messages[2], "rgb8")
            instance_rgb = self.bridge.imgmsg_to_cv2(messages[3], "rgb8")
            if not np.all(semantic_rgb[:, :, 0] == semantic_rgb[:, :, 1]) or not np.all(semantic_rgb[:, :, 1] == semantic_rgb[:, :, 2]):
                raise RuntimeError("semantic labels are not repeated-channel IDs")
            paths = {"rgb": output/"rgb"/f"{stem}.png", "depth": output/"depth"/f"{stem}.npy", "semantic": output/"semantic"/f"{stem}.npy", "instance": output/"instance"/f"{stem}.npy", "camera": output/"camera"/f"{stem}.json", "tf": output/"tf"/f"{stem}.json", "capture": output/"capture"/f"{stem}.json"}
            linear, angular, phase = motion_command_for_frame(
                scene.get("oprv3_motion_profile"), index, args.linear_speed_mps,
                world_switch_triggered=self.world_switch_triggered,
                world_switch_frame_index=self.world_switch_frame_index,
                maximum_linear_speed_mps=args.linear_speed_mps,
            )
            record = {"frame_index": index, "timestamp_ns": stamp, "odom_timestamp_ns": odom_stamp, "sensor_odom_skew_ns": abs(odom_stamp - stamp), "vehicle_xy_m": list(pose[:2]), "vehicle_yaw_rad": float(pose[2]), "motion_phase": phase, "commanded_linear_x_mps": linear, "commanded_angular_z_rad_s": angular, "exact_four_sensor_timestamp": len({stamp_ns(msg) for msg in messages}) == 1, "paths": {key: str(path.relative_to(output)).replace("\\", "/") for key, path in paths.items()}, "rgb_sha256": None}
            camera = {"width": self.camera.width, "height": self.camera.height, "k": list(self.camera.k), "p": list(self.camera.p), "frame_id": self.camera.header.frame_id}
            self.pending_writes.append((paths, record, rgb, depth, semantic_rgb[:, :, 0].copy(), decode_label(instance_rgb), camera, pose))
            self.saved.append(record)

        def flush(self):
            for paths, record, rgb, depth, semantic, instance, camera, pose in self.pending_writes:
                if not cv2.imwrite(str(paths["rgb"]), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError(f"failed to write RGB frame: {paths['rgb']}")
                np.save(paths["depth"], depth, allow_pickle=False)
                np.save(paths["semantic"], semantic, allow_pickle=False)
                np.save(paths["instance"], instance, allow_pickle=False)
                paths["camera"].write_text(json.dumps(camera, indent=2)+"\n")
                paths["tf"].write_text(json.dumps({"world_to_base_xy": list(pose[:2]), "base_to_camera_xyz_m": list(args.camera_xyz), "optical_frame": args.optical_frame}, indent=2)+"\n")
                record["rgb_sha256"] = hashlib.sha256(paths["rgb"].read_bytes()).hexdigest()
                paths["capture"].write_text(json.dumps(record, indent=2)+"\n")
            self.pending_writes.clear()

    rclpy.init(); node = Collector()
    while rclpy.ok() and len(node.saved) < args.frame_count and time.monotonic() - node.started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    finished_monotonic = time.monotonic()
    node.publisher.publish(Twist())
    flush_started = time.monotonic()
    node.flush()
    persistence_duration_s = time.monotonic() - flush_started
    dynamic_plan = scene.get("dynamic_motion_plan")
    dynamic_executed = (
        dynamic_plan is None
        or len(node.dynamic_positions) == args.frame_count
        and len({tuple(item["xyz_m"]) for item in node.dynamic_positions}) > 1
    )
    removal_plan = scene.get("dynamic_removal_plan")
    removal_executed = removal_plan is None or len(node.dynamic_removal_events) == 1
    insertion_plan = scene.get("dynamic_insertion_plan")
    insertion_executed = insertion_plan is None or len(node.dynamic_insertion_events) == 1
    adjacent_motion_gate_pass = adjacent_motion_gate(
        node.saved,
        args.frame_count,
        minimum_translation_m=args.minimum_adjacent_translation_m,
        minimum_rotation_rad=args.minimum_adjacent_rotation_rad,
    )
    observed_speeds = observed_speeds_from_records(node.saved)
    sensor_odom_skews = [item["sensor_odom_skew_ns"] for item in node.saved]
    wall_duration_s = finished_monotonic - node.started
    simulated_duration_s = (
        (node.saved[-1]["timestamp_ns"] - node.saved[0]["timestamp_ns"]) / 1e9
        if len(node.saved) > 1
        else None
    )
    effective_captured_fps = (
        (len(node.saved) - 1) / simulated_duration_s
        if simulated_duration_s and simulated_duration_s > 0.0
        else None
    )
    capture_pass = (
        len(node.saved) == args.frame_count
        and adjacent_motion_gate_pass
        and bool(sensor_odom_skews)
        and max(sensor_odom_skews) <= 50_000_000
        and all(item["exact_four_sensor_timestamp"] for item in node.saved)
        and all(item["rgb_sha256"] for item in node.saved)
        and dynamic_executed
        and removal_executed
        and insertion_executed
    )
    report = {
        "schema_version": 4,
        "scene_seed": scene["scene_seed"],
        "world_id": scene["world_id"],
        "split": scene["split"],
        "requested_frames": args.frame_count,
        "captured_frames": len(node.saved),
        "topics": list(topics),
        "camera_xyz_m": list(args.camera_xyz),
        "optical_frame": args.optical_frame,
        "commanded_linear_speed_mps": args.linear_speed_mps,
        "oprv3_motion_profile": scene.get("oprv3_motion_profile"),
        "minimum_adjacent_translation_m": args.minimum_adjacent_translation_m,
        "minimum_adjacent_rotation_rad": args.minimum_adjacent_rotation_rad,
        "capture_timing": {
            "persistence_mode": "bounded_memory_then_flush_after_motion_stop",
            "timeout_s": args.timeout,
            "wall_duration_s": wall_duration_s,
            "persistence_duration_s": persistence_duration_s,
            "simulated_duration_s": simulated_duration_s,
            "simulator_realtime_factor": (
                simulated_duration_s / wall_duration_s
                if simulated_duration_s is not None and wall_duration_s > 0.0
                else None
            ),
            "effective_captured_fps": effective_captured_fps,
        },
        "sensor_odom_sync": {
            "maximum_skew_ns": max(sensor_odom_skews) if sensor_odom_skews else None,
            "gate_maximum_skew_ns": 50_000_000,
            "pass": bool(sensor_odom_skews)
            and max(sensor_odom_skews) <= 50_000_000,
        },
        "observed_linear_speed_mps": {
            "time_basis": "synchronized_odom_timestamp",
            "samples": len(observed_speeds),
            "median": float(np.median(observed_speeds)) if observed_speeds else None,
            "p05": float(np.percentile(observed_speeds, 5)) if observed_speeds else None,
            "p95": float(np.percentile(observed_speeds, 95)) if observed_speeds else None,
        },
        "observed_absolute_yaw_change_rad": sum(
            wrapped_angle_delta(
                float(a.get("vehicle_yaw_rad", 0.0)),
                float(b.get("vehicle_yaw_rad", 0.0)),
            )
            for a, b in zip(node.saved, node.saved[1:])
        ),
        "records": node.saved,
        "adjacent_motion_gate_pass": adjacent_motion_gate_pass,
        "dynamic_motion_requested": dynamic_plan is not None,
        "dynamic_motion_executed": dynamic_executed,
        "dynamic_positions": node.dynamic_positions,
        "dynamic_removal_requested": removal_plan is not None,
        "dynamic_removal_executed": removal_executed,
        "dynamic_removal_events": node.dynamic_removal_events,
        "dynamic_insertion_requested": insertion_plan is not None,
        "dynamic_insertion_executed": insertion_executed,
        "dynamic_insertion_events": node.dynamic_insertion_events,
        "capture_pass": capture_pass,
    }
    (output/"capture_report.json").write_text(json.dumps(report, indent=2)+"\n"); print(json.dumps(report, indent=2))
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(0 if report["capture_pass"] else 2)


if __name__ == "__main__":
    main()
