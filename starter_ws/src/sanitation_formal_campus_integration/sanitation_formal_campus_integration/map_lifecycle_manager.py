"""ROS 2 manager for first-map/save and saved-map localization admission."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Bool, String
import yaml

from .map_lifecycle_core import (
    MAPPING_POSE_SOURCE,
    MapLifecycleError,
    assess_saved_pgm_observation,
    assess_grid_observation,
    load_campus_map_contract,
    materialize_saved_map_coverage_geometry,
    prepare_public_lifecycle_artifacts,
    sha256,
    validate_saved_map_artifact,
)


def _yaw(quaternion) -> float:  # type: ignore[no-untyped-def]
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class FormalMapLifecycleManager(Node):
    """Fail closed until SLAM coverage, save response and hashes all agree."""

    def __init__(self) -> None:
        super().__init__("formal_map_lifecycle_manager")
        for name, default in (
            ("mode", "mapping"),
            ("episode_manifest", ""),
            ("artifact_directory", ""),
            ("map_topic", "/map"),
            ("odom_topic", "/odom"),
            ("gps_odometry_topic", "/odometry/gps"),
            ("save_service", "/slam_toolbox/save_map"),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("observation_threshold", 0.95)
        self.declare_parameter("stable_samples_required", 3)
        self.declare_parameter("quality_period_sec", 5.0)
        # SLAM publishes every 2 s; allow three 5 s quality periods, never replay.
        # EKF/navsat run at 50/20 Hz; 5 s is a conservative hard stale ceiling.
        self.declare_parameter("map_max_age_sec", 15.0)
        self.declare_parameter("odometry_max_age_sec", 5.0)
        self.declare_parameter("fixed_start_position_tolerance_m", 0.50)
        self.declare_parameter("fixed_start_yaw_tolerance_rad", 0.35)
        self.declare_parameter("gnss_odometry_consistency_tolerance_m", 2.0)
        self.declare_parameter("support_artifacts_prepared", False)
        self.declare_parameter(
            "mapping_pose_source",
            MAPPING_POSE_SOURCE,
        )
        threshold = float(self.get_parameter("observation_threshold").value)
        samples = self.get_parameter("stable_samples_required").value
        if not 0.95 <= threshold <= 1.0 or type(samples) is not int or samples < 3:
            raise MapLifecycleError("formal mapping requires threshold >= 0.95 and >= 3 samples")
        for name in ("quality_period_sec", "map_max_age_sec", "odometry_max_age_sec"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise MapLifecycleError(f"{name} must be positive and finite")
        for name, ceiling in (("map_max_age_sec", 15.0), ("odometry_max_age_sec", 5.0)):
            if float(self.get_parameter(name).value) > ceiling:
                raise MapLifecycleError(f"{name} cannot exceed {ceiling} seconds")
        self._mapping_pose_source = str(
            self.get_parameter("mapping_pose_source").value
        )
        if self._mapping_pose_source != MAPPING_POSE_SOURCE:
            raise MapLifecycleError("unsupported mapping_pose_source")
        self._mode = str(self.get_parameter("mode").value)
        if self._mode not in {"mapping", "cleaning"}:
            raise MapLifecycleError("mode must be mapping or cleaning")
        self._contract = load_campus_map_contract(
            str(self.get_parameter("episode_manifest").value)
        )
        self._root = Path(str(self.get_parameter("artifact_directory").value))
        if not str(self._root):
            raise MapLifecycleError("artifact_directory is required")
        if self._mode == "mapping":
            if bool(self.get_parameter("support_artifacts_prepared").value):
                required = (
                    "geofence_keepout.yaml",
                    "geofence_keepout.pgm",
                    "neutral_speed.yaml",
                    "neutral_speed.pgm",
                    "mission_geometry.yaml",
                    "materialization_contract.yaml",
                )
                missing = [
                    name for name in required if not (self._root / name).is_file()
                ]
                if missing:
                    raise MapLifecycleError(
                        f"preprepared public lifecycle artifacts missing: {missing}"
                    )
            else:
                prepare_public_lifecycle_artifacts(self._contract, self._root)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status = self.create_publisher(
            String, "/formal_mapping/lifecycle_status", latched
        )
        self._ready = self.create_publisher(Bool, "/formal_mapping/map_ready", latched)
        self._latest_map: OccupancyGrid | None = None
        self._start_ok = False
        self._start_checked = False
        self._latest_odom_xy: tuple[float, float] | None = None
        self._latest_gps_xy: tuple[float, float] | None = None
        self._map_received_at: float | None = None
        self._odom_received_at: float | None = None
        self._gps_received_at: float | None = None
        self._odom_stamp_ns: int | None = None
        self._gps_stamp_ns: int | None = None
        self._map_stamp_ns: int | None = None
        self._consumed_map_stamp_ns: int | None = None
        self._stable = 0
        self._saving = False
        self._finished = False
        self._last_quality_monotonic = 0.0
        if self._mode == "cleaning":
            manifest = validate_saved_map_artifact(self._root, self._contract)
            self._finished = True
            self._publish("ready_for_localization_cleaning", True, manifest)
            return
        if (self._root / "map_lifecycle_manifest.json").exists():
            raise MapLifecycleError(
                "mapping mode cannot overwrite a finalized saved map; use a new artifact directory"
            )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("gps_odometry_topic").value),
            self._on_gps_odom,
            20,
        )
        self._save_client = self.create_client(
            SaveMap, str(self.get_parameter("save_service").value)
        )
        self.create_timer(0.5, self._evaluate)
        self._publish("waiting_for_fixed_start_and_slam_map", False, {})

    def _publish(self, status: str, ready: bool, details: dict) -> None:
        payload = {
            "schema_version": 1,
            "mode": self._mode,
            "status": status,
            "ready": ready,
            "map_id": self._contract.map_id,
            "product_map_source": "slam_toolbox_lidar_wheel_imu_with_gnss_gate",
            "world_truth_used_for_control": False,
            "mapping_ignored_dirt": True,
            "mapping_pose_source": self._mapping_pose_source,
            "gnss_mapping_reference_observed": self._latest_gps_xy is not None,
            **details,
        }
        text = String()
        text.data = json.dumps(payload, sort_keys=True)
        self._status.publish(text)
        flag = Bool()
        flag.data = ready
        self._ready.publish(flag)

    def _valid_source_header(self, message, stream: str, frame: str, maximum_age: float) -> bool:
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = getattr(self, f"_{stream}_stamp_ns")
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        valid = (
            message.header.frame_id == frame and stamp.sec >= 0
            and 0 <= stamp.nanosec < 1_000_000_000 and stamp_ns > 0
            and (previous is None or stamp_ns > previous)
            and 0 <= age <= maximum_age
        )
        if valid:
            setattr(self, f"_{stream}_stamp_ns", stamp_ns)
        return valid

    def _inputs_fresh(self, now: float) -> bool:
        source_now = self.get_clock().now().nanoseconds
        for stream, parameter, ceiling in (
            ("map", "map_max_age_sec", 15.0),
            ("odom", "odometry_max_age_sec", 5.0),
            ("gps", "odometry_max_age_sec", 5.0),
        ):
            maximum = float(self.get_parameter(parameter).value)
            received = getattr(self, f"_{stream}_received_at")
            stamp = getattr(self, f"_{stream}_stamp_ns")
            if (not 0 < maximum <= ceiling or received is None or stamp is None
                    or not 0 <= now - received <= maximum
                    or not 0 <= (source_now - stamp) / 1e9 <= maximum):
                return False
        return True

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        if (self._valid_source_header(message, "odom", "odom", 5.0)
                and math.isfinite(pose.position.x) and math.isfinite(pose.position.y)):
            self._latest_odom_xy = (pose.position.x, pose.position.y)
            self._odom_received_at = time.monotonic()
        else:
            self._latest_odom_xy = None
            self._odom_received_at = None
            self._stable = 0
            return
        if self._start_checked:
            return
        quaternion = pose.orientation
        components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if (not all(math.isfinite(value) for value in components)
                or not math.isclose(sum(value*value for value in components), 1.0, abs_tol=1e-3)):
            self._stable = 0
            return
        tolerance = float(
            self.get_parameter("fixed_start_position_tolerance_m").value
        )
        yaw_tolerance = float(
            self.get_parameter("fixed_start_yaw_tolerance_rad").value
        )
        self._start_ok = (
            math.hypot(pose.position.x, pose.position.y) <= tolerance
            and abs(math.atan2(math.sin(_yaw(pose.orientation)), math.cos(_yaw(pose.orientation))))
            <= yaw_tolerance
        )
        self._start_checked = True

    def _on_gps_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        if (self._valid_source_header(message, "gps", "odom", 5.0)
                and math.isfinite(pose.position.x) and math.isfinite(pose.position.y)):
            self._latest_gps_xy = (pose.position.x, pose.position.y)
            self._gps_received_at = time.monotonic()
        else:
            self._latest_gps_xy = None
            self._gps_received_at = None
            self._stable = 0

    def _on_map(self, message: OccupancyGrid) -> None:
        if self._finished:
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        origin = message.info.origin
        quaternion = origin.orientation
        values = (origin.position.x, origin.position.y, origin.position.z,
                  quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        valid = (
            self._valid_source_header(message, "map", "map", 15.0)
            and all(math.isfinite(value) for value in values)
            and math.isclose(sum(value * value for value in values[3:]), 1.0, abs_tol=1e-3)
            and abs(quaternion.x) <= 1e-6 and abs(quaternion.y) <= 1e-6
            and message.info.width > 0 and message.info.height > 0
            and len(message.data) == message.info.width * message.info.height
            and all(-1 <= value <= 100 for value in message.data)
            and math.isfinite(message.info.resolution)
            and 0 < message.info.resolution <= 0.10
        )
        if not valid:
            self._latest_map = None
            self._map_received_at = None
            self._stable = 0
            self._publish("invalid_or_replayed_slam_map", False, {})
            return
        self._latest_map = message
        self._map_stamp_ns = stamp_ns
        self._map_received_at = time.monotonic()

    def _evaluate(self) -> None:
        if self._finished or self._saving or self._latest_map is None:
            return
        now = time.monotonic()
        threshold = float(self.get_parameter("observation_threshold").value)
        required = self.get_parameter("stable_samples_required").value
        if not 0.95 <= threshold <= 1.0 or type(required) is not int or required < 3:
            self._stable = 0
            self._publish("invalid_mapping_quality_parameters", False, {})
            return
        if not self._inputs_fresh(now):
            self._stable = 0
            self._publish("waiting_for_fresh_mapping_inputs", False, {})
            return
        if self._map_stamp_ns == self._consumed_map_stamp_ns:
            return
        if now - self._last_quality_monotonic < float(
            self.get_parameter("quality_period_sec").value
        ):
            return
        self._last_quality_monotonic = now
        self._consumed_map_stamp_ns = self._map_stamp_ns
        if not self._start_ok:
            self._stable = 0
            self._publish("fixed_start_gate_failed", False, {})
            return
        if self._latest_odom_xy is None or self._latest_gps_xy is None:
            self._stable = 0
            self._publish("waiting_for_gnss_mapping_reference", False, {})
            return
        gnss_disagreement_m = math.dist(self._latest_odom_xy, self._latest_gps_xy)
        gnss_tolerance_m = float(
            self.get_parameter("gnss_odometry_consistency_tolerance_m").value
        )
        if not math.isfinite(gnss_disagreement_m) or gnss_disagreement_m > gnss_tolerance_m:
            self._stable = 0
            self._publish(
                "gnss_odometry_consistency_gate_failed",
                False,
                {
                    "gnss_odometry_disagreement_m": gnss_disagreement_m,
                    "gnss_odometry_tolerance_m": gnss_tolerance_m,
                },
            )
            return
        try:
            quality = self._assess_latest_map()
        except MapLifecycleError as exc:
            self._stable = 0
            self._publish("invalid_slam_grid_quality", False, {"error": str(exc)})
            return
        self._stable = self._stable + 1 if quality.passed else 0
        details = {
            "observed_cells": quality.observed_cells,
            "field_cells": quality.field_cells,
            "observed_area_m2": quality.observed_area_m2,
            "field_sampled_area_m2": quality.field_sampled_area_m2,
            "observed_fraction": quality.observed_fraction,
            "stable_gate_samples": self._stable,
            "gnss_odometry_disagreement_m": gnss_disagreement_m,
            "gnss_odometry_tolerance_m": gnss_tolerance_m,
        }
        required = int(self.get_parameter("stable_samples_required").value)
        if self._stable < required:
            self._publish("exploring_until_observed_fraction_gate", False, details)
            return
        if not self._save_client.service_is_ready():
            self._publish("quality_passed_waiting_for_slam_save_service", False, details)
            return
        self._saving = True
        request = SaveMap.Request()
        request.name.data = str((self._root / "occupancy").resolve())
        future = self._save_client.call_async(request)
        future.add_done_callback(lambda result: self._on_save(result, details))
        self._publish("saving_quality_gated_map", False, details)

    def _assess_latest_map(self):  # type: ignore[no-untyped-def]
        message = self._latest_map
        return assess_grid_observation(
            message.data,
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            origin_yaw=_yaw(message.info.origin.orientation),
            geofence=self._contract.geofence,
            threshold=float(self.get_parameter("observation_threshold").value),
        )

    def _on_save(self, future, details: dict) -> None:  # type: ignore[no-untyped-def]
        try:
            now = time.monotonic()
            if self._stable < 3 or self._latest_map is None or not self._inputs_fresh(now):
                raise MapLifecycleError("mapping inputs expired or became invalid while saving")
            threshold = float(self.get_parameter("observation_threshold").value)
            required = self.get_parameter("stable_samples_required").value
            if (not 0.95 <= threshold <= 1.0 or type(required) is not int or required < 3
                    or self._stable < required or not self._assess_latest_map().passed):
                raise MapLifecycleError("mapping quality changed while saving")
            response = future.result()
            if response is None or int(response.result) != 0:
                raise MapLifecycleError("slam_toolbox save_map returned failure")
            map_yaml = self._root / "occupancy.yaml"
            metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
            image_name = metadata.get("image") if isinstance(metadata, dict) else None
            if (
                not isinstance(image_name, str)
                or not image_name
                or Path(image_name).is_absolute()
                or Path(image_name).name != image_name
                or "/" in image_name
                or "\\" in image_name
            ):
                raise MapLifecycleError("saved map YAML has no image")
            image_path = self._root / image_name
            if not image_path.is_file():
                raise MapLifecycleError("saved map image is missing")
            materialize_saved_map_coverage_geometry(self._root, self._contract)
            pgm_observation = assess_saved_pgm_observation(
                metadata,
                image_path.read_bytes(),
                geofence=self._contract.geofence,
                threshold=float(self.get_parameter("observation_threshold").value),
            )
            if not pgm_observation.passed:
                raise MapLifecycleError("saved occupancy PGM did not meet the observation threshold")
            hashes = {
                map_yaml.name: sha256(map_yaml),
                image_path.name: sha256(image_path),
                "mission_geometry.yaml": sha256(self._root / "mission_geometry.yaml"),
                "materialization_contract.yaml": sha256(
                    self._root / "materialization_contract.yaml"
                ),
                "geofence_keepout.yaml": sha256(
                    self._root / "geofence_keepout.yaml"
                ),
                "geofence_keepout.pgm": sha256(
                    self._root / "geofence_keepout.pgm"
                ),
                "neutral_speed.yaml": sha256(
                    self._root / "neutral_speed.yaml"
                ),
                "neutral_speed.pgm": sha256(
                    self._root / "neutral_speed.pgm"
                ),
                "coverage_geometry.yaml": sha256(
                    self._root / "coverage_geometry.yaml"
                ),
                "coverage_free_space.pgm": sha256(
                    self._root / "coverage_free_space.pgm"
                ),
            }
            manifest = {
                "schema_version": 1,
                "status": "ready_for_localization_cleaning",
                "episode_id": self._contract.episode_id,
                "map_id": self._contract.map_id,
                "occupancy_map": map_yaml.name,
                "observed_fraction": pgm_observation.observed_fraction,
                "observed_area_m2": pgm_observation.observed_area_m2,
                "field_sampled_area_m2": pgm_observation.field_sampled_area_m2,
                "saved_pgm_observed_fraction": pgm_observation.observed_fraction,
                "saved_pgm_observed_cells": pgm_observation.observed_cells,
                "saved_pgm_field_cells": pgm_observation.field_cells,
                "quality_threshold": float(
                    self.get_parameter("observation_threshold").value
                ),
                "stable_gate_samples": self._stable,
                "fixed_start_verified": True,
                "world_truth_used_for_control": False,
                "mapping_ignored_dirt": True,
                "mapping_pose_source": self._mapping_pose_source,
                "gnss_mapping_reference_observed": True,
                "gnss_odometry_disagreement_m": details[
                    "gnss_odometry_disagreement_m"
                ],
                "sha256": hashes,
            }
            temporary = self._root / ".map_lifecycle_manifest.json.tmp"
            temporary.write_text(
                json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self._root / "map_lifecycle_manifest.json")
            self._finished = True
            self._publish("ready_for_localization_cleaning", True, manifest)
        except Exception as exc:  # fail closed and permit a later retry
            self._stable = 0
            self._publish("map_save_or_integrity_gate_failed", False, {"error": str(exc)})
        finally:
            self._saving = False


def main(args=None) -> None:  # type: ignore[no-untyped-def]
    rclpy.init(args=args)
    node = FormalMapLifecycleManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
