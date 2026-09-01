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
    assess_grid_observation,
    load_campus_map_contract,
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
        self.declare_parameter("fixed_start_position_tolerance_m", 0.50)
        self.declare_parameter("fixed_start_yaw_tolerance_rad", 0.35)
        self.declare_parameter("gnss_odometry_consistency_tolerance_m", 2.0)
        self.declare_parameter("support_artifacts_prepared", False)
        self.declare_parameter(
            "mapping_pose_source",
            MAPPING_POSE_SOURCE,
        )
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

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        if math.isfinite(pose.position.x) and math.isfinite(pose.position.y):
            self._latest_odom_xy = (pose.position.x, pose.position.y)
        if self._start_checked:
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
        if math.isfinite(pose.position.x) and math.isfinite(pose.position.y):
            self._latest_gps_xy = (pose.position.x, pose.position.y)

    def _on_map(self, message: OccupancyGrid) -> None:
        self._latest_map = message

    def _evaluate(self) -> None:
        if self._finished or self._saving or self._latest_map is None:
            return
        now = time.monotonic()
        if now - self._last_quality_monotonic < float(
            self.get_parameter("quality_period_sec").value
        ):
            return
        self._last_quality_monotonic = now
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
        message = self._latest_map
        quality = assess_grid_observation(
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

    def _on_save(self, future, details: dict) -> None:  # type: ignore[no-untyped-def]
        try:
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
            }
            manifest = {
                "schema_version": 1,
                "status": "ready_for_localization_cleaning",
                "episode_id": self._contract.episode_id,
                "map_id": self._contract.map_id,
                "occupancy_map": map_yaml.name,
                "observed_fraction": details["observed_fraction"],
                "observed_area_m2": details["observed_area_m2"],
                "field_sampled_area_m2": details["field_sampled_area_m2"],
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
