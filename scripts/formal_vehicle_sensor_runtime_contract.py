#!/usr/bin/env python3
"""Dependency-free contracts for formal-vehicle sensor runtime evidence."""

from __future__ import annotations

import math
from typing import Any, Mapping


A300_WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)

FORMAL_SENSOR_GROUPS: dict[str, tuple[str, ...]] = {
    "utm30lx_2d_lidar": ("/sensors/lidar_2d/scan",),
    "mid360_3d_lidar": ("/sensors/lidar_3d/points",),
    "front_d435": (
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/front_rgbd/infra1/image_rect_raw",
        "/sensors/front_rgbd/infra2/image_rect_raw",
    ),
    "wrist_d435": (
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/sensors/wrist_rgbd/infra1/image_rect_raw",
        "/sensors/wrist_rgbd/infra2/image_rect_raw",
    ),
    "rear_left_fisheye": ("/sensors/rear_left_fisheye/image_raw",),
    "rear_right_fisheye": ("/sensors/rear_right_fisheye/image_raw",),
    "zed_f9p_gnss": ("/sensors/gnss/fix",),
    "imu": ("/sensors/imu/data",),
}

STREAM_CONTRACTS: dict[str, dict[str, Any]] = {
    "/sensors/lidar_2d/scan": {
        "frame_id": "lidar_2d_link", "nominal_hz": 40.0,
    },
    "/sensors/lidar_3d/points": {
        "frame_id": "lidar_3d_link", "nominal_hz": 10.0,
    },
    "/sensors/front_rgbd/depth/image_rect_raw/image": {
        "frame_id": "front_rgbd_depth_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/front_rgbd/depth/image_rect_raw/depth_image": {
        "frame_id": "front_rgbd_depth_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/front_rgbd/depth/image_rect_raw/camera_info": {
        "frame_id": "front_rgbd_depth_optical_frame", "size": (848, 480),
    },
    "/sensors/front_rgbd/infra1/image_rect_raw": {
        "frame_id": "front_rgbd_infra1_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/front_rgbd/infra1/image_rect_raw/camera_info": {
        "frame_id": "front_rgbd_infra1_optical_frame", "size": (848, 480),
    },
    "/sensors/front_rgbd/infra2/image_rect_raw": {
        "frame_id": "front_rgbd_infra2_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/front_rgbd/infra2/image_rect_raw/camera_info": {
        "frame_id": "front_rgbd_infra2_optical_frame", "size": (848, 480),
    },
    "/sensors/wrist_rgbd/depth/image_rect_raw/image": {
        "frame_id": "wrist_rgbd_depth_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image": {
        "frame_id": "wrist_rgbd_depth_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info": {
        "frame_id": "wrist_rgbd_depth_optical_frame", "size": (848, 480),
    },
    "/sensors/wrist_rgbd/infra1/image_rect_raw": {
        "frame_id": "wrist_rgbd_infra1_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/wrist_rgbd/infra1/image_rect_raw/camera_info": {
        "frame_id": "wrist_rgbd_infra1_optical_frame", "size": (848, 480),
    },
    "/sensors/wrist_rgbd/infra2/image_rect_raw": {
        "frame_id": "wrist_rgbd_infra2_optical_frame", "nominal_hz": 30.0,
        "size": (848, 480),
    },
    "/sensors/wrist_rgbd/infra2/image_rect_raw/camera_info": {
        "frame_id": "wrist_rgbd_infra2_optical_frame", "size": (848, 480),
    },
    "/sensors/rear_left_fisheye/image_raw": {
        "frame_id": "rear_left_fisheye_optical_frame", "nominal_hz": 30.0,
        "size": (1920, 1080),
    },
    "/sensors/rear_left_fisheye/camera_info": {
        "frame_id": "rear_left_fisheye_optical_frame", "size": (1920, 1080),
    },
    "/sensors/rear_right_fisheye/image_raw": {
        "frame_id": "rear_right_fisheye_optical_frame", "nominal_hz": 30.0,
        "size": (1920, 1080),
    },
    "/sensors/rear_right_fisheye/camera_info": {
        "frame_id": "rear_right_fisheye_optical_frame", "size": (1920, 1080),
    },
    "/sensors/gnss/fix": {
        "frame_id": "gnss_antenna_link", "nominal_hz": 10.0,
    },
    "/sensors/imu/data": {
        "frame_id": "imu_link", "nominal_hz": 200.0,
    },
    "/formal_vehicle/encoders/a300/counts": {},
    "/formal_vehicle/encoders/a300/joint_states": {},
    "/odom/unfiltered": {"frame_id": "odom"},
}

# A three-stamp window is intentionally sufficient for most multi-megabyte
# image and point-cloud streams so their subscriptions can be retired quickly.
# The front RGB stream and 200 Hz IMU use longer windows to avoid judging their
# source cadence from transient subscription/startup scheduling.  Ten RGB
# stamps span 300 ms at the configured rate; fifty IMU stamps span 245 ms.
SOURCE_FREQUENCY_SAMPLE_TARGETS: dict[str, int] = {
    "/sensors/front_rgbd/depth/image_rect_raw/image": 10,
    "/sensors/wrist_rgbd/depth/image_rect_raw/image": 10,
    "/sensors/imu/data": 50,
}


def observed_frequency_hz(stamps_ns: list[int] | tuple[int, ...]) -> float | None:
    """Calculate frequency from strictly increasing source timestamps."""

    unique = sorted(set(int(value) for value in stamps_ns if int(value) > 0))
    if len(unique) < 3 or unique[-1] <= unique[0]:
        return None
    return (len(unique) - 1) * 1_000_000_000.0 / (unique[-1] - unique[0])


def validate_runtime_contract(
    samples: Mapping[str, int],
    metadata: Mapping[str, Mapping[str, Any]],
    observed_hz: Mapping[str, float | None],
) -> dict[str, Any]:
    """Validate identity, frame, frequency, range and wheel encoder evidence."""

    missing = sorted(topic for topic in STREAM_CONTRACTS if samples.get(topic, 0) <= 0)
    frame_errors = {
        topic: {
            "expected": contract["frame_id"],
            "observed": metadata.get(topic, {}).get("frame_id"),
        }
        for topic, contract in STREAM_CONTRACTS.items()
        if "frame_id" in contract
        and metadata.get(topic, {}).get("frame_id") != contract["frame_id"]
    }
    size_errors = {
        topic: {
            "expected": list(contract["size"]),
            "observed": [
                metadata.get(topic, {}).get("width"),
                metadata.get(topic, {}).get("height"),
            ],
        }
        for topic, contract in STREAM_CONTRACTS.items()
        if "size" in contract
        and (
            metadata.get(topic, {}).get("width"),
            metadata.get(topic, {}).get("height"),
        ) != contract["size"]
    }
    # Runtime transport can drop frames under load. Source timestamps must
    # nevertheless demonstrate at least half the configured nominal cadence;
    # the exact configured rates remain a separate expanded-URDF FOV gate.
    frequency_errors = {
        topic: {
            "nominal_hz": contract["nominal_hz"],
            "minimum_observed_hz": contract["nominal_hz"] * 0.5,
            "observed_hz": observed_hz.get(topic),
        }
        for topic, contract in STREAM_CONTRACTS.items()
        if "nominal_hz" in contract
        and (
            observed_hz.get(topic) is None
            or float(observed_hz[topic]) < contract["nominal_hz"] * 0.5
        )
    }
    lidar = metadata.get("/sensors/lidar_2d/scan", {})
    lidar_contract = (
        math.isclose(float(lidar.get("range_min_m", math.nan)), 0.1, abs_tol=1e-6)
        and math.isclose(float(lidar.get("range_max_m", math.nan)), 30.0, abs_tol=1e-6)
        and math.isclose(float(lidar.get("angle_min_rad", math.nan)), -2.356194, abs_tol=1e-6)
        and math.isclose(float(lidar.get("angle_max_rad", math.nan)), 2.356194, abs_tol=1e-6)
        and int(lidar.get("range_count", 0)) == 1080
    )
    cloud = metadata.get("/sensors/lidar_3d/points", {})
    mid360_cloud_contract = (
        int(cloud.get("width", 0)) * int(cloud.get("height", 0)) > 0
        and int(cloud.get("point_step", 0)) > 0
    )
    gnss = metadata.get("/sensors/gnss/fix", {})
    gnss_contract = all(
        math.isfinite(float(gnss.get(key, math.nan)))
        for key in ("latitude", "longitude", "altitude")
    )
    imu_contract = metadata.get("/sensors/imu/data", {}).get("finite_measurement") is True

    counts = metadata.get("/formal_vehicle/encoders/a300/counts", {})
    encoder_states = metadata.get(
        "/formal_vehicle/encoders/a300/joint_states", {}
    )
    expected_label = "joint_order:" + ",".join(A300_WHEEL_JOINTS)
    encoder_contract = (
        counts.get("layout_label") == expected_label
        and int(counts.get("data_length", 0)) == len(A300_WHEEL_JOINTS)
        and tuple(encoder_states.get("joint_names", ())) == A300_WHEEL_JOINTS
        and encoder_states.get("finite_position_velocity") is True
    )
    odom = metadata.get("/odom/unfiltered", {})
    odom_contract = (
        odom.get("frame_id") == "odom"
        and odom.get("child_frame_id") == "base_footprint"
    )
    sensor_group_observation = {
        group: {
            topic: int(samples.get(topic, 0)) for topic in topics
        }
        for group, topics in FORMAL_SENSOR_GROUPS.items()
    }
    all_sensor_groups_observed = len(sensor_group_observation) == 8 and all(
        all(count > 0 for count in topic_counts.values())
        for topic_counts in sensor_group_observation.values()
    )
    passed_checks = {
        "all_eight_formal_sensor_groups_observed": all_sensor_groups_observed,
        "all_required_streams_observed": not missing,
        "all_sensor_frames_exact": not frame_errors,
        "all_image_and_camera_info_dimensions_exact": not size_errors,
        "runtime_source_timestamp_rates_at_least_half_nominal": not frequency_errors,
        "utm30lx_runtime_range_and_270deg_scan_exact": lidar_contract,
        "mid360_runtime_pointcloud_nonempty": mid360_cloud_contract,
        "gnss_runtime_measurement_finite": gnss_contract,
        "imu_runtime_measurement_finite": imu_contract,
        "a300_four_wheel_encoder_feedback_structured_and_finite": encoder_contract,
        "raw_plant_odometry_frames_correct": odom_contract,
    }
    return {
        "passed": all(passed_checks.values()),
        "passed_checks": passed_checks,
        "missing_topics": missing,
        "frame_errors": frame_errors,
        "dimension_errors": size_errors,
        "frequency_errors": frequency_errors,
        "formal_sensor_group_observation": sensor_group_observation,
    }
