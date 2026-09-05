#!/usr/bin/env python3
"""Finalize the bounded Gazebo-native dynamic sensor diagnostic."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


PROBES = {
    "imu": "/sensors/imu/data",
    "utm30_gpu_lidar": "/sensors/lidar_2d/scan",
    "front_rgbd_depth": "/sensors/front_rgbd/depth/image_rect_raw/image",
}


def _integer(path: Path) -> int:
    try:
        return int(path.read_text(encoding="utf-8").strip())
    except (FileNotFoundError, ValueError):
        return -1


def finalize(
    output_root: Path, launch_alive: bool, memory_watchdog_result: int = 0
) -> dict[str, object]:
    observations = {}
    for name, topic in PROBES.items():
        status = _integer(output_root / f"{name}.status")
        byte_count = _integer(output_root / f"{name}.bytes")
        observations[name] = {
            "topic": topic,
            "exit_status": status,
            "captured_bytes": byte_count,
            "observed": status == 0 and byte_count > 0,
        }
    imu = observations["imu"]["observed"]
    lidar = observations["utm30_gpu_lidar"]["observed"]
    camera = observations["front_rgbd_depth"]["observed"]
    if memory_watchdog_result == 86:
        status = "MEMORY_WATCHDOG_BREACHED"
    elif memory_watchdog_result != 0:
        status = "MEMORY_WATCHDOG_FAILED"
    elif not launch_alive:
        status = "DYNAMIC_SENSOR_PROCESS_FAILED"
    elif imu and lidar and camera:
        status = "GZ_NATIVE_DYNAMIC_SENSOR_SMOKE_PASSED"
    elif imu and not lidar and not camera:
        status = "DYNAMIC_SPAWN_RENDER_SENSOR_LIFECYCLE_BLOCKED"
    elif imu and lidar and not camera:
        status = "DYNAMIC_SPAWN_CAMERA_RENDERING_BLOCKED"
    elif not imu:
        status = "DYNAMIC_SENSOR_BASELINE_BLOCKED"
    else:
        status = "DYNAMIC_SENSOR_MIXED_BLOCKED"
    passed = status == "GZ_NATIVE_DYNAMIC_SENSOR_SMOKE_PASSED"
    report = {
        "report_id": "tzcup_gz_native_dynamic_sensor_smoke_v1",
        "status": status,
        "passed": passed,
        "formal_eligible": False,
        "spawn_mode": "dynamic_usercommands",
        "transport_plane": "gazebo_native_without_high_bandwidth_ros_bridges",
        "launch_alive_after_probes": launch_alive,
        "memory_watchdog_result": memory_watchdog_result,
        "observations": observations,
        "claim_boundary": (
            "Diagnostic-only source-message evidence. It cannot satisfy the "
            "formal preembedded world, ROS bridge, DDS, frame, rate, FOV, or "
            "session-bound sensor acceptance contracts."
        ),
    }
    output = output_root / "gz_native_sensor_smoke.json"
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--launch-alive", type=int, choices=(0, 1), required=True)
    parser.add_argument("--memory-watchdog-result", type=int, required=True)
    args = parser.parse_args()
    report = finalize(
        args.output_root, bool(args.launch_alive), args.memory_watchdog_result
    )
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
