#!/usr/bin/env python3
"""Read a named model pose directly from a Gazebo Pose_V topic."""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from pathlib import Path


def _protobuf_scalar(block: str, field: str, default: float = 0.0) -> float:
    match = re.search(
        rf"^\s*{re.escape(field)}:\s*([-+0-9.eE]+)\s*$", block, re.MULTILINE
    )
    return float(match.group(1)) if match else default


def _named_pose_block(message: str, name: str) -> str:
    marker = f'name: "{name}"'
    marker_index = message.find(marker)
    if marker_index < 0:
        raise RuntimeError(f"Gazebo pose/info did not contain model {name}")
    start = message.rfind("pose {", 0, marker_index)
    if start < 0:
        raise RuntimeError(f"Gazebo pose/info block for {name} is malformed")
    depth = 0
    for index in range(start + len("pose "), len(message)):
        if message[index] == "{":
            depth += 1
        elif message[index] == "}":
            depth -= 1
            if depth == 0:
                return message[start : index + 1]
    raise RuntimeError(f"Gazebo pose/info block for {name} is unterminated")


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_named_model_pose(
    *, world_name: str, model_name: str, timeout_s: float = 10.0
) -> dict[str, float]:
    executable = shutil.which("gz")
    if executable is None:
        vendor = Path("/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
        if vendor.exists():
            executable = str(vendor)
    if executable is None:
        raise RuntimeError("Gazebo CLI not found")
    result = subprocess.run(
        [
            executable,
            "topic",
            "-e",
            "-t",
            f"/world/{world_name}/pose/info",
            "-n",
            "1",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    block = _named_pose_block(result.stdout, model_name)
    position_match = re.search(r"position\s*\{(?P<body>.*?)\}", block, re.DOTALL)
    orientation_match = re.search(
        r"orientation\s*\{(?P<body>.*?)\}", block, re.DOTALL
    )
    if position_match is None or orientation_match is None:
        raise RuntimeError("Gazebo model pose has no position or orientation")
    position = position_match.group("body")
    orientation = orientation_match.group("body")
    return {
        "x": _protobuf_scalar(position, "x"),
        "y": _protobuf_scalar(position, "y"),
        "z": _protobuf_scalar(position, "z"),
        "yaw": _quaternion_yaw(
            _protobuf_scalar(orientation, "x"),
            _protobuf_scalar(orientation, "y"),
            _protobuf_scalar(orientation, "z"),
            _protobuf_scalar(orientation, "w", 1.0),
        ),
    }
