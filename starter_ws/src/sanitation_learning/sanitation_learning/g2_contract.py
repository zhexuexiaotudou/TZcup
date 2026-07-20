from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


PRODUCTION_TOPICS = {
    "camera_info": "/camera/color/camera_info",
    "rgb": "/camera/color/image_raw",
    "depth": "/camera/depth/image_rect_raw",
    "points": "/camera/depth/color/points",
}


def _xacro_arg_defaults(root: ET.Element) -> dict[str, str]:
    suffix = "}arg"
    return {
        element.attrib["name"]: element.attrib["default"]
        for element in root.iter()
        if (element.tag == "arg" or element.tag.endswith(suffix))
        and "name" in element.attrib
        and "default" in element.attrib
    }


def _numbers(text: str | None, defaults: dict[str, str] | None = None) -> list[float]:
    resolved = text or ""
    for name, value in (defaults or {}).items():
        resolved = re.sub(r"\$\(\s*arg\s+" + re.escape(name) + r"\s*\)", value, resolved)
    unresolved = re.findall(r"\$\([^)]*\)", resolved)
    if unresolved:
        raise ValueError(f"unresolved Xacro arguments in numeric vector: {unresolved}")
    return [float(value) for value in resolved.split()]


def read_production_camera_contract(xacro_path: str | Path) -> dict:
    """Extract the simulated production camera contract from the vehicle Xacro."""
    path = Path(xacro_path)
    root = ET.parse(path).getroot()
    defaults = _xacro_arg_defaults(root)
    joint = next(
        (candidate for candidate in root.findall("./joint")
         if candidate.find("./child") is not None
         and candidate.find("./child").attrib.get("link") == "camera_link"),
        None,
    )
    gazebo = root.find("./gazebo[@reference='camera_link']")
    if joint is None or gazebo is None:
        raise ValueError("camera_joint or camera_link Gazebo sensor is missing")
    sensor = gazebo.find("./sensor[@type='rgbd_camera']")
    camera = sensor.find("./camera") if sensor is not None else None
    image = camera.find("./image") if camera is not None else None
    if sensor is None or camera is None or image is None:
        raise ValueError("production rgbd camera definition is incomplete")
    origin = joint.find("./origin")
    parent = joint.find("./parent")
    child = joint.find("./child")
    optical_joint = root.find("./joint[@name='camera_depth_joint']")
    optical_child = optical_joint.find("./child") if optical_joint is not None else None
    source_sha = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "schema_version": 1,
        "source": str(path).replace("\\", "/"),
        "source_sha256": source_sha,
        "parent_frame": parent.attrib["link"],
        "camera_link": child.attrib["link"],
        "optical_frame": optical_child.attrib["link"],
        "extrinsics": {
            "xyz_m": _numbers(origin.attrib.get("xyz"), defaults),
            "rpy_rad": _numbers(origin.attrib.get("rpy"), defaults),
        },
        "native_resolution": {
            "width": int(image.findtext("width")),
            "height": int(image.findtext("height")),
        },
        "horizontal_fov_rad": float(camera.findtext("horizontal_fov")),
        "update_rate_hz": float(sensor.findtext("update_rate")),
        "near_clip_m": float(camera.findtext("./clip/near")),
        "far_clip_m": float(camera.findtext("./clip/far")),
        "gazebo_base_topic": sensor.findtext("topic"),
        "production_ros_topics": dict(PRODUCTION_TOPICS),
    }


def validate_sim_launch_topics(launch_path: str | Path, contract: dict) -> None:
    text = Path(launch_path).read_text(encoding="utf-8")
    missing = [topic for topic in contract["production_ros_topics"].values() if topic not in text]
    if missing:
        raise ValueError(f"production camera topic remaps missing from launch: {missing}")


def write_contract(xacro_path: str | Path, launch_path: str | Path, output: str | Path) -> dict:
    contract = read_production_camera_contract(xacro_path)
    validate_sim_launch_topics(launch_path, contract)
    contract["launch_source"] = str(Path(launch_path)).replace("\\", "/")
    contract["launch_source_sha256"] = hashlib.sha256(Path(launch_path).read_bytes()).hexdigest()
    contract["g2_rule"] = (
        "RGB, depth, semantic GT, and instance GT sensors are training-only co-located "
        "sensors with identical pose, intrinsics, resolution, and update rate; GT topics "
        "must never enter production control or perception inputs."
    )
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(contract, indent=2) + "\n", encoding="utf-8")
    return contract
