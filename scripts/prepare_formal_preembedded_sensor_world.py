#!/usr/bin/env python3
"""Build a source-bound world with the formal vehicle present at world load.

Gazebo Harmonic's Sensors system can miss sensors introduced through the
UserCommands / ``ros_gz_sim create`` path.  This utility makes that failure
mode explicit: it converts the frozen URDF once, restores every URDF
``<gazebo reference=...>`` sensor to its physical link, and embeds the model
in the world *before* the Sensors system begins updating.

It does not fabricate transport samples.  A caller must still observe every
formal stream through the normal Gazebo-to-ROS bridges.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


class PreparationError(RuntimeError):
    """Raised when a source-bound preembedded world cannot be produced."""


CANONICAL_CONTROLLER_URI = (
    "package://sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
)
FORMAL_CONTROLLER_FILENAME = "gz_ros2_control-system"
FORMAL_CONTROLLER_NAME = "gz_ros2_control::GazeboSimROS2ControlPlugin"
CONTROLLER_RUNTIME_RELATIVE = Path(
    "share/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _strict_absolute_path(path: Path, label: str, *, directory: bool = False) -> Path:
    if not path.is_absolute():
        raise PreparationError(f"{label} must be absolute: {path}")
    if path.is_symlink():
        raise PreparationError(f"{label} must not be a symlink: {path}")
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise PreparationError(f"{label} does not exist: {path}") from error
    if directory and not resolved.is_dir():
        raise PreparationError(f"{label} is not a directory: {resolved}")
    if not directory and not resolved.is_file():
        raise PreparationError(f"{label} is not a regular file: {resolved}")
    return resolved


def bind_controller_parameters(
    model: ET.Element,
    controller_config: Path,
    runtime_install_root: Path,
) -> dict[str, str]:
    """Bind gz_ros2_control to the exact controller YAML in the frozen runtime."""

    install_root = _strict_absolute_path(
        runtime_install_root, "runtime install root", directory=True
    )
    config = _strict_absolute_path(controller_config, "controller config")
    expected = (install_root / CONTROLLER_RUNTIME_RELATIVE).resolve()
    if config != expected:
        raise PreparationError(
            "controller config is not the frozen runtime package artifact: "
            f"expected {expected}, got {config}"
        )
    try:
        config.relative_to(install_root)
    except ValueError as error:
        raise PreparationError(
            f"controller config escapes frozen runtime install root: {config}"
        ) from error

    control_candidates = [
        plugin
        for plugin in model.findall(".//plugin")
        if plugin.get("filename") == FORMAL_CONTROLLER_FILENAME
        or plugin.get("name") == FORMAL_CONTROLLER_NAME
    ]
    plugins = [
        plugin
        for plugin in control_candidates
        if plugin.get("filename") == FORMAL_CONTROLLER_FILENAME
        and plugin.get("name") == FORMAL_CONTROLLER_NAME
    ]
    if len(control_candidates) != 1 or len(plugins) != 1:
        raise PreparationError(
            "expected exactly one complete formal gz_ros2_control plugin and "
            "no competing control authority, "
            f"found {len(control_candidates)} candidates ({len(plugins)} formal)"
        )
    parameters = plugins[0].findall("parameters")
    if len(parameters) != 1:
        raise PreparationError(
            "formal gz_ros2_control plugin must have exactly one parameters element"
        )
    original = (parameters[0].text or "").strip()
    if original != CANONICAL_CONTROLLER_URI:
        raise PreparationError(
            "unexpected portable controller parameter reference: "
            f"expected {CANONICAL_CONTROLLER_URI}, got {original!r}"
        )
    parameters[0].text = str(config)
    return {
        "plugin_filename": plugins[0].get("filename", ""),
        "plugin_name": plugins[0].get("name", ""),
        "portable_source_uri": original,
        "runtime_install_root": str(install_root),
        "resolved_controller_config": str(config),
        "controller_config_relative_to_install": config.relative_to(
            install_root
        ).as_posix(),
        "controller_config_sha256": _sha256(config),
    }


def sensor_attachment_contract(urdf: ET.Element) -> dict[str, tuple[str, str | None, str]]:
    """Return sensor name -> (owning link, local pose, type) from URDF extensions."""

    result: dict[str, tuple[str, str | None, str]] = {}
    for gazebo in urdf.findall("gazebo"):
        reference = gazebo.get("reference")
        if not reference:
            continue
        for sensor in gazebo.findall("sensor"):
            name = sensor.get("name")
            if not name:
                raise PreparationError("URDF gazebo sensor is missing a name")
            if name in result:
                raise PreparationError(f"duplicate URDF gazebo sensor: {name}")
            pose = sensor.findtext("pose")
            result[name] = (reference, pose.strip() if pose else None, sensor.get("type", ""))
    if not result:
        raise PreparationError("URDF has no <gazebo reference=...><sensor> contract")
    return result


def _parent_map(root: ET.Element) -> dict[ET.Element, ET.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def _pose_values(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = tuple(float(item) for item in (text or "0 0 0 0 0 0").split())
    if len(values) != 6:
        raise PreparationError(f"expected six-value pose, got: {text!r}")
    return values  # type: ignore[return-value]


def _normalized_pose(text: str) -> str:
    return " ".join(f"{value:.12g}" for value in _pose_values(text))


def _rotation(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _sensor_link_pose(global_sensor_pose: str, local_sensor_pose: str | None) -> str:
    """Return root->link pose given converted root->sensor and link->sensor."""

    gx, gy, gz, gr, gp, gyaw = _pose_values(global_sensor_pose)
    lx, ly, lz, lr, lp, lyaw = _pose_values(local_sensor_pose)
    global_rotation = _rotation(gr, gp, gyaw)
    local_rotation = _rotation(lr, lp, lyaw)
    inverse_local = [[local_rotation[j][i] for j in range(3)] for i in range(3)]
    rotation = _matmul(global_rotation, inverse_local)
    local_translation = [lx, ly, lz]
    translated = [
        gx - sum(rotation[i][j] * local_translation[j] for j in range(3))
        for i, gx in enumerate((gx, gy, gz))
    ]
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    values = tuple(
        0.0 if abs(value) < 1e-12 else value
        for value in (*translated, roll, pitch, yaw)
    )
    return " ".join(f"{value:.12g}" for value in values)


def _urdf_parent_links(urdf: ET.Element) -> dict[str, str]:
    result: dict[str, str] = {}
    for joint in urdf.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        if child is not None and parent is not None and child.get("link") and parent.get("link"):
            result[child.get("link", "")] = parent.get("link", "")
    return result


def _surviving_ancestor(target: str, links: dict[str | None, ET.Element], parents: dict[str, str]) -> str:
    candidate = target
    visited: set[str] = set()
    while candidate not in links:
        if candidate in visited or candidate not in parents:
            raise PreparationError(
                f"URDF sensor reference {target} has no surviving SDF ancestor"
            )
        visited.add(candidate)
        candidate = parents[candidate]
    return candidate


def restore_sensor_attachments(
    model: ET.Element,
    attachments: dict[str, tuple[str, str | None, str]],
    urdf: ET.Element,
) -> list[dict[str, str]]:
    """Move converted sensors back to their URDF reference links.

    sdformat's URDF conversion bakes pose into ``base_footprint`` and leaves
    ``<gazebo reference>`` semantics behind.  The original extension describes
    the sensor pose in the reference-link frame, so preserve that local pose
    when the sensor is reattached.  This is essential for the wrist RGB-D
    sensor to follow the six-axis arm rather than the vehicle root.
    """

    links = {link.get("name"): link for link in model.findall("link")}
    reduced_frames = {frame.get("name"): frame for frame in model.findall("frame")}
    urdf_parents = _urdf_parent_links(urdf)
    parents = _parent_map(model)
    converted = {
        sensor.get("name"): sensor
        for sensor in model.findall(".//sensor")
        if sensor.get("name")
    }
    missing = sorted(set(attachments) - set(converted))
    if missing:
        raise PreparationError(
            "gz sdf conversion omitted formal sensors: " + ", ".join(missing)
        )

    restored: list[dict[str, str]] = []
    for name in sorted(attachments):
        target_link, source_pose, _sensor_type = attachments[name]
        target = links.get(target_link)
        sensor = converted[name]
        current = parents.get(sensor)
        if current is None or current.tag != "link":
            raise PreparationError(f"converted sensor {name} has no owning link")
        attachment_status = "restored_urdf_reference_link"
        # sdformat reduces fixed joint chains, including camera and lidar
        # brackets, and bakes their initial poses into a surviving link.  Restore
        # an inertialess sensor holder plus fixed joint so sources follow their
        # intended parent if an articulated ancestor moves (notably the wrist).
        if target is None:
            ancestor = _surviving_ancestor(target_link, links, urdf_parents)
            # A fixed link can survive conversion as a direct SDF frame. Replace
            # that frame because a sensor must be a child of a link and SDF
            # rejects a frame/link name collision.
            reduced = reduced_frames.pop(target_link, None)
            if reduced is not None:
                model.remove(reduced)
            target = ET.Element("link", {"name": target_link})
            pose = ET.SubElement(target, "pose")
            pose.text = _sensor_link_pose(
                sensor.findtext("pose", default="0 0 0 0 0 0"), source_pose
            )
            model.append(target)
            fixed_joint = ET.Element(
                "joint", {"name": f"formal_sensor_attachment_{target_link}", "type": "fixed"}
            )
            ET.SubElement(fixed_joint, "parent").text = ancestor
            ET.SubElement(fixed_joint, "child").text = target_link
            model.append(fixed_joint)
            links[target_link] = target
            attachment_status = "restored_reconstructed_fixed_reference_link"
        if current is not target:
            current.remove(sensor)
            target.append(sensor)
        pose = sensor.find("pose")
        if pose is None:
            pose = ET.SubElement(sensor, "pose")
        pose.text = source_pose or "0 0 0 0 0 0"
        restored.append(
            {
                "sensor": name,
                "converted_link": current.get("name", ""),
                "restored_link": target_link,
                "local_pose": pose.text,
                "attachment_status": attachment_status,
            }
        )
    return restored


def build_preembedded_world(
    source_world: Path,
    converted_sdf: str,
    urdf: Path,
    only_sensors: set[str] | None = None,
    model_pose: str = "0 0 0.005 0 0 0",
    *,
    restore_attachments: bool = True,
) -> tuple[ET.ElementTree, list[dict[str, str]], ET.Element]:
    """Embed one corrected converted model in a copy of ``source_world``."""

    try:
        world_tree = ET.parse(source_world)
        converted_root = ET.fromstring(converted_sdf)
        urdf_root = ET.parse(urdf).getroot()
    except ET.ParseError as error:
        raise PreparationError(f"invalid XML while preparing sensor world: {error}") from error
    world = world_tree.getroot().find("world")
    if world is None:
        raise PreparationError("expected a world SDF")
    restored, model = append_preembedded_model(
        world,
        converted_root,
        urdf_root,
        only_sensors=only_sensors,
        model_pose=model_pose,
        restore_attachments=restore_attachments,
    )
    return world_tree, restored, model


def append_preembedded_model(
    world: ET.Element,
    converted_root: ET.Element,
    urdf_root: ET.Element,
    *,
    only_sensors: set[str] | None = None,
    model_pose: str = "0 0 0.005 0 0 0",
    restore_attachments: bool = True,
) -> tuple[list[dict[str, str]], ET.Element]:
    """Append one source-bound model without starting Gazebo.

    The formal grasp scene needs its vehicle and material cube to exist before
    Harmonic's Sensors/Contact systems start.  Keep the one-model conversion
    contract, but permit a second independently converted model in the same
    world rather than inventing a grasp-only world writer.
    """

    robot_name = (urdf_root.get("name") or "").strip()
    if not robot_name:
        raise PreparationError("expanded URDF robot name is missing")
    models = converted_root.findall("model")
    if len(models) != 1:
        raise PreparationError("expected exactly one direct converted model SDF")
    model = models[0]
    model_name = (model.get("name") or "").strip()
    if model_name != robot_name:
        raise PreparationError(
            "converted model name differs from expanded URDF robot name: "
            f"expected {robot_name!r}, got {model_name!r}"
        )
    if any(candidate.get("name") == robot_name for candidate in world.findall("model")):
        raise PreparationError(f"source world already contains model: {robot_name}")
    attachments = sensor_attachment_contract(urdf_root)
    restored = (
        restore_sensor_attachments(model, attachments, urdf_root)
        if restore_attachments
        else []
    )
    pose = model.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        model.insert(0, pose)
    pose.text = _normalized_pose(model_pose)
    if only_sensors is not None:
        unknown = sorted(only_sensors - set(attachments))
        if unknown:
            raise PreparationError("unknown diagnostic sensor: " + ", ".join(unknown))
        parents = _parent_map(model)
        for sensor in model.findall(".//sensor"):
            if sensor.get("name") not in only_sensors:
                parents[sensor].remove(sensor)
        restored = [row for row in restored if row["sensor"] in only_sensors]
    world.append(model)
    return restored, model


def convert_urdf(gz: str, urdf: Path) -> str:
    result = subprocess.run(
        [gz, "sdf", "-p", str(urdf)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise PreparationError(f"gz sdf conversion failed ({result.returncode}): {detail}")
    return result.stdout


def _write_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False, newline="\n"
    ) as handle:
        handle.write(content)
        staged = Path(handle.name)
    staged.replace(path)


def run(args: argparse.Namespace) -> dict[str, object]:
    source_world = Path(args.source_world).resolve()
    urdf = Path(args.vehicle_urdf).resolve()
    output_world = Path(args.output_world).resolve()
    output_report = Path(args.report).resolve()
    controller_config = Path(args.controller_config)
    runtime_install_root = Path(args.runtime_install_root)
    for path in (source_world, urdf):
        if not path.is_file():
            raise PreparationError(f"missing required source: {path}")
    if output_world.exists() or output_report.exists():
        raise PreparationError(
            "refusing to overwrite preembedded sensor evidence; use a new output path"
        )
    converted = convert_urdf(args.gz, urdf)
    only_sensors = set(args.only_sensor) if args.only_sensor else None
    diagnostic_raw_layout = bool(
        getattr(args, "diagnostic_skip_attachment_restoration", False)
    )
    model_pose = _normalized_pose(args.model_pose)
    world_tree, restored, model = build_preembedded_world(
        source_world,
        converted,
        urdf,
        only_sensors=only_sensors,
        model_pose=model_pose,
        restore_attachments=not diagnostic_raw_layout,
    )
    controller_binding = bind_controller_parameters(
        model, controller_config, runtime_install_root
    )
    additional_model: dict[str, object] | None = None
    additional_urdf_arg = getattr(args, "additional_urdf", None)
    if additional_urdf_arg:
        additional_urdf = Path(additional_urdf_arg).resolve()
        if not additional_urdf.is_file():
            raise PreparationError(f"missing additional model URDF: {additional_urdf}")
        additional_root = ET.parse(additional_urdf).getroot()
        additional_restored, additional = append_preembedded_model(
            world_tree.getroot().find("world"),
            ET.fromstring(convert_urdf(args.gz, additional_urdf)),
            additional_root,
            model_pose=getattr(args, "additional_model_pose", "0 0 0.017 0 0 0"),
        )
        additional_model = {
            "urdf": str(additional_urdf),
            "urdf_sha256": _sha256(additional_urdf),
            "model_name": additional.get("name"),
            "model_initial_pose": _normalized_pose(
                getattr(args, "additional_model_pose", "0 0 0.017 0 0 0")
            ),
            "sensors_restored_to_urdf_reference_links": additional_restored,
            "sensor_count": len(additional.findall(".//sensor")),
        }
    ET.indent(world_tree, space="  ")
    _write_atomic(output_world, ET.tostring(world_tree.getroot(), encoding="unicode") + "\n")
    report = {
        "report_id": "tzcup_formal_preembedded_sensor_world_v1",
        "status": (
            "DIAGNOSTIC_NOT_FORMAL_PREEMBEDDED_WORLD"
            if diagnostic_raw_layout
            else "FORMAL_PREEMBEDDED_SENSOR_WORLD_READY"
        ),
        "passed": True,
        "formal_eligible": not diagnostic_raw_layout,
        "claim_boundary": (
            "World preparation only. It proves neither Gazebo Transport nor ROS "
            "stream delivery; the formal 12-stream runtime collector remains required."
        ),
        "source_world": str(source_world),
        "source_world_sha256": _sha256(source_world),
        "vehicle_urdf": str(urdf),
        "vehicle_urdf_sha256": _sha256(urdf),
        "controller_runtime_binding": controller_binding,
        "output_world": str(output_world),
        "output_world_sha256": _sha256(output_world),
        "sensors_restored_to_urdf_reference_links": restored,
        "sensor_count": len(model.findall(".//sensor")),
        "spawn_mode": "preembedded_before_gazebo_sensors_system",
        "model_initial_pose": model_pose,
        "model_name": model.get("name"),
        "diagnostic_sensor_filter": sorted(only_sensors) if only_sensors else None,
        "diagnostic_skip_attachment_restoration": diagnostic_raw_layout,
        "additional_model": additional_model,
    }
    _write_atomic(output_report, json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-world", required=True)
    parser.add_argument("--vehicle-urdf", required=True)
    parser.add_argument("--output-world", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument(
        "--controller-config", required=True,
        help="Absolute installed controller YAML from the frozen runtime overlay.",
    )
    parser.add_argument(
        "--runtime-install-root", required=True,
        help="Absolute install root verified by the formal runtime closure.",
    )
    parser.add_argument("--gz", default="gz", help="Gazebo CLI used for URDF -> SDF")
    parser.add_argument(
        "--additional-urdf",
        help="Optional second source-bound URDF to embed in the same world.",
    )
    parser.add_argument(
        "--additional-model-pose", default="0 0 0.017 0 0 0",
        help="Initial pose for --additional-urdf.",
    )
    parser.add_argument(
        "--only-sensor", action="append", default=[],
        help="Optional source-only diagnostic filter; never use for formal acceptance.",
    )
    parser.add_argument(
        "--diagnostic-skip-attachment-restoration",
        action="store_true",
        help=(
            "Keep gz sdf's converted sensor layout and omit reconstructed fixed "
            "attachment joints. Diagnostic only; output is ineligible for formal acceptance."
        ),
    )
    parser.add_argument(
        "--model-pose", default="0 0 0.005 0 0 0",
        help="Model pose replacing ros_gz_sim create's historical -z 0.005 placement.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    try:
        run(parse_args(argv))
    except PreparationError as error:
        print(f"preembedded sensor world failed: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
