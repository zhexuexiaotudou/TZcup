#!/usr/bin/env python3
"""Convert the formal 19-camera visual world to on-demand capture.

The converter is deliberately fail-closed.  It accepts only the exact formal
visual-camera contract and only changes each ``<camera>`` element by inserting
``<triggered>true</triggered>``.  Gazebo then derives each default trigger topic
as ``<image topic>/trigger`` because no explicit trigger-topic override exists.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Sequence


EXPECTED_TOPICS = (
    "/formal_visual/front_left",
    "/formal_visual/rear_right",
    "/formal_visual/top_cleaning",
    "/formal_visual/sensor_tower_detail",
    "/formal_visual/front_sensor_detail",
    "/formal_visual/arm_mount_detail",
    "/formal_visual/dry_deposition_detail",
    "/formal_visual/cleaning_head_detail",
    "/formal_visual/rear_service_detail",
    "/formal_visual/power_compute_detail",
    "/formal_visual/storage_recovery_detail",
    "/formal_visual/rear_left_sensor_detail",
    "/formal_visual/rear_right_sensor_detail",
    "/formal_visual/drivetrain_detail",
    "/formal_visual/inertial_power_detail",
    "/formal_visual/dry_deposition_internal",
    "/formal_visual/power_safety_internal",
    "/formal_visual/charge_interface_detail",
    "/formal_visual/drain_interface_detail",
)


class TriggeredWorldError(RuntimeError):
    """Raised when conversion cannot preserve the formal visual contract."""


def _normalized_path(path: Path, *, strict: bool) -> str:
    try:
        resolved = path.resolve(strict=strict)
    except (FileNotFoundError, OSError) as error:
        raise TriggeredWorldError(f"path cannot be resolved: {path}: {error}") from error
    return os.path.normcase(str(resolved))


def _validate_paths(source: Path, output: Path, report: Path | None = None) -> None:
    if not source.exists() or not source.is_file():
        raise TriggeredWorldError(f"source world is not a regular file: {source}")
    if source.is_symlink():
        raise TriggeredWorldError(f"refusing source-world alias path: {source}")

    paths = [
        ("source world", source, _normalized_path(source, strict=True)),
        ("output world", output, _normalized_path(output, strict=False)),
    ]
    if report is not None:
        paths.append(("report", report, _normalized_path(report, strict=False)))
    normalized = [item[2] for item in paths]
    if len(set(normalized)) != len(normalized):
        rendered = ", ".join(f"{name}={path}" for name, path, _ in paths)
        raise TriggeredWorldError(f"refusing aliased input/output paths: {rendered}")

    for name, path, _ in paths[1:]:
        if path.exists() or path.is_symlink():
            raise TriggeredWorldError(f"refusing stale {name}: {path}")


def _parser() -> ET.XMLParser:
    return ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))


def _text(element: ET.Element, path: str) -> str:
    value = element.findtext(path)
    return "" if value is None else value.strip()


def _camera_rows(world: ET.Element) -> list[tuple[ET.Element, ET.Element, str]]:
    sensors = list(world.findall(".//sensor"))
    if len(sensors) != len(EXPECTED_TOPICS):
        raise TriggeredWorldError(
            "source visual world must contain only its 19 camera sensors"
        )
    if any(sensor.get("type") != "camera" for sensor in sensors):
        raise TriggeredWorldError("source visual world contains a non-camera sensor")

    rows: list[tuple[ET.Element, ET.Element, str]] = []
    topics: list[str] = []
    for sensor in sensors:
        cameras = sensor.findall("camera")
        if len(cameras) != 1:
            raise TriggeredWorldError(
                "each formal camera sensor must contain exactly one camera element"
            )
        topic = _text(sensor, "topic")
        topics.append(topic)
        rows.append((sensor, cameras[0], topic))

    if len(set(topics)) != len(topics):
        raise TriggeredWorldError("source visual world contains duplicate camera topics")
    if tuple(topics) != EXPECTED_TOPICS:
        missing = sorted(set(EXPECTED_TOPICS) - set(topics))
        unexpected = sorted(set(topics) - set(EXPECTED_TOPICS))
        raise TriggeredWorldError(
            "source visual topics do not match the formal 19-camera contract: "
            f"missing={missing}, unexpected={unexpected}, order_exact={tuple(topics) == EXPECTED_TOPICS}"
        )
    return rows


def _camera_contract(world: ET.Element) -> dict[str, dict[str, object]]:
    contract: dict[str, dict[str, object]] = {}
    for model in world.findall("model"):
        model_pose = _text(model, "pose")
        for link in model.findall("link"):
            link_pose = _text(link, "pose")
            for sensor in link.findall("sensor"):
                if sensor.get("type") != "camera":
                    continue
                camera = sensor.find("camera")
                if camera is None:
                    continue
                topic = _text(sensor, "topic")
                contract[topic] = {
                    "model_name": model.get("name", ""),
                    "model_pose": model_pose,
                    "link_name": link.get("name", ""),
                    "link_pose": link_pose,
                    "sensor_name": sensor.get("name", ""),
                    "sensor_pose": _text(sensor, "pose"),
                    "horizontal_fov": _text(camera, "horizontal_fov"),
                    "width": _text(camera, "image/width"),
                    "height": _text(camera, "image/height"),
                    "format": _text(camera, "image/format"),
                    "camera_without_trigger": _element_contract(camera),
                }
    return contract


def _element_contract(element: ET.Element) -> dict[str, object]:
    """Return a whitespace-insensitive, JSON-safe element representation."""

    excluded = {"triggered", "trigger_topic", "triggered_topic"}
    children = [
        _element_contract(child)
        for child in list(element)
        if isinstance(child.tag, str) and child.tag not in excluded
    ]
    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": children,
    }


def _contract_sha256(contract: dict[str, dict[str, object]]) -> str:
    payload = json.dumps(contract, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def prepare_world(source: Path, output: Path) -> dict[str, object]:
    """Create a fresh triggered-camera world and return its binding report."""

    _validate_paths(source, output)
    source_bytes = source.read_bytes()
    tree = ET.ElementTree(ET.fromstring(source_bytes, parser=_parser()))
    root = tree.getroot()
    if root.tag != "sdf":
        raise TriggeredWorldError("source is not an SDF document")
    worlds = root.findall("world")
    if len(worlds) != 1:
        raise TriggeredWorldError("source SDF must contain exactly one world")
    world = worlds[0]

    rows = _camera_rows(world)
    before = _camera_contract(world)
    if set(before) != set(EXPECTED_TOPICS):
        raise TriggeredWorldError("camera placement contract is incomplete")

    trigger_bindings: list[dict[str, object]] = []
    for sensor, camera, topic in rows:
        if (
            sensor.findall("triggered")
            or sensor.findall("trigger_topic")
            or sensor.findall("triggered_topic")
        ):
            raise TriggeredWorldError(
                f"source camera trigger configuration is outside <camera>: {topic}"
            )
        if camera.findall("triggered"):
            raise TriggeredWorldError(
                f"source camera is already triggered and is not a pristine input: {topic}"
            )
        if camera.findall("trigger_topic") or camera.findall("triggered_topic"):
            raise TriggeredWorldError(
                f"source camera overrides the default trigger topic: {topic}"
            )
        triggered = ET.Element("triggered")
        triggered.text = "true"
        # Match the canonical gz-sensors triggered-camera SDF ordering.
        camera.insert(0, triggered)
        trigger_bindings.append(
            {
                "image_topic": topic,
                "trigger_topic": f"{topic}/trigger",
                "uses_default_trigger_topic": True,
            }
        )

    after = _camera_contract(world)
    if before != after:
        raise TriggeredWorldError(
            "camera pose, resolution, FOV, topic or non-trigger configuration drifted"
        )
    output_rows = _camera_rows(world)
    for sensor, camera, topic in output_rows:
        if (
            sensor.findall("triggered")
            or sensor.findall("trigger_topic")
            or sensor.findall("triggered_topic")
        ):
            raise TriggeredWorldError(
                f"camera trigger configuration escaped <camera>: {topic}"
            )
        triggered = camera.findall("triggered")
        if len(triggered) != 1 or (triggered[0].text or "").strip().lower() != "true":
            raise TriggeredWorldError(f"camera was not exactly trigger-enabled: {topic}")
        if camera.findall("trigger_topic") or camera.findall("triggered_topic"):
            raise TriggeredWorldError(
                f"camera no longer uses its default trigger topic: {topic}"
            )

    ET.indent(tree, space="  ")
    output_bytes = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    reparsed_root = ET.fromstring(output_bytes, parser=_parser())
    reparsed_worlds = reparsed_root.findall("world")
    if len(reparsed_worlds) != 1:
        raise TriggeredWorldError("serialized triggered world is invalid")
    reparsed_world = reparsed_worlds[0]
    _camera_rows(reparsed_world)
    if _camera_contract(reparsed_world) != before:
        raise TriggeredWorldError("serialized camera contract drifted")

    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output.open("xb") as stream:
            stream.write(output_bytes)
    except FileExistsError as error:
        raise TriggeredWorldError(f"refusing stale output world: {output}") from error

    contract_hash = _contract_sha256(before)
    return {
        "report_id": "tzcup_formal_triggered_visual_world_v1",
        "status": "FORMAL_TRIGGERED_VISUAL_WORLD_PREPARED",
        "passed": True,
        "source_world": str(source.resolve(strict=True)),
        "output_world": str(output.resolve(strict=True)),
        "source_world_sha256": _sha256_bytes(source_bytes),
        "output_world_sha256": _sha256_bytes(output_bytes),
        "camera_count": len(output_rows),
        "camera_contract_sha256_before": contract_hash,
        "camera_contract_sha256_after": _contract_sha256(
            _camera_contract(reparsed_world)
        ),
        "all_camera_contract_fields_preserved": True,
        "all_cameras_triggered": True,
        "all_cameras_use_default_trigger_topic": True,
        "trigger_bindings": trigger_bindings,
    }


def _write_report(report_path: Path, report: dict[str, object]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        with report_path.open("xb") as stream:
            stream.write(payload)
    except FileExistsError as error:
        raise TriggeredWorldError(f"refusing stale report: {report_path}") from error


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)

    _validate_paths(args.source_world, args.output_world, args.report)
    report = prepare_world(args.source_world, args.output_world)
    _write_report(args.report, report)
    print(report["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
