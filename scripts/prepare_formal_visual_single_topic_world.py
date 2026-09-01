#!/usr/bin/env python3
"""Create a diagnostic visual world containing exactly one camera sensor."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


class DiagnosticWorldError(RuntimeError):
    """Raised when the source world does not match the visual contract."""


def _sensor_topic(sensor: ET.Element) -> str:
    topic = sensor.find("topic")
    return "" if topic is None or topic.text is None else topic.text.strip()


def prepare_world(source: Path, output: Path, topic: str) -> dict[str, object]:
    if not topic.startswith("/formal_visual/"):
        raise DiagnosticWorldError("topic must be under /formal_visual/")
    if output.exists() or output.is_symlink():
        raise DiagnosticWorldError(f"refusing stale diagnostic world: {output}")

    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise DiagnosticWorldError("source SDF has no world")

    visual_models: list[tuple[ET.Element, list[str]]] = []
    all_topics: list[str] = []
    all_sensors = list(world.findall(".//sensor"))
    for model in list(world.findall("model")):
        topics = [
            _sensor_topic(sensor)
            for sensor in model.findall(".//sensor[@type='camera']")
        ]
        topics = [item for item in topics if item.startswith("/formal_visual/")]
        if topics:
            visual_models.append((model, topics))
            all_topics.extend(topics)

    if len(all_topics) != 19 or len(set(all_topics)) != 19:
        raise DiagnosticWorldError(
            "source world must contain exactly 19 unique formal visual cameras"
        )
    if len(all_sensors) != 19:
        raise DiagnosticWorldError(
            "source visual world must contain only its 19 camera sensors"
        )
    if topic not in all_topics:
        raise DiagnosticWorldError(f"requested topic is absent from source world: {topic}")

    removed_models: list[str] = []
    retained_models: list[str] = []
    for model, topics in visual_models:
        name = model.get("name", "")
        if topics == [topic]:
            retained_models.append(name)
        else:
            world.remove(model)
            removed_models.append(name)

    remaining_sensors = list(world.findall(".//sensor"))
    remaining = [_sensor_topic(sensor) for sensor in remaining_sensors]
    if (
        remaining != [topic]
        or len(retained_models) != 1
        or remaining_sensors[0].get("type") != "camera"
    ):
        raise DiagnosticWorldError("single-topic world reduction was not exact")

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return {
        "report_id": "tzcup_formal_visual_single_topic_world_v1",
        "status": "FORMAL_VISUAL_SINGLE_TOPIC_WORLD_PREPARED",
        "passed": True,
        "source_world": str(source),
        "output_world": str(output),
        "retained_topic": topic,
        "retained_models": retained_models,
        "removed_visual_camera_model_count": len(removed_models),
        "remaining_formal_visual_camera_count": len(remaining),
        "remaining_total_sensor_count": len(remaining_sensors),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    resolved_paths = {
        args.source_world.resolve(strict=False),
        args.output_world.resolve(strict=False),
        args.report.resolve(strict=False),
    }
    if len(resolved_paths) != 3:
        raise DiagnosticWorldError(
            "source world, output world and report must be distinct paths"
        )
    if args.report.exists() or args.report.is_symlink():
        raise DiagnosticWorldError(f"refusing stale report: {args.report}")
    report = prepare_world(args.source_world, args.output_world, args.topic)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(report["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
