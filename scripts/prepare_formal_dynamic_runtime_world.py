#!/usr/bin/env python3
"""Add the Gazebo contact system without changing campus world contents."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


CONTACT_PLUGIN_FILENAME = "gz-sim-contact-system"
CONTACT_PLUGIN_NAME = "gz::sim::systems::Contact"


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _normalized(element: ET.Element) -> tuple[Any, ...]:
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_normalized(child) for child in element),
    )


def prepare_runtime_world(source: Path, output: Path) -> dict[str, Any]:
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("source SDF has no world element")
    source_without_contact = copy.deepcopy(root)
    source_world = source_without_contact.find("world")
    assert source_world is not None
    source_plugins = [
        plugin
        for plugin in world.findall("plugin")
        if plugin.attrib.get("name") == CONTACT_PLUGIN_NAME
    ]
    if len(source_plugins) > 1:
        raise ValueError("source world contains duplicate Contact systems")
    for plugin in list(source_world.findall("plugin")):
        if plugin.attrib.get("name") == CONTACT_PLUGIN_NAME:
            source_world.remove(plugin)

    added = not source_plugins
    if added:
        plugin = ET.Element(
            "plugin",
            {"filename": CONTACT_PLUGIN_FILENAME, "name": CONTACT_PLUGIN_NAME},
        )
        plugin_count = len(world.findall("plugin"))
        world.insert(plugin_count, plugin)

    output_without_contact = copy.deepcopy(root)
    output_world = output_without_contact.find("world")
    assert output_world is not None
    for plugin in list(output_world.findall("plugin")):
        if plugin.attrib.get("name") == CONTACT_PLUGIN_NAME:
            output_world.remove(plugin)
    unchanged = _normalized(source_without_contact) == _normalized(
        output_without_contact
    )
    if not unchanged:
        raise AssertionError("runtime world changed beyond Contact system insertion")

    contact_plugins = [
        plugin
        for plugin in world.findall("plugin")
        if plugin.attrib.get("name") == CONTACT_PLUGIN_NAME
    ]
    if len(contact_plugins) != 1:
        raise AssertionError("runtime world must contain exactly one Contact system")
    walker_ids = sorted(
        model.attrib.get("name", "")
        for model in world.findall("model")
        if model.attrib.get("name", "").startswith("walker_")
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return {
        "schema_version": 1,
        "source_world": str(source.resolve()),
        "source_world_sha256": _digest(source),
        "runtime_world": str(output.resolve()),
        "runtime_world_sha256": _digest(output),
        "world_name": world.attrib.get("name"),
        "contact_system_plugin_count": len(contact_plugins),
        "contact_system_added": added,
        "world_preserved_except_contact_system": unchanged,
        "pedestrian_model_count": len(walker_ids),
        "pedestrian_model_ids": walker_ids,
        "claim_boundary": (
            "Evaluator/runtime instrumentation adds only Gazebo's Contact system. "
            "All campus models, poses, collisions, sensors and walker identities "
            "remain those of the admitted public cleaning world."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    manifest = prepare_runtime_world(args.source, args.output)
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
