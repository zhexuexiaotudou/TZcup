#!/usr/bin/env python3
"""Create a first-task mapping world without parked dynamic pedestrians.

This is an evaluator-side environment preparation step.  The product mapping
graph still receives only lidar, odometry and the public geofence contract.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CONTACT_PLUGIN_FILENAME = "gz-sim-contact-system"
CONTACT_PLUGIN_NAME = "gz::sim::systems::Contact"
MAPPING_PHYSICS_STEP_S = 0.005


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _ensure_mapping_physics(world: ET.Element) -> float:
    """Require the bounded 5 ms mapping physics profile exactly once."""
    physics_nodes = world.findall("physics")
    if len(physics_nodes) > 1:
        raise ValueError("mapping world contains multiple physics profiles")
    if not physics_nodes:
        physics = ET.Element("physics", {"name": "mapping_5ms", "type": "ignored"})
        ET.SubElement(physics, "max_step_size").text = str(MAPPING_PHYSICS_STEP_S)
        ET.SubElement(physics, "real_time_factor").text = "1.0"
        # Keep the SDF physics declaration ahead of systems/plugins and retain
        # every source-world model, collision and visual unchanged.
        world.insert(0, physics)
        return MAPPING_PHYSICS_STEP_S

    step_text = physics_nodes[0].findtext("max_step_size")
    if step_text is None:
        raise ValueError("mapping world physics profile has no max_step_size")
    try:
        step = float(step_text)
    except ValueError as exc:
        raise ValueError("mapping world physics max_step_size is invalid") from exc
    if not math.isfinite(step) or not math.isclose(
        step, MAPPING_PHYSICS_STEP_S, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError(
            "mapping world physics max_step_size must be "
            f"{MAPPING_PHYSICS_STEP_S:g}, got {step_text!r}"
        )
    return step


def prepare(source: Path, episode_manifest: Path, output: Path) -> dict:
    manifest = json.loads(episode_manifest.read_text(encoding="utf-8"))
    expected = int(manifest.get("counts", {}).get("pedestrians", -1))
    if expected < 0:
        raise ValueError("public episode manifest has no pedestrian count")
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("SDF has no world element")
    walkers = [
        model
        for model in world.findall("model")
        if (model.get("name") or "").startswith("walker_")
    ]
    if len(walkers) != expected:
        raise ValueError(
            f"world walker count {len(walkers)} disagrees with manifest {expected}"
        )
    names: list[str] = []
    for model in walkers:
        name = model.get("name") or ""
        static = (model.findtext("static") or "").strip().lower()
        if static != "true":
            raise ValueError(f"mapping-world walker is not parked/static: {name}")
        names.append(name)
        world.remove(model)
    contact_plugins = [
        plugin
        for plugin in world.findall("plugin")
        if plugin.get("name") == CONTACT_PLUGIN_NAME
    ]
    if len(contact_plugins) > 1:
        raise ValueError("source world contains duplicate Contact systems")
    contact_added = not contact_plugins
    if contact_added:
        world.insert(
            len(world.findall("plugin")),
            ET.Element(
                "plugin",
                {"filename": CONTACT_PLUGIN_FILENAME, "name": CONTACT_PLUGIN_NAME},
            ),
        )
    physics_step = _ensure_mapping_physics(world)
    if len(world.findall("physics")) != 1:
        raise ValueError("mapping world must contain exactly one physics profile")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    check_world = ET.parse(output).getroot().find("world")
    remaining = [] if check_world is None else [
        model.get("name")
        for model in check_world.findall("model")
        if (model.get("name") or "").startswith("walker_")
    ]
    if remaining:
        raise ValueError(f"mapping world still contains walkers: {remaining}")
    check_contacts = [] if check_world is None else [
        plugin
        for plugin in check_world.findall("plugin")
        if plugin.get("name") == CONTACT_PLUGIN_NAME
    ]
    if len(check_contacts) != 1:
        raise ValueError("mapping world must contain exactly one Contact system")
    check_physics = [] if check_world is None else check_world.findall("physics")
    if len(check_physics) != 1:
        raise ValueError("mapping world must contain exactly one physics profile")
    return {
        "schema_version": 1,
        "status": "FORMAL_MAPPING_WORLD_PEDESTRIAN_EXCLUSION_PASSED",
        "passed": True,
        "environment_preparation_only": True,
        "product_control_input": False,
        "reason": (
            "Parked static walker models would become first-map ghost obstacles; "
            "dynamic-pedestrian cleaning acceptance uses the unmodified world."
        ),
        "source_world": str(source),
        "source_world_sha256": _sha256(source),
        "mapping_world": str(output),
        "mapping_world_sha256": _sha256(output),
        "manifest_expected_pedestrians": expected,
        "removed_pedestrian_models": sorted(names),
        "removed_pedestrian_count": len(names),
        "remaining_pedestrian_count": 0,
        "contact_system_plugin_count": 1,
        "contact_system_added": contact_added,
        "physics_step": physics_step,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    report = prepare(args.source, args.episode_manifest, args.output)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
