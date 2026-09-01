from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from prepare_formal_dynamic_runtime_world import prepare_runtime_world


def _write_world(path: Path, *, with_contact: bool = False) -> None:
    contact = (
        '<plugin filename="gz-sim-contact-system" '
        'name="gz::sim::systems::Contact" />'
        if with_contact
        else ""
    )
    path.write_text(
        f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="campus_formal">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
  {contact}
  <model name="ground"><static>true</static></model>
  <model name="walker_b"><static>true</static><pose>1 2 0 0 0 0</pose></model>
  <model name="walker_a"><static>true</static><pose>3 4 0 0 0 0</pose></model>
</world></sdf>
""",
        encoding="utf-8",
    )


def test_adds_only_contact_system_and_records_source_identity(tmp_path: Path) -> None:
    source = tmp_path / "source.sdf"
    output = tmp_path / "runtime.sdf"
    _write_world(source)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    report = prepare_runtime_world(source, output)
    root = ET.parse(output).getroot()
    world = root.find("world")
    assert world is not None
    assert report["source_world_sha256"] == digest
    assert report["contact_system_added"] is True
    assert report["world_preserved_except_contact_system"] is True
    assert report["pedestrian_model_ids"] == ["walker_a", "walker_b"]
    assert len(
        [
            plugin
            for plugin in world.findall("plugin")
            if plugin.attrib.get("name") == "gz::sim::systems::Contact"
        ]
    ) == 1


def test_existing_contact_system_is_not_duplicated(tmp_path: Path) -> None:
    source = tmp_path / "source.sdf"
    output = tmp_path / "runtime.sdf"
    _write_world(source, with_contact=True)
    report = prepare_runtime_world(source, output)
    assert report["contact_system_added"] is False
    assert report["contact_system_plugin_count"] == 1
    assert report["world_preserved_except_contact_system"] is True
