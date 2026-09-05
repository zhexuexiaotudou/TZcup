import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/prepare_formal_visual_single_topic_world.py"
SPEC = importlib.util.spec_from_file_location("prepare_single_visual_world", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_prepared_world_contains_only_requested_formal_camera(tmp_path: Path) -> None:
    source = ROOT / (
        "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    output = tmp_path / "single.sdf"
    report = MODULE.prepare_world(source, output, "/formal_visual/front_left")

    tree = ET.parse(output)
    topics = [
        (sensor.findtext("topic") or "").strip()
        for sensor in tree.findall(".//sensor[@type='camera']")
    ]
    assert topics == ["/formal_visual/front_left"]
    assert report["remaining_formal_visual_camera_count"] == 1
    assert report["remaining_total_sensor_count"] == 1
    assert report["removed_visual_camera_model_count"] == 18
    assert tree.find(".//plugin[@filename='gz-sim-sensors-system']") is not None
    assert tree.find(".//model[@name='studio_ground']") is not None


def test_prepared_world_rejects_unknown_topic_and_stale_output(tmp_path: Path) -> None:
    source = ROOT / (
        "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    output = tmp_path / "single.sdf"
    with pytest.raises(MODULE.DiagnosticWorldError, match="absent"):
        MODULE.prepare_world(source, output, "/formal_visual/not_registered")

    output.write_text("stale", encoding="utf-8")
    with pytest.raises(MODULE.DiagnosticWorldError, match="stale"):
        MODULE.prepare_world(source, output, "/formal_visual/front_left")


def test_prepared_world_rejects_any_extra_sensor(tmp_path: Path) -> None:
    source = ROOT / (
        "starter_ws/src/sanitation_vehicle_description/worlds/"
        "formal_vehicle_visual_acceptance.sdf"
    )
    modified = tmp_path / "extra_sensor.sdf"
    text = source.read_text(encoding="utf-8").replace(
        "</link>\n    </model>",
        '<sensor name="unexpected" type="depth_camera"><topic>/other</topic>'
        "<camera><image><width>1</width><height>1</height></image></camera>"
        "</sensor></link>\n    </model>",
        1,
    )
    modified.write_text(text, encoding="utf-8")
    with pytest.raises(MODULE.DiagnosticWorldError, match="only its 19"):
        MODULE.prepare_world(
            modified, tmp_path / "single.sdf", "/formal_visual/front_left"
        )
