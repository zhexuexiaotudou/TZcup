import importlib.util
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/prepare_formal_triggered_visual_world.py"
SPEC = importlib.util.spec_from_file_location("prepare_triggered_visual_world", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/worlds/"
    "formal_vehicle_visual_acceptance.sdf"
)


def _sensor_contract(path: Path) -> dict[str, tuple[str, ...]]:
    root = ET.parse(path).getroot()
    result: dict[str, tuple[str, ...]] = {}
    for model in root.findall("./world/model"):
        for link in model.findall("link"):
            for sensor in link.findall("sensor[@type='camera']"):
                topic = (sensor.findtext("topic") or "").strip()
                camera = sensor.find("camera")
                assert camera is not None
                result[topic] = (
                    (model.findtext("pose") or "").strip(),
                    (link.findtext("pose") or "").strip(),
                    (sensor.findtext("pose") or "").strip(),
                    (camera.findtext("horizontal_fov") or "").strip(),
                    (camera.findtext("image/width") or "").strip(),
                    (camera.findtext("image/height") or "").strip(),
                    (camera.findtext("image/format") or "").strip(),
                )
    return result


def test_converts_all_19_cameras_without_contract_drift(tmp_path: Path) -> None:
    output = tmp_path / "triggered.sdf"
    before = _sensor_contract(SOURCE)
    report = MODULE.prepare_world(SOURCE, output)
    after = _sensor_contract(output)

    assert tuple(after) == MODULE.EXPECTED_TOPICS
    assert len(after) == 19
    assert after == before
    assert report["camera_count"] == 19
    assert report["camera_contract_sha256_before"] == report[
        "camera_contract_sha256_after"
    ]
    assert report["all_camera_contract_fields_preserved"] is True
    assert report["all_cameras_triggered"] is True
    assert report["all_cameras_use_default_trigger_topic"] is True

    tree = ET.parse(output)
    sensors = tree.findall(".//sensor")
    assert len(sensors) == 19
    for sensor in sensors:
        topic = (sensor.findtext("topic") or "").strip()
        camera = sensor.find("camera")
        assert camera is not None
        assert sensor.findall("triggered") == []
        assert sensor.findall("trigger_topic") == []
        assert sensor.findall("triggered_topic") == []
        assert [item.text for item in camera.findall("triggered")] == ["true"]
        assert camera.find("trigger_topic") is None
        assert camera.find("triggered_topic") is None
        binding = next(
            row for row in report["trigger_bindings"] if row["image_topic"] == topic
        )
        assert binding == {
            "image_topic": topic,
            "trigger_topic": f"{topic}/trigger",
            "uses_default_trigger_topic": True,
        }


def test_rejects_extra_sensor_and_duplicate_topic(tmp_path: Path) -> None:
    extra = tmp_path / "extra.sdf"
    source_text = SOURCE.read_text(encoding="utf-8")
    extra.write_text(
        source_text.replace(
            "</link>\n    </model>",
            '<sensor name="unexpected" type="imu"><topic>/unexpected</topic>'
            "</sensor></link>\n    </model>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(MODULE.TriggeredWorldError, match="only its 19"):
        MODULE.prepare_world(extra, tmp_path / "extra-output.sdf")

    duplicate = tmp_path / "duplicate.sdf"
    duplicate.write_text(
        source_text.replace(
            "/formal_visual/rear_right", "/formal_visual/front_left", 1
        ),
        encoding="utf-8",
    )
    with pytest.raises(MODULE.TriggeredWorldError, match="duplicate"):
        MODULE.prepare_world(duplicate, tmp_path / "duplicate-output.sdf")


def test_rejects_override_existing_trigger_stale_and_alias_paths(
    tmp_path: Path,
) -> None:
    overridden = tmp_path / "override.sdf"
    overridden.write_text(
        SOURCE.read_text(encoding="utf-8").replace(
            "<camera><horizontal_fov>",
            "<camera><trigger_topic>/wrong</trigger_topic><horizontal_fov>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(MODULE.TriggeredWorldError, match="overrides"):
        MODULE.prepare_world(overridden, tmp_path / "override-output.sdf")

    already_triggered = tmp_path / "already-triggered.sdf"
    already_triggered.write_text(
        SOURCE.read_text(encoding="utf-8").replace(
            "<camera><horizontal_fov>",
            "<camera><triggered>true</triggered><horizontal_fov>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(MODULE.TriggeredWorldError, match="already triggered"):
        MODULE.prepare_world(already_triggered, tmp_path / "triggered-output.sdf")

    misplaced = tmp_path / "misplaced-triggered.sdf"
    misplaced.write_text(
        SOURCE.read_text(encoding="utf-8").replace(
            "<camera><horizontal_fov>",
            "<triggered>true</triggered><camera><horizontal_fov>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(MODULE.TriggeredWorldError, match="outside <camera>"):
        MODULE.prepare_world(misplaced, tmp_path / "misplaced-output.sdf")

    stale = tmp_path / "stale.sdf"
    stale.write_text("stale", encoding="utf-8")
    with pytest.raises(MODULE.TriggeredWorldError, match="stale"):
        MODULE.prepare_world(SOURCE, stale)

    alias_source = tmp_path / "formal_vehicle_visual_acceptance.sdf"
    shutil.copyfile(SOURCE, alias_source)
    with pytest.raises(MODULE.TriggeredWorldError, match="aliased"):
        MODULE.prepare_world(alias_source, alias_source.parent / "." / alias_source.name)


def test_cli_writes_fresh_bound_report_and_rejects_report_alias(tmp_path: Path) -> None:
    output = tmp_path / "triggered.sdf"
    report_path = tmp_path / "triggered.json"
    assert (
        MODULE.main(
            [
                "--source-world",
                str(SOURCE),
                "--output-world",
                str(output),
                "--report",
                str(report_path),
            ]
        )
        == 0
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "FORMAL_TRIGGERED_VISUAL_WORLD_PREPARED"
    assert report["output_world_sha256"] == MODULE._sha256_bytes(output.read_bytes())

    alias_output = tmp_path / "alias.sdf"
    with pytest.raises(MODULE.TriggeredWorldError, match="aliased"):
        MODULE.main(
            [
                "--source-world",
                str(SOURCE),
                "--output-world",
                str(alias_output),
                "--report",
                str(alias_output.parent / "." / alias_output.name),
            ]
        )
