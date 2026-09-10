import json
from pathlib import Path

import pytest

from prepare_formal_mapping_world import prepare


def _inputs(tmp_path: Path, *, expected: int = 2) -> tuple[Path, Path]:
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf version='1.10'><world name='campus_formal'>"
        "<model name='building'><static>true</static></model>"
        "<model name='walker_a'><static>true</static></model>"
        "<model name='walker_b'><static>true</static></model>"
        "</world></sdf>",
        encoding="utf-8",
    )
    manifest = tmp_path / "episode.json"
    manifest.write_text(
        json.dumps({"counts": {"pedestrians": expected}}), encoding="utf-8"
    )
    return world, manifest


def test_mapping_world_removes_exact_manifest_walker_count(tmp_path):
    world, manifest = _inputs(tmp_path)
    output = tmp_path / "mapping.sdf"
    report = prepare(world, manifest, output)
    assert report["removed_pedestrian_count"] == 2
    assert report["remaining_pedestrian_count"] == 0
    assert report["product_control_input"] is False
    assert report["contact_system_plugin_count"] == 1
    assert report["physics_step"] == pytest.approx(0.005)
    text = output.read_text(encoding="utf-8")
    assert "walker_" not in text
    assert "building" in text
    assert "gz-sim-contact-system" in text
    assert 'physics name="mapping_5ms" type="ignored"' in text
    assert "<max_step_size>0.005</max_step_size>" in text


def test_mapping_world_fails_closed_on_manifest_count_mismatch(tmp_path):
    world, manifest = _inputs(tmp_path, expected=3)
    with pytest.raises(ValueError, match="disagrees"):
        prepare(world, manifest, tmp_path / "mapping.sdf")


@pytest.mark.parametrize(
    ("physics", "error"),
    [
        ("<physics name='missing' type='ignored'/>", "no max_step_size"),
        (
            "<physics name='one_ms' type='ignored'><max_step_size>0.001</max_step_size></physics>",
            "must be 0.005",
        ),
        (
            "<physics name='first' type='ignored'><max_step_size>0.005</max_step_size></physics>"
            "<physics name='second' type='ignored'><max_step_size>0.005</max_step_size></physics>",
            "multiple physics profiles",
        ),
    ],
)
def test_mapping_world_fails_closed_on_incompatible_source_physics(tmp_path, physics, error):
    world, manifest = _inputs(tmp_path)
    world.write_text(
        "<sdf version='1.10'><world name='campus_formal'>"
        + physics
        + "<model name='building'><static>true</static></model>"
        "<model name='walker_a'><static>true</static></model>"
        "<model name='walker_b'><static>true</static></model>"
        "</world></sdf>",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=error):
        prepare(world, manifest, tmp_path / "mapping.sdf")


def test_mapping_world_reuses_single_compatible_source_physics(tmp_path):
    world, manifest = _inputs(tmp_path)
    world.write_text(
        "<sdf version='1.10'><world name='campus_formal'>"
        "<physics name='mapping_5ms' type='ignored'><max_step_size>0.005</max_step_size>"
        "<real_time_factor>1.0</real_time_factor></physics>"
        "<model name='building'><static>true</static></model>"
        "<model name='walker_a'><static>true</static></model>"
        "<model name='walker_b'><static>true</static></model>"
        "</world></sdf>",
        encoding="utf-8",
    )
    output = tmp_path / "mapping.sdf"
    report = prepare(world, manifest, output)
    assert report["physics_step"] == pytest.approx(0.005)
    assert output.read_text(encoding="utf-8").count("<physics") == 1
