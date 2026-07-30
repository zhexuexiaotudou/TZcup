from pathlib import Path

import pytest

from sanitation_debug_visualization.model import (
    MarkerSpec,
    build_static_specs,
    load_yaml,
    predicted_specs,
    status_text,
    transform_specs_to_vehicle,
    vehicle_specs,
)


ROOT = Path(__file__).resolve().parents[2]


def fixture_path(package_name: str, relative_path: str) -> Path:
    source_path = ROOT / package_name / relative_path
    if source_path.is_file():
        return source_path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package_name)) / relative_path


def fixtures():
    registry = load_yaml(
        fixture_path(
            "sanitation_perception",
            "config/garbage_registry.yaml",
        )
    )
    scene = load_yaml(
        fixture_path(
            "sanitation_ground_truth",
            "config/stage5a_scene.yaml",
        )
    )
    mission = load_yaml(
        fixture_path(
            "sanitation_tasks",
            "config/demo_area.yaml",
        )
    )
    return registry, scene, mission


def test_static_scene_exposes_targets_obstacles_and_zones():
    registry, scene, mission = fixtures()
    specs = build_static_specs(registry, scene, mission)
    namespaces = {spec.namespace for spec in specs}
    assert {
        "GT_targets",
        "GT_labels",
        "GT_halos",
        "obstacles",
        "zones",
    } <= namespaces
    labels = {spec.text for spec in specs if spec.kind == "text"}
    assert "GT | BOTTLE" in labels
    assert "GT | PUDDLE" in labels
    assert "OBSTACLE | PEDESTRIAN" in labels
    bottle = next(
        spec
        for spec in specs
        if spec.namespace == "GT_targets" and spec.key == "trash_bottle_01_shape"
    )
    assert bottle.position[:2] == pytest.approx((2.6, -0.55))


def test_cleaned_target_changes_status_and_color():
    registry, scene, mission = fixtures()
    bottle_uuid = registry["models"]["trash_bottle_01"]["uuid"]
    specs = build_static_specs(registry, scene, mission, {bottle_uuid})
    bottle_label = next(
        spec
        for spec in specs
        if spec.namespace == "GT_labels" and spec.key == "trash_bottle_01_label"
    )
    bottle_shape = next(
        spec
        for spec in specs
        if spec.namespace == "GT_targets" and spec.key == "trash_bottle_01_shape"
    )
    assert bottle_label.text == "CLEANED | BOTTLE"
    assert bottle_shape.color[1] > 0.9


def test_prediction_labels_show_source_and_confidence():
    specs = predicted_specs(
        [
            {
                "uuid": "track-1",
                "class_id": "metal_can",
                "confidence": 0.876,
                "position": (1.0, 2.0, 0.1),
                "size": (0.07, 0.07, 0.12),
            }
        ]
    )
    label = next(spec for spec in specs if spec.kind == "text")
    assert label.text == "PRED | CAN 0.88"


def test_marker_ids_are_stable_and_namespace_sensitive():
    first = MarkerSpec(namespace="a", key="one", kind="cube").marker_id
    repeated = MarkerSpec(namespace="a", key="one", kind="cube").marker_id
    other = MarkerSpec(namespace="b", key="one", kind="cube").marker_id
    assert first == repeated
    assert first != other


def test_status_text_is_operator_readable():
    text = status_text(
        prediction_count=3,
        truth_visible_count=5,
        cleaned_count=1,
        brush_enabled=True,
        coverage_state="RUNNING",
        spot_state="deferred | QUEUED 2",
    )
    assert "PRED: 3" in text
    assert "BRUSH: ON" in text
    assert "COVERAGE: RUNNING" in text


def test_vehicle_marker_is_visible_without_robot_model_tf():
    specs = vehicle_specs(1.2, -0.4, 0.5)
    assert {spec.kind for spec in specs} == {"cube", "arrow", "text"}
    assert all(spec.position[:2] == (1.2, -0.4) for spec in specs)
    assert next(spec for spec in specs if spec.kind == "text").text == "VEHICLE"


def test_vehicle_frame_transform_keeps_world_fixed_while_robot_moves():
    source = [
        MarkerSpec(
            namespace="target",
            key="one",
            kind="cube",
            position=(3.0, 2.0, 0.1),
            yaw=0.5,
            points=((3.0, 2.0, 0.0),),
        )
    ]
    transformed = transform_specs_to_vehicle(source, 1.0, 2.0, 0.0)[0]
    assert transformed.position == pytest.approx((2.0, 0.0, 0.1))
    assert transformed.points[0] == pytest.approx((2.0, 0.0, 0.0))
    assert transformed.yaw == pytest.approx(0.5)

    rotated = transform_specs_to_vehicle(source, 1.0, 2.0, 1.57079632679)[0]
    assert rotated.position[:2] == pytest.approx((0.0, -2.0), abs=1e-9)


def test_static_target_labels_are_staggered_for_operator_readability():
    registry, scene, mission = fixtures()
    specs = build_static_specs(registry, scene, mission)
    bottle_shape = next(
        spec
        for spec in specs
        if spec.namespace == "GT_targets" and spec.key == "trash_bottle_01_shape"
    )
    bottle_label = next(
        spec
        for spec in specs
        if spec.namespace == "GT_labels" and spec.key == "trash_bottle_01_label"
    )
    assert bottle_label.position[:2] != bottle_shape.position[:2]
