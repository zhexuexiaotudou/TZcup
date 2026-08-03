import pytest

from sanitation_coverage.taught_route import compile_taught_route, seal_taught_route


SAFE = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]


def route_payload():
    return seal_taught_route({
        "route_id": "corridor_a",
        "version": "1.0.0",
        "frame_id": "map",
        "allowed_direction": "FORWARD",
        "poses": [
            {"x": -1.0, "y": 0.0, "yaw": 0.0, "speed_limit_mps": 0.3, "brush_enabled": False},
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed_limit_mps": 0.25, "brush_enabled": True},
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "speed_limit_mps": 0.25, "brush_enabled": True},
        ],
        "no_clean_sections": [],
        "interaction_points": [{"pose_index": 1, "kind": "gate"}],
        "recovery_points": [{"pose_index": 0, "kind": "staging"}],
    })


def test_taught_route_compiles_to_nav2_semantic_components():
    plan = compile_taught_route(route_payload(), SAFE)

    assert plan.route_mode == "TAUGHT_ROUTE"
    assert len(plan.components) == 2
    assert not plan.components[0].brush_enabled
    assert plan.components[1].brush_enabled
    assert all(item.metadata["executor"] == "Nav2 FollowPath" for item in plan.components)
    assert all(item.metadata["collision_checked"] for item in plan.components)


def test_taught_route_hash_tampering_fails_closed():
    payload = route_payload()
    payload["poses"][1]["x"] = 0.5

    with pytest.raises(ValueError, match="sha256 mismatch"):
        compile_taught_route(payload, SAFE)


def test_taught_route_outside_safe_polygon_fails_closed():
    payload = route_payload()
    payload["poses"][2]["x"] = 3.0
    payload = seal_taught_route(payload)

    with pytest.raises(ValueError, match="footprint-safe polygon"):
        compile_taught_route(payload, SAFE)


def test_no_clean_section_rejects_brush_enabled_pose():
    payload = route_payload()
    payload["no_clean_sections"] = [{"start_index": 1, "end_index": 2}]
    payload = seal_taught_route(payload)

    with pytest.raises(ValueError, match="no-clean section"):
        compile_taught_route(payload, SAFE)
