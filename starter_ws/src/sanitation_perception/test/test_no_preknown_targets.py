import json

from sanitation_perception.dynamic_trash_map import DynamicTrashMap


def test_new_mission_is_empty_even_if_a_persisted_other_mission_exists(tmp_path):
    old_map = DynamicTrashMap.start_new("old-mission")
    persisted = tmp_path / "old-map.json"
    old_map.persist(persisted)

    new_map = DynamicTrashMap.start_new("new-mission")
    assert new_map.count == 0
    assert new_map.snapshot()["preknown_target_coordinates_used"] is False

    payload = json.loads(persisted.read_text(encoding="utf-8"))
    payload["targets"] = [{"uuid": "registry-target"}]
    persisted.write_text(json.dumps(payload), encoding="utf-8")
    try:
        DynamicTrashMap.resume_same_mission(persisted, "new-mission")
    except ValueError as exc:
        assert "different mission" in str(exc)
    else:
        raise AssertionError("cross-mission target restore was allowed")


def test_no_registry_bootstrap_api_is_exposed():
    assert not hasattr(DynamicTrashMap, "from_gazebo_registry")
    assert not hasattr(DynamicTrashMap, "from_evaluation_registry")
