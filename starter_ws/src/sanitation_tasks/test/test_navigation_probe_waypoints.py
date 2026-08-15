import json

import pytest

from sanitation_tasks.route_config import load_waypoints


def test_load_waypoints_from_json_or_file(tmp_path):
    payload = [[1, 2, 0], [3.5, 4.5, 1.57]]
    assert load_waypoints(json.dumps(payload)) == [
        (1.0, 2.0, 0.0),
        (3.5, 4.5, 1.57),
    ]
    path = tmp_path / "route.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    assert load_waypoints("[]", str(path)) == [
        (1.0, 2.0, 0.0),
        (3.5, 4.5, 1.57),
    ]


@pytest.mark.parametrize("payload", [[], [[1, 2]], [[1, 2, float("nan")]]])
def test_load_waypoints_rejects_invalid_payload(payload):
    with pytest.raises(ValueError):
        load_waypoints(json.dumps(payload))
