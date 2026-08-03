import math
import pytest

from sanitation_coverage.coverage_components import ComponentType
from sanitation_coverage.skid_steer_connector import plan_skid_steer_connector


SAFE = [(-1, -1), (3, -1), (3, 3), (-1, 3)]


def test_rtr_connector_uses_semantic_rotate_shift_rotate():
    components = plan_skid_steer_connector("c0", (0, 0), 0, (0, 1), math.pi, SAFE, False)
    assert [item.kind for item in components] == [ComponentType.ROTATE, ComponentType.SHIFT, ComponentType.ROTATE]
    assert all(not item.brush_enabled for item in components)


def test_large_heading_change_prefers_bounded_backup():
    components = plan_skid_steer_connector("c1", (1, 1), 0, (0, 1), 0, SAFE, True)
    assert ComponentType.BACKUP in [item.kind for item in components]


def test_translation_outside_safe_polygon_fails_closed():
    with pytest.raises(ValueError, match="safe polygon"):
        plan_skid_steer_connector("c2", (0, 0), 0, (4, 0), 0, SAFE)
