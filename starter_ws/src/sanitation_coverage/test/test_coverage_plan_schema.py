import pytest

from sanitation_coverage.coverage_components import ComponentType, CoverageComponent
from sanitation_coverage.coverage_plan import CoveragePlan


def test_plan_round_trip_preserves_semantics_and_length():
    plan = CoveragePlan(
        mission_id="demo",
        frame_id="map",
        components=(
            CoverageComponent("s0", ComponentType.SWATH, ((0, 0), (2, 0)), True, "CLEAN"),
            CoverageComponent("r0", ComponentType.ROTATE, ((2, 0),), False, "TURN", {"delta_yaw_rad": 3.14}),
            CoverageComponent("x0", ComponentType.SHIFT, ((2, 0), (2, 0.5)), False, "SHIFT"),
        ),
    )
    restored = CoveragePlan.from_dict(plan.to_dict())
    assert restored.components[0].kind is ComponentType.SWATH
    assert restored.total_length_m == pytest.approx(2.5)
    assert restored.to_dict()["schema"] == "tzcup.coverage_plan.v1"


def test_non_cleaning_component_cannot_enable_brush():
    with pytest.raises(ValueError, match="brush_enabled"):
        CoverageComponent("bad", ComponentType.TRANSIT, ((0, 0), (1, 0)), True)
