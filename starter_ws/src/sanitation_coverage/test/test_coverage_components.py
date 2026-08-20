import math

import pytest

from sanitation_coverage.coverage_components import (
    connector_handoff_replan_decision,
)


def test_small_static_connector_handoff_remains_precomputed():
    decision = connector_handoff_replan_decision(
        (10.40, 20.10, 0.20),
        (10.00, 20.00, 0.00),
    )
    assert decision["requires_replan"] is False
    assert decision["position_error_m"] == pytest.approx(math.hypot(0.4, 0.1))
    assert decision["heading_error_rad"] == pytest.approx(0.20)


def test_large_heading_error_requires_live_connector_replan():
    decision = connector_handoff_replan_decision(
        (7.79, 22.10, -2.4088),
        (7.68, 21.80, math.pi),
    )
    assert decision["requires_replan"] is True
    assert decision["position_error_m"] < 0.75
    assert decision["heading_error_rad"] == pytest.approx(0.7328, abs=1e-4)


def test_large_position_error_requires_live_connector_replan():
    decision = connector_handoff_replan_decision(
        (11.00, 20.00, math.pi - 0.05),
        (10.00, 20.00, -math.pi + 0.05),
    )
    assert decision["requires_replan"] is True
    assert decision["position_error_m"] == pytest.approx(1.0)
    assert decision["heading_error_rad"] == pytest.approx(0.10)
