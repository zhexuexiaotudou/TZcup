import math
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from formal_safety_status_schema import validate_speed_qualification_status


def _payload(**overrides):
    return {
        "effective_max_linear_velocity_mps": 0.45,
        "operation_speed_profile": "",
        "speed_qualification_state": "none",
        **overrides,
    }


def test_default_cap_is_accepted():
    validate_speed_qualification_status(_payload())


def test_exact_qualified_cap_is_accepted():
    validate_speed_qualification_status(_payload(
        effective_max_linear_velocity_mps=1.0,
        operation_speed_profile="dry_cleaning_competition_candidate",
        speed_qualification_state="isolated_same_map_dry_coverage",
    ))


@pytest.mark.parametrize("cap", [True, 1, 0.0, -0.1, 1.1, math.nan, math.inf])
def test_invalid_cap_is_rejected(cap):
    with pytest.raises(ValueError, match="effective_max_linear_velocity_mps"):
        validate_speed_qualification_status(_payload(effective_max_linear_velocity_mps=cap))


@pytest.mark.parametrize("field, value", [
    ("operation_speed_profile", "unknown"),
    ("operation_speed_profile", ["mapping_safe"]),
    ("speed_qualification_state", "unknown"),
    ("speed_qualification_state", None),
])
def test_unknown_speed_enums_are_rejected(field, value):
    with pytest.raises(ValueError):
        validate_speed_qualification_status(_payload(**{field: value}))


@pytest.mark.parametrize("overrides", [
    {"effective_max_linear_velocity_mps": 0.7},
    {"effective_max_linear_velocity_mps": 1.0, "operation_speed_profile": "mapping_safe"},
    {
        "effective_max_linear_velocity_mps": 1.0,
        "speed_qualification_state": "isolated_same_map_dry_coverage",
    },
    {"effective_max_linear_velocity_mps": 1.0, "speed_qualification_state": "none"},
])
def test_unqualified_speed_above_default_is_rejected(overrides):
    with pytest.raises(ValueError, match="requires the exact dry qualification"):
        validate_speed_qualification_status(_payload(**overrides))
