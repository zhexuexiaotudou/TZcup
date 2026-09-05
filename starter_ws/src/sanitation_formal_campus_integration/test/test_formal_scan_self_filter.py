import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sanitation_formal_campus_integration.scan_self_filter_core import (
    filter_ranges,
    is_self_return,
    parse_masks,
)


MASKS = parse_masks((
    math.radians(-135.0), math.radians(-123.5), 0.18,
    math.radians(119.0), math.radians(135.0), 0.31,
))


def test_committed_filter_contract_matches_mesh_ray_audit():
    config_path = Path(__file__).resolve().parents[1] / "config" / (
        "formal_utm30lx_self_filter.yaml"
    )
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "formal_scan_self_filter"
    ]["ros__parameters"]
    assert params["input_topic"] == "/scan"
    assert params["output_topic"] == "/scan/navigation"
    assert params["mesh_ray_occluded_count"] == 110
    assert params["mesh_ray_total_count"] == 1081
    assert params["normalize_positive_infinity"] is True
    assert params["expected_sensor_range_max_m"] == pytest.approx(30.0)
    assert params["no_return_replacement_m"] == pytest.approx(12.0)
    committed = parse_masks(params["angular_range_masks_rad"])
    assert len(committed) == len(MASKS)
    for actual, expected in zip(committed, MASKS):
        assert actual == pytest.approx(expected)


def test_mesh_derived_self_hits_are_removed_but_real_ranges_are_preserved():
    assert is_self_return(
        angle_rad=math.radians(-130.0), range_m=0.10, masks=MASKS
    )
    assert is_self_return(
        angle_rad=math.radians(125.0), range_m=0.20, masks=MASKS
    )
    assert not is_self_return(
        angle_rad=math.radians(-130.0), range_m=0.50, masks=MASKS
    )
    assert not is_self_return(
        angle_rad=math.radians(0.0), range_m=0.10, masks=MASKS
    )
    assert not is_self_return(
        angle_rad=math.radians(125.0), range_m=math.inf, masks=MASKS
    )


def test_filter_changes_only_angle_and_range_intersection():
    values, count, normalized = filter_ranges(
        angle_min=math.radians(-135.0),
        angle_increment=math.radians(45.0),
        ranges=(0.10, 0.10, 0.10, 0.10, 0.10, 0.20, 0.50),
        masks=MASKS,
    )
    assert count == 1
    assert normalized == 0
    assert math.isnan(values[0])
    assert values[1:] == [0.10, 0.10, 0.10, 0.10, 0.20, 0.50]


def test_physical_no_return_becomes_exact_karto_threshold_but_self_mask_stays_nan():
    values, self_count, normalized = filter_ranges(
        angle_min=math.radians(-135.0),
        angle_increment=math.radians(45.0),
        ranges=(0.10, math.inf, math.nan, -math.inf, 5.0),
        masks=MASKS,
        range_max=30.0,
        normalize_positive_infinity=True,
        no_return_replacement_m=12.0,
    )
    assert math.isnan(values[0])
    assert values[1] == pytest.approx(12.0)
    assert math.isnan(values[2])
    assert values[3] == -math.inf
    assert values[4] == pytest.approx(5.0)
    assert self_count == 1
    assert normalized == 1


def test_no_return_normalization_fails_closed_without_physical_range():
    with pytest.raises(ValueError, match="finite range_max"):
        filter_ranges(
            angle_min=0.0,
            angle_increment=0.1,
            ranges=(math.inf,),
            masks=(),
            normalize_positive_infinity=True,
        )


def test_no_return_normalization_rejects_replacement_outside_physical_range():
    with pytest.raises(ValueError, match="replacement"):
        filter_ranges(
            angle_min=0.0,
            angle_increment=0.1,
            ranges=(math.inf,),
            masks=(),
            range_max=30.0,
            normalize_positive_infinity=True,
            no_return_replacement_m=30.0,
        )


def test_exact_karto_threshold_expands_bbox_without_occupied_endpoint_ring():
    threshold = 12.0
    karto_tolerance = 1.0e-6
    normalized = filter_ranges(
        angle_min=-math.pi,
        angle_increment=math.pi / 2.0,
        ranges=(math.inf,) * 4,
        masks=(),
        range_max=30.0,
        normalize_positive_infinity=True,
        no_return_replacement_m=threshold,
    )[0]
    assert all(value <= threshold for value in normalized)
    assert all(not (value < threshold - karto_tolerance) for value in normalized)


@pytest.mark.parametrize(
    "values",
    (
        (0.0, 1.0),
        (1.0, 0.0, 0.1),
        (0.0, 1.0, -0.1),
        (0.0, 1.0, 0.1, 0.5, 1.5, 0.2),
    ),
)
def test_invalid_masks_fail_closed(values):
    with pytest.raises(ValueError):
        parse_masks(values)
