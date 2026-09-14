"""Independent numerical checks of the formal bristle-sweep proxy, not field efficacy."""
from copy import deepcopy
import math

import pytest

from formal_cleaning_geometry import (
    empirical_cleaning_metrics,
    load_cleaning_geometry,
    sample_from_ground_dirt_status,
    samples_in_mission_frame,
    transform_sample_to_mission,
)


NAMES = ("left_side_brush", "right_side_brush", "central_roller")
OUTER = [[-2., -2.], [3., -2.], [3., 2.], [-2., 2.]]


def sample(t=0., x=0., y=0., yaw=0., active=NAMES):
    return {"stamp_s": t, "x": x, "y": y, "yaw": yaw,
            "lift_position_m": .1, "layout_ready": True,
            "tools": {name: {"enabled": name in active,
                             "observed_speed_rad_s": 3., "clearance_m": 0.}
                      for name in NAMES}}


def measure(samples, **kwargs):
    return empirical_cleaning_metrics(OUTER, samples, resolution=.005, **kwargs)


def area(samples, **kwargs):
    return measure(samples, **kwargs)["covered_area_m2"]


def test_authoritative_profile_uses_each_brush_radius_not_vehicle_width():
    geometry = load_cleaning_geometry()
    for name in NAMES[:2]:
        assert geometry["tools"][name]["radius_m"] == pytest.approx(.15)
    assert geometry["tools"]["central_roller"]["width_m"] == pytest.approx(.62)
    assert geometry["tools"]["central_roller"]["radius_m"] == pytest.approx(.10)


@pytest.mark.parametrize("active,expected", [
    (("left_side_brush",), math.pi * .15**2),
    (("central_roller",), .2 * .62),
    (NAMES, 2 * math.pi * .15**2 + .2 * .62),
])
def test_stationary_footprints_have_independently_computed_area(active, expected):
    assert area([sample(active=active)]) == pytest.approx(expected, abs=.006)


def test_one_metre_side_brush_sweep_matches_capsule_area():
    samples = [sample(t=i*.1, x=i*.1, active=(NAMES[0],)) for i in range(11)]
    assert area(samples) == pytest.approx(.30 + math.pi*.15**2, abs=.009)


def test_repeated_passes_are_a_union_not_summed_brush_area():
    one = [sample(t=i*.1, x=i*.1) for i in range(11)]
    retrace = [sample(t=1+i*.1, x=1-i*.1) for i in range(1, 11)]
    assert area(one+retrace) == pytest.approx(area(one), abs=.002)


@pytest.mark.parametrize("mutation", [
    lambda s: s.update(lift_position_m=0.),
    lambda s: s.update(layout_ready=False),
    lambda s: [tool.update(enabled=False) for tool in s["tools"].values()],
    lambda s: [tool.update(observed_speed_rad_s=0.) for tool in s["tools"].values()],
    lambda s: [tool.update(clearance_m=.1) for tool in s["tools"].values()],
])
def test_inactive_or_unready_tools_add_no_coverage(mutation):
    s = sample()
    mutation(s)
    assert area([s]) == 0.


def test_off_sample_breaks_path_and_cannot_bridge_unobserved_sweep():
    first, last = sample(x=0.), sample(t=.2, x=1.)
    off = sample(t=.1, x=.5, active=())
    expected = area([first]) + area([last])
    assert area([first, off, last], max_speed_m_s=20.) == pytest.approx(expected, abs=.002)


@pytest.mark.parametrize("end_time,max_speed", [(1., 3.), (.1, 3.), (0., 30.)])
def test_time_gap_teleport_or_nonincreasing_stamp_never_bridge(end_time, max_speed):
    first, last = sample(x=0.), sample(t=end_time, x=1.)
    assert area([first, last], max_speed_m_s=max_speed) <= area([first])+area([last])+.002


def test_missing_and_nonfinite_evidence_is_fail_closed():
    for field, value in (("yaw", float("nan")), ("stamp_s", float("inf"))):
        bad = sample(t=.1, x=.5)
        bad[field] = value
        result = measure([sample(), bad, sample(t=.2, x=1.)], max_speed_m_s=20.)
        assert result["valid"] is False
        assert result["invalid_sample_count"] >= 1
        assert result["covered_area_m2"] <= area([sample()])+area([sample(x=1.)])+.002
    missing = sample()
    del missing["tools"]["central_roller"]["clearance_m"]
    assert measure([missing])["valid"] is False
    assert area([missing]) == 0.


def test_rotation_only_sweep_includes_intermediate_roller_orientations():
    geometry = deepcopy(load_cleaning_geometry())
    geometry["tools"]["central_roller"]["center_xy_m"] = [0., 0.]
    endpoints = [sample(yaw=0., active=(NAMES[2],)),
                 sample(t=.1, yaw=math.pi/2, active=(NAMES[2],))]
    joined = area(endpoints, geometry=geometry)
    disconnected = area([endpoints[0], dict(endpoints[1], stamp_s=1.)], geometry=geometry)
    assert joined > disconnected + .015


def test_yaw_wrap_takes_short_rotation_across_pi():
    geometry = deepcopy(load_cleaning_geometry())
    geometry["tools"]["central_roller"]["center_xy_m"] = [0., 0.]
    endpoints = [sample(yaw=math.radians(179), active=(NAMES[2],)),
                 sample(t=.1, yaw=math.radians(-179), active=(NAMES[2],))]
    assert area(endpoints, geometry=geometry) < .15


def ground_dirt_status(*, x=0., y=0., yaw=0., enabled=True, layout=True,
                       lift=.1, speeds=(3., 3., 3.), clearances=(0., 0., 0.)):
    geometry = load_cleaning_geometry()
    c, s = math.cos(yaw), math.sin(yaw)

    def world(offset):
        return x + c * offset[0] - s * offset[1], y + s * offset[0] + c * offset[1]

    left, right, roller = (world(geometry["tools"][name]["center_xy_m"]) for name in NAMES)
    thresholds = geometry["runtime_thresholds"]

    def ready(index):
        return (enabled and layout and lift >= thresholds["minimum_lift_position_m"]
                and abs(speeds[index]) >= thresholds["minimum_rotation_rad_s"]
                and thresholds["minimum_contact_clearance_m"] <= clearances[index]
                <= thresholds["maximum_contact_clearance_m"])
    return {
        "enabled": enabled, "cell_layout_ready": layout, "lift_position_m": lift,
        "left_velocity_rad_s": speeds[0], "right_velocity_rad_s": speeds[1],
        "roller_velocity_rad_s": speeds[2], "left_ready": ready(0),
        "right_ready": ready(1), "roller_ready": ready(2),
        "left_clearance_m": clearances[0], "right_clearance_m": clearances[1],
        "roller_clearance_m": clearances[2], "left_world_x": left[0], "left_world_y": left[1],
        "right_world_x": right[0], "right_world_y": right[1],
        "roller_world_x": roller[0], "roller_world_y": roller[1],
    }


def test_ground_dirt_status_adapter_recovers_pose_and_independent_tool_states():
    status = ground_dirt_status(x=2.5, y=-1.25, yaw=.7, speeds=(3., 0., 3.))
    adapted = sample_from_ground_dirt_status(status, 17.25)
    assert adapted["stamp_s"] == 17.25
    assert adapted["x"] == pytest.approx(2.5)
    assert adapted["y"] == pytest.approx(-1.25)
    assert adapted["yaw"] == pytest.approx(.7)
    assert adapted["tools"]["left_side_brush"]["enabled"] is True
    assert adapted["tools"]["right_side_brush"]["enabled"] is False
    assert adapted["tools"]["central_roller"]["enabled"] is True


@pytest.mark.parametrize("mutation", [
    lambda status: status.update(left_velocity_rad_s=float("nan")),
    lambda status: status.pop("roller_world_y"),
    lambda status: status.update(right_world_y=status["right_world_y"] + .02),
    lambda status: status.update(roller_world_x=status["roller_world_x"] + .02),
    lambda status: status.update(left_ready=not status["left_ready"]),
])
def test_ground_dirt_status_adapter_fails_closed_for_missing_nan_or_inconsistent_geometry(mutation):
    status = ground_dirt_status(yaw=.2)
    mutation(status)
    with pytest.raises(ValueError):
        sample_from_ground_dirt_status(status, 1.)


def test_empty_samples_and_sampling_discontinuities_are_invalid():
    assert measure([])["valid"] is False
    discontinuous = measure([sample(), sample(t=.1, x=1.)], max_speed_m_s=3.)
    assert discontinuous["continuity_break_count"] == 3
    assert discontinuous["valid"] is False


def test_many_samples_use_a_compact_raster_mask_and_remain_union_based():
    samples = [sample(t=index * .01, active=(NAMES[0],)) for index in range(1000)]
    result = empirical_cleaning_metrics(OUTER, samples, resolution=.05)
    assert result["valid"] is True
    assert result["covered_area_m2"] == pytest.approx(math.pi * .15**2, abs=.02)


def test_world_status_samples_use_inverse_frozen_start_transform_for_map_mission():
    world_sample = sample(t=1., x=13., y=-3., yaw=math.pi / 2)
    mission = {"frame_id": "map", "source_fixed_start_pose": [10., -5., math.pi / 2]}
    mapped = transform_sample_to_mission(world_sample, mission)
    assert mapped["x"] == pytest.approx(2.)
    assert mapped["y"] == pytest.approx(-3.)
    assert mapped["yaw"] == pytest.approx(0.)
    assert list(samples_in_mission_frame([world_sample], mission)) == [mapped]


@pytest.mark.parametrize("mission", [
    {}, {"frame_id": "odom"}, {"frame_id": "map"},
    {"frame_id": "map", "source_fixed_start_pose": [0., 0., float("nan")]},
])
def test_unknown_or_unsealed_mission_frames_are_rejected(mission):
    with pytest.raises(ValueError):
        transform_sample_to_mission(sample(), mission)
