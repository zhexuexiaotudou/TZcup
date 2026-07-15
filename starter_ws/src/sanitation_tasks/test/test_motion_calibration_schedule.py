import math

from sanitation_tasks.motion_calibration_runner import build_schedule


def test_schedule_contains_all_thirteen_required_segments():
    schedule = build_schedule()
    names = []
    for action in schedule:
        if action.include_in_metrics and action.segment not in names:
            names.append(action.segment)
    assert len(names) == 13
    assert names[0] == "stationary_20s"
    assert names[-2:] == ["rectangle_10x5", "figure_eight_r2"]


def test_each_segment_has_a_three_second_rest():
    rests = [action for action in build_schedule() if not action.include_in_metrics]
    assert len(rests) == 13
    assert all(action.duration == 3.0 for action in rests)


def test_required_line_and_turn_commands_are_exact():
    schedule = build_schedule()
    moving = {action.segment: action for action in schedule if action.action in {"line", "turn"}}
    assert moving["forward_5m_0p20"].duration == 25.0
    assert moving["forward_5m_0p45"].linear == 0.45
    assert moving["reverse_5m_0p20"].linear == -0.20
    assert math.isclose(
        moving["turn_positive_360_0p25"].duration * 0.25,
        2.0 * math.pi,
    )
