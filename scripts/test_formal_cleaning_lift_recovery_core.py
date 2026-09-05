from __future__ import annotations

import pytest

from formal_cleaning_lift_recovery_core import (
    CleaningLiftRecoverySupervisor,
    trajectory_duration_s,
)


def test_duration_respects_p16_rated_speed_and_remaining_travel() -> None:
    assert trajectory_duration_s(0.0) == pytest.approx(20.9)
    assert trajectory_duration_s(0.095542468) == pytest.approx(1.0)
    assert trajectory_duration_s(0.100) == pytest.approx(0.1)


def test_default_recovery_contract_keeps_the_certified_target_and_tight_bounds() -> None:
    supervisor = CleaningLiftRecoverySupervisor()
    assert supervisor.target_position_m == pytest.approx(0.100)
    assert supervisor.position_tolerance_m == pytest.approx(0.0002)
    assert supervisor.plateau_tolerance_m == pytest.approx(0.0001)
    assert supervisor.plateau_duration_sim_s == pytest.approx(0.5)
    assert supervisor.max_reissues == 3


def test_position_inside_0_2_mm_target_tolerance_does_not_reissue() -> None:
    supervisor = CleaningLiftRecoverySupervisor()
    assert supervisor.observe(
        sim_time_s=1.0, actual_position_m=0.0998, safety_permit=False
    ) is None
    assert supervisor.observe(
        sim_time_s=1.6, actual_position_m=0.0998, safety_permit=True
    ) is None
    assert supervisor.reissue_count == 0


def test_no_reissue_without_a_safety_cancellation() -> None:
    supervisor = CleaningLiftRecoverySupervisor()
    assert supervisor.observe(
        sim_time_s=1.0, actual_position_m=0.0955, safety_permit=True
    ) is None
    assert supervisor.observe(
        sim_time_s=2.0, actual_position_m=0.0955, safety_permit=True
    ) is None
    assert supervisor.reissue_count == 0


def test_reissue_waits_for_permit_recovery_and_a_sim_time_plateau() -> None:
    supervisor = CleaningLiftRecoverySupervisor()
    assert supervisor.observe(
        sim_time_s=1.0, actual_position_m=0.0955, safety_permit=False
    ) is None
    assert supervisor.observe(
        sim_time_s=1.1, actual_position_m=0.0955, safety_permit=True
    ) is None
    # A real position change restarts the plateau window.
    assert supervisor.observe(
        sim_time_s=1.5, actual_position_m=0.0957, safety_permit=True
    ) is None
    assert supervisor.observe(
        sim_time_s=1.9, actual_position_m=0.0957, safety_permit=True
    ) is None
    reissue = supervisor.observe(
        sim_time_s=2.0, actual_position_m=0.0957, safety_permit=True
    )
    assert reissue is not None
    assert reissue.attempt == 1
    assert reissue.target_position_m == pytest.approx(0.100)
    assert reissue.duration_s >= (0.100 - 0.0957) / 0.0048


def test_reissue_is_bounded_and_target_position_never_changes() -> None:
    supervisor = CleaningLiftRecoverySupervisor(
        plateau_duration_sim_s=0.0, max_reissues=2
    )
    for attempt in (1, 2):
        assert supervisor.observe(
            sim_time_s=float(attempt),
            actual_position_m=0.095,
            safety_permit=False,
        ) is None
        reissue = supervisor.observe(
            sim_time_s=float(attempt),
            actual_position_m=0.095,
            safety_permit=True,
        )
        assert reissue is not None
        assert reissue.attempt == attempt
        assert reissue.target_position_m == pytest.approx(0.100)

    assert supervisor.observe(
        sim_time_s=3.0, actual_position_m=0.095, safety_permit=False
    ) is None
    assert supervisor.observe(
        sim_time_s=3.0, actual_position_m=0.095, safety_permit=True
    ) is None
    assert supervisor.exhausted is True
