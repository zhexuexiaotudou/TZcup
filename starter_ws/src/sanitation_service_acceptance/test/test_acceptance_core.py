from __future__ import annotations

import pytest

from sanitation_service_acceptance.acceptance_core import evaluate_scenario
from sanitation_service_acceptance.acceptance_core import scenario_profile
from sanitation_service_acceptance.acceptance_core import SCENARIOS
from sanitation_service_acceptance.acceptance_core import WASTEWATER_CAPACITY_KG


def passing_observation(scenario: str) -> dict:
    profile = scenario_profile(scenario)
    charge_should_allow = scenario in {
        'charge_allow',
        'mutual_interlock_charge_wins',
    }
    drain_should_allow = scenario == 'drain_allow'
    return {
        'joint_state_samples': 20,
        'tank_level_samples': 20,
        'max_sensed_tank_level_fraction': 1.0,
        'tank_mass_samples': 20,
        'tank_mass_first_kg': 8.30,
        'tank_mass_last_kg': 7.10 if drain_should_allow else 8.30,
        'service_drained_volume_samples': 20,
        'service_drained_volume_first_l': 0.0,
        'service_drained_volume_last_l': 1.20 if drain_should_allow else 0.0,
        'battery_state_samples': 20,
        'battery_soc_first': 0.80,
        'battery_soc_last': 0.802 if charge_should_allow else 0.80,
        'traction_permitted_samples': 20,
        'traction_permitted_true_samples': 0,
        'charge_raw_publishers': 1,
        'drain_raw_publishers': 1,
        'world_truth_consumed': False,
        'charge_nonempty_contacts': 10 if profile.station_connected else 0,
        'drain_nonempty_contacts': 10 if profile.station_connected else 0,
        'charge_door_position_rad': profile.charge_door_rad,
        'charge_lock_position_m': profile.charge_lock_m,
        'drain_cap_position_rad': profile.drain_cap_rad,
        'drain_valve_position_rad': 1.5 if drain_should_allow else 0.0,
        'charge_enable_seen': charge_should_allow,
        'charge_connected_seen': charge_should_allow,
        'max_charge_request_w': 650.0 if charge_should_allow else 0.0,
        'drain_permitted_seen': drain_should_allow,
        'drain_open_seen': drain_should_allow,
    }


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_each_complete_physical_scenario_can_pass(scenario: str) -> None:
    gates = evaluate_scenario(scenario, passing_observation(scenario))
    assert gates
    assert all(gates.values())


def test_world_truth_consumption_always_fails_closed() -> None:
    observed = passing_observation('charge_allow')
    observed['world_truth_consumed'] = True
    assert not evaluate_scenario('charge_allow', observed)['no_world_truth_consumed']


def test_missing_raw_bridge_fails_even_a_rejection_scenario() -> None:
    observed = passing_observation('charge_reject_no_contact')
    observed['charge_raw_publishers'] = 0
    assert not evaluate_scenario('charge_reject_no_contact', observed)[
        'charge_raw_bridge_unique'
    ]


def test_allow_scenario_cannot_pass_without_physical_contact() -> None:
    observed = passing_observation('drain_allow')
    observed['drain_nonempty_contacts'] = 0
    assert not evaluate_scenario('drain_allow', observed)[
        'drain_contact_matches_scenario'
    ]


def test_final_wastewater_capacity_and_full_sensor_gate_are_frozen() -> None:
    assert WASTEWATER_CAPACITY_KG == 8.30
    observed = passing_observation('drain_allow')
    observed['max_sensed_tank_level_fraction'] = 0.98
    assert not evaluate_scenario('drain_allow', observed)[
        'initial_8_30kg_capacity_reaches_full_sensor'
    ]


def test_charge_allow_requires_soc_gain_and_continuous_traction_inhibit() -> None:
    observed = passing_observation('charge_allow')
    observed['battery_soc_last'] = observed['battery_soc_first']
    observed['traction_permitted_true_samples'] = 1
    gates = evaluate_scenario('charge_allow', observed)
    assert not gates['battery_soc_matches_charge_result']
    assert not gates['traction_is_inhibited_during_charge_request']


def test_drain_allow_requires_measured_mass_conservation() -> None:
    observed = passing_observation('drain_allow')
    observed['service_drained_volume_last_l'] = 0.8
    gates = evaluate_scenario('drain_allow', observed)
    assert not gates['drain_mass_conservation_within_0_02kg']


def test_rejected_drain_cannot_silently_lose_tank_mass() -> None:
    observed = passing_observation('drain_reject_cap_closed')
    observed['tank_mass_last_kg'] = 8.0
    gates = evaluate_scenario('drain_reject_cap_closed', observed)
    assert not gates['rejected_drain_preserves_tank_mass']
