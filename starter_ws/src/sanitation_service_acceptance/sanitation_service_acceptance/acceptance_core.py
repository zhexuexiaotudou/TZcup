"""Truth-free scenario profiles and fail-closed service-interface verdicts."""

from __future__ import annotations

from dataclasses import dataclass
import math


SCENARIOS = (
    'charge_allow',
    'charge_reject_no_contact',
    'charge_reject_door_closed',
    'charge_reject_lock_open',
    'drain_allow',
    'drain_reject_no_contact',
    'drain_reject_cap_closed',
    'mutual_interlock_charge_wins',
)
WASTEWATER_CAPACITY_KG = 8.30


@dataclass(frozen=True)
class ScenarioProfile:
    station_connected: bool
    charge_door_rad: float
    charge_lock_m: float
    drain_cap_rad: float
    charge_requested: bool
    drain_requested: bool
    emergency_stop_active: bool
    main_power_requested: bool


def scenario_profile(name: str) -> ScenarioProfile:
    if name not in SCENARIOS:
        raise ValueError(f'unsupported service acceptance scenario: {name}')
    connected = 'no_contact' not in name
    if name.startswith('charge_'):
        return ScenarioProfile(
            station_connected=connected,
            charge_door_rad=0.0 if name == 'charge_reject_door_closed' else 1.82,
            charge_lock_m=0.0 if name == 'charge_reject_lock_open' else 0.006,
            drain_cap_rad=0.0,
            charge_requested=True,
            drain_requested=False,
            emergency_stop_active=True,
            main_power_requested=False,
        )
    if name == 'mutual_interlock_charge_wins':
        return ScenarioProfile(
            station_connected=True,
            charge_door_rad=1.82,
            charge_lock_m=0.006,
            drain_cap_rad=0.55,
            charge_requested=True,
            drain_requested=True,
            emergency_stop_active=True,
            main_power_requested=False,
        )
    return ScenarioProfile(
        station_connected=connected,
        charge_door_rad=0.0,
        charge_lock_m=0.0,
        drain_cap_rad=0.0 if name == 'drain_reject_cap_closed' else 0.55,
        charge_requested=False,
        drain_requested=True,
        emergency_stop_active=False,
        main_power_requested=True,
    )


def evaluate_scenario(name: str, observed: dict) -> dict[str, bool]:
    """Evaluate product observations only; no world/model state is accepted."""
    profile = scenario_profile(name)
    gates = {
        'joint_state_observed': observed.get('joint_state_samples', 0) > 0,
        'full_tank_sensor_observed': observed.get('tank_level_samples', 0) > 0,
        'initial_8_30kg_capacity_reaches_full_sensor': (
            float(observed.get('max_sensed_tank_level_fraction', 0.0)) >= 0.99
        ),
        'charge_raw_bridge_unique': observed.get('charge_raw_publishers') == 1,
        'drain_raw_bridge_unique': observed.get('drain_raw_publishers') == 1,
        'a300_bms_state_observed': observed.get('battery_state_samples', 0) > 0,
        'tank_mass_observed': observed.get('tank_mass_samples', 0) > 0,
        'service_drained_volume_observed': (
            observed.get('service_drained_volume_samples', 0) > 0
        ),
        'no_world_truth_consumed': observed.get('world_truth_consumed') is False,
    }
    charge_contact = observed.get('charge_nonempty_contacts', 0) > 0
    drain_contact = observed.get('drain_nonempty_contacts', 0) > 0

    def number(key: str) -> float:
        value = observed.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            return float(value)
        return math.nan

    door = number('charge_door_position_rad')
    lock = number('charge_lock_position_m')
    cap = number('drain_cap_position_rad')
    valve = number('drain_valve_position_rad')
    soc_first = number('battery_soc_first')
    soc_last = number('battery_soc_last')
    tank_first = number('tank_mass_first_kg')
    tank_last = number('tank_mass_last_kg')
    drained_first = number('service_drained_volume_first_l')
    drained_last = number('service_drained_volume_last_l')

    if name.startswith('charge_') or name == 'mutual_interlock_charge_wins':
        gates.update(
            {
                'charge_contact_matches_scenario': charge_contact
                == profile.station_connected,
                'charge_door_matches_command': abs(door - profile.charge_door_rad)
                <= 0.12,
                'charge_lock_matches_command': abs(lock - profile.charge_lock_m)
                <= 0.0015,
            }
        )
        should_allow = name == 'charge_allow' or name == 'mutual_interlock_charge_wins'
        gates['charge_enable_matches_expected'] = bool(
            observed.get('charge_enable_seen', False)
        ) is should_allow
        gates['charge_connected_matches_expected'] = bool(
            observed.get('charge_connected_seen', False)
        ) is should_allow
        gates['charge_power_matches_expected'] = (
            float(observed.get('max_charge_request_w', 0.0)) >= 649.0
            if should_allow
            else float(observed.get('max_charge_request_w', 0.0)) <= 1e-6
        )
        gates['traction_is_inhibited_during_charge_request'] = (
            observed.get('traction_permitted_samples', 0) > 0
            and observed.get('traction_permitted_true_samples', 0) == 0
        )
        gates['battery_soc_matches_charge_result'] = (
            soc_last - soc_first >= 5e-5
            if should_allow
            else abs(soc_last - soc_first) <= 1e-6
        )

    if name.startswith('drain_') or name == 'mutual_interlock_charge_wins':
        gates.update(
            {
                'drain_contact_matches_scenario': drain_contact
                == profile.station_connected,
                'drain_cap_matches_command': abs(cap - profile.drain_cap_rad) <= 0.12,
            }
        )
        should_allow = name == 'drain_allow'
        gates['drain_permit_matches_expected'] = bool(
            observed.get('drain_permitted_seen', False)
        ) is should_allow
        gates['drain_valve_matches_expected'] = (
            valve >= 1.35 if should_allow else abs(valve) <= 0.12
        )
        gates['drain_plant_command_matches_expected'] = bool(
            observed.get('drain_open_seen', False)
        ) is should_allow
        tank_drop_kg = tank_first - tank_last
        drained_delta_l = drained_last - drained_first
        if should_allow:
            gates['drain_removes_measured_tank_mass'] = tank_drop_kg >= 0.30
            gates['drain_reports_measured_removed_volume'] = drained_delta_l >= 0.30
            gates['drain_mass_conservation_within_0_02kg'] = (
                abs(tank_drop_kg - drained_delta_l) <= 0.02
            )
        else:
            gates['rejected_drain_preserves_tank_mass'] = abs(tank_drop_kg) <= 1e-3
            gates['rejected_drain_reports_zero_removed_volume'] = (
                abs(drained_delta_l) <= 1e-3
            )
    return gates
