"""Drive and collect one physical service-interface acceptance scenario."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, Float64, String

from .acceptance_core import (
    evaluate_scenario,
    scenario_profile,
    SCENARIOS,
    WASTEWATER_CAPACITY_KG,
)


CHARGE_RAW = '/formal_vehicle/service/raw/charge_plug_contact'
DRAIN_RAW = '/formal_vehicle/service/raw/drain_hose_contact'
SENSED_TANK_LEVEL = (
    '/model/tzcup_formal_sanitation_vehicle/water_recovery/'
    'sensed_tank_level_fraction'
)
TANK_MASS = '/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg'
SERVICE_DRAINED_VOLUME = (
    '/model/tzcup_formal_sanitation_vehicle/water_recovery/'
    'service_drained_volume_l'
)
BATTERY_STATE = '/formal_vehicle/power/battery_state'
TRACTION_PERMITTED = '/formal_vehicle/power/traction_permitted'


class ServiceAcceptanceCollector(Node):
    def __init__(self, scenario: str, output: Path, settle_sec: float, sample_sec: float) -> None:
        super().__init__('formal_service_acceptance_collector')
        self.scenario = scenario
        self.profile = scenario_profile(scenario)
        self.output = output
        self.settle_sec = settle_sec
        self.sample_sec = sample_sec
        self.started = time.monotonic()
        self.observed = {
            'joint_state_samples': 0,
            'tank_level_samples': 0,
            'max_sensed_tank_level_fraction': 0.0,
            'tank_mass_samples': 0,
            'tank_mass_first_kg': None,
            'tank_mass_last_kg': None,
            'tank_mass_min_kg': None,
            'service_drained_volume_samples': 0,
            'service_drained_volume_first_l': None,
            'service_drained_volume_last_l': None,
            'service_drained_volume_max_l': None,
            'battery_state_samples': 0,
            'battery_soc_first': None,
            'battery_soc_last': None,
            'battery_soc_min': None,
            'battery_soc_max': None,
            'traction_permitted_samples': 0,
            'traction_permitted_true_samples': 0,
            'charge_contact_messages': 0,
            'charge_nonempty_contacts': 0,
            'drain_contact_messages': 0,
            'drain_nonempty_contacts': 0,
            'charge_door_position_rad': None,
            'charge_lock_position_m': None,
            'drain_cap_position_rad': None,
            'drain_valve_position_rad': None,
            'charge_enable_seen': False,
            'charge_connected_seen': False,
            'max_charge_request_w': 0.0,
            'drain_permitted_seen': False,
            'drain_open_seen': False,
            'world_truth_consumed': False,
        }
        self.subscription_topics = (
            CHARGE_RAW,
            DRAIN_RAW,
            '/joint_states',
            '/formal_vehicle/power/charge_enable',
            '/formal_vehicle/power/charge_connected',
            '/formal_vehicle/power/charge_request_w',
            '/formal_vehicle/power/charge_status_json',
            '/formal_vehicle/service/drain_status_json',
            SENSED_TANK_LEVEL,
            TANK_MASS,
            SERVICE_DRAINED_VOLUME,
            BATTERY_STATE,
            TRACTION_PERMITTED,
        )
        self.estop = self.create_publisher(
            Bool, '/formal_vehicle/simulation/command/emergency_stop', 10
        )
        self.estop_reset = self.create_publisher(
            Bool, '/formal_vehicle/simulation/command/emergency_stop_reset', 10
        )
        self.main_power = self.create_publisher(
            Bool, '/formal_vehicle/simulation/command/main_power', 10
        )
        self.charge_request = self.create_publisher(
            Bool, '/formal_vehicle/simulation/command/charge_connected', 10
        )
        self.drain_request = self.create_publisher(
            Bool, '/safety/command/service_drain_open', 10
        )
        self.door_command = self.create_publisher(
            Float64,
            '/formal_vehicle/evaluation/service/charge_door_position_rad',
            10,
        )
        self.lock_command = self.create_publisher(
            Float64,
            '/formal_vehicle/evaluation/service/charge_lock_position_m',
            10,
        )
        self.cap_command = self.create_publisher(
            Float64,
            '/formal_vehicle/evaluation/service/drain_cap_position_rad',
            10,
        )
        self.create_subscription(
            Contacts, CHARGE_RAW, self._charge_contact, qos_profile_sensor_data
        )
        self.create_subscription(
            Contacts, DRAIN_RAW, self._drain_contact, qos_profile_sensor_data
        )
        self.create_subscription(JointState, '/joint_states', self._joints, 20)
        self.create_subscription(Float64, SENSED_TANK_LEVEL, self._tank_level, 20)
        self.create_subscription(Float64, TANK_MASS, self._tank_mass, 20)
        self.create_subscription(
            Float64, SERVICE_DRAINED_VOLUME, self._service_drained_volume, 20
        )
        self.create_subscription(BatteryState, BATTERY_STATE, self._battery_state, 20)
        self.create_subscription(
            Bool, TRACTION_PERMITTED, self._traction_permitted, 20
        )
        self.create_subscription(
            Bool, '/formal_vehicle/power/charge_enable', self._charge_enable, 20
        )
        self.create_subscription(
            Bool,
            '/formal_vehicle/power/charge_connected',
            self._charge_connected,
            20,
        )
        self.create_subscription(
            Float64,
            '/formal_vehicle/power/charge_request_w',
            self._charge_power,
            20,
        )
        self.create_subscription(
            String,
            '/formal_vehicle/power/charge_status_json',
            self._ignore_status,
            20,
        )
        self.create_subscription(
            String,
            '/formal_vehicle/service/drain_status_json',
            self._drain_status,
            20,
        )
        self.timer = self.create_timer(0.05, self._tick)

    def _collecting(self) -> bool:
        return time.monotonic() - self.started >= self.settle_sec

    def _charge_contact(self, message: Contacts) -> None:
        if self._collecting():
            self.observed['charge_contact_messages'] += 1
            self.observed['charge_nonempty_contacts'] += int(bool(message.contacts))

    def _drain_contact(self, message: Contacts) -> None:
        if self._collecting():
            self.observed['drain_contact_messages'] += 1
            self.observed['drain_nonempty_contacts'] += int(bool(message.contacts))

    def _joints(self, message: JointState) -> None:
        if not self._collecting():
            return
        positions = dict(zip(message.name, message.position))
        self.observed['joint_state_samples'] += 1
        for joint, key in (
            ('charge_port_door_hinge_joint', 'charge_door_position_rad'),
            ('charge_connector_lock_joint', 'charge_lock_position_m'),
            ('wastewater_drain_service_cap_joint', 'drain_cap_position_rad'),
            ('wastewater_drain_valve_joint', 'drain_valve_position_rad'),
        ):
            if joint in positions:
                self.observed[key] = float(positions[joint])

    def _charge_enable(self, message: Bool) -> None:
        if self._collecting():
            self.observed['charge_enable_seen'] |= bool(message.data)

    def _tank_level(self, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value):
            return
        # Capture the full pre-drain state during settling. Restricting this
        # to the later window makes a valid drain episode erase its own
        # initial-capacity evidence.
        self.observed['tank_level_samples'] += 1
        self.observed['max_sensed_tank_level_fraction'] = max(
            self.observed['max_sensed_tank_level_fraction'], value
        )

    def _tank_mass(self, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value):
            return
        if self.observed['tank_mass_first_kg'] is None:
            self.observed['tank_mass_first_kg'] = value
        self.observed['tank_mass_samples'] += 1
        self.observed['tank_mass_last_kg'] = value
        current_min = self.observed['tank_mass_min_kg']
        self.observed['tank_mass_min_kg'] = (
            value if current_min is None else min(float(current_min), value)
        )

    def _service_drained_volume(self, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value):
            return
        if self.observed['service_drained_volume_first_l'] is None:
            self.observed['service_drained_volume_first_l'] = value
        self.observed['service_drained_volume_samples'] += 1
        self.observed['service_drained_volume_last_l'] = value
        current_max = self.observed['service_drained_volume_max_l']
        self.observed['service_drained_volume_max_l'] = (
            value if current_max is None else max(float(current_max), value)
        )

    def _battery_state(self, message: BatteryState) -> None:
        value = float(message.percentage)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            return
        if self.observed['battery_soc_first'] is None:
            self.observed['battery_soc_first'] = value
        self.observed['battery_state_samples'] += 1
        self.observed['battery_soc_last'] = value
        current_min = self.observed['battery_soc_min']
        current_max = self.observed['battery_soc_max']
        self.observed['battery_soc_min'] = (
            value if current_min is None else min(float(current_min), value)
        )
        self.observed['battery_soc_max'] = (
            value if current_max is None else max(float(current_max), value)
        )

    def _traction_permitted(self, message: Bool) -> None:
        self.observed['traction_permitted_samples'] += 1
        self.observed['traction_permitted_true_samples'] += int(bool(message.data))

    def _charge_connected(self, message: Bool) -> None:
        if self._collecting():
            self.observed['charge_connected_seen'] |= bool(message.data)

    def _charge_power(self, message: Float64) -> None:
        if self._collecting():
            self.observed['max_charge_request_w'] = max(
                self.observed['max_charge_request_w'], float(message.data)
            )

    def _ignore_status(self, _message: String) -> None:
        pass

    def _drain_status(self, message: String) -> None:
        if not self._collecting():
            return
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.observed['drain_permitted_seen'] |= bool(status.get('permitted'))
        self.observed['drain_open_seen'] |= bool(status.get('water_recovery_drain_open'))

    def _tick(self) -> None:
        self.estop.publish(Bool(data=self.profile.emergency_stop_active))
        self.estop_reset.publish(Bool(data=not self.profile.emergency_stop_active))
        self.main_power.publish(Bool(data=self.profile.main_power_requested))
        self.charge_request.publish(Bool(data=self.profile.charge_requested))
        self.drain_request.publish(Bool(data=self.profile.drain_requested))
        self.door_command.publish(Float64(data=self.profile.charge_door_rad))
        self.lock_command.publish(Float64(data=self.profile.charge_lock_m))
        self.cap_command.publish(Float64(data=self.profile.drain_cap_rad))
        if time.monotonic() - self.started < self.settle_sec + self.sample_sec:
            return
        self.observed['charge_raw_publishers'] = len(
            self.get_publishers_info_by_topic(CHARGE_RAW)
        )
        self.observed['drain_raw_publishers'] = len(
            self.get_publishers_info_by_topic(DRAIN_RAW)
        )
        gates = evaluate_scenario(self.scenario, self.observed)
        artifact = {
            'schema': 'tzcup.formal_service_interface_episode.v1',
            'scenario': self.scenario,
            'result': 'PASS' if all(gates.values()) else 'FAIL',
            'gates': gates,
            'observed': self.observed,
            'command_profile': self.profile.__dict__,
            'wastewater_capacity_kg': WASTEWATER_CAPACITY_KG,
            'subscription_topics': list(self.subscription_topics),
            'truth_boundary': (
                'Evaluator commands service hardware and observes product Contacts, '
                'JointState and status only; product nodes consume no world truth.'
            ),
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            json.dumps(artifact, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True, choices=SCENARIOS)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--settle-sec', type=float, default=10.0)
    parser.add_argument('--sample-sec', type=float, default=6.0)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = ServiceAcceptanceCollector(
        parsed.scenario, parsed.output, parsed.settle_sec, parsed.sample_sec
    )
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
