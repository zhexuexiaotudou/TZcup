#!/usr/bin/env python3
"""Fail-closed semantic readiness gate before formal water collectors start."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from formal_cleaning_motor_telemetry import decode_cleaning_motor_telemetry


ACTIVE_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
    "cleaning_controller",
    "storage_controller",
    "service_controller",
}
INACTIVE_CONTROLLERS = {"brush_controller", "recovery_controller"}
REQUIRED_NODES = {"/whole_vehicle_safety_manager"}
SERVICE_DRAIN_MANAGER_NODE = "/service_drain_safety_manager"
TYPED_TOPIC = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot"
def controller_contract_checks(states: dict[str, str]) -> dict[str, bool]:
    return {
        "main_six_controllers_active": all(
            states.get(name) == "active" for name in ACTIVE_CONTROLLERS
        ),
        "brush_and_recovery_configured_inactive": all(
            states.get(name) == "inactive" for name in INACTIVE_CONTROLLERS
        ),
    }


def controller_states_from_response(response: object) -> dict[str, str]:
    return {
        str(controller.name): str(controller.state)
        for controller in response.controller
    }


class ReadinessProbe(Node):
    def __init__(self) -> None:
        super().__init__("formal_water_preoperational_readiness")
        self.decoded_frame: dict[str, object] | None = None
        self.decode_errors: list[str] = []
        self.controller_states: dict[str, str] = {}
        self.controller_errors: list[str] = []
        self.controller_future = None
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.create_subscription(Float64MultiArray, TYPED_TOPIC, self._on_frame, 100)

    def _on_frame(self, message: Float64MultiArray) -> None:
        try:
            decoded = decode_cleaning_motor_telemetry(message.data)
        except Exception as error:
            self.decode_errors.append(f"{type(error).__name__}: {error}")
            return
        if (
            int(decoded["physics_update_sequence"]) > 0
            and not bool(decoded["physics_update_stale"])
        ):
            self.decoded_frame = decoded

    def update_controller_states(self) -> None:
        if self.controller_future is not None:
            if not self.controller_future.done():
                return
            try:
                self.controller_states = controller_states_from_response(
                    self.controller_future.result()
                )
            except Exception as error:
                self.controller_errors.append(f"{type(error).__name__}: {error}")
            self.controller_future = None
        if self.controller_client.wait_for_service(timeout_sec=0.05):
            self.controller_future = self.controller_client.call_async(
                ListControllers.Request()
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument(
        "--service-drain-manager",
        choices=("present", "absent"),
        default="present",
    )
    args = parser.parse_args()
    rclpy.init()
    node = ReadinessProbe()
    deadline = time.monotonic() + args.timeout_s
    checks: dict[str, bool] = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.update_controller_states()
        node_names = {
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in node.get_node_names_and_namespaces()
        }
        publisher_info = node.get_publishers_info_by_topic(TYPED_TOPIC)
        manager_present = SERVICE_DRAIN_MANAGER_NODE in node_names
        checks = {
            **controller_contract_checks(node.controller_states),
            "required_safety_manager_nodes_present": REQUIRED_NODES <= node_names,
            "service_drain_manager_matches_expected_presence": manager_present
            == (args.service_drain_manager == "present"),
            "typed_snapshot_has_ros_publisher": bool(publisher_info),
            "typed_snapshot_rev_positive_and_non_stale": node.decoded_frame is not None,
            "typed_snapshot_has_zero_decode_errors": not node.decode_errors,
            "controller_service_has_zero_errors": not node.controller_errors,
        }
        if all(checks.values()):
            break
        time.sleep(0.10)
    passed = bool(checks) and all(checks.values())
    report = {
        "schema_version": 1,
        "status": "FORMAL_WATER_PREOPERATIONAL_READINESS_PASSED" if passed else "FAILED",
        "passed": passed,
        "checks": checks,
        "controller_states": node.controller_states,
        "controller_state_source": "/controller_manager/list_controllers",
        "controller_service_errors": node.controller_errors,
        "required_nodes": sorted(REQUIRED_NODES),
        "service_drain_manager_node": SERVICE_DRAIN_MANAGER_NODE,
        "expected_service_drain_manager_presence": args.service_drain_manager,
        "typed_topic": TYPED_TOPIC,
        "typed_frame": node.decoded_frame,
        "decode_errors": node.decode_errors,
    }
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
