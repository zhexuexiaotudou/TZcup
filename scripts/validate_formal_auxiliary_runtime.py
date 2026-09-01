#!/usr/bin/env python3
"""Exercise the formal vehicle power, charge, E-stop and lighting interfaces."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
PASSED_STATUS = "FORMAL_AUXILIARY_POWER_LIGHTING_RUNTIME_PASSED"
FAILED_STATUS = "FORMAL_AUXILIARY_POWER_LIGHTING_RUNTIME_FAILED"


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, object], dict[str, object]]:
    """Reject auxiliary evidence detached from the current formal session."""

    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from auxiliary source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from auxiliary session")
    return binding, bound_session


def evaluate(phases: dict[str, dict[str, Any]]) -> dict[str, Any]:
    enabled = phases.get("enabled", {})
    timed_out = phases.get("operator_timeout", {})
    checks = {
        "enabled_relay_and_power_branches": enabled.get("relay_enabled") is True
        and enabled.get("branches")
        == {"safety": True, "low_voltage": True, "high_power": True},
        "enabled_physical_switchgear_measured_closed": enabled.get(
            "main_isolator_closed"
        )
        is True
        and enabled.get("main_isolator_feedback_fresh") is True
        and enabled.get("main_contactor_closed") is True
        and enabled.get("main_contactor_feedback_fresh") is True,
        "requested_work_tail_warning_lights_on": enabled.get("lighting")
        == {"work": True, "tail": True, "four_corner_warning": True}
        and enabled.get("applied_lighting")
        == {"work": True, "tail": True, "four_corner_warning": True},
        "operator_timeout_fails_safe": timed_out.get("operator_command_fresh")
        is False
        and timed_out.get("emergency_stop_active") is True
        and timed_out.get("relay_enabled") is False
        and timed_out.get("branches", {}).get("high_power") is False,
        "estop_forces_warning_light": timed_out.get("lighting", {}).get(
            "four_corner_warning"
        )
        is True
        and timed_out.get("applied_lighting")
        == {"work": False, "tail": False, "four_corner_warning": True},
        "runtime_bound_to_all_product_positions": set(
            timed_out.get("bindings", {})
        )
        == {
            "work_lights",
            "tail_lights",
            "four_corner_warning_lights",
            "charge_interface",
            "fused_power_distribution",
            "isolated_low_voltage_power",
            "safety_relay",
            "main_power_isolator",
            "main_power_contactor",
            "emergency_stop",
        },
        "simulation_claim_boundary_preserved": all(
            phase.get("evidence_authority") == "SIMULATION_ENGINEERING_ONLY"
            and phase.get("interface_class") == "product_simulation"
            and isinstance(phase.get("battery_voltage_v"), (int, float))
            and math.isfinite(float(phase["battery_voltage_v"]))
            for phase in phases.values()
        ),
    }
    passed = all(checks.values())
    return {
        "report_id": "tzcup_formal_auxiliary_power_lighting_runtime_v2",
        "status": PASSED_STATUS if passed else FAILED_STATUS,
        "passed": passed,
        "checks": checks,
        "phases": phases,
        "claim_boundary": (
            "This is ROS runtime evidence for the formal simulation power-distribution, "
            "E-stop relay, physical switchgear and applied lighting state machine. Physical "
            "charge-plug, charge-door, connector-lock and SOC-gain interlocks are validated "
            "by the separate formal service-interface acceptance. It does not claim a measured battery "
            "voltage, electrical schematic certification, physical charger, real lamp luminous "
            "intensity or hardware safety relay performance."
        ),
    }


def run_probe(
    output: Path,
    startup_timeout_s: float,
    *,
    in_process_product_node: bool = False,
    runtime_binding_path: Path | None = None,
    snapshot_path: Path | None = None,
    session_path: Path | None = None,
) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("formal_auxiliary_runtime_probe")
            prefix = "/formal_vehicle/simulation/command/"
            self._command_publishers = {
                name: self.create_publisher(Bool, prefix + name, 10)
                for name in (
                    "emergency_stop",
                    "emergency_stop_reset",
                    "emergency_stop_plunger_pressed",
                    "main_power",
                    "charge_connected",
                    "work_lights",
                    "tail_lights",
                    "warning_lights",
                )
            }
            self.latest: dict[str, Any] | None = None
            self.samples = 0
            self._applied_lighting = {
                "work": False,
                "tail": False,
                "four_corner_warning": False,
            }
            self._applied_lighting_samples = 0
            self._spin_once = lambda timeout: rclpy.spin_once(
                self, timeout_sec=timeout
            )
            self.create_subscription(
                String,
                "/formal_vehicle/auxiliary/status_json",
                self._on_status,
                10,
            )
            self._applied_subscriptions = []
            for key, topic in (
                ("work", "/formal_vehicle/lighting/work_lights_applied"),
                ("tail", "/formal_vehicle/lighting/tail_lights_applied"),
                (
                    "four_corner_warning",
                    "/formal_vehicle/lighting/warning_lights_applied",
                ),
            ):
                self._applied_subscriptions.append(
                    self.create_subscription(
                        Bool,
                        topic,
                        lambda message, key=key: self._on_applied_lighting(
                            key, message
                        ),
                        10,
                    )
                )

        def _on_applied_lighting(self, key: str, message: Bool) -> None:
            self._applied_lighting[key] = bool(message.data)
            self._applied_lighting_samples += 1

        def _on_status(self, message: String) -> None:
            value = json.loads(message.data)
            if value.get("schema") != "tzcup.formal_auxiliary_product_state.v1":
                return
            self.latest = value
            self.samples += 1

        def command(self, values: dict[str, bool]) -> None:
            for name, publisher in self._command_publishers.items():
                default = name == "emergency_stop_reset" and not bool(
                    values.get("emergency_stop", True)
                )
                publisher.publish(Bool(data=bool(values.get(name, default))))

        def command_bridges_ready(self) -> bool:
            # These command topics have no in-process consumers. A matched
            # subscriber therefore proves that the ROS-to-Gazebo auxiliary
            # bridge exists before a safety transition is exercised.
            return all(
                self._command_publishers[name].get_subscription_count() > 0
                for name in (
                    "emergency_stop",
                    "emergency_stop_reset",
                    "emergency_stop_plunger_pressed",
                )
            )

        def phase(
            self,
            values: dict[str, bool] | None,
            duration_s: float,
            *,
            minimum_samples: int = 3,
            predicate=None,
        ) -> dict[str, Any]:
            start_samples = self.samples
            deadline = time.monotonic() + duration_s
            while rclpy.ok() and time.monotonic() < deadline:
                if values is not None:
                    self.command(values)
                self._spin_once(0.05)
                if self.latest is not None:
                    candidate = json.loads(json.dumps(self.latest))
                    candidate["applied_lighting"] = dict(self._applied_lighting)
                    if (
                        predicate is not None
                        and self.samples - start_samples >= minimum_samples
                        and predicate(candidate)
                    ):
                        return candidate
            if self.latest is None or self.samples - start_samples < minimum_samples:
                raise RuntimeError("auxiliary status stream did not provide enough fresh samples")
            result = json.loads(json.dumps(self.latest))
            result["applied_lighting"] = dict(self._applied_lighting)
            return result

    enabled_command = {
        "emergency_stop": False,
        "main_power": True,
        "charge_connected": False,
        "work_lights": True,
        "tail_lights": True,
        "warning_lights": True,
    }
    charging_command = {
        "emergency_stop": True,
        "main_power": False,
        "charge_connected": True,
        "work_lights": False,
        "tail_lights": False,
        "warning_lights": False,
    }

    runtime_evidence: tuple[dict[str, object], dict[str, object]] | None = None
    if runtime_binding_path is not None:
        if snapshot_path is None or session_path is None:
            raise ValueError("snapshot and session are required with runtime binding")
        runtime_evidence = _bound_runtime_evidence(
            snapshot_path, session_path, runtime_binding_path
        )

    rclpy.init()
    product_node = None
    bms_node = None
    executor = None
    if in_process_product_node:
        from sanitation_safety.simulation_safety_inputs import SimulationSafetyInputs
        from sanitation_power_system.a300_bms_node import A300BmsNode

        product_node = SimulationSafetyInputs()
        bms_node = A300BmsNode()
    node = Probe()
    if product_node is not None:
        executor = SingleThreadedExecutor()
        executor.add_node(product_node)
        executor.add_node(bms_node)
        executor.add_node(node)
        node._spin_once = lambda timeout: executor.spin_once(timeout_sec=timeout)
    phases: dict[str, dict[str, Any]] = {}
    report: dict[str, Any]
    try:
        deadline = time.monotonic() + startup_timeout_s
        def product_runtime_ready() -> bool:
            if node.latest is None:
                return False
            bumpers = node.latest.get("bumper_inputs", {})
            product_output_bridges_ready = product_node is None or all(
                product_node._product_publishers[name].get_subscription_count() > 0
                for name in (
                    "safety_branch",
                    "main_contactor_command",
                    "work_lights",
                    "tail_lights",
                    "warning_lights",
                )
            )
            return (
                node.command_bridges_ready()
                and product_output_bridges_ready
                and node.latest.get("battery_state_fresh") is True
                and node.latest.get("main_isolator_feedback_fresh") is True
                and node.latest.get("main_contactor_feedback_fresh") is True
                and bumpers.get("front_raw_bridge_available") is True
                and bumpers.get("rear_raw_bridge_available") is True
                and node._applied_lighting_samples > 0
            )

        while rclpy.ok() and not product_runtime_ready() and time.monotonic() < deadline:
            node.command(charging_command)
            node._spin_once(0.05)
        if not product_runtime_ready():
            raise TimeoutError(
                "formal auxiliary product state and physical dependencies did not become ready"
            )
        # The real 90 degree isolator is speed-limited to 1.2 rad/s. Wait on
        # measured state, not a guessed delay, before accepting the downstream
        # contactor and applied lamp outputs.
        phases["enabled"] = node.phase(
            enabled_command,
            30.0,
            predicate=lambda value: (
                value.get("emergency_stop_active") is False
                and value.get("relay_enabled") is True
                and value.get("main_isolator_closed") is True
                and value.get("main_contactor_closed") is True
                and value.get("branches")
                == {"safety": True, "low_voltage": True, "high_power": True}
                and value.get("applied_lighting")
                == {"work": True, "tail": True, "four_corner_warning": True}
            ),
        )
        # Stop refreshing the two fail-safe operator commands. The product node
        # must independently latch E-stop and remove main/high power.
        phases["operator_timeout"] = node.phase(
            None,
            5.0,
            predicate=lambda value: (
                value.get("operator_command_fresh") is False
                and value.get("emergency_stop_active") is True
                and value.get("relay_enabled") is False
                and value.get("applied_lighting")
                == {"work": False, "tail": False, "four_corner_warning": True}
            ),
        )
        report = evaluate(phases)
        report["runtime_sample_count"] = node.samples
    except Exception as exc:
        report = evaluate(phases)
        report["error"] = str(exc)
        report["runtime_sample_count"] = node.samples
    finally:
        if executor is not None:
            executor.remove_node(node)
            if product_node is not None:
                executor.remove_node(product_node)
            if bms_node is not None:
                executor.remove_node(bms_node)
            executor.shutdown()
        node.destroy_node()
        if product_node is not None:
            product_node.destroy_node()
        if bms_node is not None:
            bms_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if runtime_evidence is not None:
        binding, _ = runtime_evidence
        report["runtime_gate_binding"] = binding
        report["acceptance_session_binding"] = binding["acceptance_session_binding"]
        report["runtime_closure_binding"] = binding["runtime_closure_binding"]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    parser.add_argument("--in-process-product-node", action="store_true")
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    args = parser.parse_args()
    return run_probe(
        args.output,
        args.startup_timeout,
        in_process_product_node=args.in_process_product_node,
        runtime_binding_path=args.runtime_binding,
        snapshot_path=args.snapshot,
        session_path=args.session,
    )


if __name__ == "__main__":
    raise SystemExit(main())
