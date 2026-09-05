#!/usr/bin/env python3
"""Drive 20 sequential rich perception requests against an already-running world.

This product-side probe never spawns, teleports or identifies simulator models.
The environment must independently expose 20 physical cubes at the requested
perception poses, wrist rechecks, contact gates and the observation-only bin.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from formal_runtime_gate_binding import load_binding


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


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


def _session_binding(session_path: Path, source_binding: dict[str, str]) -> dict[str, object]:
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict):
        raise ValueError("formal acceptance session root is not an object")
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    snapshot = session.get("snapshot")
    if not isinstance(snapshot, dict):
        raise ValueError("formal acceptance session has no frozen snapshot")
    for key in (
        "snapshot_manifest_sha256",
        "source_inventory_sha256",
        "expanded_urdf_sha256",
    ):
        if snapshot.get(key) != source_binding.get(key):
            raise ValueError(f"formal acceptance session snapshot mismatch: {key}")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    if started_epoch_ns > time.time_ns():
        raise ValueError("formal acceptance session is future dated")
    return {
        "session_manifest_sha256": hashlib.sha256(session_path.read_bytes()).hexdigest(),
        "started_epoch_ns": started_epoch_ns,
        "status_at_gate_start": session["status"],
    }


def _active_overlay_identity(expected_install_root: Path) -> dict[str, object]:
    """Reject a probe whose executor resolves outside the frozen overlay."""

    expected = expected_install_root.resolve(strict=True)
    if expected.is_symlink():
        raise ValueError("bound frozen runtime install root is a symbolic link")
    prefixes = [
        Path(value).resolve(strict=True)
        for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if value
    ]
    if not prefixes:
        raise ValueError("AMENT_PREFIX_PATH is empty after frozen-overlay setup")
    package_marker = Path(
        "share/ament_index/resource_index/packages/sanitation_manipulation"
    )
    resolved_package_prefix = next(
        (prefix for prefix in prefixes if (prefix / package_marker).is_file()), None
    )
    if resolved_package_prefix != expected:
        raise ValueError(
            "sanitation_manipulation resolves outside the bound frozen overlay: "
            f"{resolved_package_prefix} != {expected}"
        )
    return {
        "active_ament_prefix_first": str(prefixes[0]),
        "resolved_sanitation_manipulation_prefix": str(resolved_package_prefix),
        "expected_runtime_install_root": str(expected),
        "active_overlay_matches_runtime_binding": True,
    }


def _evaluator_payload_matches(
    status: object,
    *,
    expected_count: int,
    expected_mass_kg: float,
    mass_tolerance_kg: float = 1.0e-5,
) -> bool:
    """Confirm the physical bodies and load are unchanged at a retry boundary."""

    if not isinstance(status, dict):
        return False
    try:
        physical_mass_kg = float(status.get("physical_contained_mass_kg", -1.0))
    except (TypeError, ValueError):
        return False
    return (
        status.get("sensor_ready") is True
        and status.get("candidate_model_count") == 20
        and status.get("inside_candidate_count") == expected_count
        and status.get("inertial_candidate_count") == expected_count
        and abs(physical_mass_kg - expected_mass_kg) <= mass_tolerance_kg
    )


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_20_cube_grasp_acceptance_probe")
        self.request = self.create_publisher(String, "/active_cleaning/grasp_request", 10)
        self.wrist = self.create_publisher(String, "/perception/wrist/grasp_recheck", 10)
        self.estop_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.main_power_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.safe_zero_command = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.create_subscription(String, "/active_cleaning/grasp_result", self._result, 10)
        self.create_subscription(DiagnosticArray, "/manipulation/formal_grasp_status", self._status, 10)
        self.create_subscription(Bool, "/manipulation/base_motion_inhibited", self._inhibit, 10)
        self.create_subscription(Bool, "/safety/actuators_enabled", self._safety, 10)
        self.create_subscription(
            String,
            "/formal_acceptance/evaluator/dry_bin/status_json",
            self._evaluator_bin_status,
            10,
        )
        self.results: dict[str, dict] = {}
        self.awaiting_recheck = False
        self.executor_busy: bool | None = None
        self.inhibit_events: list[bool] = []
        self.safety_permitted = False
        self.evaluator_bin_status: dict[str, object] | None = None
        self.create_timer(0.05, self._heartbeat)

    def _heartbeat(self) -> None:
        # Operator controls and a zero base request only.  The safety manager
        # retains sole authority for actuator permission and gated motion.
        self.estop_command.publish(Bool(data=False))
        self.estop_reset_command.publish(Bool(data=True))
        self.main_power_command.publish(Bool(data=True))
        self.safe_zero_command.publish(Twist())

    def _result(self, message: String) -> None:
        payload = json.loads(message.data)
        self.results[str(payload.get("target_id", ""))] = payload

    def _status(self, message: DiagnosticArray) -> None:
        for row in message.status:
            if row.name != "formal_physical_grasp_executor":
                continue
            values = {item.key: item.value for item in row.values}
            self.awaiting_recheck = values.get("reason") == "awaiting_wrist_near_field_recheck"
            self.executor_busy = values.get("busy") == "true"

    def _inhibit(self, message: Bool) -> None:
        self.inhibit_events.append(bool(message.data))

    def _safety(self, message: Bool) -> None:
        self.safety_permitted = bool(message.data)

    def _evaluator_bin_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.evaluator_bin_status = value

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", required=True, type=Path)
    parser.add_argument("--per-target-timeout", type=float, default=180.0)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if manifest.get("task_count") != 20 or len(manifest.get("requests", [])) != 20:
        raise SystemExit("manifest must contain exactly 20 requests")
    maximum_attempts = manifest.get("runtime_requirements", {}).get(
        "maximum_attempts_per_target"
    )
    if maximum_attempts != 2:
        raise SystemExit("manifest must limit every target to at most two attempts")
    source_binding = _source_binding(args.snapshot)
    session_binding = _session_binding(args.session, source_binding)
    runtime_binding = load_binding(args.runtime_binding)
    bound_session = runtime_binding["acceptance_session_binding"]
    closure = runtime_binding["runtime_closure_binding"]
    if bound_session.get("snapshot") != source_binding:
        raise SystemExit("runtime binding snapshot differs from the 20-cube source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != session_binding["session_manifest_sha256"]
        or bound_session.get("session_started_epoch_ns")
        != session_binding["started_epoch_ns"]
    ):
        raise SystemExit("runtime binding session differs from the 20-cube session")
    expected_install = closure.get("runtime_install_root")
    if not isinstance(expected_install, str) or not expected_install:
        raise SystemExit("runtime closure has no frozen install identity")
    if closure.get("symbolic_link_count") != 0:
        raise SystemExit("runtime closure allows symbolic links")
    active_overlay = _active_overlay_identity(Path(expected_install))
    manifest_sha256 = hashlib.sha256(args.manifest.read_bytes()).hexdigest()
    if args.manifest.stat().st_mtime_ns < int(session_binding["started_epoch_ns"]):
        raise SystemExit("20-cube manifest predates the formal acceptance session")
    rclpy.init()
    node = Probe()
    report = {
        "report_id": "tzcup_formal_target_conditioned_20_cube_runtime_v1",
        "passed": False,
        "status": "FORMAL_20_CUBE_TARGET_CONDITIONED_GRASP_FAILED",
        "manifest_id": manifest.get("manifest_id"),
        "source_binding": source_binding,
        "acceptance_session_binding": session_binding,
        "runtime_gate_binding": runtime_binding,
        "active_overlay": active_overlay,
        "scene_manifest_sha256": manifest_sha256,
        "truth_used_for_product_control": False,
        "evaluator_truth_used_only_for_acceptance_scoring": True,
        "expected_target_count": 20,
        "expected_final_physical_resident_mass_kg": manifest.get(
            "expected_final_physical_resident_mass_kg"
        ),
        "expected_final_aggregate_dry_mass_kg": manifest.get(
            "expected_final_aggregate_dry_mass_kg"
        ),
        "results": [],
    }
    measured_cumulative_mass_kg = 0.0
    try:
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            node.spin_once()
            initial = node.evaluator_bin_status
            if (
                node.request.get_subscription_count() >= 1
                and node.safety_permitted
                and node.executor_busy is False
                and isinstance(initial, dict)
                and initial.get("sensor_ready") is True
                and initial.get("candidate_model_count") == 20
            ):
                break
        if node.request.get_subscription_count() < 1:
            raise RuntimeError("formal grasp executor unavailable")
        if not node.safety_permitted:
            raise RuntimeError("whole-vehicle safety permit unavailable")
        if node.executor_busy is not False:
            raise RuntimeError("formal grasp executor status unavailable or busy")
        initial = node.evaluator_bin_status
        if not isinstance(initial, dict):
            raise RuntimeError("evaluator dry-bin status unavailable")
        expected_initial_count = manifest["scene_contract"]["initial_contained_object_count"]
        expected_initial_mass = manifest["scene_contract"]["initial_contained_mass_kg"]
        if initial.get("candidate_model_count") != 20:
            raise RuntimeError(f"formal scene does not contain 20 physical candidate bodies: {initial}")
        if initial.get("dry_accounting_mode") != "physical_resident":
            raise RuntimeError(f"formal scene selected a non-physical dry ledger: {initial}")
        if initial.get("resident_load_path") != "independent_rigid_bodies_contact":
            raise RuntimeError(f"formal scene has no rigid-body contact load path: {initial}")
        if initial.get("accounting_valid") is not True:
            raise RuntimeError(f"formal scene dry ledger is not valid: {initial}")
        if initial.get("inside_candidate_count") != expected_initial_count:
            raise RuntimeError(f"formal scene starts with litter already in the bin: {initial}")
        if initial.get("inertial_candidate_count") != expected_initial_count:
            raise RuntimeError(f"formal scene initial inertial count is invalid: {initial}")
        if abs(float(initial.get("physical_contained_mass_kg", -1.0)) - expected_initial_mass) > 1.0e-9:
            raise RuntimeError(f"formal scene starts with unexpected physical bin mass: {initial}")
        report["initial_evaluator_bin_status"] = initial
        for request in manifest["requests"]:
            target_id = request["target_id"]
            product_request = {key: value for key, value in request.items() if key != "acceptance"}
            encoded = json.dumps(product_request, sort_keys=True)
            inhibit_start = len(node.inhibit_events)
            expected_count = request["acceptance"]["expected_count_after"]
            expected_cumulative = request["acceptance"]["expected_cumulative_mass_kg"]
            expected_increment = request["acceptance"]["expected_increment_kg"]
            expected_prior_count = expected_count - 1
            expected_prior_mass = expected_cumulative - expected_increment
            attempts: list[dict[str, object]] = []
            result: dict[str, object] | None = None
            recheck_sent = False
            for attempt_number in range(1, maximum_attempts + 1):
                idle_deadline = time.monotonic() + 5.0
                while time.monotonic() < idle_deadline and node.executor_busy is not False:
                    node.spin_once()
                if node.executor_busy is not False:
                    raise RuntimeError(f"{target_id} executor did not become idle")
                node.results.pop(target_id, None)
                node.awaiting_recheck = False
                node.request.publish(String(data=encoded))
                target_deadline = time.monotonic() + args.per_target_timeout
                attempt_recheck_sent = False
                while target_id not in node.results and time.monotonic() < target_deadline:
                    node.spin_once()
                    if node.awaiting_recheck and not attempt_recheck_sent:
                        node.wrist.publish(String(data=encoded))
                        attempt_recheck_sent = True
                if target_id not in node.results:
                    raise TimeoutError(f"{target_id} attempt {attempt_number} did not complete")
                result = node.results[target_id]
                recheck_sent = recheck_sent or attempt_recheck_sent
                attempt_evidence = result.get("evidence", {})
                attempt_row: dict[str, object] = {
                    "attempt_number": attempt_number,
                    "verified_in_bin": result.get("verified_in_bin") is True,
                    "reason": result.get("reason"),
                    "wrist_recheck_sent": attempt_recheck_sent,
                    "executor_declared_retryable_without_operator": (
                        isinstance(attempt_evidence, dict)
                        and attempt_evidence.get("retryable_without_operator") is True
                    ),
                }
                attempts.append(attempt_row)
                if result.get("verified_in_bin") is True:
                    break
                if attempt_number >= maximum_attempts:
                    break

                # The bounded second attempt is legal only when the executor
                # restored transport/gate state before physical grasp and the
                # evaluator independently proves that no body or mass entered
                # the bin during the failed attempt.
                retry_deadline = time.monotonic() + 5.0
                retry_payload_unchanged = False
                while time.monotonic() < retry_deadline:
                    node.spin_once()
                    retry_payload_unchanged = _evaluator_payload_matches(
                        node.evaluator_bin_status,
                        expected_count=expected_prior_count,
                        expected_mass_kg=expected_prior_mass,
                    )
                    if retry_payload_unchanged and node.executor_busy is False:
                        break
                retryable = (
                    isinstance(attempt_evidence, dict)
                    and attempt_evidence.get("retryable_without_operator") is True
                    and isinstance(attempt_evidence.get("failure_recovery"), dict)
                    and attempt_evidence["failure_recovery"].get("completed") is True
                    and retry_payload_unchanged
                    and node.executor_busy is False
                )
                attempt_row["retry_payload_unchanged"] = retry_payload_unchanged
                attempt_row["retry_authorized"] = retryable
                if not retryable:
                    break
            if result is None:
                raise RuntimeError(f"{target_id} produced no attempt result")
            evidence = result.get("evidence", {})
            bin_verification = evidence.get("dry_bin_verification", {})
            mass = bin_verification.get("measured_increment_kg")
            expected = expected_increment
            measured_material = bin_verification.get(
                "post_deposit_material_from_load_increment"
            )
            if isinstance(mass, (int, float)):
                measured_cumulative_mass_kg += float(mass)
            measured_count = bin_verification.get("contained_object_count")
            # The product result is load-cell equivalent data.  The separate
            # evaluator channel proves that 20 spawned rigid bodies remain in
            # the world and that exactly one additional inertial body is now
            # inside the physical bin; it is never an executor input.
            evaluator_deadline = time.monotonic() + 5.0
            evaluator = node.evaluator_bin_status
            while time.monotonic() < evaluator_deadline:
                node.spin_once()
                evaluator = node.evaluator_bin_status
                if (
                    isinstance(evaluator, dict)
                    and evaluator.get("candidate_model_count") == 20
                    and evaluator.get("inside_candidate_count") == expected_count
                    and evaluator.get("inertial_candidate_count") == expected_count
                ):
                    break
            evaluator_count_matches = (
                isinstance(evaluator, dict)
                and evaluator.get("candidate_model_count") == 20
                and evaluator.get("inside_candidate_count") == expected_count
                and evaluator.get("inertial_candidate_count") == expected_count
            )
            evaluator_mass_matches = (
                isinstance(evaluator, dict)
                and abs(float(evaluator.get("physical_contained_mass_kg", -1.0)) - expected_cumulative)
                <= 1.0e-5
            )
            inhibit_cycle = node.inhibit_events[inhibit_start:]
            row = {
                "target_id": target_id,
                "attempt_count": len(attempts),
                "maximum_attempts": maximum_attempts,
                "attempts": attempts,
                "bounded_attempt_contract_passed": 1 <= len(attempts) <= maximum_attempts,
                "verified_in_bin": result.get("verified_in_bin") is True,
                "wrist_recheck_sent": recheck_sent,
                "product_material": request["material"],
                "actual_material_evaluator_only": request["acceptance"]["actual_material_evaluator_only"],
                "post_deposit_material_from_load_increment": measured_material,
                "material_matches": measured_material
                == request["acceptance"]["actual_material_evaluator_only"],
                "measured_increment_kg": mass,
                "expected_increment_kg": expected,
                "mass_matches": isinstance(mass, (int, float)) and abs(float(mass) - expected) <= 1.0e-5,
                "contained_object_count": measured_count,
                "expected_count_after": expected_count,
                "count_matches": measured_count == expected_count,
                "measured_cumulative_mass_kg": round(measured_cumulative_mass_kg, 9),
                "expected_cumulative_mass_kg": expected_cumulative,
                "cumulative_mass_matches": abs(
                    measured_cumulative_mass_kg - expected_cumulative
                )
                <= 1.0e-5,
                "evaluator_physical_body_count_matches": evaluator_count_matches,
                "evaluator_physical_mass_matches": evaluator_mass_matches,
                "base_motion_inhibit_cycle_observed": True in inhibit_cycle and False in inhibit_cycle,
            }
            report["results"].append(row)
            if not all(
                (
                    row["verified_in_bin"],
                    row["bounded_attempt_contract_passed"],
                    row["wrist_recheck_sent"],
                    row["material_matches"],
                    row["mass_matches"],
                    row["count_matches"],
                    row["cumulative_mass_matches"],
                    row["evaluator_physical_body_count_matches"],
                    row["evaluator_physical_mass_matches"],
                    row["base_motion_inhibit_cycle_observed"],
                )
            ):
                raise RuntimeError(f"{target_id} failed: {row}")
        report["base_motion_inhibit_observed"] = True in node.inhibit_events and False in node.inhibit_events
        if not report["base_motion_inhibit_observed"]:
            raise RuntimeError("base motion inhibit true/false lifecycle was not observed")
        report["completed_target_count"] = len(report["results"])
        report["final_measured_physical_resident_mass_kg"] = round(
            measured_cumulative_mass_kg, 9
        )
        report["physical_resident_mass_chain_passed"] = (
            report["completed_target_count"] == report["expected_target_count"]
            and abs(
                measured_cumulative_mass_kg
                - float(report["expected_final_physical_resident_mass_kg"])
            )
            <= 1.0e-5
        )
        report["final_evaluator_bin_status"] = node.evaluator_bin_status
        report["physical_rigid_body_payload_retained"] = (
            isinstance(node.evaluator_bin_status, dict)
            and node.evaluator_bin_status.get("dry_accounting_mode")
            == "physical_resident"
            and node.evaluator_bin_status.get("resident_load_path")
            == "independent_rigid_bodies_contact"
            and node.evaluator_bin_status.get("accounting_valid") is True
            and node.evaluator_bin_status.get("candidate_model_count") == 20
            and node.evaluator_bin_status.get("inside_candidate_count") == 20
            and node.evaluator_bin_status.get("inertial_candidate_count") == 20
        )
        # The product-side measured increments and evaluator-only rigid-body
        # mass must independently close on the same final total.  This is a
        # physical-resident ledger, not a DynamicPayload aggregate update:
        # the retained bodies transfer vehicle load through their contacts.
        report["final_evaluator_physical_mass_kg"] = (
            node.evaluator_bin_status.get("physical_contained_mass_kg")
            if isinstance(node.evaluator_bin_status, dict)
            else None
        )
        report["physical_final_mass_matches_resident_chain"] = _evaluator_payload_matches(
            node.evaluator_bin_status,
            expected_count=20,
            expected_mass_kg=float(report["expected_final_physical_resident_mass_kg"]),
        ) and abs(
            measured_cumulative_mass_kg
            - float(report["expected_final_physical_resident_mass_kg"])
        ) <= 1.0e-5
        report["physical_resident_mass_chain_passed"] = (
            report["physical_resident_mass_chain_passed"]
            and report["physical_final_mass_matches_resident_chain"]
        )
        if not report["physical_rigid_body_payload_retained"]:
            raise RuntimeError("20 physical rigid bodies were not retained in the dry bin")
        if not report["physical_resident_mass_chain_passed"]:
            raise RuntimeError(
                "20-cube physical-resident mass chain did not close against "
                "the evaluator-only physical rigid-body mass"
            )
        report["passed"] = True
        report["status"] = "FORMAL_20_CUBE_TARGET_CONDITIONED_GRASP_PASSED"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
