#!/usr/bin/env python3
"""Validate live Gazebo joint-state evidence for the four body service doors."""

from __future__ import annotations

import argparse
import json
import math
import re
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import median
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_LOGICAL_PATH = "reports/engineering/formal_vehicle_snapshot_manifest.json"

PASSED_STATUS = "FORMAL_BODYWORK_SERVICE_DOOR_RUNTIME_PASSED"
FAILED_STATUS = "FORMAL_BODYWORK_SERVICE_DOOR_RUNTIME_FAILED"
DOORS = {
    "power": ("bodywork_power_service_door_hinge_joint", "bodywork_power_service_door_latch_joint", 0.0, 1.745329252, 1.2),
    "compute": ("bodywork_compute_service_door_hinge_joint", "bodywork_compute_service_door_latch_joint", -1.745329252, 0.0, -1.2),
    "wet": ("bodywork_wet_service_door_hinge_joint", "bodywork_wet_service_door_latch_joint", -1.745329252, 0.0, -1.2),
    "rear_dry": ("bodywork_rear_dry_service_door_hinge_joint", "bodywork_rear_dry_service_door_latch_joint", -1.745329252, 0.0, -1.2),
}
PHASES = (
    "initial_locked",
    "locked_open_rejected",
    "unlocked",
    "open",
    "closed_unlocked",
    "transport_locked",
    "relock_open_rejected",
)


def _advancing_complete_samples(samples: object, minimum: int = 5) -> bool:
    if not isinstance(samples, list) or len(samples) < minimum:
        return False
    window = samples[-minimum:]
    expected = {item for spec in DOORS.values() for item in spec[:2]}
    times: list[int] = []
    for sample in window:
        if not isinstance(sample, dict) or set(sample.get("positions_rad", {})) != expected:
            return False
        value = sample.get("sim_clock_ns")
        if not isinstance(value, int):
            return False
        times.append(value)
    return all(left < right for left, right in zip(times, times[1:]))


def _phase_delivery_and_timing_valid(
    evidence: dict[str, Any], snapshot_manifest: Path | None
) -> bool:
    limits = evidence.get("urdf_velocity_limits_rad_per_s")
    contract = evidence.get("phase_timing_contract")
    if not isinstance(limits, dict) or not isinstance(contract, dict):
        return False
    try:
        minimum = float(contract["minimum_duration_s"])
        margin = float(contract["settling_margin_s"])
        fresh = int(contract["minimum_fresh_samples"])
    except (KeyError, TypeError, ValueError):
        return False
    if not math.isfinite(minimum) or minimum <= 0 or not math.isfinite(margin) or margin < 0 or fresh < 5:
        return False
    joints = {item for spec in DOORS.values() for item in spec[:2]}
    if set(limits) != joints:
        return False
    manifest_value = evidence.get("snapshot_manifest")
    if (
        not isinstance(manifest_value, str)
        or manifest_value != SNAPSHOT_LOGICAL_PATH
        or Path(manifest_value).is_absolute()
        or ".." in Path(manifest_value).parts
    ):
        return False
    manifest_path = snapshot_manifest if snapshot_manifest is not None else ROOT / manifest_value
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        output = manifest["outputs"]["reports/engineering/formal_competition_vehicle.urdf"]
        urdf_path = manifest_path.parents[2] / "reports/engineering/formal_competition_vehicle.urdf"
        if (
            hashlib.sha256(urdf_path.read_bytes()).hexdigest() != output["sha256"]
            or hashlib.sha256(manifest_path.read_bytes()).hexdigest()
            != evidence.get("source_binding", {}).get("snapshot_manifest_sha256")
            or evidence.get("source_binding", {}).get("expanded_urdf_sha256") != output["sha256"]
        ):
            return False
        root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
        bound_limits = {
            joint.get("name"): float(joint.find("limit").get("velocity"))
            for joint in root.findall("joint")
            if joint.get("name") in joints and joint.find("limit") is not None
        }
    except (OSError, KeyError, TypeError, ValueError, ET.ParseError):
        return False
    if set(bound_limits) != joints or any(
        not math.isclose(float(limits[name]), value, abs_tol=1e-12)
        for name, value in bound_limits.items()
    ):
        return False
    try:
        if any(not math.isfinite(float(limits[joint])) or float(limits[joint]) <= 0 for joint in joints):
            return False
    except (TypeError, ValueError):
        return False
    previous = {door: {"hinge": 0.0, "latch": 0.0} for door in DOORS}
    for name in PHASES:
        phase = evidence.get("phases", {}).get(name, {})
        if not isinstance(phase, dict):
            return False
        targets = phase.get("commanded_targets_rad")
        if not isinstance(targets, dict):
            return False
        required = 0.0
        for door, spec in DOORS.items():
            row = targets.get(door)
            if not isinstance(row, dict):
                return False
            for kind, joint in (("hinge", spec[0]), ("latch", spec[1])):
                try:
                    required = max(required, abs(float(row[kind]) - previous[door][kind]) / float(limits[joint]))
                except (KeyError, TypeError, ValueError):
                    return False
        try:
            requested = float(phase["simulated_duration_requested_s"])
            start = int(phase["sim_clock_start_ns"])
            end = int(phase["sim_clock_end_ns"])
        except (KeyError, TypeError, ValueError):
            return False
        if requested + 1e-9 < max(minimum, required + margin) or end - start + 1 < math.ceil(requested * 1e9):
            return False
        echo = phase.get("plugin_target_echo")
        if not isinstance(echo, dict) or set(echo) != set(DOORS):
            return False
        for door, row in targets.items():
            detail = echo.get(door)
            if not isinstance(detail, dict):
                return False
            for kind in ("hinge", "latch"):
                item = detail.get(kind)
                if not isinstance(item, dict) or item.get("delivered_after_previous_generation") is not True:
                    return False
                try:
                    if float(item["received_messages"]) <= float(item["previous_received_messages"]):
                        return False
                    if not math.isclose(float(item["received_target_rad"]), float(row[kind]), abs_tol=1e-6):
                        return False
                except (KeyError, TypeError, ValueError):
                    return False
        if not _advancing_complete_samples(phase.get("fresh_complete_samples_after_plugin_echo"), fresh):
            return False
        previous = targets
    ready = evidence.get("target_transport", {}).get("fresh_complete_samples_after_ready")
    return _advancing_complete_samples(ready, fresh)


def _gazebo_sidecar_valid(evidence: dict[str, Any]) -> bool:
    sidecar = evidence.get("gazebo_joint_state_sidecar")
    if not isinstance(sidecar, dict) or sidecar.get("status") != "PASSED":
        return False
    expected = {item for spec in DOORS.values() for item in spec[:2]}
    topic = sidecar.get("gazebo_transport_topic")
    discovered = sidecar.get("discovered_topic")
    raw_names: list[str] = []
    for body in re.findall(r"(?ms)^\s*joint\s*\{(.*?)^\s*\}", str(sidecar.get("topic_sample_output", ""))):
        match = re.search(r'^\s*name:\s*"([^"]+)"\s*$', body, re.MULTILINE)
        if match is not None:
            raw_names.append(match.group(1))
    return (
        sidecar.get("gz_partition") == evidence.get("gazebo_partition")
        and isinstance(topic, str) and isinstance(discovered, str)
        and (discovered == topic or discovered.endswith(topic))
        and sidecar.get("topic_candidates") == [discovered]
        and sidecar.get("message_type") == "gz.msgs.Model"
        and sidecar.get("publisher_count") == 1
        and set(sidecar.get("observed_joint_names", [])) == expected
        and len(sidecar.get("observed_joint_names", [])) == len(expected)
        and raw_names == sidecar.get("observed_joint_names")
        and isinstance(sidecar.get("launcher_pid"), int) and sidecar.get("launcher_pid") > 0
        and isinstance(sidecar.get("launcher_liveness_checks"), list)
        and len(sidecar.get("launcher_liveness_checks")) >= 2
        and all(value is True for value in sidecar.get("launcher_liveness_checks"))
        and all(isinstance(sidecar.get(key), str) for key in (
            "topic_list_output", "topic_info_output", "topic_sample_output"
        ))
    )


def _phase_samples(evidence: dict[str, Any], name: str) -> list[dict[str, Any]]:
    phase = evidence.get("phases", {}).get(name, {})
    samples = phase.get("joint_state_samples", []) if isinstance(phase, dict) else []
    return samples if isinstance(samples, list) else []


def _final_positions(evidence: dict[str, Any], phase: str) -> dict[str, float]:
    samples = _phase_samples(evidence, phase)
    result: dict[str, float] = {}
    for joint in {item for spec in DOORS.values() for item in spec[:2]}:
        values = [
            float(sample["positions_rad"][joint])
            for sample in samples[-5:]
            if isinstance(sample, dict)
            and isinstance(sample.get("positions_rad"), dict)
            and joint in sample["positions_rad"]
        ]
        if values:
            result[joint] = median(values)
    return result


def _commands_match(evidence: dict[str, Any]) -> bool:
    phases = evidence.get("phases", {})
    if not isinstance(phases, dict):
        return False
    expected: dict[str, tuple[float, float]] = {
        "initial_locked": (0.0, 0.0),
        "locked_open_rejected": (math.nan, 0.0),
        "unlocked": (0.0, 0.6),
        "open": (math.nan, 0.6),
        "closed_unlocked": (0.0, 0.6),
        "transport_locked": (0.0, 0.0),
        "relock_open_rejected": (math.nan, 0.0),
    }
    for phase_name, (hinge_expected, latch_expected) in expected.items():
        commands = phases.get(phase_name, {}).get("commanded_targets_rad", {})
        if not isinstance(commands, dict):
            return False
        for door, (_, _, _, _, open_target) in DOORS.items():
            row = commands.get(door)
            if not isinstance(row, dict):
                return False
            required_hinge = open_target if math.isnan(hinge_expected) else hinge_expected
            if not math.isclose(float(row.get("hinge", math.nan)), required_hinge, abs_tol=1e-9):
                return False
            if not math.isclose(float(row.get("latch", math.nan)), latch_expected, abs_tol=1e-9):
                return False
    return True


def _timestamps_are_fresh_and_ordered(evidence: dict[str, Any]) -> bool:
    previous = -1
    for phase in PHASES:
        samples = _phase_samples(evidence, phase)
        timestamps = [
            sample.get("received_monotonic_ns")
            for sample in samples
            if isinstance(sample, dict)
        ]
        if (
            len(timestamps) != len(samples)
            or len(timestamps) < 5
            or any(not isinstance(value, int) for value in timestamps)
            or any(left >= right for left, right in zip(timestamps, timestamps[1:]))
            or (timestamps and timestamps[0] <= previous)
        ):
            return False
        previous = timestamps[-1]
    return True


def _all_target_publishers_have_bridge_subscribers(evidence: dict[str, Any]) -> bool:
    for phase in PHASES:
        row = evidence.get("phases", {}).get(phase, {})
        counts = row.get("ros_publisher_subscription_counts", {}) if isinstance(row, dict) else {}
        if not isinstance(counts, dict) or len(counts) != len(DOORS) * 2:
            return False
        if any(not isinstance(value, int) or value < 1 for value in counts.values()):
            return False
    return True


def _plugin_lifecycle_configuration_observed(evidence: dict[str, Any]) -> bool:
    telemetry = evidence.get("plugin_diagnostics", {})
    lifecycle = telemetry.get("lifecycle", []) if isinstance(telemetry, dict) else []
    if not isinstance(lifecycle, list):
        return False
    def numeric(record: dict[str, Any], key: str) -> float | None:
        try:
            value = float(record.get(key, 0.0))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    configured = any(
        isinstance(record, dict)
        and record.get("event") == "configured"
        and numeric(record, "configured") == 1.0
        and numeric(record, "doors") == float(len(DOORS))
        for record in lifecycle
    )
    subscriptions = {
        record.get("door")
        for record in lifecycle
        if isinstance(record, dict)
        and record.get("event") == "subscription"
        and numeric(record, "hinge_subscribed") == 1.0
        and numeric(record, "latch_subscribed") == 1.0
    }
    return configured and subscriptions == set(DOORS)


def _plugin_reports_received_targets_and_force_writes(evidence: dict[str, Any]) -> bool:
    telemetry = evidence.get("plugin_diagnostics", {})
    records = telemetry.get("records", []) if isinstance(telemetry, dict) else []
    by_door: dict[str, list[dict[str, Any]]] = {door: [] for door in DOORS}
    for record in records:
        if isinstance(record, dict) and record.get("door") in by_door:
            by_door[record["door"]].append(record)
    for door in DOORS:
        door_records = by_door[door]
        if not door_records:
            return False
        required = {
            "sim_time_sec", "received_hinge_messages", "received_latch_messages",
            "received_hinge_target_rad", "received_latch_target_rad",
            "requested_hinge_rad", "requested_latch_rad",
            "effective_hinge_rad", "effective_latch_rad",
            "hinge_position_rad", "latch_position_rad",
            "hinge_force_nm", "latch_force_nm",
            "hinge_force_writes", "latch_force_writes",
            "postupdate_hinge_force_present", "postupdate_latch_force_present",
            "postupdate_hinge_force_nm", "postupdate_latch_force_nm",
        }
        if any(not required <= set(record) for record in door_records):
            return False
        try:
            if any(
                not math.isfinite(float(record[key]))
                for record in door_records
                for key in required
            ):
                return False
        except (TypeError, ValueError):
            return False
        for key in (
            "sim_time_sec", "received_hinge_messages", "received_latch_messages",
            "hinge_force_writes", "latch_force_writes",
        ):
            try:
                values = [float(record[key]) for record in door_records]
            except (TypeError, ValueError):
                return False
            if any(not math.isfinite(value) or value < 0.0 for value in values) or any(
                left > right for left, right in zip(values, values[1:])
            ):
                return False
        sim_times = [float(record["sim_time_sec"]) for record in door_records]
        if any(left >= right for left, right in zip(sim_times, sim_times[1:])):
            return False
        record = door_records[-1]
        try:
            latest_counts = [
                float(record.get(key, 0.0))
                for key in (
                    "received_hinge_messages", "received_latch_messages",
                    "hinge_force_writes", "latch_force_writes",
                )
            ]
        except (TypeError, ValueError):
            return False
        if any(value < 1.0 for value in latest_counts):
            return False
        # The first and last snapshots can be startup/teardown boundaries.
        # Require a stable in-between readback; it is still only an ECM
        # observation, not a claim about the final writer or physics response.
        try:
            stable_postupdate_readback = any(
                float(record["postupdate_hinge_force_present"]) == 1.0
                and float(record["postupdate_latch_force_present"]) == 1.0
                and math.isfinite(float(record["postupdate_hinge_force_nm"]))
                and math.isfinite(float(record["postupdate_latch_force_nm"]))
                for record in door_records[1:-1]
            )
        except (KeyError, TypeError, ValueError):
            return False
        if len(door_records) < 3 or not stable_postupdate_readback:
            return False
        try:
            latch_unlocked = any(
                math.isclose(float(record["effective_latch_rad"]), 0.6, abs_tol=0.02)
                for record in door_records
                if "effective_latch_rad" in record
            )
            hinge_opened = any(
                math.isclose(
                    float(record["effective_hinge_rad"]), DOORS[door][4], abs_tol=0.02
                )
                for record in door_records
                if "effective_hinge_rad" in record
            )
        except (TypeError, ValueError):
            return False
        if not latch_unlocked:
            return False
        if not hinge_opened:
            return False
    return True


def _failed_report(evidence: object, error: Exception) -> dict[str, Any]:
    """Preserve a machine-readable failed report when malformed evidence hits a validator edge."""

    value = evidence if isinstance(evidence, dict) else {}
    return {
        "report_id": "tzcup_formal_service_door_runtime_v1",
        "status": FAILED_STATUS,
        "passed": False,
        "checks": {"validator_completed": False},
        "validation_error": f"{type(error).__name__}: {error}",
        "source_binding": _json_safe(value.get("source_binding", {})),
        "evidence_authority": _json_safe(value.get("evidence_authority")),
        "sample_counts": {},
        "final_positions_rad": {},
        # Do not echo malformed raw samples or diagnostics into a report that
        # must be consumable by strict JSON parsers.
        "phases": {},
        "plugin_diagnostics": {},
        "gazebo_joint_state_sidecar": {},
        "claim_boundary": "Malformed or non-finite runtime evidence is rejected fail-closed.",
    }


def _json_safe(value: object) -> object:
    """Convert diagnostic metadata to JSON values without preserving NaN/Inf."""

    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def report_json(evidence: object, report: dict[str, Any]) -> tuple[dict[str, Any], str]:
    """Return the final strict-JSON report and its serialized representation."""

    try:
        return report, json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    except (TypeError, ValueError) as exc:
        final_report = _failed_report(evidence, exc)
        return final_report, json.dumps(
            final_report, indent=2, sort_keys=True, allow_nan=False
        ) + "\n"


def evaluate(
    evidence: dict[str, Any], snapshot_manifest: Path | None = None
) -> dict[str, Any]:
    try:
        return _evaluate(evidence, snapshot_manifest)
    except (ArithmeticError, KeyError, TypeError, ValueError) as exc:
        return _failed_report(evidence, exc)


def _evaluate(evidence: dict[str, Any], snapshot_manifest: Path | None) -> dict[str, Any]:
    sample_counts = {name: len(_phase_samples(evidence, name)) for name in PHASES}
    finals = {name: _final_positions(evidence, name) for name in PHASES}
    expected_joints = {item for spec in DOORS.values() for item in spec[:2]}

    samples_complete = all(count >= 5 for count in sample_counts.values()) and all(
        set(position) == expected_joints for position in finals.values()
    )
    within_limits = True
    for phase in PHASES:
        for sample in _phase_samples(evidence, phase):
            positions = sample.get("positions_rad", {}) if isinstance(sample, dict) else {}
            if set(positions) != expected_joints:
                within_limits = False
                continue
            for hinge, latch, lower, upper, _ in DOORS.values():
                hinge_value = float(positions[hinge])
                latch_value = float(positions[latch])
                if (
                    not math.isfinite(hinge_value)
                    or not math.isfinite(latch_value)
                    or hinge_value < lower - 0.03
                    or hinge_value > upper + 0.03
                    or abs(latch_value) > 0.815398163
                ):
                    within_limits = False

    def all_closed(phase: str) -> bool:
        return all(abs(finals.get(phase, {}).get(spec[0], math.inf)) <= 0.08 for spec in DOORS.values())

    def all_locked(phase: str) -> bool:
        return all(abs(finals.get(phase, {}).get(spec[1], math.inf)) <= 0.08 for spec in DOORS.values())

    def all_unlocked(phase: str) -> bool:
        return all(abs(finals.get(phase, {}).get(spec[1], 0.0)) >= 0.35 for spec in DOORS.values())

    opened = all(
        abs(finals.get("open", {}).get(hinge, 0.0)) >= 0.90
        and (finals["open"][hinge] > 0.0) == (target > 0.0)
        for hinge, _, _, _, target in DOORS.values()
    )
    checks = {
        "source_bound_to_expanded_urdf": isinstance(evidence.get("source_binding"), dict)
        and re.fullmatch(r"[0-9a-f]{64}", str(evidence["source_binding"].get("expanded_urdf_sha256", ""))) is not None,
        "physical_joint_state_authority": (
            evidence.get("evidence_authority") == "GAZEBO_MODEL_JOINT_STATE_BRIDGE"
            and evidence.get("physical_joint_state_topic")
            == "/formal/service_door_joint_states"
        ),
        "all_phases_have_fresh_complete_samples": samples_complete,
        "joint_samples_are_strictly_ordered_across_phases": (
            _timestamps_are_fresh_and_ordered(evidence)
        ),
        "command_sequence_unlocks_before_opening": _commands_match(evidence),
        "all_target_publishers_have_ros_bridge_subscribers": (
            _all_target_publishers_have_bridge_subscribers(evidence)
        ),
        "startup_and_phase_delivery_use_fresh_advancing_simulated_time": (
            _phase_delivery_and_timing_valid(evidence, snapshot_manifest)
        ),
        "independent_gazebo_joint_state_sidecar_is_complete": _gazebo_sidecar_valid(evidence),
        "plugin_lifecycle_configuration_observed": (
            _plugin_lifecycle_configuration_observed(evidence)
        ),
        "plugin_reports_received_targets_and_force_writes": (
            _plugin_reports_received_targets_and_force_writes(evidence)
        ),
        "all_samples_remain_inside_urdf_limits": within_limits,
        "initial_transport_state_locked_and_closed": all_closed("initial_locked") and all_locked("initial_locked"),
        "locked_hinges_reject_open_command": all_closed("locked_open_rejected") and all_locked("locked_open_rejected"),
        "all_latches_physically_unlock": all_closed("unlocked") and all_unlocked("unlocked"),
        "all_hinges_physically_open_after_unlock": opened and all_unlocked("open"),
        "all_hinges_physically_close_before_relock": all_closed("closed_unlocked") and all_unlocked("closed_unlocked"),
        "transport_state_relocks_at_zero": all_closed("transport_locked") and all_locked("transport_locked"),
        "relocked_hinges_reject_open_command": all_closed("relock_open_rejected") and all_locked("relock_open_rejected"),
    }
    passed = all(checks.values())
    return {
        "report_id": "tzcup_formal_service_door_runtime_v1",
        "status": PASSED_STATUS if passed else FAILED_STATUS,
        "passed": passed,
        "checks": checks,
        "source_binding": evidence.get("source_binding", {}),
        "evidence_authority": evidence.get("evidence_authority"),
        "sample_counts": sample_counts,
        "final_positions_rad": finals,
        "phases": evidence.get("phases", {}),
        "plugin_diagnostics": evidence.get("plugin_diagnostics", {}),
        "gazebo_joint_state_sidecar": evidence.get("gazebo_joint_state_sidecar", {}),
        "claim_boundary": (
            "This proves bounded-force Gazebo motion of all four physical door hinges and latches, "
            "including measured unlock-before-open and zero-angle transport relock. It does not "
            "claim real handle ergonomics, seal compression, fatigue life or certified retention force. "
            "Plugin diagnostics only locate message and command-component behavior; they do not "
            "alone identify the final ECM writer or prove physics consumption."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--snapshot-manifest", type=Path)
    args = parser.parse_args()
    evidence = json.loads(args.input.read_text(encoding="utf-8"))
    result = evaluate(evidence, args.snapshot_manifest)
    final_report, text = report_json(evidence, result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if final_report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
