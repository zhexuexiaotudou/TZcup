"""Evidence contracts for PC_ONNX split-loopback emulation."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Mapping


RUNTIME_BACKENDS = frozenset({"PC_ONNX", "JOURNEY6_OE"})
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
RUN_ID_PATTERN = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$"
)
FORMAL_STATUS_EVALUATOR_BLOCKER = (
    "trusted_run_bound_attestation_collector_not_implemented"
)


def validate_run_id(run_id: str) -> str:
    normalized = str(run_id).strip().lower()
    if RUN_ID_PATTERN.fullmatch(normalized) is None:
        raise ValueError("HIL run_id must be a non-empty UUID")
    return normalized


def synthetic_sensor_publishers_allowed(sensor_source: str) -> bool:
    if sensor_source not in {"synthetic_transport_probe", "gazebo"}:
        raise ValueError("unsupported HIL sensor source")
    return sensor_source == "synthetic_transport_probe"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _number_at_least(value: object, minimum: float) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and number >= minimum


def _verified_file_under_root(
    root: Path, relative: object, expected_sha256: object
) -> bool:
    if not isinstance(relative, str) or not relative:
        return False
    expected = str(expected_sha256).lower()
    if not SHA256_PATTERN.fullmatch(expected):
        return False
    root_resolved = root.resolve()
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root_resolved)
    except ValueError:
        return False
    return candidate.is_file() and _sha256(candidate) == expected


def _referenced_evidence(
    root: Path, section: object, *, pass_key: str, run_id: str
) -> tuple[dict[str, object], str] | None:
    if not isinstance(section, Mapping) or section.get("pass") is not True:
        return None
    relative = section.get("evidence_file")
    expected_sha = str(section.get("evidence_sha256", "")).lower()
    if not isinstance(relative, str) or not SHA256_PATTERN.fullmatch(expected_sha):
        return None
    root_resolved = root.resolve()
    evidence_path = (root / relative).resolve()
    try:
        evidence_path.relative_to(root_resolved)
    except ValueError:
        return None
    if not evidence_path.is_file() or _sha256(evidence_path) != expected_sha:
        return None
    try:
        payload = json.loads(evidence_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if (
        not isinstance(payload, dict)
        or payload.get(pass_key) is not True
        or payload.get("run_id") != run_id
    ):
        return None
    return payload, expected_sha


def audit_model_qualification_manifest(
    manifest_path: Path, *, model_id: str, model_sha256: str, run_id: str
) -> dict[str, object]:
    """Bind model readiness to hashed, content-checked qualification evidence."""
    result: dict[str, object] = {
        "manifest_sha256": None,
        "run_id": run_id,
        "model_id": model_id,
        "model_sha256": model_sha256,
        "pt_onnx_parity_pass": False,
        "pc_inference_pass": False,
        "full_stack_evidence_sha256": None,
        "full_stack_pass": False,
    }
    try:
        run_id = validate_run_id(run_id)
    except ValueError:
        return result
    result["run_id"] = run_id
    if not manifest_path.is_file() or not SHA256_PATTERN.fullmatch(model_sha256):
        return result
    result["manifest_sha256"] = _sha256(manifest_path)
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return result
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        return result
    if manifest.get("run_id") != run_id:
        return result
    if manifest.get("model_id") != model_id:
        return result
    if str(manifest.get("model_sha256", "")).lower() != model_sha256:
        return result
    root = manifest_path.parent
    parity = _referenced_evidence(
        root,
        manifest.get("pt_onnx_parity"),
        pass_key="pt_onnx_parity_pass",
        run_id=run_id,
    )
    pc_inference = _referenced_evidence(
        root,
        manifest.get("pc_inference"),
        pass_key="pc_inference_pass",
        run_id=run_id,
    )
    full_stack = _referenced_evidence(
        root,
        manifest.get("full_stack"),
        pass_key="full_stack_pass",
        run_id=run_id,
    )
    if parity is not None:
        payload, _ = parity
        result["pt_onnx_parity_pass"] = (
            payload.get("model_id") == model_id
            and str(payload.get("model_sha256", "")).lower() == model_sha256
        )
    if pc_inference is not None:
        payload, _ = pc_inference
        result["pc_inference_pass"] = all(
            (
                payload.get("model_id") == model_id,
                str(payload.get("model_sha256", "")).lower() == model_sha256,
                payload.get("real_execution") is True,
                _number_at_least(payload.get("inference_count"), 1.0),
            )
        )
    if full_stack is not None:
        payload, evidence_sha = full_stack
        result["full_stack_evidence_sha256"] = evidence_sha
        result["full_stack_pass"] = all(
            (
                payload.get("model_id") == model_id,
                str(payload.get("model_sha256", "")).lower() == model_sha256,
                payload.get("real_execution") is True,
                _number_at_least(payload.get("duration_s"), 1800.0),
            )
        )
    return result


def audit_gazebo_sensor_provenance(
    manifest_path: Path, *, run_id: str
) -> dict[str, object]:
    """Verify hashed launch, process, and publisher provenance for Gazebo."""
    result: dict[str, object] = {
        "audited_launch": False,
        "gazebo_process_verified": False,
        "publisher_endpoints_verified": False,
        "pc_sensor_and_plant_only": False,
        "evidence_sha256": None,
        "run_id": run_id,
    }
    try:
        run_id = validate_run_id(run_id)
    except ValueError:
        return result
    result["run_id"] = run_id
    if not manifest_path.is_file():
        return result
    result["evidence_sha256"] = _sha256(manifest_path)
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return result
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        return result
    if manifest.get("run_id") != run_id:
        return result
    root = manifest_path.parent
    launch = _referenced_evidence(
        root,
        manifest.get("audited_launch_evidence"),
        pass_key="audited_launch",
        run_id=run_id,
    )
    processes = _referenced_evidence(
        root,
        manifest.get("gazebo_process_evidence"),
        pass_key="gazebo_process_verified",
        run_id=run_id,
    )
    endpoints = _referenced_evidence(
        root,
        manifest.get("publisher_endpoint_evidence"),
        pass_key="publisher_endpoints_verified",
        run_id=run_id,
    )
    if launch is not None:
        payload, _ = launch
        result["audited_launch"] = (
            payload.get("pc_sensor_and_plant_only") is True
            and payload.get("forbidden_algorithm_nodes") == []
            and _verified_file_under_root(
                root,
                payload.get("launch_file"),
                payload.get("launch_file_sha256"),
            )
        )
        result["pc_sensor_and_plant_only"] = result["audited_launch"]
    if processes is not None:
        payload, _ = processes
        result["gazebo_process_verified"] = bool(
            payload.get("gazebo_processes")
        ) and payload.get("algorithm_processes") == []
    if endpoints is not None:
        payload, _ = endpoints
        required = {
            "/hil/clock",
            "/hil/camera/color",
            "/hil/camera/depth",
            "/hil/camera/camera_info",
            "/hil/tf",
            "/hil/tf_static",
        }
        observed = payload.get("publisher_topics", [])
        result["publisher_endpoints_verified"] = (
            isinstance(observed, list)
            and required <= set(observed)
            and payload.get("publisher_process_links_verified") is True
            and payload.get("harness_sensor_publishers_present") is False
            and payload.get("unexpected_publishers") == []
        )
    return result


def validate_qos_evidence(text: str) -> bool:
    """Validate live ``ros2 topic info -v`` endpoint evidence."""
    blocks: dict[str, str] = {}
    for section in text.split("TOPIC=")[1:]:
        topic, _, body = section.partition("\n")
        blocks[topic.strip()] = body
    sensor_endpoints = (
        ("Publisher count: 1", 1),
        ("Subscription count: 1", 1),
        ("Reliability: BEST_EFFORT", 2),
        ("Durability: VOLATILE", 2),
    )
    expectations = {
        "/hil/camera/color": sensor_endpoints,
        "/hil/camera/depth": sensor_endpoints,
        "/hil/camera/camera_info": sensor_endpoints,
        "/hil/tf": sensor_endpoints,
        "/hil/tf_static": (
            ("Publisher count: 1", 1),
            ("Subscription count: 1", 1),
            ("Reliability: RELIABLE", 2),
            ("Durability: TRANSIENT_LOCAL", 2),
        ),
        "/hil/vehicle/ackermann_command": (
            ("Publisher count: 2", 1),
            ("Subscription count: 1", 1),
            ("Reliability: RELIABLE", 3),
            ("Lifespan: 120000000 nanoseconds", 3),
            ("Deadline: 80000000 nanoseconds", 3),
        ),
        "/hil/vehicle/validated_ackermann_command": (
            ("Publisher count: 1", 1),
            ("Subscription count: 1", 1),
            ("Reliability: RELIABLE", 2),
            ("Lifespan: 120000000 nanoseconds", 2),
            ("Deadline: 80000000 nanoseconds", 2),
        ),
        "/hil/health": (
            ("Publisher count: 1", 1),
            ("Subscription count: 1", 1),
            ("Reliability: RELIABLE", 2),
            ("Durability: VOLATILE", 2),
        ),
    }
    return all(
        topic in blocks
        and all(blocks[topic].count(token) >= minimum for token, minimum in required)
        for topic, required in expectations.items()
    )


@dataclass(frozen=True)
class RuntimeIdentity:
    runtime_backend: str
    not_journey6_runtime: bool

    def validate(self) -> None:
        if self.runtime_backend not in RUNTIME_BACKENDS:
            raise ValueError(f"unsupported loopback runtime backend: {self.runtime_backend}")
        expected = self.runtime_backend == "PC_ONNX"
        if self.not_journey6_runtime is not expected:
            raise ValueError(
                "runtime identity mismatch: PC_ONNX must be marked not_journey6_runtime"
            )


@dataclass
class SensorContractAudit:
    sync_tolerance_s: float = 0.005
    last_clock_s: float = -math.inf
    last_color_stamp_s: float = -math.inf
    last_depth_stamp_s: float = -math.inf
    clock_rollback_count: int = 0
    sensor_timestamp_rollback_count: int = 0
    color_count: int = 0
    depth_count: int = 0
    camera_info_count: int = 0
    tf_count: int = 0
    tf_static_count: int = 0
    synchronized_pair_count: int = 0
    rejected_unsynchronized_pair_count: int = 0
    _colors: list[float] = field(default_factory=list, repr=False)
    _depths: list[float] = field(default_factory=list, repr=False)

    def observe_clock(self, stamp_s: float) -> None:
        stamp = self._finite_stamp(stamp_s)
        if stamp < self.last_clock_s:
            self.clock_rollback_count += 1
        self.last_clock_s = max(self.last_clock_s, stamp)

    def observe_camera_info(self, stamp_s: float) -> None:
        self._finite_stamp(stamp_s)
        self.camera_info_count += 1

    def observe_tf(self, stamp_s: float, *, static: bool) -> None:
        self._finite_stamp(stamp_s)
        if static:
            self.tf_static_count += 1
        else:
            self.tf_count += 1

    def observe_color(self, stamp_s: float) -> None:
        stamp = self._finite_stamp(stamp_s)
        if stamp <= self.last_color_stamp_s:
            self.sensor_timestamp_rollback_count += 1
        self.last_color_stamp_s = max(self.last_color_stamp_s, stamp)
        self.color_count += 1
        self._colors.append(stamp)
        self._match()

    def observe_depth(self, stamp_s: float) -> None:
        stamp = self._finite_stamp(stamp_s)
        if stamp <= self.last_depth_stamp_s:
            self.sensor_timestamp_rollback_count += 1
        self.last_depth_stamp_s = max(self.last_depth_stamp_s, stamp)
        self.depth_count += 1
        self._depths.append(stamp)
        self._match()

    @staticmethod
    def _finite_stamp(stamp_s: float) -> float:
        stamp = float(stamp_s)
        if not math.isfinite(stamp) or stamp < 0.0:
            raise ValueError("sensor timestamp must be finite and nonnegative")
        return stamp

    def _match(self) -> None:
        while self._colors and self._depths:
            delta = self._colors[0] - self._depths[0]
            if abs(delta) <= self.sync_tolerance_s:
                self._colors.pop(0)
                self._depths.pop(0)
                self.synchronized_pair_count += 1
            elif delta < 0.0:
                self._colors.pop(0)
                self.rejected_unsynchronized_pair_count += 1
            else:
                self._depths.pop(0)
                self.rejected_unsynchronized_pair_count += 1

    def snapshot(self) -> dict[str, object]:
        return {
            "clock_monotonic": self.clock_rollback_count == 0,
            "sensor_timestamps_monotonic": self.sensor_timestamp_rollback_count == 0,
            "clock_rollback_count": self.clock_rollback_count,
            "sensor_timestamp_rollback_count": self.sensor_timestamp_rollback_count,
            "color_count": self.color_count,
            "depth_count": self.depth_count,
            "camera_info_count": self.camera_info_count,
            "tf_count": self.tf_count,
            "tf_static_count": self.tf_static_count,
            "synchronized_pair_count": self.synchronized_pair_count,
            "rejected_unsynchronized_pair_count": self.rejected_unsynchronized_pair_count,
        }


def derive_algorithm_host_full_stack_pass(report: Mapping[str, object]) -> bool:
    """Single source of truth; blocked until a trusted run collector exists."""
    # Current process/topic snapshots are not cryptographically bound to a
    # monotonic run window or board attestation. Never promote handwritten
    # booleans or replayable JSON into a formal full-stack pass.
    return False


def evaluate_loopback_report(report: Mapping[str, object]) -> dict[str, bool]:
    """Derive V2 readiness states without weakening the legacy J6 state."""
    runtime = RuntimeIdentity(
        runtime_backend=str(report.get("runtime_backend", "")),
        not_journey6_runtime=report.get("not_journey6_runtime") is True,
    )
    runtime.validate()
    # Formal evaluation is deliberately unavailable until a trusted collector
    # binds run_id, time window, node/process/endpoint raw records, model
    # qualification, and official board/runtime attestation. Keeping all states
    # false is safer than retaining a JSON path that can be hand-authored true.
    return {
        "J6_LOOPBACK_TRANSPORT_READY": False,
        "J6_LOOPBACK_ALGORITHM_READY": False,
        "J6_LOOPBACK_HIL_EMULATION_READY": False,
        "J6_LOOPBACK_HIL_READY": False,
    }

    # Candidate-gate logic below is retained as the specification for the
    # future trusted collector, but is unreachable until the blocker is removed.
    transport = report.get("transport", {})
    algorithm = report.get("algorithm", {})
    safety = report.get("safety", {})
    if not isinstance(transport, Mapping) or not isinstance(algorithm, Mapping):
        raise ValueError("loopback transport and algorithm evidence must be mappings")
    if not isinstance(safety, Mapping):
        raise ValueError("loopback safety evidence must be a mapping")
    platform = algorithm.get("platform", {})
    if not isinstance(platform, Mapping):
        raise ValueError("loopback algorithm platform evidence must be a mapping")
    qualification = algorithm.get("model_qualification", {})
    if not isinstance(qualification, Mapping):
        raise ValueError("model qualification evidence must be a mapping")
    sensor_provenance = report.get("sensor_provenance", {})
    if not isinstance(sensor_provenance, Mapping):
        raise ValueError("sensor provenance evidence must be a mapping")

    full_safety_matrix = all(
        (
            safety.get("steady_state_pc_duplicate_algorithm_nodes") == 0,
            safety.get("pc_blacklist_injection_detected") is True,
            safety.get("pc_blacklist_safe_stop") is True,
            safety.get("ground_truth_control_violation_count") == 0,
            safety.get("nonzero_authority_pass") is True,
            safety.get("command_timeout_safe_stop") is True,
            safety.get("actual_network_loss_safe_stop") is True,
            safety.get("network_reconnect_requires_manual_resume") is True,
            safety.get("no_stale_command_replay") is True,
            safety.get("estop_safe_stop") is True,
        )
    )
    common_transport_ready = all(
        (
            report.get("actual_ros2_processes") is True,
            float(report.get("duration_s", 0.0)) >= 1800.0,
            report.get("sensor_source") == "gazebo",
            sensor_provenance.get("audited_launch") is True,
            sensor_provenance.get("gazebo_process_verified") is True,
            sensor_provenance.get("publisher_endpoints_verified") is True,
            sensor_provenance.get("pc_sensor_and_plant_only") is True,
            SHA256_PATTERN.fullmatch(
                str(sensor_provenance.get("evidence_sha256", "")).lower()
            )
            is not None,
            transport.get("qos_contract_pass") is True,
            transport.get("sensor_timestamps_monotonic") is True,
            transport.get("clock_monotonic") is True,
            transport.get("tf_received") is True,
            transport.get("tf_static_received") is True,
            transport.get("image_depth_sync_pass") is True,
            int(transport.get("synchronized_pair_count", 0)) >= 10,
            full_safety_matrix,
        )
    )
    pc_onnx_platform_ready = all(
        (
            platform.get("os_id") == "ubuntu",
            platform.get("os_version_id") == "22.04",
            platform.get("ros_distro") == "humble",
        )
    )
    transport_ready = all(
        (
            common_transport_ready,
            runtime.runtime_backend == "PC_ONNX",
            runtime.not_journey6_runtime,
            pc_onnx_platform_ready,
        )
    )
    algorithm_ready = all(
        (
            transport_ready,
            runtime.runtime_backend == "PC_ONNX",
            runtime.not_journey6_runtime,
            algorithm.get("model_loaded") is True,
            algorithm.get("provider") in {"CPUExecutionProvider", "CUDAExecutionProvider"},
            algorithm.get("fallback_used") is False,
            int(algorithm.get("inference_count", 0)) > 0,
            algorithm.get("required_model_id_match") is True,
            SHA256_PATTERN.fullmatch(
                str(qualification.get("manifest_sha256", "")).lower()
            )
            is not None,
            qualification.get("model_id") == algorithm.get("model_id"),
            qualification.get("model_sha256") == algorithm.get("model_sha256"),
            qualification.get("pt_onnx_parity_pass") is True,
            qualification.get("pc_inference_pass") is True,
            SHA256_PATTERN.fullmatch(
                str(qualification.get("full_stack_evidence_sha256", "")).lower()
            )
            is not None,
            qualification.get("full_stack_pass") is True,
        )
    )
    fault_matrix_ready = common_transport_ready and full_safety_matrix
    emulation_ready = all(
        (
            algorithm_ready,
            fault_matrix_ready,
        )
    )
    official_j6_runtime = all(
        (
            runtime.runtime_backend == "JOURNEY6_OE",
            not runtime.not_journey6_runtime,
            report.get("official_journey6_runtime_evidence") is True,
            common_transport_ready,
            algorithm.get("model_loaded") is True,
            int(algorithm.get("inference_count", 0)) > 0,
            fault_matrix_ready,
        )
    )
    return {
        "J6_LOOPBACK_TRANSPORT_READY": transport_ready,
        "J6_LOOPBACK_ALGORITHM_READY": algorithm_ready,
        "J6_LOOPBACK_HIL_EMULATION_READY": emulation_ready,
        "J6_LOOPBACK_HIL_READY": bool(official_j6_runtime),
    }


__all__ = [
    "FORMAL_STATUS_EVALUATOR_BLOCKER",
    "RUNTIME_BACKENDS",
    "SHA256_PATTERN",
    "RuntimeIdentity",
    "SensorContractAudit",
    "audit_gazebo_sensor_provenance",
    "audit_model_qualification_manifest",
    "derive_algorithm_host_full_stack_pass",
    "evaluate_loopback_report",
    "synthetic_sensor_publishers_allowed",
    "validate_run_id",
    "validate_qos_evidence",
]
