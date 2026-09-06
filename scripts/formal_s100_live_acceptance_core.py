#!/usr/bin/env python3
"""Pure-Python rules for the fail-closed RDK S100P live-runtime gate."""

from __future__ import annotations

import hashlib
import json
import platform
import re
from pathlib import Path
from typing import Any


RAW_REPORT_ID = "tzcup_formal_rdk_s100_live_runtime_raw_v1"
FINAL_REPORT_ID = "tzcup_formal_rdk_s100_live_product_runtime_v1"
RAW_COLLECTED = "FORMAL_RDK_S100_LIVE_RUNTIME_COLLECTED"
RAW_BLOCKED = "FORMAL_RDK_S100_LIVE_RUNTIME_BLOCKED"
FINAL_PASSED = "FORMAL_RDK_S100_LIVE_PRODUCT_RUNTIME_PASSED"
FINAL_BLOCKED = "FORMAL_RDK_S100_LIVE_PRODUCT_RUNTIME_BLOCKED"
TARGET_BOARD = "RDK S100P"
TARGET_SOC = "Journey 6P"

REQUIRED_MODEL_ROLES = {
    "dosod_hbm",
    "dosod_vocabulary",
    "edgesam_encoder_hbm",
    "edgesam_decoder_hbm",
}
REQUIRED_NODES = {"hobot_dosod", "mono_edgesam", "open_vocab_product_adapter"}
REQUIRED_TOPICS = {
    "/perception/garbage/detections_2d": "vision_msgs/msg/Detection2DArray",
    "/perception/ground_dirt/masks": "sensor_msgs/msg/Image",
    "/perception/garbage/targets": "sanitation_perception_interfaces/msg/GarbageTargetArray",
    "/perception/wrist/grasp_recheck": "std_msgs/msg/String",
    "/perception/open_vocab/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
    "/perception/open_vocab/dosod_boxes": "vision_msgs/msg/Detection2DArray",
}
PROHIBITED_TRUTH_TOKENS = ("ground_truth", "evaluator", "/world/", "model_states")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return payload


def snapshot_identity(path: Path) -> dict[str, str]:
    payload = json_object(path)
    source_hash = payload.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or not SHA256_RE.fullmatch(source_hash.lower()):
        raise ValueError("snapshot manifest has no valid source_inventory_sha256")
    return {
        "snapshot_manifest_sha256": sha256_path(path),
        "source_inventory_sha256": source_hash.lower(),
    }


def _require_session_runtime_closure_binding(
    payload: dict[str, Any], expected_runtime_closure: dict[str, str] | None
) -> None:
    """Require the board hand-off to retain the session's original closure."""

    if not isinstance(expected_runtime_closure, dict):
        raise ValueError("active runtime closure binding is required")
    session_closure = payload.get("runtime_closure_binding")
    if not isinstance(session_closure, dict):
        raise ValueError("acceptance session has no runtime closure binding")
    if (
        session_closure.get("manifest_sha256")
        != expected_runtime_closure.get("runtime_closure_manifest_sha256")
        or session_closure.get("closure_sha256")
        != expected_runtime_closure.get("runtime_closure_sha256")
    ):
        raise ValueError("acceptance session runtime closure does not match the active closure")


def acceptance_session_binding(
    path: Path,
    expected_snapshot: dict[str, str],
    expected_runtime_closure: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Return the immutable facts that bind a board collection to one session."""
    payload = json_object(path)
    started = payload.get("started_epoch_ns")
    if (
        payload.get("report_id") != "tzcup_formal_final_acceptance_session_v1"
        or payload.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or not isinstance(started, int)
        or started <= 0
        or payload.get("snapshot") != expected_snapshot
    ):
        raise ValueError("acceptance session is not the active frozen-snapshot session")
    _require_session_runtime_closure_binding(payload, expected_runtime_closure)
    return {
        "session_manifest_sha256": sha256_path(path),
        "session_started_epoch_ns": started,
        "session_status_at_collection": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "snapshot": dict(expected_snapshot),
    }


def active_session_identity(
    path: Path,
    expected_snapshot: dict[str, str],
    expected_runtime_closure: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Read the stable identity of the current session without trusting its phase hash.

    A retained session intentionally transitions RUNNING -> PENDING during the
    external-gate hand-off, which changes the session file hash.  The immutable
    identity is therefore its start timestamp plus frozen snapshot, while the
    raw collector still records the RUNNING-file digest it observed.
    """
    payload = json_object(path)
    started = payload.get("started_epoch_ns")
    if (
        payload.get("report_id") != "tzcup_formal_final_acceptance_session_v1"
        or payload.get("status")
        not in {"FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"}
        or not isinstance(started, int)
        or started <= 0
        or payload.get("snapshot") != expected_snapshot
    ):
        raise ValueError("acceptance session is not the current frozen-snapshot session")
    _require_session_runtime_closure_binding(payload, expected_runtime_closure)
    return {
        "session_started_epoch_ns": started,
        "snapshot": dict(expected_snapshot),
    }


def runtime_closure_binding(path: Path) -> dict[str, str]:
    """Return the two immutable digests required from the frozen runtime closure."""
    payload = json_object(path)
    closure_digest = payload.get("closure_sha256")
    if (
        payload.get("kind") != "tzcup_formal_final_runtime_closure"
        or payload.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
        or not isinstance(closure_digest, str)
        or not SHA256_RE.fullmatch(closure_digest.lower())
    ):
        raise ValueError("runtime closure is not a valid frozen closure manifest")
    return {
        "runtime_closure_manifest_sha256": sha256_path(path),
        "runtime_closure_sha256": closure_digest.lower(),
    }


def probe_hardware(root: Path = Path("/"), *, machine: str | None = None) -> dict[str, Any]:
    """Read immutable Linux hardware identity. ``root`` exists only for tests."""
    actual_machine = (machine or platform.machine()).lower()
    model_path = root / "proc/device-tree/model"
    compatible_path = root / "proc/device-tree/compatible"

    def read_dt(path: Path) -> tuple[str, str | None]:
        if not path.is_file():
            return "", None
        raw = path.read_bytes()
        return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip(), hashlib.sha256(raw).hexdigest()

    model, model_hash = read_dt(model_path)
    compatible, compatible_hash = read_dt(compatible_path)
    normalized_model = model.lower()
    normalized_compatible = compatible.lower()
    identity = f"{normalized_model} {normalized_compatible}"
    has_s100p = any(token in normalized_model for token in ("s100p", "rdk s100p", "rdk-s100p"))
    has_explicit_journey6p = any(token in identity for token in ("journey 6p", "journey6p", "journey-6p"))
    # The production RDK S100P V1P0 device tree observed on 2026-08-30 uses
    # ``D-Robotics RDK S100P V1P0`` for model and the platform-family token
    # ``drobot,s100-rdk`` for compatible; it does not repeat ``journey6p``.
    # Accept that exact fail-closed composition, while continuing to reject a
    # generic S100 model or an arbitrary ARM64 host with the family token.
    has_live_s100p_family_compatible = "drobot,s100-rdk" in normalized_compatible.split()
    has_journey6p_identity = has_explicit_journey6p or (has_s100p and has_live_s100p_family_compatible)
    attested = (
        actual_machine in {"aarch64", "arm64"}
        and bool(model_hash)
        and bool(compatible_hash)
        and has_s100p
        and has_journey6p_identity
    )
    blockers = []
    if actual_machine not in {"aarch64", "arm64"}:
        blockers.append(f"architecture is {actual_machine}, expected aarch64")
    if not model_hash or not compatible_hash:
        blockers.append("device-tree model/compatible identity files are missing")
    if not has_s100p:
        blockers.append("device-tree does not identify an RDK S100P")
    if not has_journey6p_identity:
        blockers.append("device-tree does not identify the S100P/Journey 6P platform")
    return {
        "attested": attested,
        "architecture": actual_machine,
        "board": TARGET_BOARD if attested else None,
        "soc": TARGET_SOC if attested else None,
        "sku": model if attested else None,
        "device_tree_model": model,
        "device_tree_model_sha256": model_hash,
        "device_tree_compatible": compatible,
        "device_tree_compatible_sha256": compatible_hash,
        "soc_identity_basis": (
            "explicit_journey6p_device_tree"
            if has_explicit_journey6p
            else "s100p_model_plus_drobot_s100_rdk_compatible"
            if has_journey6p_identity
            else None
        ),
        "blockers": blockers,
    }


def _number(value: Any) -> float | None:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return None


def validate_raw(
    payload: dict[str, Any],
    expected_snapshot: dict[str, str],
    expected_session: dict[str, Any] | None = None,
    expected_runtime_closure: dict[str, str] | None = None,
    dosod_bundle_root: Path | None = None,
) -> list[str]:
    failures: list[str] = []

    def fail(message: str) -> None:
        if message not in failures:
            failures.append(message)

    if payload.get("schema_version") != 1 or payload.get("report_id") != RAW_REPORT_ID:
        fail("raw artifact identity/schema is invalid")
    if payload.get("status") != RAW_COLLECTED or payload.get("collection_complete") is not True:
        fail("collector did not complete on live hardware")

    collector = payload.get("collector")
    expected_collector_hash = sha256_path(Path(__file__).with_name("collect_formal_s100_live_runtime.py"))
    if not isinstance(collector, dict) or collector.get("script_sha256") != expected_collector_hash:
        fail("raw artifact was not produced by this exact collector revision")

    hardware = payload.get("hardware")
    if not isinstance(hardware, dict) or hardware.get("attested") is not True:
        fail("RDK S100P hardware identity is not attested")
    else:
        if hardware.get("architecture") not in {"aarch64", "arm64"}:
            fail("hardware architecture is not aarch64")
        if hardware.get("board") != TARGET_BOARD or hardware.get("soc") != TARGET_SOC:
            fail(f"board/SOC identity is not {TARGET_BOARD}/{TARGET_SOC}")
        for field in ("device_tree_model_sha256", "device_tree_compatible_sha256"):
            value = hardware.get(field)
            if not isinstance(value, str) or not SHA256_RE.fullmatch(value.lower()):
                fail(f"hardware.{field} is missing or invalid")

    source = payload.get("source_binding")
    if not isinstance(source, dict):
        fail("source binding is missing")
    else:
        for field, expected in expected_snapshot.items():
            if source.get(field) != expected:
                fail(f"source binding {field} does not match frozen snapshot")

    session = payload.get("acceptance_session_binding")
    if not isinstance(session, dict):
        fail("acceptance-session binding is missing")
    else:
        if session.get("snapshot") != expected_snapshot:
            fail("acceptance-session binding snapshot does not match frozen snapshot")
        if session.get("session_status_at_collection") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
            fail("acceptance-session binding is not from a running session")
        if not isinstance(session.get("session_started_epoch_ns"), int) or session["session_started_epoch_ns"] <= 0:
            fail("acceptance-session binding start time is invalid")
        digest = session.get("session_manifest_sha256")
        if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest.lower()):
            fail("acceptance-session binding manifest digest is invalid")
        elif expected_session is not None and (
            session.get("session_started_epoch_ns")
            != expected_session.get("session_started_epoch_ns")
            or session.get("snapshot") != expected_session.get("snapshot")
        ):
            fail("acceptance-session binding does not match the active session")

    closure = payload.get("runtime_closure_binding")
    if not isinstance(closure, dict):
        fail("runtime-closure binding is missing")
    else:
        for field in ("runtime_closure_manifest_sha256", "runtime_closure_sha256"):
            value = closure.get(field)
            if not isinstance(value, str) or not SHA256_RE.fullmatch(value.lower()):
                fail(f"runtime-closure binding {field} is invalid")
        if expected_runtime_closure is not None and closure != expected_runtime_closure:
            fail("runtime-closure binding does not match the active frozen closure")

    system = payload.get("system_image")
    if not isinstance(system, dict):
        fail("system image evidence is missing")
    else:
        for field in ("kernel_release", "os_release_sha256", "runtime_inventory"):
            if not system.get(field):
                fail(f"system image {field} is missing")
        release = system.get("os_release")
        if not isinstance(release, dict) or not release.get("ID") or not any(release.get(key) for key in ("IMAGE_ID", "BUILD_ID", "VERSION_ID")):
            fail("system image ID/build/version is missing")
        runtime_inventory = system.get("runtime_inventory")
        if isinstance(runtime_inventory, dict):
            ros = runtime_inventory.get("ros2")
            bpu_rows = [runtime_inventory.get("hbrt4"), runtime_inventory.get("hrt_model_exec")]
            if not isinstance(ros, dict) or ros.get("returncode") != 0:
                fail("ROS 2 runtime inventory command failed")
            if not any(isinstance(row, dict) and row.get("returncode") == 0 for row in bpu_rows):
                fail("no Journey 6 BPU runtime executable was identified")

    models = payload.get("models")
    model_hashes: dict[str, str] = {}
    if not isinstance(models, list):
        fail("model inventory is missing")
    else:
        roles = {row.get("role") for row in models if isinstance(row, dict)}
        if roles != REQUIRED_MODEL_ROLES:
            fail("model inventory does not contain exactly the four required roles")
        for row in models:
            if not isinstance(row, dict):
                fail("model inventory contains a non-object row")
                continue
            digest = row.get("sha256")
            if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest.lower()):
                fail(f"model {row.get('role')} has no valid SHA-256")
            else:
                model_hashes[str(row.get("role"))] = digest.lower()
            if not isinstance(row.get("byte_size"), int) or row["byte_size"] <= 0:
                fail(f"model {row.get('role')} has invalid byte size")

    # The board file inventory alone cannot prove that DOSOD HBM came through
    # the contract-bound compile/parity/metric chain.  The live collector
    # records the three producer receipts before the timed run; finalization
    # consumes their mutually bound digests rather than accepting an HBM hash
    # in isolation.
    dosod_evidence = payload.get("dosod_hbm_evidence")
    if not isinstance(dosod_evidence, dict) or set(dosod_evidence) != {"compile", "parity", "metric", "full_admission"}:
        fail("DOSOD compile/parity/metric receipt chain is missing")
    else:
        expected_ids = {
            "compile": ("tzcup_s100p_dosod_hbm_compile_receipt_v1", "COMPILED_NOT_BOARD_ACCEPTED"),
            "parity": ("tzcup_dosod_hbm_x86_nash_parity_v1", "PARITY_PASSED"),
            "metric": ("tzcup_dosod_quantized_metric_regression_v1", "REGRESSION_PASSED"),
        }
        for name, (report_id, status) in expected_ids.items():
            row = dosod_evidence.get(name)
            if not isinstance(row, dict) or row.get("report_id") != report_id or row.get("status") != status:
                fail(f"DOSOD {name} receipt identity/status is invalid")
                continue
            if not isinstance(row.get("path"), str) or not isinstance(row.get("sha256"), str) or not SHA256_RE.fullmatch(row["sha256"].lower()):
                fail(f"DOSOD {name} receipt path/digest is invalid")
            if row.get("hbm_sha256") != model_hashes.get("dosod_hbm"):
                fail(f"DOSOD {name} receipt does not bind the collected HBM")
        full = dosod_evidence.get("full_admission")
        receipt_fields = {"compile_receipt": "compile", "parity_report": "parity", "metric_report": "metric"}
        if not isinstance(full, dict) or full.get("hbm_sha256") != model_hashes.get("dosod_hbm") or any(
            full.get(f"{field}_sha256") != dosod_evidence.get(name, {}).get("sha256")
            for field, name in receipt_fields.items()
        ):
            fail("DOSOD full admission summary is not bound to receipt triad")
        if dosod_bundle_root is not None:
            try:
                from verify_dosod_compile_parity_metric_chain import verify_dosod_compile_parity_metric_chain
                verified = verify_dosod_compile_parity_metric_chain(dosod_bundle_root, Path(str(next((row.get("path") for row in models if isinstance(row, dict) and row.get("role") == "dosod_hbm"), ""))))
                if not isinstance(full, dict) or any(full.get(key) != verified.get(key) for key in verified):
                    fail("DOSOD full admission bundle recheck differs from raw summary")
            except Exception:
                fail("DOSOD full admission bundle recheck failed")

    graph = payload.get("ros_graph")
    if not isinstance(graph, dict):
        fail("ROS graph evidence is missing")
    else:
        names = {str(name).strip("/").split("/")[-1] for name in graph.get("nodes", [])}
        missing_nodes = sorted(REQUIRED_NODES - names)
        if missing_nodes:
            fail(f"required ROS nodes missing: {', '.join(missing_nodes)}")
        topics = graph.get("topics")
        if not isinstance(topics, dict):
            fail("ROS topic/type graph is missing")
        else:
            for topic, expected_type in REQUIRED_TOPICS.items():
                types = topics.get(topic, [])
                if expected_type not in types:
                    fail(f"required ROS topic/type missing: {topic} [{expected_type}]")
        graph_text = json.dumps(graph, sort_keys=True).lower()
        if any(token in graph_text for token in PROHIBITED_TRUTH_TOKENS):
            fail("ROS graph contains simulator/evaluator truth surfaces")

    telemetry = payload.get("inference_telemetry")
    if not isinstance(telemetry, dict):
        fail("inference telemetry is missing")
    else:
        for component in ("dosod", "edgesam"):
            row = telemetry.get(component)
            if not isinstance(row, dict):
                fail(f"{component} inference telemetry is missing")
                continue
            if row.get("backend") not in {"bpu", "cpu"}:
                fail(f"{component} backend is not explicitly bpu or cpu")
            elif row.get("backend") != "bpu":
                fail(f"{component} used CPU; the formal S100 product profile requires BPU")
            digest = row.get("model_sha256")
            if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest.lower()):
                fail(f"{component} runtime model hash is missing")
            elif component == "dosod" and digest.lower() != model_hashes.get("dosod_hbm"):
                fail("dosod runtime model hash does not match the collected HBM")
            if component == "dosod":
                vocabulary_digest = row.get("vocabulary_sha256")
                if (
                    not isinstance(vocabulary_digest, str)
                    or not SHA256_RE.fullmatch(vocabulary_digest.lower())
                ):
                    fail("dosod runtime vocabulary hash is missing")
                elif vocabulary_digest.lower() != model_hashes.get("dosod_vocabulary"):
                    fail("dosod runtime vocabulary hash does not match the collected vocabulary")
            if component == "edgesam":
                runtime_hashes = row.get("model_sha256s")
                expected_hashes = {
                    model_hashes.get("edgesam_encoder_hbm"),
                    model_hashes.get("edgesam_decoder_hbm"),
                }
                if not isinstance(runtime_hashes, list) or set(runtime_hashes) != expected_hashes:
                    fail("edgesam runtime model hashes do not match encoder and decoder HBM files")
            samples = row.get("samples")
            fps = _number(row.get("fps"))
            latency = _number(row.get("latency_ms_p95"))
            if not isinstance(samples, int) or samples <= 0 or fps is None or fps <= 0:
                fail(f"{component} has no measured live frames")
            if latency is None or latency <= 0:
                fail(f"{component} has no measured positive p95 latency")
            elif latency > 1000.0:
                fail(f"{component} p95 latency exceeded 1000 ms")
            minimum_fps = 2.0 if component == "dosod" else 1.0
            if fps is not None and fps < minimum_fps:
                fail(f"{component} measured FPS is below {minimum_fps:g} Hz")
            if row.get("runtime_process_maps_backend_match") is not True:
                fail(f"{component} backend is not corroborated by live process maps")

        output_counts = telemetry.get("product_output_counts")
        nonempty_counts = telemetry.get("product_nonempty_counts")
        for topic in (
            "/perception/garbage/detections_2d",
            "/perception/ground_dirt/masks",
            "/perception/garbage/targets",
        ):
            if not isinstance(output_counts, dict) or not isinstance(output_counts.get(topic), int) or output_counts[topic] <= 0:
                fail(f"product adapter emitted no live messages on {topic}")
            if (
                not isinstance(nonempty_counts, dict)
                or not isinstance(nonempty_counts.get(topic), int)
                or nonempty_counts[topic] <= 0
            ):
                fail(f"product adapter emitted no nonempty live products on {topic}")

    duration = _number(payload.get("sustained_run", {}).get("duration_sec") if isinstance(payload.get("sustained_run"), dict) else None)
    if duration is None or duration < 1800.0:
        fail("sustained run is shorter than 1800 seconds")
    sustained = payload.get("sustained_run")
    if isinstance(sustained, dict):
        if sustained.get("node_disappearances") != 0:
            fail("required ROS nodes disappeared during the run")
        if sustained.get("process_restarts") != 0:
            fail("perception processes restarted during the run")
        if sustained.get("inference_failures") != 0:
            fail("inference failures occurred during the run")

    resources = payload.get("resources")
    if not isinstance(resources, dict):
        fail("memory evidence is missing")
    else:
        for field in ("system_memory_total_bytes", "system_memory_available_min_bytes", "perception_rss_peak_bytes"):
            if not isinstance(resources.get(field), int) or resources[field] <= 0:
                fail(f"resource metric {field} is missing")
        total = resources.get("system_memory_total_bytes")
        available = resources.get("system_memory_available_min_bytes")
        if isinstance(total, int) and isinstance(available, int) and available / total < 0.05:
            fail("available system memory fell below 5 percent")

    thermal = payload.get("thermal")
    if not isinstance(thermal, dict) or not isinstance(thermal.get("samples"), int) or thermal["samples"] <= 0:
        fail("thermal sampling evidence is missing")
    else:
        peak = _number(thermal.get("peak_celsius"))
        if peak is None:
            fail("thermal peak is missing")
        elif peak > 85.0:
            fail("thermal peak exceeded 85 C")

    if payload.get("truth_boundary", {}).get("simulator_or_evaluator_truth_used") is not False:
        fail("truth-boundary declaration is absent or unsafe")
    return failures


def build_final_report(
    raw: dict[str, Any],
    expected_snapshot: dict[str, str],
    raw_path: Path,
    expected_session: dict[str, Any] | None = None,
    expected_runtime_closure: dict[str, str] | None = None,
    dosod_bundle_root: Path | None = None,
) -> dict[str, Any]:
    failures = validate_raw(raw, expected_snapshot, expected_session, expected_runtime_closure, dosod_bundle_root)
    return {
        "schema_version": 1,
        "report_id": FINAL_REPORT_ID,
        "status": FINAL_BLOCKED if failures else FINAL_PASSED,
        "passed": not failures,
        "source_binding": expected_snapshot,
        "acceptance_session_binding": raw.get("acceptance_session_binding"),
        "runtime_closure_binding": raw.get("runtime_closure_binding"),
        "raw_evidence": {
            "path": str(raw_path),
            "sha256": sha256_path(raw_path),
            "collector_report_id": raw.get("report_id"),
            "collector_status": raw.get("status"),
        },
        "checks": {
            "hardware_identity": not any("hardware" in row.lower() or "board/soc" in row.lower() for row in failures),
            "system_image": not any("system image" in row.lower() for row in failures),
            "model_hashes": not any("model" in row.lower() for row in failures),
            "dosod_compile_parity_metric_chain": not any("dosod" in row.lower() for row in failures),
            "ros_graph": not any("ros" in row.lower() for row in failures),
            "actual_backend_and_inference": not any(
                token in row.lower() for row in failures for token in ("backend", "inference", "live frames", "latency")
            ),
            "sustained_resources_and_thermal": not any(
                token in row.lower() for row in failures for token in ("1800", "memory", "thermal", "restarted", "disappeared")
            ),
            "truth_boundary": not any("truth" in row.lower() for row in failures),
            "snapshot_binding": not any("source binding" in row.lower() for row in failures),
            "acceptance_session_binding": not any(
                "acceptance-session" in row.lower() for row in failures
            ),
            "runtime_closure_binding": not any(
                "runtime-closure" in row.lower() for row in failures
            ),
        },
        "blockers": failures,
        "claim_boundary": (
            "PASS proves one collected live RDK S100 run satisfied this gate. "
            "The JSON/hash chain is tamper-evident, not TPM-backed remote attestation."
        ),
    }
