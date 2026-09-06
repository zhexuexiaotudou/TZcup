#!/usr/bin/env python3
"""Collect live RDK S100P evidence; refuse to run as a collector elsewhere.

The collector launches no product nodes.  DOSOD, EdgeSAM and the product
adapter must already be running and must publish per-inference DiagnosticArray
records on /perception/open_vocab/diagnostics.  Required diagnostic values are
documented in docs/formal-s100-live-acceptance.md.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any

from formal_s100_live_acceptance_core import (
    RAW_BLOCKED,
    RAW_COLLECTED,
    RAW_REPORT_ID,
    acceptance_session_binding,
    probe_hardware,
    runtime_closure_binding,
    sha256_path,
    snapshot_identity,
)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _sha256(path: Path) -> str:
    return sha256_path(path)


def collect_dosod_receipt_chain(
    compile_receipt: Path, parity_report: Path, metric_report: Path, dosod_hbm: Path
) -> dict[str, dict[str, Any]]:
    """Read already-produced offline evidence; this collector never produces it."""
    paths = {"compile": compile_receipt, "parity": parity_report, "metric": metric_report}
    documents: dict[str, dict[str, Any]] = {}
    for name, path in paths.items():
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"DOSOD {name} receipt is missing or linked")
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError(f"DOSOD {name} receipt is not an object")
        documents[name] = value
    hbm_sha = _sha256(dosod_hbm)
    compile_row, parity_row, metric_row = documents["compile"], documents["parity"], documents["metric"]
    if compile_row.get("receipt_id") != "tzcup_s100p_dosod_hbm_compile_receipt_v1" or compile_row.get("status") != "COMPILED_NOT_BOARD_ACCEPTED" or compile_row.get("output_sha256") != hbm_sha:
        raise ValueError("DOSOD compile receipt does not bind this HBM")
    if parity_row.get("report_id") != "tzcup_dosod_hbm_x86_nash_parity_v1" or parity_row.get("status") != "PARITY_PASSED" or parity_row.get("hbm", {}).get("sha256") != hbm_sha or parity_row.get("compile_receipt_sha256") != _sha256(compile_receipt):
        raise ValueError("DOSOD parity receipt does not bind this compile/HBM")
    if metric_row.get("report_id") != "tzcup_dosod_quantized_metric_regression_v1" or metric_row.get("status") != "REGRESSION_PASSED" or metric_row.get("hbm", {}).get("sha256") != hbm_sha or metric_row.get("compile_receipt_sha256") != _sha256(compile_receipt) or metric_row.get("parity_report_sha256") != _sha256(parity_report):
        raise ValueError("DOSOD metric receipt does not bind this compile/parity/HBM")
    return {
        name: {"path": str(path.resolve()), "sha256": _sha256(path), "report_id": documents[name].get("report_id", documents[name].get("receipt_id")), "status": documents[name].get("status"), "hbm_sha256": hbm_sha}
        for name, path in paths.items()
    }


def run_text(command: list[str], timeout: float = 30.0) -> dict[str, Any]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)
        return {"command": command, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "returncode": None, "stdout": "", "stderr": str(exc)}


def os_release() -> dict[str, Any]:
    path = Path("/etc/os-release")
    raw = path.read_bytes() if path.is_file() else b""
    fields: dict[str, str] = {}
    for line in raw.decode("utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            fields[key] = value.strip().strip('"')
    uname = run_text(["uname", "-a"])
    runtime = {
        name: run_text(command)
        for name, command in {
            "hbrt4": ["hbrt4-run-model", "--version"],
            "hrt_model_exec": ["hrt_model_exec", "--version"],
            "ros2": ["ros2", "--help"],
            "dpkg_horizon": ["dpkg-query", "-W", "hobot*", "horizon*"],
        }.items()
    }
    return {
        "os_release": fields,
        "os_release_sha256": __import__("hashlib").sha256(raw).hexdigest() if raw else None,
        "kernel_release": os.uname().release,
        "kernel_version": os.uname().version,
        "uname": uname,
        "runtime_inventory": runtime,
    }


def parse_topic_types(text: str) -> dict[str, list[str]]:
    topics: dict[str, list[str]] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or " [" not in line or not line.endswith("]"):
            continue
        name, types = line.split(" [", 1)
        topics[name] = [value.strip() for value in types[:-1].split(",") if value.strip()]
    return topics


def ros_graph() -> dict[str, Any]:
    node_result = run_text(["ros2", "node", "list"])
    nodes = sorted({line.strip() for line in node_result["stdout"].splitlines() if line.strip()})
    topic_result = run_text(["ros2", "topic", "list", "-t"])
    details = {node: run_text(["ros2", "node", "info", node]) for node in nodes}
    return {
        "nodes": nodes,
        "topics": parse_topic_types(topic_result["stdout"]),
        "node_list_command": node_result,
        "topic_list_command": topic_result,
        "node_info": details,
    }


def ros_node_names() -> list[str]:
    result = run_text(["ros2", "node", "list"], timeout=10.0)
    return sorted({line.strip() for line in result["stdout"].splitlines() if line.strip()})


def process_inventory() -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            cmdline = (proc / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", errors="replace")
        except OSError:
            continue
        lowered = cmdline.lower()
        component = next((name for name in ("hobot_dosod", "mono_edgesam", "open_vocab_product_adapter") if name in lowered), None)
        if component is None:
            continue
        try:
            maps = (proc / "maps").read_text(encoding="utf-8", errors="replace")
        except OSError:
            maps = ""
        backend_libraries = sorted(
            {token for token in maps.split() if any(mark in token.lower() for mark in ("hbrt", "hb_dnn", "libdnn", "onnxruntime"))}
        )
        rss = 0
        try:
            for line in (proc / "status").read_text(encoding="utf-8", errors="replace").splitlines():
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1]) * 1024
        except (OSError, ValueError, IndexError):
            pass
        rows.append(
            {
                "pid": int(proc.name),
                "component": component,
                "cmdline": cmdline,
                "cmdline_sha256": __import__("hashlib").sha256(cmdline.encode()).hexdigest(),
                "backend_libraries": backend_libraries,
                "rss_bytes": rss,
            }
        )
    return {"processes": rows}


def memory_sample() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        if ":" in line:
            name, remainder = line.split(":", 1)
            parts = remainder.split()
            if parts and parts[0].isdigit():
                values[name] = int(parts[0]) * 1024
    return values


def thermal_sample() -> list[dict[str, Any]]:
    rows = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            raw = float((zone / "temp").read_text().strip())
            celsius = raw / 1000.0 if raw > 500 else raw
            zone_type = (zone / "type").read_text().strip()
        except (OSError, ValueError):
            continue
        rows.append({"zone": zone.name, "type": zone_type, "celsius": celsius})
    return rows


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def collect_live(duration_sec: float, sample_period_sec: float) -> dict[str, Any]:
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(f"required ROS 2 Python packages are unavailable: {exc}") from exc

    rclpy.init(args=None)
    node = rclpy.create_node("formal_s100_live_runtime_collector")
    latencies: dict[str, list[float]] = {"dosod": [], "edgesam": []}
    backends: dict[str, set[str]] = {"dosod": set(), "edgesam": set()}
    hashes: dict[str, set[str]] = {"dosod": set(), "edgesam": set()}
    vocabulary_hashes: set[str] = set()
    inference_failures = 0
    counts: dict[str, int] = {}
    nonempty_counts: dict[str, int] = {}

    def diagnostics_callback(message: Any) -> None:
        nonlocal inference_failures
        for status in message.status:
            name = status.name.lower()
            component = "dosod" if "dosod" in name else "edgesam" if "edgesam" in name else None
            if component is None:
                continue
            values = {entry.key: entry.value for entry in status.values}
            required = {"backend", "model_sha256", "latency_ms", "inference_ok"}
            if component == "dosod":
                required.add("vocabulary_sha256")
            if not required.issubset(values):
                inference_failures += 1
                continue
            try:
                latencies[component].append(float(values["latency_ms"]))
            except ValueError:
                inference_failures += 1
                continue
            backends[component].add(values["backend"].strip().lower())
            hashes[component].update(value.strip().lower() for value in values["model_sha256"].split(",") if value.strip())
            if component == "dosod":
                vocabulary_hashes.add(values["vocabulary_sha256"].strip().lower())
            raw_level = status.level
            level = raw_level[0] if isinstance(raw_level, (bytes, bytearray)) and raw_level else int(raw_level)
            if values["inference_ok"].strip().lower() not in {"1", "true", "yes"} or level >= 2:
                inference_failures += 1

    node.create_subscription(DiagnosticArray, "/perception/open_vocab/diagnostics", diagnostics_callback, 50)
    observed_topics = {
        "/perception/garbage/detections_2d": "vision_msgs/msg/Detection2DArray",
        "/perception/ground_dirt/masks": "sensor_msgs/msg/Image",
        "/perception/garbage/targets": "sanitation_perception_interfaces/msg/GarbageTargetArray",
        "/perception/wrist/grasp_recheck": "std_msgs/msg/String",
    }
    subscriptions = []
    for topic, type_name in observed_topics.items():
        counts[topic] = 0
        nonempty_counts[topic] = 0

        def callback(message: Any, *, key: str = topic) -> None:
            counts[key] += 1
            if key.endswith("detections_2d"):
                nonempty = bool(message.detections)
            elif key.endswith("ground_dirt/masks"):
                nonempty = any(int(value) >= 2 for value in message.data)
            elif key.endswith("garbage/targets"):
                nonempty = bool(message.targets)
            else:
                nonempty = bool(getattr(message, "data", ""))
            if nonempty:
                nonempty_counts[key] += 1

        subscriptions.append(node.create_subscription(get_message(type_name), topic, callback, 50))

    graph_start = ros_graph()
    initial_processes = process_inventory()
    initial_pids = {row["component"]: row["pid"] for row in initial_processes["processes"]}
    memory_samples: list[dict[str, int]] = []
    thermal_samples: list[list[dict[str, Any]]] = []
    node_disappearances = 0
    process_restarts = 0
    rss_peak = 0
    start = time.monotonic()
    next_sample = start
    next_graph_check = start
    try:
        while time.monotonic() - start < duration_sec:
            rclpy.spin_once(node, timeout_sec=min(0.1, sample_period_sec))
            now = time.monotonic()
            if now >= next_sample:
                memory_samples.append(memory_sample())
                thermal_samples.append(thermal_sample())
                inventory = process_inventory()
                rss_peak = max(rss_peak, max((row["rss_bytes"] for row in inventory["processes"]), default=0))
                current = {row["component"]: row["pid"] for row in inventory["processes"]}
                process_restarts += sum(1 for key, pid in initial_pids.items() if current.get(key) not in {None, pid})
                next_sample = now + sample_period_sec
            if now >= next_graph_check:
                live_names = {name.strip("/").split("/")[-1] for name in ros_node_names()}
                node_disappearances += sum(1 for name in ("hobot_dosod", "mono_edgesam", "open_vocab_product_adapter") if name not in live_names)
                next_graph_check = now + 10.0
    finally:
        elapsed = time.monotonic() - start
        graph_end = ros_graph()
        final_processes = process_inventory()
        node.destroy_node()
        rclpy.shutdown()

    process_rows = final_processes["processes"]
    rss_peak = max(rss_peak, max((row["rss_bytes"] for row in process_rows), default=0))
    temperatures = [row["celsius"] for sample in thermal_samples for row in sample]

    def telemetry(component: str) -> dict[str, Any]:
        values = latencies[component]
        backend_values = sorted(backends[component])
        component_rows = [row for row in process_rows if component in row["component"]]
        libraries = [library.lower() for row in component_rows for library in row["backend_libraries"]]
        backend = backend_values[0] if len(backend_values) == 1 else None
        maps_match = bool(component_rows) and (
            (backend == "bpu" and any(any(token in library for token in ("hbrt", "hb_dnn", "libdnn")) for library in libraries))
            or (backend == "cpu" and not any(any(token in library for token in ("hbrt", "hb_dnn", "libdnn")) for library in libraries))
        )
        row = {
            "backend": backend,
            "samples": len(values),
            "fps": len(values) / elapsed if elapsed > 0 else 0.0,
            "latency_ms_p50": percentile(values, 0.50) if values else None,
            "latency_ms_p95": percentile(values, 0.95) if values else None,
            "latency_ms_p99": percentile(values, 0.99) if values else None,
            "latency_ms_max": max(values) if values else None,
            "runtime_process_maps_backend_match": maps_match,
        }
        if component == "dosod":
            row["model_sha256"] = next(iter(hashes[component]), None) if len(hashes[component]) == 1 else None
            row["vocabulary_sha256"] = (
                next(iter(vocabulary_hashes), None)
                if len(vocabulary_hashes) == 1
                else None
            )
        else:
            row["model_sha256s"] = sorted(hashes[component])
            row["model_sha256"] = sorted(hashes[component])[0] if hashes[component] else None
        return row

    return {
        "ros_graph": graph_end,
        "inference_telemetry": {
            "dosod": telemetry("dosod"),
            "edgesam": telemetry("edgesam"),
            "product_output_counts": counts,
            "product_nonempty_counts": nonempty_counts,
        },
        "sustained_run": {
            "duration_sec": elapsed,
            "sample_period_sec": sample_period_sec,
            "process_restarts": process_restarts,
            "node_disappearances": node_disappearances,
            "inference_failures": inference_failures,
        },
        "resources": {
            "system_memory_total_bytes": max((sample.get("MemTotal", 0) for sample in memory_samples), default=0),
            "system_memory_available_min_bytes": min((sample.get("MemAvailable", 0) for sample in memory_samples), default=0),
            "perception_rss_peak_bytes": rss_peak,
            "sample_count": len(memory_samples),
            "process_inventory_start": initial_processes,
            "process_inventory_end": final_processes,
        },
        "thermal": {
            "samples": len(thermal_samples),
            "peak_celsius": max(temperatures) if temperatures else None,
            "zones_last_sample": thermal_samples[-1] if thermal_samples else [],
        },
        "ros_graph_start": graph_start,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--acceptance-session", type=Path)
    parser.add_argument("--runtime-closure", type=Path)
    parser.add_argument("--dosod-hbm", type=Path)
    parser.add_argument("--dosod-vocabulary", type=Path)
    parser.add_argument("--edgesam-encoder-hbm", type=Path)
    parser.add_argument("--edgesam-decoder-hbm", type=Path)
    parser.add_argument("--dosod-compile-receipt", type=Path)
    parser.add_argument("--dosod-parity-report", type=Path)
    parser.add_argument("--dosod-metric-report", type=Path)
    parser.add_argument("--duration-sec", type=float, default=1800.0)
    parser.add_argument("--sample-period-sec", type=float, default=1.0)
    args = parser.parse_args()

    if args.output.exists():
        print(json.dumps({"status": "INVALID", "error": f"refusing to overwrite retained raw evidence: {args.output}"}, indent=2))
        return 2

    hardware = probe_hardware()
    base: dict[str, Any] = {
        "schema_version": 1,
        "report_id": RAW_REPORT_ID,
        "status": RAW_BLOCKED,
        "collection_complete": False,
        "hardware": hardware,
        "collector": {"script_path": str(Path(__file__).resolve()), "script_sha256": sha256_path(Path(__file__)), "pid": os.getpid()},
        "truth_boundary": {"simulator_or_evaluator_truth_used": False},
    }
    if not hardware["attested"]:
        base["blockers"] = hardware["blockers"] + ["collector refuses non-RDK-S100P hardware"]
        atomic_json(args.output, base)
        print(json.dumps(base, indent=2, sort_keys=True))
        return 4

    required_paths = {
        "snapshot": args.snapshot,
        "acceptance_session": args.acceptance_session,
        "runtime_closure": args.runtime_closure,
        "dosod_hbm": args.dosod_hbm,
        "dosod_vocabulary": args.dosod_vocabulary,
        "edgesam_encoder_hbm": args.edgesam_encoder_hbm,
        "edgesam_decoder_hbm": args.edgesam_decoder_hbm,
        "dosod_compile_receipt": args.dosod_compile_receipt,
        "dosod_parity_report": args.dosod_parity_report,
        "dosod_metric_report": args.dosod_metric_report,
    }
    missing = [name for name, path in required_paths.items() if path is None or not path.is_file()]
    if missing:
        base["blockers"] = [f"required on-board input missing: {name}" for name in missing]
        atomic_json(args.output, base)
        return 4
    if args.duration_sec <= 0 or args.sample_period_sec <= 0:
        base["blockers"] = ["duration and sampling period must be positive"]
        atomic_json(args.output, base)
        return 4

    try:
        base["source_binding"] = snapshot_identity(args.snapshot)
        base["runtime_closure_binding"] = runtime_closure_binding(args.runtime_closure)
        base["acceptance_session_binding"] = acceptance_session_binding(
            args.acceptance_session,
            base["source_binding"],
            base["runtime_closure_binding"],
        )
        base["system_image"] = os_release()
        base["models"] = [
            {"role": role, "path": str(path.resolve()), "sha256": sha256_path(path), "byte_size": path.stat().st_size}
            for role, path in required_paths.items()
            if role in {
                "dosod_hbm",
                "dosod_vocabulary",
                "edgesam_encoder_hbm",
                "edgesam_decoder_hbm",
            }
        ]
        base["dosod_hbm_evidence"] = collect_dosod_receipt_chain(
            args.dosod_compile_receipt, args.dosod_parity_report,
            args.dosod_metric_report, args.dosod_hbm,
        )
        base.update(collect_live(args.duration_sec, args.sample_period_sec))
        base["status"] = RAW_COLLECTED
        base["collection_complete"] = True
        base["blockers"] = []
    except Exception as exc:  # fail closed while retaining diagnostic evidence
        base["blockers"] = [f"live collection failed: {type(exc).__name__}: {exc}"]
        atomic_json(args.output, base)
        print(json.dumps(base, indent=2, sort_keys=True))
        return 4
    atomic_json(args.output, base)
    print(json.dumps(base, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
