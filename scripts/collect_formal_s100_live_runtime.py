#!/usr/bin/env python3
"""Collect live RDK S100P evidence; refuse to run as a collector elsewhere.

The collector launches no product nodes.  DOSOD, EdgeSAM and the product
adapter must already be running and must publish per-inference DiagnosticArray
records on /perception/open_vocab/diagnostics.  Required diagnostic values are
documented in docs/formal-s100-live-acceptance.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from formal_s100_live_acceptance_core import (
    RAW_BLOCKED,
    RAW_COLLECTED,
    RAW_REPORT_ID,
    HRT_MODEL_EXEC_PATH,
    REQUIRED_SHORT_INPUT_TOPICS,
    REQUIRED_SHORT_DIAGNOSTIC_COMPONENTS,
    REQUIRED_SHORT_NODES,
    REQUIRED_SHORT_OUTPUT_TOPICS,
    REQUIRED_SHORT_PROJECT_OUTPUTS,
    REQUIRED_MODEL_ROLES,
    SHORT_DIAGNOSTIC_BLOCKED,
    SHORT_DIAGNOSTIC_DURATION_SEC,
    SHORT_DIAGNOSTIC_PASSED,
    SHORT_DIAGNOSTIC_REPORT_ID,
    TROS_ABI_IMPORTS,
    TROS_SETUP_PATHS,
    acceptance_session_binding,
    path_identity,
    probe_hardware,
    regular_nonlink_with_safe_ancestors,
    runtime_closure_binding,
    sha256_path,
    short_diagnostic_binding,
    snapshot_identity,
)
from verify_dosod_compile_parity_metric_chain import verify_dosod_compile_parity_metric_chain


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
        "hrt_model_exec": run_text([HRT_MODEL_EXEC_PATH, "--version"]),
        "ros2": run_text(["ros2", "--help"]),
        "dpkg_horizon": run_text(["dpkg-query", "-W", "hobot*", "horizon*"]),
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
        component = next((name for name in REQUIRED_SHORT_NODES if name in lowered), None)
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


def _stamp_ns(message: Any) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None and hasattr(message, "clock"):
        stamp = message.clock
    if stamp is None:
        transforms = getattr(message, "transforms", [])
        stamp = getattr(getattr(transforms[0], "header", None), "stamp", None) if transforms else None
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    return int(seconds) * 1_000_000_000 + int(nanoseconds) if isinstance(seconds, int) and isinstance(nanoseconds, int) else 0


def _bpu_device_identity() -> dict[str, Any]:
    path = Path("/dev/bpu_core0")
    try:
        mode = path.lstat().st_mode
        return {
            "path": str(path), "is_symlink": stat.S_ISLNK(mode),
            "is_character_device": stat.S_ISCHR(mode), "st_rdev_major": os.major(path.stat().st_rdev),
            "st_rdev_minor": os.minor(path.stat().st_rdev),
        }
    except OSError as exc:
        return {"path": str(path), "is_symlink": None, "is_character_device": False, "error": str(exc)}


def _tros_abi_identity() -> dict[str, Any]:
    setup = next((Path(value) for value in TROS_SETUP_PATHS if Path(value).is_file() and not Path(value).is_symlink()), None)
    if setup is None:
        return {"setup": {"path": None, "present": False}, "imports": {}}
    program = (
        "import importlib,importlib.metadata,json,pathlib,re,subprocess,sys; names=" + repr(TROS_ABI_IMPORTS) + "; rows={}\n"
        "for name in names:\n"
        " m=importlib.import_module(name); module_path=str(getattr(m,\"__file__\",\"\"))\n"
        " try:\n  meta=importlib.metadata.distribution(name); metadata={\"method\":\"importlib.metadata\",\"path\":str(meta.locate_file(\"METADATA\")),\"version\":str(meta.version)}\n"
        " except importlib.metadata.PackageNotFoundError:\n  try:\n   import ament_index_python.packages as aip; prefix=pathlib.Path(aip.get_package_prefix(name)); package_xml=prefix/'share'/name/'package.xml'; match=re.search(r'<version>([^<]+)</version>',package_xml.read_text(encoding='utf-8')); metadata={\"method\":\"ament_package_xml\",\"path\":str(package_xml),\"version\":match.group(1) if match else \"\"}\n  except Exception:\n   owner=subprocess.check_output(['dpkg-query','-S',module_path],text=True).split(':',1)[0]; version=subprocess.check_output(['dpkg-query','-W','-f=${Version}',owner],text=True).strip(); metadata={\"method\":\"dpkg_query\",\"path\":module_path,\"version\":version}\n"
        " rows[name]={\"module_path\":module_path,\"package_metadata\":metadata}\n"
        "print(json.dumps({\"python_executable\":sys.executable,\"python_version\":sys.version.split()[0],\"imports\":rows},sort_keys=True))"
    )
    result = run_text([
        "/bin/bash", "-lc",
        f"source {shlex.quote(str(setup))}; exec {shlex.quote(sys.executable)} -c {shlex.quote(program)}",
    ])
    try:
        details = json.loads(result["stdout"])
    except (KeyError, TypeError, json.JSONDecodeError):
        details = {"imports": {}, "error": result.get("stderr", "TROS ABI probe did not return JSON")}
    details["setup"] = path_identity(setup)
    details["command"] = result
    return details


def _node_identities() -> dict[str, dict[str, Any]]:
    result = run_text(["ros2", "node", "list"], timeout=10.0)
    names = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    return {
        role: {"name": next((name for name in names if name.rstrip("/").split("/")[-1] == leaf), None), "count": sum(name.rstrip("/").split("/")[-1] == leaf for name in names)}
        for role, leaf in REQUIRED_SHORT_NODES.items()
    }


def parse_dosod_model_info(stdout: str) -> dict[str, list[int]]:
    """Accept only explicit named DOSOD score/box shapes from official model_info."""
    rows: dict[str, list[int]] = {}
    # Current Journey 6 ``hrt_model_exec`` prints each tensor as a bounded
    # multiline output block (``name:`` followed by ``valid shape:``).  Parse
    # those blocks first so an ONNX path or model description cannot be
    # mistaken for a runtime output declaration.
    for match in re.finditer(
        r"(?ims)^output\[\d+\]:\s*(.*?)(?=^output\[\d+\]:|^-{10,}\s*$|\Z)",
        stdout,
    ):
        block = match.group(1)
        name_match = re.search(r"(?im)^name:\s*(scores|boxes)\s*$", block)
        shape_match = re.search(
            r"(?im)^valid shape:\s*\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)\s*$",
            block,
        )
        if name_match and shape_match:
            rows[name_match.group(1).lower()] = [
                int(value) for value in shape_match.groups()
            ]
    # Retain compatibility with older official builds that emitted one-line
    # square-bracket shapes.  Never overwrite a parsed current-format block.
    for name in ("scores", "boxes"):
        if name in rows:
            continue
        match = re.search(
            rf"(?im)^(?:name:\s*)?{name}\b[^\n]*?\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]",
            stdout,
        )
        if match:
            rows[name] = [int(value) for value in match.groups()]
    return rows


def dosod_model_info(dosod_hbm: Path) -> dict[str, Any]:
    result = run_text([HRT_MODEL_EXEC_PATH, "model_info", f"--model_file={dosod_hbm}"])
    stdout = str(result.get("stdout", ""))
    stderr = str(result.get("stderr", ""))
    shapes = parse_dosod_model_info(stdout)
    observed = 4 if result.get("returncode") == 0 and shapes == {"scores": [1, 8400, 4], "boxes": [1, 8400, 4]} else None
    # ``model_info`` is normally tiny.  Refuse a pathological output rather
    # than recording only an unverifiable digest in the short receipt.
    if len(stdout.encode("utf-8")) > 65_536 or len(stderr.encode("utf-8")) > 65_536:
        stdout = ""
        stderr = "model_info output exceeded bounded receipt limit"
        shapes = {}
        observed = None
    return {
        "argv": result.get("command"), "returncode": result.get("returncode"),
        "stdout": stdout, "stderr": stderr,
        "stdout_sha256": hashlib.sha256(stdout.encode("utf-8")).hexdigest(),
        "stderr_sha256": hashlib.sha256(stderr.encode("utf-8")).hexdigest(),
        "output_shapes": shapes, "observed_class_count": observed,
    }


def unsafe_required_model_inputs(model_paths: dict[str, Path]) -> list[str]:
    """Return every required model whose exact path is not a safe regular file."""
    return sorted(
        role for role in REQUIRED_MODEL_ROLES
        if role not in model_paths or not regular_nonlink_with_safe_ancestors(model_paths[role])
    )


def collect_short_diagnostic(
    duration_sec: float,
    source_binding: dict[str, str],
    session_binding: dict[str, Any],
    closure_binding: dict[str, str],
    model_paths: dict[str, Path],
) -> dict[str, Any]:
    """Observe a bounded, real board graph before the formal 1800-second run."""
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(f"required ROS 2 Python packages are unavailable: {exc}") from exc

    rclpy.init(args=None)
    node = rclpy.create_node("formal_s100_short_diagnostic_collector")
    inputs = {
        role: {
            "topic": topic, "type": type_name, "count": 0, "last_stamp_ns": 0, "stamps_ns": [], "frames_by_stamp": {},
            **({"freshness_policy": "static_map_allowed"} if role == "map" else {}),
        }
        for role, (topic, type_name) in REQUIRED_SHORT_INPUT_TOPICS.items()
    }
    outputs = {
        role: {"topic": topic, "type": type_name, "count": 0, "nonempty_count": 0, "stamps_ns": []}
        for role, (topic, type_name) in REQUIRED_SHORT_OUTPUT_TOPICS.items()
    }
    project_outputs = {
        role: {"topic": topic, "type": type_name, "count": 0, "nonempty_count": 0, "stamps_ns": []}
        for role, (topic, type_name) in REQUIRED_SHORT_PROJECT_OUTPUTS.items()
    }
    clock = {"topic": "/clock", "type": "rosgraph_msgs/msg/Clock", "stamps_ns": []}
    diagnostics = {
        "topic": "/perception/open_vocab/diagnostics", "type": "diagnostic_msgs/msg/DiagnosticArray",
        "count": 0, "errors": 0, "healthy_components": set(),
    }
    subscriptions: list[Any] = []

    def input_callback(message: Any, *, role: str) -> None:
        stamp = _stamp_ns(message)
        if stamp <= 0:
            return
        row = inputs[role]
        row["count"] += 1
        row["last_stamp_ns"] = stamp
        row["stamps_ns"] = (row["stamps_ns"] + [stamp])[-128:]
        row["frames_by_stamp"][str(stamp)] = str(getattr(getattr(message, "header", None), "frame_id", ""))
        if role == "camera_info":
            row["width"] = int(getattr(message, "width", 0))
            row["height"] = int(getattr(message, "height", 0))
            row["k"] = [float(value) for value in getattr(message, "k", [])]
        if role == "tf":
            transform_stamps = [_stamp_ns(transform) for transform in getattr(message, "transforms", [])]
            row["transform_stamps_ns"] = (row.get("transform_stamps_ns", []) + [value for value in transform_stamps if value > 0])[-128:]

    def output_callback(message: Any, *, role: str) -> None:
        stamp = _stamp_ns(message)
        if stamp <= 0:
            return
        row = outputs[role]
        row["count"] += 1
        row["stamps_ns"] = (row["stamps_ns"] + [stamp])[-128:]
        if bool(getattr(message, "targets", [])):
            row["nonempty_count"] += 1

    def project_output_callback(message: Any, *, role: str) -> None:
        stamp = _stamp_ns(message)
        if stamp <= 0:
            return
        row = project_outputs[role]
        row["count"] += 1
        row["stamps_ns"] = (row["stamps_ns"] + [stamp])[-128:]
        values = getattr(message, "detections", None)
        if values is None:
            values = getattr(message, "targets", None)
        if bool(values):
            row["nonempty_count"] += 1

    def clock_callback(message: Any) -> None:
        stamp = _stamp_ns(message)
        if stamp > 0:
            clock["stamps_ns"] = (clock["stamps_ns"] + [stamp])[-16:]

    def diagnostics_callback(message: Any) -> None:
        diagnostics["count"] += 1
        for status in message.status:
            name = str(status.name).lower()
            matched = next((role for role in REQUIRED_SHORT_DIAGNOSTIC_COMPONENTS if role in name), None)
            if matched is None:
                continue
            level = int(status.level)
            if level >= 2:
                diagnostics["errors"] += 1
            else:
                diagnostics["healthy_components"].add(matched)

    for role, (topic, type_name) in REQUIRED_SHORT_INPUT_TOPICS.items():
        subscriptions.append(node.create_subscription(get_message(type_name), topic, lambda message, role=role: input_callback(message, role=role), 20))
    for role, (topic, type_name) in REQUIRED_SHORT_OUTPUT_TOPICS.items():
        subscriptions.append(node.create_subscription(get_message(type_name), topic, lambda message, role=role: output_callback(message, role=role), 20))
    for role, (topic, type_name) in REQUIRED_SHORT_PROJECT_OUTPUTS.items():
        subscriptions.append(node.create_subscription(get_message(type_name), topic, lambda message, role=role: project_output_callback(message, role=role), 20))
    subscriptions.append(node.create_subscription(get_message("rosgraph_msgs/msg/Clock"), "/clock", clock_callback, 20))
    subscriptions.append(node.create_subscription(DiagnosticArray, diagnostics["topic"], diagnostics_callback, 50))
    start = time.monotonic()
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    try:
        while time.monotonic() - start < duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - start
        node.destroy_node()
        rclpy.shutdown()
    inventory = process_inventory()["processes"]
    processes = {}
    for role in REQUIRED_SHORT_NODES:
        rows = [row for row in inventory if row["component"] == role]
        processes[role] = rows[0] if len(rows) == 1 else {"count": len(rows)}
    required_stamp_rows = (
        [inputs[role]["stamps_ns"] for role in ("rgb", "depth", "camera_info")]
        + [outputs[role]["stamps_ns"] for role in REQUIRED_SHORT_OUTPUT_TOPICS]
        + [project_outputs[role]["stamps_ns"] for role in REQUIRED_SHORT_PROJECT_OUTPUTS]
    )
    common_stamps = set.intersection(*(set(rows) for rows in required_stamp_rows)) if required_stamp_rows else set()
    common_stamp = max(common_stamps) if common_stamps else 0
    camera_frame = inputs["rgb"]["frames_by_stamp"].get(str(common_stamp), "")
    try:
        transform = tf_buffer.lookup_transform("map", camera_frame, Time(nanoseconds=common_stamp), timeout=Duration(seconds=0.2)) if common_stamp and camera_frame else None
        tf_exact_binding = {
            "map_frame": "map", "camera_frame": camera_frame, "source_stamp_ns": common_stamp,
            "transform_stamp_ns": _stamp_ns(transform) if transform is not None else 0,
            "lookup_succeeded": transform is not None,
        }
    except Exception as exc:
        tf_exact_binding = {"map_frame": "map", "camera_frame": camera_frame, "source_stamp_ns": common_stamp, "transform_stamp_ns": 0, "lookup_succeeded": False, "error": str(exc)}
    return {
        "schema_version": 1,
        "report_id": SHORT_DIAGNOSTIC_REPORT_ID,
        "status": SHORT_DIAGNOSTIC_PASSED,
        "completed": True,
        "collected_epoch_ns": time.time_ns(),
        "duration_sec": elapsed,
        "source_binding": source_binding,
        "acceptance_session_binding": session_binding,
        "runtime_closure_binding": closure_binding,
        "hardware": probe_hardware(),
        "collector": {
            "script_path": str(Path(__file__).resolve()),
            "script_sha256": sha256_path(Path(__file__)),
            "pid": os.getpid(),
        },
        "bpu_device": _bpu_device_identity(),
        "hrt_model_exec": {**path_identity(Path(HRT_MODEL_EXEC_PATH)), "version": run_text([HRT_MODEL_EXEC_PATH, "--version"])},
        "tros_abi": _tros_abi_identity(),
        "nodes": _node_identities(),
        "processes": processes,
        "inputs": inputs,
        "clock": clock,
        "outputs": outputs,
        "models": [
            {"role": role, "path": str(path.resolve()), "sha256": sha256_path(path), "byte_size": path.stat().st_size}
            for role, path in sorted(model_paths.items())
        ],
        "dosod_model_info": dosod_model_info(model_paths["dosod_hbm"]),
        "same_stamp_chain": {
            "roles": ["rgb", "depth", "camera_info", "dosod", "edgesam", "boxes", "targets"],
            "stamp_ns": common_stamp,
        },
        "project_outputs": project_outputs,
        "tf_exact_binding": tf_exact_binding,
        "diagnostics": {**diagnostics, "healthy_components": sorted(diagnostics["healthy_components"])},
        "blockers": [],
    }


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
                node_disappearances += sum(1 for name in REQUIRED_NODES if name not in live_names)
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
    parser.add_argument("--dosod-admission-bundle", type=Path)
    parser.add_argument("--duration-sec", type=float, default=1800.0)
    parser.add_argument("--sample-period-sec", type=float, default=1.0)
    parser.add_argument("--short-diagnostic-only", action="store_true")
    parser.add_argument("--short-diagnostic", type=Path)
    parser.add_argument("--short-diagnostic-duration-sec", type=float, default=SHORT_DIAGNOSTIC_DURATION_SEC)
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
        "dosod_admission_bundle": args.dosod_admission_bundle,
    }
    short_required_names = (
        "snapshot", "acceptance_session", "runtime_closure",
        "dosod_hbm", "dosod_vocabulary", "edgesam_encoder_hbm", "edgesam_decoder_hbm",
    )
    required_for_mode = (
        {name: required_paths[name] for name in short_required_names}
        if args.short_diagnostic_only else required_paths
    )
    binding_names = ("snapshot", "acceptance_session", "runtime_closure")
    missing_bindings = [
        name
        for name in binding_names
        if required_paths[name] is None or not required_paths[name].is_file()
    ]
    if missing_bindings:
        base["blockers"] = [
            f"required on-board input missing: {name}" for name in missing_bindings
        ]
        atomic_json(args.output, base)
        return 4
    try:
        # Bind every board-side failure that occurs after the immutable formal
        # hand-off has been supplied.  In particular, a missing compile/parity/
        # metric chain must not produce an anonymous artifact that cannot be
        # associated with the active snapshot and runtime closure.
        base["source_binding"] = snapshot_identity(args.snapshot)
        base["runtime_closure_binding"] = runtime_closure_binding(args.runtime_closure)
        base["acceptance_session_binding"] = acceptance_session_binding(
            args.acceptance_session,
            base["source_binding"],
            base["runtime_closure_binding"],
        )
    except Exception as exc:  # fail closed while retaining diagnostic evidence
        base["blockers"] = [f"live collection failed: {type(exc).__name__}: {exc}"]
        atomic_json(args.output, base)
        print(json.dumps(base, indent=2, sort_keys=True))
        return 4
    missing = [
        name for name, path in required_for_mode.items()
        if path is None or (not path.is_dir() if name == "dosod_admission_bundle" else not path.is_file())
    ]
    if missing:
        base["blockers"] = [f"required on-board input missing: {name}" for name in missing]
        atomic_json(args.output, base)
        return 4
    unsafe_models = unsafe_required_model_inputs(
        {role: required_paths[role] for role in REQUIRED_MODEL_ROLES if role in required_for_mode}
    )
    if unsafe_models:
        base["blockers"] = [f"required model input is linked, unsafe, or nonregular: {role}" for role in sorted(unsafe_models)]
        atomic_json(args.output, base)
        return 4
    if args.duration_sec <= 0 or args.sample_period_sec <= 0:
        base["blockers"] = ["duration and sampling period must be positive"]
        atomic_json(args.output, base)
        return 4

    if args.short_diagnostic_duration_sec < SHORT_DIAGNOSTIC_DURATION_SEC:
        base["blockers"] = ["short diagnostic duration is below the fixed admission interval"]
        atomic_json(args.output, base)
        return 4

    try:
        model_paths = {role: required_paths[role] for role in REQUIRED_MODEL_ROLES}
        if args.short_diagnostic_only:
            # This is intentionally a separate retained receipt.  The 1800s
            # collector accepts it only via ``--short-diagnostic`` below.
            short = collect_short_diagnostic(
                args.short_diagnostic_duration_sec,
                base["source_binding"],
                base["acceptance_session_binding"],
                base["runtime_closure_binding"],
                model_paths,
            )
            if short["hardware"].get("attested") is not True:
                raise RuntimeError("short diagnostic board hardware is not attested")
            from formal_s100_live_acceptance_core import validate_short_diagnostic
            failures = validate_short_diagnostic(
                short, base["source_binding"], base["acceptance_session_binding"],
                base["runtime_closure_binding"], now_epoch_ns=time.time_ns(),
            )
            if failures:
                short["status"] = SHORT_DIAGNOSTIC_BLOCKED
                short["completed"] = False
                short["blockers"] = failures
                atomic_json(args.output, short)
                print(json.dumps(short, indent=2, sort_keys=True))
                return 4
            atomic_json(args.output, short)
            print(json.dumps(short, indent=2, sort_keys=True))
            return 0
        if args.short_diagnostic is None:
            raise ValueError("formal 1800-second collection requires --short-diagnostic")
        base["short_diagnostic_binding"] = short_diagnostic_binding(
            args.short_diagnostic,
            base["source_binding"], base["acceptance_session_binding"],
            base["runtime_closure_binding"],
        )
        base["system_image"] = os_release()
        base["models"] = [
            {"role": role, "path": str(path.resolve()), "sha256": sha256_path(path), "byte_size": path.stat().st_size}
            for role, path in sorted(model_paths.items())
        ]
        base["dosod_hbm_evidence"] = collect_dosod_receipt_chain(
            args.dosod_compile_receipt, args.dosod_parity_report,
            args.dosod_metric_report, args.dosod_hbm,
        )
        base["dosod_hbm_evidence"]["full_admission"] = verify_dosod_compile_parity_metric_chain(
            args.dosod_admission_bundle, args.dosod_hbm
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
