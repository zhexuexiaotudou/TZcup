from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/capture_formal_sensor_loopback_attestation.py"
SPEC = importlib.util.spec_from_file_location("sensor_loopback_attestation", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _process(proc: Path, pid: int, cmdline: list[str], environment: dict[str, str]) -> None:
    root = proc / str(pid)
    root.mkdir(parents=True)
    (root / "cmdline").write_bytes(b"\0".join(item.encode() for item in cmdline) + b"\0")
    (root / "environ").write_bytes(
        b"\0".join(f"{key}={value}".encode() for key, value in environment.items()) + b"\0"
    )


def _fixture(
    tmp_path: Path,
    *,
    bridge_ip: str = "127.0.0.1",
    relay: bool = False,
    inherited_discovery: dict[str, str] | None = None,
):
    proc = tmp_path / "proc"
    proc.mkdir()
    session = tmp_path / "session.json"
    closure = tmp_path / "closure.json"
    session.write_text(json.dumps({"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"}), encoding="utf-8")
    closure.write_text(json.dumps({"status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"}), encoding="utf-8")
    partition = "probe_partition"
    uri = "file:///repo/config/cyclonedds_localhost.xml"
    base = {
        "GZ_IP": "127.0.0.1",
        "IGN_IP": "127.0.0.1",
        "GZ_PARTITION": partition,
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "CYCLONEDDS_URI": uri,
    }
    if inherited_discovery:
        base.update(inherited_discovery)
    _process(proc, 101, ["/usr/bin/ruby", "/opt/gz", "sim", "-r"], base)
    bridge = {**base, "GZ_IP": bridge_ip}
    if relay:
        bridge["GZ_RELAY"] = "10.0.0.2"
    _process(proc, 102, ["/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"], bridge)
    return proc, partition, uri, session, closure


def test_accepts_live_gazebo_and_bridge_with_exact_loopback_environment(tmp_path: Path) -> None:
    proc, partition, uri, session, closure = _fixture(tmp_path)
    result = MODULE.attest(
        partition=partition,
        expected_cyclonedds_uri=uri,
        session=session,
        closure_manifest=closure,
        timeout_s=0,
        proc_root=proc,
    )
    assert result["passed"] is True
    assert {row["role"] for row in result["processes"]} == {
        "gazebo_sim",
        "ros_gz_parameter_bridge",
    }
    assert result["ros_discovery_variables_required_absent"] == [
        "ROS_LOCALHOST_ONLY",
        "ROS_STATIC_PEERS",
    ]


def test_rejects_non_loopback_bridge_or_relay(tmp_path: Path) -> None:
    proc, partition, uri, session, closure = _fixture(
        tmp_path, bridge_ip="172.20.0.2", relay=True
    )
    result = MODULE.attest(
        partition=partition,
        expected_cyclonedds_uri=uri,
        session=session,
        closure_manifest=closure,
        timeout_s=0,
        proc_root=proc,
    )
    assert result["passed"] is False
    assert {row["code"] for row in result["blockers"]} == {
        "LOOPBACK_ENVIRONMENT_MISMATCH",
        "RELAY_ENVIRONMENT_PRESENT",
    }


def test_rejects_inherited_legacy_or_static_ros_discovery(tmp_path: Path) -> None:
    proc, partition, uri, session, closure = _fixture(
        tmp_path,
        inherited_discovery={
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_STATIC_PEERS": "192.0.2.10",
        },
    )
    result = MODULE.attest(
        partition=partition,
        expected_cyclonedds_uri=uri,
        session=session,
        closure_manifest=closure,
        timeout_s=0,
        proc_root=proc,
    )
    assert result["passed"] is False
    blockers = [
        row for row in result["blockers"] if row["code"] == "ROS_DISCOVERY_ENVIRONMENT_PRESENT"
    ]
    assert len(blockers) == 2
    assert {frozenset(row["environment"].items()) for row in blockers} == {
        frozenset({"ROS_LOCALHOST_ONLY": "1", "ROS_STATIC_PEERS": "192.0.2.10"}.items())
    }


def test_rejects_missing_required_runtime_role(tmp_path: Path) -> None:
    proc, partition, uri, session, closure = _fixture(tmp_path)
    (proc / "102" / "environ").unlink()
    result = MODULE.attest(
        partition=partition,
        expected_cyclonedds_uri=uri,
        session=session,
        closure_manifest=closure,
        timeout_s=0,
        proc_root=proc,
    )
    assert result["passed"] is False
    assert result["blockers"] == [
        {"code": "MISSING_PROCESS_ROLE", "role": "ros_gz_parameter_bridge"}
    ]


def test_sensor_runner_attests_after_launch_before_collect_and_archives_sidecar() -> None:
    runner = (ROOT / "scripts/run_formal_vehicle_sensor_runtime.sh").read_text(encoding="utf-8")
    orchestrator = (ROOT / "scripts/run_formal_final_acceptance.py").read_text(encoding="utf-8")
    launch = runner.index("launch_pid=$!")
    attest = runner.index("capture_formal_sensor_loopback_attestation.py")
    collect = runner.index("collect_formal_vehicle_sensor_runtime.py")
    assert launch < attest < collect
    assert ".loopback_attestation.json" in runner
    assert ".loopback_attestation.json" in orchestrator
