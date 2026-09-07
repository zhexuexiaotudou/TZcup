"""Small shared primitives for HBM evidence producers.

The producers deliberately use only their explicit output directory.  A
``BLOCKED`` report is evidence of a failed/unfinished operation, never a pass.
"""

from __future__ import annotations

import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Mapping

MEMORY_WATCHDOG_THRESHOLDS = {"min_mem_available": 3145728, "max_swap_used": 1048576, "max_group_rss": 9437184}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"json_object_required:{path}")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    pending.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def fresh_directory(path: Path, label: str) -> None:
    """Create a new evidence root without accepting an old run or link."""

    for ancestor in (path, *path.parents):
        if ancestor.is_symlink():
            raise ValueError(f"{label}_symlink_forbidden")
    if path.exists():
        raise ValueError(f"{label}_must_not_preexist")
    path.mkdir(parents=True, exist_ok=False)


def normal_file(path: Path, label: str) -> None:
    for ancestor in (path, *path.parents):
        if ancestor.is_symlink():
            raise ValueError(f"{label}_symlink_forbidden")
    if not path.is_file():
        raise ValueError(f"{label}_missing")


def path_under(root: Path, candidate: str, label: str) -> Path:
    path = (root / candidate).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"{label}_path_escape")
    normal_file(path, label)
    return path


def memory_watchdog_evidence(root: Path, execution: dict[str, Any]) -> dict[str, Any]:
    """Retain any fresh watchdog artifacts, including evidence from failures."""
    value = execution.get("memory_watchdog")
    result: dict[str, Any] = {"json": None, "log": None, "returncode": None, "status": None, "pgid": execution.get("pgid")}
    if not isinstance(value, dict):
        return result
    result.update({key: value.get(key) for key in ("returncode", "status")})
    for source, target in (("json_path", "json"), ("log_path", "log")):
        path = Path(str(value.get(source, "")))
        if path.is_relative_to(root.resolve()) and path.is_file() and not path.is_symlink():
            result[target] = {"relative_path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "byte_size": path.stat().st_size}
    return result


def require_completed_memory_watchdog(root: Path, value: dict[str, Any], label: str) -> None:
    if not isinstance(value.get("json"), dict) or not isinstance(value.get("log"), dict):
        raise ValueError(f"{label}_evidence_missing")
    report = load_object(root / value["json"]["relative_path"])
    if (value.get("returncode") != 0 or value.get("status") != "FORMAL_MEMORY_WATCHDOG_COMPLETED"
            or report.get("status") != "FORMAL_MEMORY_WATCHDOG_COMPLETED"
            or report.get("target_pgid") != value.get("pgid") or report.get("surviving_group_processes") != 0
            or report.get("breach_exit_code") != 86 or report.get("thresholds_kib") != MEMORY_WATCHDOG_THRESHOLDS):
        raise ValueError(f"{label}_failed")


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def _stop_group(pgid: int, details: dict[str, Any]) -> None:
    if not _group_alive(pgid):
        details["zero_survivor"] = True
        return
    details["term_sent"] = True
    os.killpg(pgid, signal.SIGTERM)
    deadline = time.monotonic() + 10
    while _group_alive(pgid) and time.monotonic() < deadline:
        time.sleep(0.05)
    if _group_alive(pgid):
        details["kill_sent"] = True
        os.killpg(pgid, signal.SIGKILL)
        deadline = time.monotonic() + 2
        while _group_alive(pgid) and time.monotonic() < deadline:
            time.sleep(0.05)
    details["zero_survivor"] = not _group_alive(pgid)


def _watchdog_details(process: subprocess.Popen[str], watchdog: dict[str, Path], environment: Mapping[str, str] | None) -> dict[str, Any]:
    script, json_path, log_path = watchdog["script"], watchdog["json"], watchdog["log"]
    normal_file(script, "memory_watchdog_script")
    for path, label in ((json_path, "memory_watchdog_json"), (log_path, "memory_watchdog_log")):
        if path.exists() or path.is_symlink():
            raise ValueError(f"{label}_must_be_fresh")
    pgid = os.getpgid(process.pid)
    watcher = subprocess.Popen([str(script), "--leader-pid", str(process.pid), "--pgid", str(pgid),
                                "--json", str(json_path), "--log", str(log_path)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
                               env=environment)
    return {"process": watcher, "pgid": pgid, "json_path": str(json_path.resolve()), "log_path": str(log_path.resolve())}


def _finish_watchdog(state: dict[str, Any]) -> dict[str, Any]:
    watcher: subprocess.Popen[str] = state.pop("process")
    try:
        watchdog_rc = watcher.wait(timeout=15)
    except subprocess.TimeoutExpired:
        watcher.terminate()
        try:
            watchdog_rc = watcher.wait(timeout=10)
        except subprocess.TimeoutExpired:
            watcher.kill(); watchdog_rc = watcher.wait()
    details: dict[str, Any] = {**state, "returncode": watchdog_rc, "status": None,
                               "json_sha256": None, "log_sha256": None}
    json_path, log_path = Path(details["json_path"]), Path(details["log_path"])
    try:
        normal_file(json_path, "memory_watchdog_json"); normal_file(log_path, "memory_watchdog_log")
        report = load_object(json_path)
        details.update({"status": report.get("status"), "json_sha256": sha256_file(json_path),
                        "log_sha256": sha256_file(log_path), "breach_exit_code": report.get("breach_exit_code"),
                        "surviving_group_processes": report.get("surviving_group_processes")})
    except (OSError, ValueError, json.JSONDecodeError):
        pass
    return details


def run_owned_process(command: list[str], *, timeout_seconds: float, memory_watchdog: dict[str, Path] | None = None,
                      start_new_session: bool = True, environment: Mapping[str, str] | None = None) -> tuple[int | None, str, str, dict[str, Any]]:
    """Run one child, optionally attaching the existing watchdog to its exact PGID."""
    if os.name != "posix":
        raise ValueError("posix_process_group_supervision_required")
    started = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               start_new_session=start_new_session, env=environment)
    pgid = os.getpgid(process.pid) if start_new_session else None
    details: dict[str, Any] = {"pgid": pgid, "deadline_seconds": timeout_seconds, "term_grace_seconds": 10,
                               "timed_out": False, "term_sent": False, "kill_sent": False, "zero_survivor": False}
    try:
        watchdog = _watchdog_details(process, memory_watchdog, environment) if memory_watchdog is not None else None
    except Exception:
        if start_new_session:
            _stop_group(pgid, details)
        else:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        raise
    try:
        stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        details["timed_out"] = details["term_sent"] = True
        if start_new_session:
            os.killpg(pgid, signal.SIGTERM)
        else:
            process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            details["kill_sent"] = True
            if start_new_session:
                os.killpg(pgid, signal.SIGKILL)
            else:
                process.kill()
            stdout, stderr = process.communicate()
    if start_new_session:
        _stop_group(pgid, details)
    else:
        # This PID shares an outer PGID.  Only that outer owner may claim that
        # the whole group has no survivors.
        details["direct_process_reaped"] = process.poll() is not None
    if watchdog is not None:
        details["memory_watchdog"] = _finish_watchdog(watchdog)
    details["elapsed_seconds"] = time.monotonic() - started
    return process.returncode, stdout or "", stderr or "", details
