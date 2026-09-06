#!/usr/bin/env python3
"""Record an independent Gazebo Model joint-state proof for service-door runs."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path


def _joint_names_from_model_text(text: str) -> list[str]:
    """Extract every joint name from the Model protobuf text, including duplicates."""

    names: list[str] = []
    for body in re.findall(r"(?ms)^\s*joint\s*\{(.*?)^\s*\}", text):
        match = re.search(r'^\s*name:\s*"([^"]+)"\s*$', body, re.MULTILINE)
        if match is not None:
            names.append(match.group(1))
    return names


def _publisher_count_from_topic_info(text: str) -> int:
    """Count publisher endpoints in legacy and Gazebo Sim 8 topic output."""

    count = 0
    in_publishers = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped in ("Publishers:", "Publishers [Address, Message Type]:"):
            in_publishers = True
            continue
        if not in_publishers:
            continue
        # Gazebo 8 prints endpoint rows under a different heading and some
        # versions do not indent them.  A top-level ``Something:`` heading
        # always ends the publisher table, rather than becoming a publisher.
        if re.fullmatch(r"[A-Za-z][^:]*:", stripped):
            break
        # Count endpoint records, not arbitrary non-empty rows.  Legacy
        # transport uses URI endpoints; Gazebo 8 uses an address,type row.
        if re.match(r"[A-Za-z][A-Za-z0-9+.-]*://\S+$", stripped) or re.fullmatch(
            r"[^,\s][^,]*,\s*[^,\s].*", stripped
        ):
            count += 1
    return count


def _run(args: list[str], partition: str) -> subprocess.CompletedProcess[str]:
    environment = dict(os.environ, GZ_PARTITION=partition)
    return subprocess.run(args, text=True, capture_output=True, env=environment, timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--joint", action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launcher-pid", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    result: dict[str, object] = {
        "status": "FAILED", "gz_partition": args.partition,
        "gazebo_transport_topic": args.topic, "topic_candidates": [],
        "discovered_topic": None, "message_type": None, "publisher_count": 0,
        "observed_joint_names": [], "topic_list_output": "", "topic_info_output": "",
        "topic_sample_output": "", "launcher_pid": args.launcher_pid,
        "launcher_liveness_checks": [],
    }
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        alive = args.launcher_pid > 0 and Path(f"/proc/{args.launcher_pid}").is_dir()
        result["launcher_liveness_checks"].append(alive)
        if not alive:
            result["error"] = "launch_pid_not_alive"
            break
        try:
            listed = _run(["gz", "topic", "-l"], args.partition)
            result["topic_list_output"] = listed.stdout + listed.stderr
            topics = [line.strip() for line in listed.stdout.splitlines() if line.strip()]
            candidates = [topic for topic in topics if topic == args.topic or topic.endswith(args.topic)]
            result["topic_candidates"] = candidates
            if listed.returncode != 0 or len(candidates) != 1:
                time.sleep(0.2)
                continue
            topic = candidates[0]
            info = _run(["gz", "topic", "-i", "-t", topic], args.partition)
            result["topic_info_output"] = info.stdout + info.stderr
            lines = info.stdout.splitlines()
            result["discovered_topic"] = topic
            result["message_type"] = "gz.msgs.Model" if any("gz.msgs.Model" in line for line in lines) else None
            publishers = _publisher_count_from_topic_info(info.stdout)
            result["publisher_count"] = publishers
            if info.returncode or result["message_type"] != "gz.msgs.Model" or publishers != 1:
                time.sleep(0.2)
                continue
            sample = _run(["gz", "topic", "-e", "-t", topic, "-n", "1"], args.partition)
            result["launcher_liveness_checks"].append(
                Path(f"/proc/{args.launcher_pid}").is_dir()
            )
            result["topic_sample_output"] = sample.stdout + sample.stderr
            observed = _joint_names_from_model_text(sample.stdout)
            result["observed_joint_names"] = observed
            expected = set(args.joint)
            if (
                sample.returncode == 0
                and len(observed) == len(expected)
                and len(observed) == len(set(observed))
                and set(observed) == expected
                and all(result["launcher_liveness_checks"])
            ):
                result["status"] = "PASSED"
                break
        except (OSError, subprocess.TimeoutExpired) as exc:
            result["error"] = str(exc)
        time.sleep(0.2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
