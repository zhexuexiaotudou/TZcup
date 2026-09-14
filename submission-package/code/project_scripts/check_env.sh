#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SANITATION_PACK_ROOT="${SANITATION_PACK_ROOT:-$PACK_ROOT}"

python3 - <<'PY'
import datetime as dt
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path


def probe(command: str, timeout: int = 20) -> dict:
    try:
        completed = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except Exception as exc:  # pragma: no cover - defensive evidence path
        return {"command": command, "returncode": 255, "stdout": "", "stderr": repr(exc)}


os_release = {}
os_release_path = Path("/etc/os-release")
if os_release_path.exists():
    for line in os_release_path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            os_release[key] = value.strip('"')

commands = {
    name: shutil.which(name)
    for name in ("python3", "ros2", "gz", "colcon", "rosdep", "vcs", "nvidia-smi")
}
probes = {
    "ros_package_count": probe(
        "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && ros2 pkg list | wc -l"
    ),
    "gazebo_version": probe("gz sim --versions || gz --versions"),
    "ros_gz_sim": probe(
        "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && ros2 pkg prefix ros_gz_sim"
    ),
    "gpu": probe(
        "nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader"
    ),
    "disk": probe("df -B1 --output=source,size,used,avail,pcent,target . | tail -n 1"),
    "memory": probe("free -b | sed -n '1,2p'"),
}

checks = {
    "ubuntu_24_04": os_release.get("ID") == "ubuntu" and os_release.get("VERSION_ID") == "24.04",
    "ros_jazzy_setup": Path("/opt/ros/jazzy/setup.bash").is_file(),
    "ros2_available": commands["ros2"] is not None,
    "gazebo_available": commands["gz"] is not None and probes["gazebo_version"]["returncode"] == 0,
    "ros_gz_available": probes["ros_gz_sim"]["returncode"] == 0,
    "python3_available": commands["python3"] is not None,
    "colcon_available": commands["colcon"] is not None,
    "rosdep_available": commands["rosdep"] is not None,
    "vcs_available": commands["vcs"] is not None,
}
required_checks = tuple(checks)
blockers = [name for name in required_checks if not checks[name]]

display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
warnings = []
if not display:
    warnings.append("No DISPLAY or WAYLAND_DISPLAY detected; GUI evidence cannot be collected here.")
if commands["nvidia-smi"] is None or probes["gpu"]["returncode"] != 0:
    warnings.append("No NVIDIA GPU was exposed; CPU/software rendering may be used for headless checks.")

report = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "target_profile": "Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic",
    "success": not blockers,
    "execution_context": {
        "platform": platform.platform(),
        "container": Path("/.dockerenv").exists(),
        "wsl_distro_name": os.environ.get("WSL_DISTRO_NAME"),
        "display": display,
    },
    "os_release": os_release,
    "commands": commands,
    "checks": checks,
    "blockers": blockers,
    "warnings": warnings,
    "probes": probes,
}

pack_root = Path(os.environ["SANITATION_PACK_ROOT"])
output_path = Path(
    os.environ.get("PREFLIGHT_OUTPUT", str(pack_root / "artifacts" / "preflight.json"))
)
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(0 if report["success"] else 2)
PY
