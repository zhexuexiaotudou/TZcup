#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
import json, os, platform, shutil, subprocess
from pathlib import Path

def cmd(args):
    try:
        return subprocess.check_output(args, stderr=subprocess.STDOUT, text=True, timeout=20).strip()
    except Exception as e:
        return f"ERROR: {e}"

os_release = {}
p = Path("/etc/os-release")
if p.exists():
    for line in p.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            os_release[k] = v.strip('"')

report = {
    "platform": platform.platform(),
    "os_release": os_release,
    "ros_distro_env": os.environ.get("ROS_DISTRO"),
    "ros2_path": shutil.which("ros2"),
    "gz_path": shutil.which("gz"),
    "colcon_path": shutil.which("colcon"),
    "vcs_path": shutil.which("vcs"),
    "nvidia_smi_path": shutil.which("nvidia-smi"),
    "ros2_version_probe": cmd(["bash", "-lc", "source /opt/ros/jazzy/setup.bash 2>/dev/null && ros2 pkg list | wc -l"]),
    "gz_version": cmd(["bash", "-lc", "gz sim --versions 2>/dev/null || gz --versions 2>/dev/null || true"]),
    "gpu": cmd(["bash", "-lc", "nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null || true"]),
    "disk": cmd(["bash", "-lc", "df -h . | tail -n 1"]),
}

out = Path(os.environ.get("SANITATION_WS", str(Path.home() / "sanitation_ws"))) / "artifacts"
out.mkdir(parents=True, exist_ok=True)
(out / "preflight.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))

ok = (
    os_release.get("VERSION_ID") == "24.04"
    and Path("/opt/ros/jazzy/setup.bash").exists()
    and shutil.which("colcon") is not None
)
raise SystemExit(0 if ok else 2)
PY
