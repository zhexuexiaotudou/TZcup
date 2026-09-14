#!/usr/bin/env bash
# Offline slam_toolbox replay for one already-closed rosbag2 MCAP.
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_day1_offline_slam_replay.sh \
  --bag-dir DIR \
  --output-dir DIR \
  --runtime-setup FILE \
  --params-file FILE \
  [--rate RATE] \
  [--domain ID]
EOF
  exit 2
}

bag_dir=
output_dir=
runtime_setup=
params_file=
rate=1.0
domain=121

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-dir) bag_dir=${2:-}; shift 2 ;;
    --output-dir) output_dir=${2:-}; shift 2 ;;
    --runtime-setup) runtime_setup=${2:-}; shift 2 ;;
    --params-file) params_file=${2:-}; shift 2 ;;
    --rate) rate=${2:-}; shift 2 ;;
    --domain) domain=${2:-}; shift 2 ;;
    *) usage ;;
  esac
done

[[ -n "$bag_dir" && -n "$output_dir" && -n "$runtime_setup" && -n "$params_file" ]] ||
  usage
[[ -f "$bag_dir/metadata.yaml" && -f "$bag_dir/bag_0.mcap" ]] || {
  echo "closed bag inputs are incomplete: $bag_dir" >&2
  exit 3
}
[[ -f "$runtime_setup" && -f "$params_file" ]] || {
  echo "runtime setup or SLAM parameters are missing" >&2
  exit 3
}
[[ "$domain" =~ ^[0-9]+$ ]] && ((domain >= 0 && domain <= 232)) || {
  echo "ROS_DOMAIN_ID must be an integer in [0, 232]" >&2
  exit 3
}
[[ "$rate" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "rate must be a positive number" >&2
  exit 3
}
[[ ! -e "$output_dir/occupancy.yaml" && ! -e "$output_dir/replay.lifecycle.json" ]] || {
  echo "refusing an existing replay output: $output_dir" >&2
  exit 3
}

install -d -m 700 "$output_dir" "$output_dir/logs"
xdg_runtime="/tmp/tzcup_offline_slam_${domain}_$$_xdg"
rm -rf "$xdg_runtime"
install -d -m 700 "$xdg_runtime"

export XDG_RUNTIME_DIR="$xdg_runtime"
export ROS_DOMAIN_ID="$domain"
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_DISABLE_DAEMON=1
export RCUTILS_LOGGING_USE_STDOUT=1

set +u
source /opt/ros/jazzy/setup.bash
source "$runtime_setup"
set -u

duration_ns=$(
  python3 - "$bag_dir/metadata.yaml" <<'PY'
import sys
import yaml

info = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
print(info["rosbag2_bagfile_information"]["duration"]["nanoseconds"])
PY
)
play_timeout_seconds=$(
  python3 - "$duration_ns" "$rate" <<'PY'
import math
import sys

duration_seconds = int(sys.argv[1]) / 1_000_000_000
rate = float(sys.argv[2])
print(max(30, math.ceil(duration_seconds / rate) + 60))
PY
)
save_delay_seconds=$(
  python3 - "$duration_ns" "$rate" <<'PY'
import sys

duration_seconds = int(sys.argv[1]) / 1_000_000_000
rate = float(sys.argv[2])
print(max(0.0, duration_seconds / rate - 2.0))
PY
)

launch_pid=
player_pid=
cleanup() {
  for pid in "$player_pid" "$launch_pid"; do
    [[ -n "$pid" ]] || continue
    kill -0 "$pid" 2>/dev/null || continue
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ "$pgid" == "$pid" ]]; then
      kill -INT -- "-$pgid" 2>/dev/null || true
    else
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "$player_pid" "$launch_pid"; do
    [[ -n "$pid" ]] || continue
    kill -0 "$pid" 2>/dev/null || continue
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ "$pgid" == "$pid" ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    else
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  rm -rf "$xdg_runtime"
}
trap cleanup EXIT

start_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
setsid ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:="$params_file" \
  >"$output_dir/logs/slam.launch.log" 2>&1 &
launch_pid=$!

service_ready=false
for _ in $(seq 1 120); do
  if timeout 3 ros2 service list 2>/dev/null | grep -qx '/slam_toolbox/save_map'; then
    service_ready=true
    break
  fi
  kill -0 "$launch_pid" 2>/dev/null || break
  sleep 0.5
done

if [[ "$service_ready" != true ]]; then
  echo "slam_toolbox save_map service did not become ready" >&2
  exit 4
fi

player_rc=0
map_save_rc=0
setsid timeout --signal=INT --kill-after=10 "$play_timeout_seconds" \
  ros2 bag play --clock --rate "$rate" "$bag_dir" \
  >"$output_dir/logs/bag_play.log" 2>&1 &
player_pid=$!

sleep "$save_delay_seconds"
set +e
timeout 60 ros2 run nav2_map_server map_saver_cli \
  -f "$output_dir/occupancy" \
  --ros-args -p use_sim_time:=true \
  >"$output_dir/logs/map_saver.log" 2>&1
map_save_rc=$?
set -e

set +e
wait "$player_pid"
player_rc=$?
set -e
player_pid=
sleep 2

end_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
python3 - "$output_dir/replay.lifecycle.json" \
  "$bag_dir/metadata.yaml" "$bag_dir/bag_0.mcap" \
  "$params_file" "$runtime_setup" \
  "$domain" "$rate" "$player_rc" "$map_save_rc" \
  "$start_utc" "$end_utc" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

output, metadata, mcap, params, runtime_setup = map(Path, sys.argv[1:6])
domain, rate = int(sys.argv[6]), float(sys.argv[7])
player_rc, map_save_rc = int(sys.argv[8]), int(sys.argv[9])

def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

payload = {
    "schema_version": 1,
    "artifact_kind": "day1_offline_slam_replay_lifecycle",
    "start_utc": sys.argv[10],
    "end_utc": sys.argv[11],
    "ros_domain_id": domain,
    "ros_localhost_only": 1,
    "playback_rate": rate,
    "bag_metadata_sha256": sha256(metadata),
    "bag_mcap_sha256": sha256(mcap),
    "slam_params_sha256": sha256(params),
    "runtime_setup_sha256": sha256(runtime_setup),
    "player_rc": player_rc,
    "map_save_rc": map_save_rc,
    "map_saved": (output / "occupancy.yaml").is_file()
    and (output / "occupancy.pgm").is_file(),
    "gazebo_started": False,
}
(output / "replay.lifecycle.json").write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY

if [[ "$map_save_rc" -ne 0 || ! -f "$output_dir/occupancy.yaml" ]]; then
  exit 5
fi
if [[ "$player_rc" -ne 0 ]]; then
  exit 6
fi
