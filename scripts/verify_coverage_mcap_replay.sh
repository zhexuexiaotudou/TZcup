#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG=""
OUTPUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ -n "${BAG}" && -d "${BAG}" ]] || { echo "--bag must name an MCAP bag directory" >&2; exit 2; }
[[ -n "${OUTPUT}" ]] || { echo "--output is required" >&2; exit 2; }
mkdir -p "${OUTPUT}"

ros2 bag info "${BAG}" > "${OUTPUT}/rosbag_info.txt"
set +e
timeout 120 ros2 bag play "${BAG}" --rate 100 --topics \
  /coverage/state /coverage/component_state /brush_enabled \
  --remap \
    /coverage/state:=/replay/coverage/state \
    /coverage/component_state:=/replay/coverage/component_state \
    /brush_enabled:=/replay/brush_enabled \
  > "${OUTPUT}/rosbag_play.log" 2>&1
play_code=$?
set -e
printf '%s\n' "${play_code}" > "${OUTPUT}/rosbag_play_exit_code.txt"

python3 "${ROOT}/scripts/coverage_mcap_replay_audit.py" \
  --bag "${BAG}" --play-exit-code "${play_code}" \
  --output "${OUTPUT}/mcap_replay_report.json"
