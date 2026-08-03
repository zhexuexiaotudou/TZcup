#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${ROOT}/starter_ws"
BASE_WORKSPACE="${WORKSPACE}"
OUTPUT=""
SEEDS="150,151,152,153,154,155,156,157,158,159"
RENDER_ENGINE="ogre2"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    --base-workspace) BASE_WORKSPACE="$2"; shift 2 ;;
    --seeds) SEEDS="$2"; shift 2 ;;
    --render-engine) RENDER_ENGINE="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ -n "${OUTPUT}" ]] || { echo "--output is required" >&2; exit 2; }
case "${RENDER_ENGINE}" in ogre2|ogre) ;; *) echo "invalid render engine" >&2; exit 2 ;; esac
mkdir -p "${OUTPUT}"
status_file="${OUTPUT}/repair_matrix_status.tsv"
[[ ! -e "${status_file}" ]] || { echo "refusing to overwrite ${status_file}" >&2; exit 3; }
printf 'seed\texit_code\tevidence\n' > "${status_file}"

IFS=',' read -ra seed_list <<< "${SEEDS}"
for seed in "${seed_list[@]}"; do
  [[ "${seed}" =~ ^[0-9]+$ ]] || { echo "invalid seed: ${seed}" >&2; exit 2; }
  export ROS_DOMAIN_ID="${seed}"
  export GZ_PARTITION="tzcup_coverage_repair_${seed}_$$"
  run_dir="${OUTPUT}/seed_${seed}"
  set +e
  bash "${ROOT}/scripts/run_visual_demo.sh" \
    --workspace "${WORKSPACE}" --base-workspace "${BASE_WORKSPACE}" \
    --output "${run_dir}" --skip-build --gazebo-only --no-gui --no-rviz \
    --no-browser --no-mcap --video off --map-size small \
    --simulation-speed fast --simulation-render-engine "${RENDER_ENGINE}" \
    --coverage-profile optimized --repair-evaluation-injection \
    --timeout 360 --seed "${seed}"
  code=$?
  set -e
  printf '%s\t%s\t%s\n' "${seed}" "${code}" "${run_dir}" >> "${status_file}"
  sleep 2
done

python3 - "${status_file}" <<'PY'
from pathlib import Path
import csv
import sys
rows = list(csv.DictReader(Path(sys.argv[1]).open(encoding="utf-8"), delimiter="\t"))
raise SystemExit(1 if any(int(row["exit_code"]) != 0 for row in rows) else 0)
PY
