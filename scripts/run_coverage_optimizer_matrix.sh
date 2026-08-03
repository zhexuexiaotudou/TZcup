#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${ROOT}/starter_ws"
BASE_WORKSPACE="${WORKSPACE}"
OUTPUT=""
OPTIMIZED_SEEDS="132,133,134,135,136"
LEGACY_SEEDS="140,141,142,143,144"
SPEED="fast"
RENDER_ENGINE="ogre2"
MCAP_SEED="132"

usage() {
  cat <<'EOF'
Usage: bash scripts/run_coverage_optimizer_matrix.sh --output DIR [options]
  --workspace DIR
  --base-workspace DIR
  --optimized-seeds CSV   default: 132,133,134,135,136
  --legacy-seeds CSV      default: 140,141,142,143,144
  --simulation-speed MODE default: fast
  --render-engine ENGINE  ogre2 or ogre
  --mcap-seed N           one optimized seed recorded for replay; empty disables
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    --base-workspace) BASE_WORKSPACE="$2"; shift 2 ;;
    --optimized-seeds) OPTIMIZED_SEEDS="$2"; shift 2 ;;
    --legacy-seeds) LEGACY_SEEDS="$2"; shift 2 ;;
    --simulation-speed) SPEED="$2"; shift 2 ;;
    --render-engine) RENDER_ENGINE="$2"; shift 2 ;;
    --mcap-seed) MCAP_SEED="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${OUTPUT}" ]] || { echo "--output is required" >&2; exit 2; }
case "${SPEED}" in normal|fast|turbo) ;; *) echo "invalid simulation speed" >&2; exit 2 ;; esac
case "${RENDER_ENGINE}" in ogre2|ogre) ;; *) echo "invalid render engine" >&2; exit 2 ;; esac
[[ -f "${WORKSPACE}/install/setup.bash" ]] || { echo "workspace install missing" >&2; exit 2; }
[[ -f "${BASE_WORKSPACE}/install/setup.bash" ]] || { echo "base workspace install missing" >&2; exit 2; }

mkdir -p "${OUTPUT}/baseline" "${OUTPUT}/selected"
status_file="${OUTPUT}/matrix_status.tsv"
if [[ -e "${status_file}" ]]; then
  echo "Refusing to overwrite retained matrix status: ${status_file}" >&2
  exit 3
fi
printf 'profile\tseed\texit_code\tevidence\n' > "${status_file}"

run_profile() {
  local profile="$1"
  local seed_csv="$2"
  local bucket="$3"
  local seed output_dir code
  IFS=',' read -ra seeds <<< "${seed_csv}"
  for seed in "${seeds[@]}"; do
    [[ "${seed}" =~ ^[0-9]+$ ]] || { echo "invalid seed: ${seed}" >&2; exit 2; }
    export ROS_DOMAIN_ID="${seed}"
    export GZ_PARTITION="tzcup_coverage_${profile}_${seed}_$$"
    output_dir="${OUTPUT}/${bucket}/seed_${seed}"
    args=(
      --workspace "${WORKSPACE}" --base-workspace "${BASE_WORKSPACE}"
      --output "${output_dir}" --skip-build --gazebo-only --no-gui --no-rviz
      --no-browser --video off --map-size small --simulation-speed "${SPEED}"
      --simulation-render-engine "${RENDER_ENGINE}"
      --coverage-profile "${profile}" --timeout 360 --seed "${seed}"
    )
    if [[ "${profile}" != "optimized" || "${seed}" != "${MCAP_SEED}" ]]; then
      args+=(--no-mcap)
    fi
    set +e
    bash "${ROOT}/scripts/run_visual_demo.sh" "${args[@]}"
    code=$?
    set -e
    printf '%s\t%s\t%s\t%s\n' \
      "${profile}" "${seed}" "${code}" "${output_dir}" >> "${status_file}"
    sleep 2
  done
}

run_profile optimized "${OPTIMIZED_SEEDS}" selected
run_profile legacy "${LEGACY_SEEDS}" baseline

python3 - "${status_file}" <<'PY'
from pathlib import Path
import csv
import sys

rows = list(csv.DictReader(Path(sys.argv[1]).open(encoding="utf-8"), delimiter="\t"))
failures = [row for row in rows if int(row["exit_code"]) != 0]
print(f"matrix completed: {len(rows)} runs, {len(failures)} launcher failures")
raise SystemExit(1 if failures else 0)
PY
