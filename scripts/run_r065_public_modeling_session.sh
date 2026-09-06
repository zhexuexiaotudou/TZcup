#!/usr/bin/env bash
# Public-only R065 runtime wrapper. It never accepts arbitrary command strings.
set -euo pipefail

: "${R065_RUN_ROOT:?new non-symlink run root is required}"
: "${R065_CLOSURE_MANIFEST:?frozen runtime closure manifest is required}"
: "${R065_RUNTIME_WS:?fresh frozen runtime workspace is required}"
: "${R065_PUBLIC_SPLIT:?public split=train|val is required}"
: "${R065_MAP_INDEX:?public map index is required}"
: "${R065_MISSION_INDEX:?public mission index is required}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ ! -L "$R065_RUN_ROOT" ]] || { echo 'BLOCKED: run-root argument must not be a symlink' >&2; exit 2; }
RAW_RUN_ROOT="$(realpath --no-symlinks -e "$R065_RUN_ROOT")"
RUN_ROOT="$(realpath -e "$R065_RUN_ROOT")"
[[ "$RAW_RUN_ROOT" == "$RUN_ROOT" ]] || { echo 'BLOCKED: run-root path must not traverse a symlink' >&2; exit 2; }
[[ ! -L "$RUN_ROOT" && -d "$RUN_ROOT" && -z "$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" && "${RUN_ROOT,,}" != *hidden* ]] || { echo 'BLOCKED: run root must be new, empty, and public' >&2; exit 2; }
[[ "$R065_PUBLIC_SPLIT" == train || "$R065_PUBLIC_SPLIT" == val ]] || { echo 'BLOCKED: split must be public train or val' >&2; exit 2; }
for value in "$R065_PUBLIC_SPLIT" "$R065_MAP_INDEX" "$R065_MISSION_INDEX"; do
  [[ "${value,,}" != *hidden* ]] || { echo 'BLOCKED: hidden input/command forbidden' >&2; exit 2; }
done

SNAPSHOT="$ROOT/reports/engineering/formal_vehicle_snapshot_manifest.json"
SESSION="$RUN_ROOT/formal_acceptance_session.json"
BINDING="$RUN_ROOT/formal_runtime_gate_binding.json"
R065_INSTALL_ROOT="$R065_RUNTIME_WS/install"

# Every fail-closed exit after run-root admission leaves one atomic blocked
# receipt. A successfully created session intentionally stays RUNNING for
# forensic use rather than being falsely finalized after a partial run.
PRIMARY_ERROR=""
trap '[[ -n "$PRIMARY_ERROR" ]] || PRIMARY_ERROR="$BASH_COMMAND"' ERR
on_exit() {
  local rc=$?
  printf 'exit_code=%s\nprimary_error=%s\n' "$rc" "$PRIMARY_ERROR" >"$RUN_ROOT/cleanup_evidence.txt"
  if [[ "$rc" -ne 0 && ! -e "$RUN_ROOT/r065_public_modeling_receipt.json" ]]; then
    python3 - "$RUN_ROOT/r065_public_modeling_receipt.json" "$rc" "$PRIMARY_ERROR" <<'PY'
import json, os, sys
from pathlib import Path
path = Path(sys.argv[1])
payload = {"report_id": "r065_public_modeling_receipt", "status": "R065_PUBLIC_MODELING_BLOCKED", "passed": False, "primary_error": sys.argv[3], "exit_code": int(sys.argv[2])}
pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
pending.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
pending.replace(path)
PY
  fi
}
trap on_exit EXIT

python3 "$ROOT/scripts/formal_acceptance_session.py" start --repository-root "$ROOT" --snapshot "$SNAPSHOT" --output "$SESSION" --runtime-closure-manifest "$R065_CLOSURE_MANIFEST" --runtime-install-root "$R065_INSTALL_ROOT" >"$RUN_ROOT/session.stdout"
source "$ROOT/scripts/formal_source_bound_preflight.sh"
formal_source_bound_preflight "$ROOT" "$R065_RUNTIME_WS" "$R065_CLOSURE_MANIFEST" "$SESSION" "$SNAPSHOT" "$BINDING" >"$RUN_ROOT/source_bound_preflight.stdout"
set +u
source /opt/ros/jazzy/setup.bash
source "$R065_INSTALL_ROOT/setup.bash"
set -u
formal_source_bound_verify_overlay "$R065_INSTALL_ROOT"
ros2 run sanitation_campus_scenario sanitation-campus-scenario generate --config "$ROOT/starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml" --profile formal --split "$R065_PUBLIC_SPLIT" --map-index "$R065_MAP_INDEX" --mission-index "$R065_MISSION_INDEX" --output "$RUN_ROOT/episode" >"$RUN_ROOT/generator.stdout"
EPISODE_MANIFEST="$RUN_ROOT/episode/public/episode_manifest.json"
WORLD="$RUN_ROOT/episode/public/world.sdf"
ENVIRONMENT_SCHEDULE="$RUN_ROOT/episode/environment/pedestrian_schedule.json"
export R065_SESSION="$SESSION" R065_RUNTIME_BINDING="$BINDING" R065_SNAPSHOT="$SNAPSHOT"
[[ -x "$ROOT/scripts/run_r065_w1_dynamic_footprint_live.sh" ]] || { echo 'BLOCKED: Sol must provide fixed W1 live runner' >&2; exit 2; }
[[ -x "$ROOT/scripts/run_r065_w2_moveit_ground_live.sh" ]] || { echo 'BLOCKED: Sol must provide fixed W2 live runner' >&2; exit 2; }
"$ROOT/scripts/run_r065_w1_dynamic_footprint_live.sh" "$RUN_ROOT" >"$RUN_ROOT/w1.stdout" 2>&1
"$ROOT/scripts/run_r065_w2_moveit_ground_live.sh" "$RUN_ROOT" >"$RUN_ROOT/w2.stdout" 2>&1
python3 "$ROOT/scripts/publish_r065_public_modeling_receipt.py" --seal-stdout "$RUN_ROOT/w2.stdout" "$RUN_ROOT/w2.json"
python3 "$ROOT/scripts/audit_public_pedestrian_geometry.py" --output "$RUN_ROOT/w3_public_audit.json" >"$RUN_ROOT/w3_public_audit.stdout"
FORMAL_DYNAMIC_EPISODE_ROOT="$RUN_ROOT/episode" FORMAL_DYNAMIC_SAVED_MAP_ROOT="$RUN_ROOT/first_map" FORMAL_ACCEPTANCE_SESSION="$SESSION" FORMAL_VEHICLE_RUNTIME_WS="$R065_RUNTIME_WS" FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="$R065_CLOSURE_MANIFEST" "$ROOT/scripts/run_formal_first_map_dynamic_prerequisite.sh" >"$RUN_ROOT/w5_mapping.stdout" 2>&1
FORMAL_DYNAMIC_EPISODE_ROOT="$RUN_ROOT/episode" FORMAL_DYNAMIC_SAVED_MAP_ROOT="$RUN_ROOT/first_map" FORMAL_DYNAMIC_OUTPUT="$RUN_ROOT/w3_live_dynamic.json" FORMAL_DYNAMIC_TELEMETRY="$RUN_ROOT/w3_dynamic_runtime/runtime_telemetry.json" FORMAL_DYNAMIC_RUNTIME_BINDING="$RUN_ROOT/w3.runtime_binding.json" FORMAL_ACCEPTANCE_SESSION_STATUS="$SESSION" FORMAL_VEHICLE_RUNTIME_WS="$R065_RUNTIME_WS" FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="$R065_CLOSURE_MANIFEST" "$ROOT/scripts/run_formal_dynamic_obstacle_avoidance.sh" >"$RUN_ROOT/w3_live_dynamic.stdout" 2>&1
mapfile -t runtime_schedules < <(find "$RUN_ROOT/w3_dynamic_runtime" -maxdepth 1 -type f -name 'pedestrian_schedule.seed_*.json' -print)
[[ "${#runtime_schedules[@]}" -eq 1 ]] || { echo 'BLOCKED: expected exactly one prepared runtime schedule' >&2; exit 2; }
RUNTIME_SCHEDULE="${runtime_schedules[0]}"
FORMAL_DYNAMIC_EPISODE_ROOT="$RUN_ROOT/episode" FORMAL_DYNAMIC_SAVED_MAP_ROOT="$RUN_ROOT/first_map" FORMAL_MAP_CLEANING_RUNTIME_ROOT="$RUN_ROOT/w5_runtime" FORMAL_MAP_LIFECYCLE_OUTPUT="$RUN_ROOT/w5.json" FORMAL_ACCEPTANCE_SESSION="$SESSION" FORMAL_VEHICLE_RUNTIME_WS="$R065_RUNTIME_WS" FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST="$R065_CLOSURE_MANIFEST" "$ROOT/scripts/run_formal_saved_map_cleaning_lifecycle.sh" >"$RUN_ROOT/w5.stdout" 2>&1

python3 "$ROOT/scripts/publish_r065_public_modeling_receipt.py" \
  --repository-root "$ROOT" --run-root "$RUN_ROOT" --session "$SESSION" \
  --runtime-binding "$BINDING" --episode-manifest "$EPISODE_MANIFEST" \
  --world "$WORLD" --environment-schedule "$ENVIRONMENT_SCHEDULE" --runtime-schedule "$RUNTIME_SCHEDULE" \
  --w1 "$RUN_ROOT/w1.json" --w2 "$RUN_ROOT/w2.json" --w3_public_audit "$RUN_ROOT/w3_public_audit.json" --w3_live_dynamic "$RUN_ROOT/w3_live_dynamic.json" --w5 "$RUN_ROOT/w5.json" \
  --output "$RUN_ROOT/r065_public_modeling_receipt.json"
