#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
WS="${SANITATION_WS:-$PACK_ROOT/.work/stage1_$STAMP}"
OUT="$PACK_ROOT/artifacts/stage1_$STAMP"

if [[ -e "$WS" ]]; then
  echo "ERROR: Stage 1 workspace already exists: $WS" >&2
  exit 3
fi

mkdir -p "$WS" "$OUT"
export SANITATION_WS="$WS"
export PIP_BREAK_SYSTEM_PACKAGES=1
export ROSDEP_SKIP_KEYS=micro_ros_agent

apt-get -o Acquire::Retries=5 update 2>&1 | tee "$OUT/apt_update.log"

rosdep_update_ok=false
for attempt in 1 2 3; do
  echo "rosdep update attempt $attempt/3" | tee -a "$OUT/rosdep_update.log"
  set +e
  rosdep update --rosdistro jazzy 2>&1 | tee -a "$OUT/rosdep_update.log"
  rosdep_status=${PIPESTATUS[0]}
  set -e
  if [[ "$rosdep_status" -eq 0 ]]; then
    rosdep_update_ok=true
    break
  fi
done

if [[ "$rosdep_update_ok" != true ]]; then
  echo "WARNING: rosdep update was partial after 3 attempts; validating the cached Jazzy database." \
    | tee -a "$OUT/rosdep_update.log"
  rosdep db 2>&1 | tee "$OUT/rosdep_database.log" >/dev/null
  rosdep resolve nav2_bringup 2>&1 | tee "$OUT/rosdep_probe_nav2_bringup.log" >/dev/null
fi

record_command() {
  printf '$ %q' "$1"
  shift
  printf ' %q' "$@"
  printf '\n'
}

{
  echo "Stage 1 reproducible build"
  echo "started_at_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "workspace=$WS"
  echo "pack_root=$PACK_ROOT"
} > "$OUT/context.txt"

record_command bash "$PACK_ROOT/scripts/install_starter.sh" | tee "$OUT/commands.log"
bash "$PACK_ROOT/scripts/install_starter.sh" 2>&1 | tee "$OUT/install_starter.log"

record_command bash "$PACK_ROOT/scripts/import_upstream.sh" | tee -a "$OUT/commands.log"
bash "$PACK_ROOT/scripts/import_upstream.sh" 2>&1 | tee "$OUT/import_upstream.log"

{
  for repository in linorobot2 opennav_coverage; do
    repo="$WS/src/$repository"
    printf '%s commit=%s dirty_files=%s\n' \
      "$repository" \
      "$(git -C "$repo" rev-parse HEAD)" \
      "$(git -C "$repo" status --porcelain | wc -l)"
  done
} | tee "$OUT/third_party_status_before.txt"

for build_number in 1 2; do
  record_command bash --noprofile --norc -c \
    "export SANITATION_WS='$WS'; bash '$PACK_ROOT/scripts/build_ws.sh'" \
    | tee -a "$OUT/commands.log"
  bash --noprofile --norc -c \
    "export SANITATION_WS='$WS'; bash '$PACK_ROOT/scripts/build_ws.sh'" \
    2>&1 | tee "$OUT/build_${build_number}.log"
done

{
  for repository in linorobot2 opennav_coverage; do
    repo="$WS/src/$repository"
    printf '%s commit=%s dirty_files=%s\n' \
      "$repository" \
      "$(git -C "$repo" rev-parse HEAD)" \
      "$(git -C "$repo" status --porcelain | wc -l)"
  done
} | tee "$OUT/third_party_status_after.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS"
colcon list --names-only | sort > "$OUT/packages.txt"
colcon test-result --all --verbose > "$OUT/test_results.txt"

export STAGE1_OUT="$OUT"
export STAGE1_WS="$WS"
python3 - <<'PY'
import datetime as dt
import json
import os
import subprocess
from pathlib import Path

out = Path(os.environ["STAGE1_OUT"])
ws = Path(os.environ["STAGE1_WS"])

repositories = {}
for name in ("linorobot2", "opennav_coverage"):
    repo = ws / "src" / name
    repositories[name] = {
        "commit": subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
        ).strip(),
        "dirty": bool(
            subprocess.check_output(
                ["git", "-C", str(repo), "status", "--porcelain"], text=True
            ).strip()
        ),
    }

summary = {
    "schema_version": 1,
    "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "success": True,
    "workspace": str(ws),
    "builds_completed": 2,
    "tests_passed": True,
    "third_party_repositories": repositories,
    "package_count": len((out / "packages.txt").read_text().splitlines()),
    "artifacts": sorted(path.name for path in out.iterdir()),
}
(out / "stage1_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
PY

echo "$OUT"
