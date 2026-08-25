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
export TZCUP_ROOT="$PACK_ROOT"
export PIP_BREAK_SYSTEM_PACKAGES=1
export ROSDEP_SKIP_KEYS=micro_ros_agent

if [[ "$(id -u)" -eq 0 ]]; then
  APT_GET=(apt-get)
else
  # Keep Git checkouts owned by the invoking developer while using the
  # conventional passwordless sudo boundary only for package metadata.
  APT_GET=(sudo -n apt-get)
fi
"${APT_GET[@]}" -o Acquire::Retries=5 update 2>&1 | tee "$OUT/apt_update.log"

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

# Several package tests intentionally load repository-level audit helpers and
# checked-in configuration. Expose those read-only trees at the same relative
# locations inside this disposable workspace so Stage 1 remains portable.
ln -s "$PACK_ROOT/scripts" "$WS/scripts"
ln -s "$PACK_ROOT/artifacts" "$WS/artifacts"
mkdir "$WS/starter_ws"
ln -s "$PACK_ROOT/starter_ws/src" "$WS/starter_ws/src"
touch "$WS/starter_ws/COLCON_IGNORE"

export UPSTREAM_PATCH_REPORT="$OUT/opennav_coverage_patch_status.json"
record_command env "UPSTREAM_PATCH_REPORT=$UPSTREAM_PATCH_REPORT" \
  bash "$PACK_ROOT/scripts/import_upstream.sh" | tee -a "$OUT/commands.log"
bash "$PACK_ROOT/scripts/import_upstream.sh" 2>&1 | tee "$OUT/import_upstream.log"

verify_opennav_patch_state() {
  python3 - "$WS/src/opennav_coverage" "$UPSTREAM_PATCH_REPORT" "$PACK_ROOT" <<'PY'
import hashlib
import json
import subprocess
import sys
from pathlib import Path

repo = Path(sys.argv[1])
report_path = Path(sys.argv[2])
pack_root = Path(sys.argv[3])
report = json.loads(report_path.read_text(encoding="utf-8"))
patch_path = pack_root / report["patch_path"]

patched_commit = subprocess.check_output(
    ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
).strip()
patched_tree = subprocess.check_output(
    ["git", "-C", str(repo), "rev-parse", "HEAD^{tree}"], text=True
).strip()
base_commit = subprocess.check_output(
    ["git", "-C", str(repo), "rev-parse", "HEAD^"], text=True
).strip()
status = subprocess.check_output(
    ["git", "-C", str(repo), "status", "--porcelain=v1"], text=True
).splitlines()
diff = subprocess.check_output(
    [
        "git", "-C", str(repo), "diff", "--binary", "--no-ext-diff",
        f"{report['base_commit']}..{report['patched_commit']}",
    ]
)
diff_sha256 = hashlib.sha256(diff).hexdigest()
patch_sha256 = hashlib.sha256(patch_path.read_bytes()).hexdigest()
base_tree = subprocess.check_output(
    ["git", "-C", str(repo), "rev-parse", f"{report['base_commit']}^{{tree}}"], text=True
).strip()
patched_files = subprocess.check_output(
    [
        "git", "-C", str(repo), "diff", "--name-only",
        f"{report['base_commit']}..{report['patched_commit']}",
    ],
    text=True,
).splitlines()
metadata_values = subprocess.check_output(
    [
        "git", "-C", str(repo), "show", "-s",
        "--format=%an%x00%ae%x00%aI%x00%cn%x00%ce%x00%cI%x00%B",
        report["patched_commit"],
    ]
).decode("utf-8").rstrip("\n").split("\x00")
actual_metadata = dict(
    zip(
        ("name", "email", "author_date", "committer_name", "committer_email", "committer_date", "message"),
        metadata_values,
        strict=True,
    )
)
expected_metadata = {
    "name": report["commit_metadata"]["name"],
    "email": report["commit_metadata"]["email"],
    "author_date": report["commit_metadata"]["author_date"],
    "committer_name": report["commit_metadata"]["name"],
    "committer_email": report["commit_metadata"]["email"],
    "committer_date": report["commit_metadata"]["committer_date"],
    "message": report["commit_metadata"]["message"],
}

errors = []
if report.get("repository") != "opennav_coverage":
    errors.append(f"unexpected repository: {report.get('repository')!r}")
if base_commit != report["base_commit"]:
    errors.append(f"patched commit parent changed: {base_commit}")
if base_tree != report["base_tree"]:
    errors.append(f"base tree changed: {base_tree}")
if patched_commit != report["patched_commit"]:
    errors.append(f"patched commit changed: {patched_commit}")
if patched_tree != report["patched_tree"]:
    errors.append(f"patched tree changed: {patched_tree}")
if patch_sha256 != report["patch_sha256"]:
    errors.append(f"patch file changed: {patch_sha256}")
if status != report["status_porcelain"]:
    errors.append(f"status changed: {status!r}")
if diff_sha256 != report["patched_diff_sha256"]:
    errors.append(f"patched diff changed: {diff_sha256}")
if status:
    errors.append(f"patched checkout is dirty: {status!r}")
if report["status_porcelain"] != []:
    errors.append(f"report does not record clean status: {report['status_porcelain']!r}")
if report["working_tree_clean"] is not True:
    errors.append("patched checkout must explicitly report working_tree_clean=true")
if patched_files != ["opennav_coverage/test/test_path.cpp"]:
    errors.append(f"actual patched files changed: {patched_files!r}")
if report["patched_files"] != patched_files:
    errors.append(f"unexpected patched files: {report['patched_files']!r}")
if actual_metadata != expected_metadata:
    errors.append(f"patched commit metadata changed: {actual_metadata!r}")
if errors:
    raise SystemExit("; ".join(errors))

print(
    "opennav_coverage "
    f"base_commit={base_commit} "
    f"patch_sha256={report['patch_sha256']} "
    f"patched_commit={patched_commit} "
    f"patched_tree={patched_tree} "
    f"patched_diff_sha256={diff_sha256} "
    "working_tree_clean=true patch_state_verified=true"
)
PY
}

verify_opennav_patch_state | tee "$OUT/opennav_coverage_patch_state_before.txt"

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

verify_opennav_patch_state | tee "$OUT/opennav_coverage_patch_state_after.txt"

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
patch_report = json.loads(
    (out / "opennav_coverage_patch_status.json").read_text(encoding="utf-8")
)

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
    if name == "opennav_coverage":
        repositories[name]["patch"] = patch_report
        repositories[name]["patch_state_verified_before"] = True
        repositories[name]["patch_state_verified_after"] = True

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
