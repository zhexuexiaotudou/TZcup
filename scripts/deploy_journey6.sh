#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --host HOST --user USER --bundle DIR [--profile auto|PROFILE] [--execute]" >&2
}

HOST=""
USER_NAME=""
BUNDLE=""
PROFILE="auto"
EXECUTE=false
while (($#)); do
  case "$1" in
    --host) HOST="$2"; shift 2 ;;
    --user) USER_NAME="$2"; shift 2 ;;
    --bundle) BUNDLE="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --execute) EXECUTE=true; shift ;;
    *) usage; exit 64 ;;
  esac
done
[[ -n "${HOST}" && -n "${USER_NAME}" && -n "${BUNDLE}" ]] || { usage; exit 64; }
[[ "${HOST}" =~ ^[A-Za-z0-9_.:-]+$ && "${USER_NAME}" =~ ^[A-Za-z0-9._-]+$ && "${PROFILE}" =~ ^(auto|journey6_[a-z0-9_]+)$ ]] || {
  echo "invalid host, user, or profile syntax" >&2
  exit 64
}
BUNDLE="$(cd -- "${BUNDLE}" && pwd)"
[[ -f "${BUNDLE}/bundle_manifest.json" && -f "${BUNDLE}/SHA256SUMS" ]] || { echo "invalid bundle directory" >&2; exit 66; }
(cd "${BUNDLE}" && sha256sum -c SHA256SUMS)
if [[ "${EXECUTE}" != true ]]; then
  echo "dry-run: checksum verified locally; no board connection or mutation performed"
  echo "target=${USER_NAME}@${HOST} profile=${PROFILE} bundle=${BUNDLE}"
  exit 0
fi

TARGET="${USER_NAME}@${HOST}"
SSH_OPTIONS=(-o StrictHostKeyChecking=yes)
BUNDLE_ID="$(python3 - "${BUNDLE}/bundle_manifest.json" <<'PY'
import json, sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
if data.get("target_family") != "journey6":
    raise SystemExit("bundle target is not journey6")
if data.get("status") == "skeleton" or data.get("external_blockers"):
    raise SystemExit("blocked_external: bundle is still a skeleton")
print(data["bundle_id"])
PY
)"
REMOTE_ROOT="/var/tmp/tzcup-j6-${BUNDLE_ID}-$$"
ssh "${SSH_OPTIONS[@]}" "${TARGET}" "mkdir -p '${REMOTE_ROOT}'"
scp "${SSH_OPTIONS[@]}" "${BUNDLE}/scripts/j6_board_inventory.py" "${TARGET}:${REMOTE_ROOT}/j6_board_inventory.py"
ssh "${SSH_OPTIONS[@]}" "${TARGET}" "python3 '${REMOTE_ROOT}/j6_board_inventory.py' --output '${REMOTE_ROOT}/J6_BOARD_INVENTORY.json'"
scp "${SSH_OPTIONS[@]}" -r "${BUNDLE}" "${TARGET}:${REMOTE_ROOT}/bundle"
ssh "${SSH_OPTIONS[@]}" -t "${TARGET}" "sudo bash '${REMOTE_ROOT}/bundle/scripts/install_candidate.sh' --source '${REMOTE_ROOT}/bundle' --inventory '${REMOTE_ROOT}/J6_BOARD_INVENTORY.json' --profile '${PROFILE}' --execute"
echo "deployment completed; board-side installer reported its rollback point"
