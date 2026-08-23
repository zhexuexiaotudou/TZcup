#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --source DIR --inventory FILE [--profile auto|PROFILE] [--prefix DIR] --execute" >&2
}

SOURCE=""
INVENTORY=""
PROFILE="auto"
PREFIX="/opt/tzcup/journey6"
EXECUTE=false
while (($#)); do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --inventory) INVENTORY="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --execute) EXECUTE=true; shift ;;
    *) usage; exit 64 ;;
  esac
done
[[ -n "${SOURCE}" && -n "${INVENTORY}" ]] || { usage; exit 64; }
[[ "${EXECUTE}" == true ]] || { echo "dry-run: no files or symlinks changed"; exit 0; }
[[ -d "${SOURCE}" && -f "${INVENTORY}" && -f "${SOURCE}/SHA256SUMS" ]] || exit 66

(cd "${SOURCE}" && sha256sum -c SHA256SUMS)
readarray -t INVENTORY_FIELDS < <(python3 - "${INVENTORY}" <<'PY'
import json, sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
if data.get("status") != "ready" or data.get("target_family") != "journey6":
    raise SystemExit("blocked_external: Journey 6 board/runtime inventory is not ready")
if data.get("board", {}).get("forbidden_family_evidence"):
    raise SystemExit("refusing RDK/S100-family board inventory")
print(data.get("target_march", "auto"))
print(data.get("target_sku", "auto"))
PY
)
ACTUAL_MARCH="${INVENTORY_FIELDS[0]}"

if [[ "${PROFILE}" == auto ]]; then
  case "${ACTUAL_MARCH}" in
    nash-e) PROFILE="journey6_nash_e" ;;
    nash-m) PROFILE="journey6_nash_m" ;;
    nash-p) PROFILE="journey6_nash_p" ;;
    *) echo "blocked_external: inventory did not resolve a supported Journey 6 march" >&2; exit 78 ;;
  esac
fi
PROFILE_FILE="${SOURCE}/profiles/${PROFILE}.yaml"
[[ -f "${PROFILE_FILE}" ]] || { echo "profile not found: ${PROFILE}" >&2; exit 78; }
PROFILE_MARCH="$(awk '/^target_march:/ {print $2; exit}' "${PROFILE_FILE}")"
[[ "${PROFILE_MARCH}" == "${ACTUAL_MARCH}" ]] || { echo "runtime/profile march mismatch" >&2; exit 78; }
PROFILE_ABI="$(awk '/^  abi:/ {print $2; exit}' "${PROFILE_FILE}")"
PROFILE_RUNTIME_VERSION="$(awk '/^  minimum_version:/ {print $2; exit}' "${PROFILE_FILE}")"
[[ -n "${PROFILE_ABI}" && "${PROFILE_ABI}" != "null" && -n "${PROFILE_RUNTIME_VERSION}" && "${PROFILE_RUNTIME_VERSION}" != "null" ]] || {
  echo "blocked_external: runtime ABI/version profile unresolved" >&2
  exit 78
}
python3 - "${INVENTORY}" "${PROFILE_ABI}" "${PROFILE_RUNTIME_VERSION}" <<'PY'
import json, re, sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
actual_abi = data.get("runtime", {}).get("abi")
actual_version = data.get("runtime", {}).get("version")
if actual_abi != sys.argv[2]:
    raise SystemExit("runtime/profile ABI mismatch")
def version(value):
    return tuple(int(part) for part in re.findall(r"\d+", value or ""))
if not actual_version or version(actual_version) < version(sys.argv[3]):
    raise SystemExit("runtime/profile version mismatch")
PY
SANITY="$(sed -n 's/^  sanity_command: //p' "${PROFILE_FILE}" | head -n1)"
WARMUP="$(sed -n 's/^  warmup_command: //p' "${PROFILE_FILE}" | head -n1)"
PARITY="$(sed -n 's/^  parity_command: //p' "${PROFILE_FILE}" | head -n1)"
HEALTHCHECK="$(sed -n 's/^  healthcheck_command: //p' "${PROFILE_FILE}" | head -n1)"
for VALUE in "${SANITY}" "${WARMUP}" "${PARITY}" "${HEALTHCHECK}"; do
  [[ -n "${VALUE}" && "${VALUE}" != "null" ]] || { echo "blocked_external: sanity/warmup/parity command unresolved" >&2; exit 78; }
done

BUNDLE_ID="$(python3 - "${SOURCE}/bundle_manifest.json" <<'PY'
import json, sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
if data.get("target_family") != "journey6" or data.get("target_sku") != "auto" or data.get("target_march") != "auto":
    raise SystemExit("invalid bundle target")
if data.get("status") == "skeleton" or data.get("external_blockers"):
    raise SystemExit("blocked_external: skeleton bundle cannot be deployed")
print(data["bundle_id"])
PY
)"
RELEASES="${PREFIX}/releases"
RELEASE="${RELEASES}/${BUNDLE_ID}"
mkdir -p "${RELEASES}"
[[ ! -e "${RELEASE}" ]] || { echo "release already exists: ${RELEASE}" >&2; exit 73; }
mv "${SOURCE}" "${RELEASE}"
ln -sfn "${RELEASE}" "${PREFIX}/candidate.new"
mv -Tf "${PREFIX}/candidate.new" "${PREFIX}/candidate"

OLD_ACTIVE=""
if [[ -L "${PREFIX}/active" ]]; then
  OLD_ACTIVE="$(readlink -f "${PREFIX}/active")"
  ln -sfn "${OLD_ACTIVE}" "${PREFIX}/last-known-good.new"
  mv -Tf "${PREFIX}/last-known-good.new" "${PREFIX}/last-known-good"
fi
mkdir -p "${PREFIX}/evidence"
write_evidence() {
  local status="$1"
  local restored="$2"
  local inventory_sha
  inventory_sha="$(sha256sum "${INVENTORY}" | awk '{print $1}')"
  python3 - "${PREFIX}/evidence/deployment-${BUNDLE_ID}.json" "${status}" "${BUNDLE_ID}" "${PROFILE}" "${ACTUAL_MARCH}" "${inventory_sha}" "${RELEASE}" "${restored}" <<'PY'
import datetime, json, sys
path, status, bundle, profile, march, inventory_sha, candidate, restored = sys.argv[1:]
data = {
    "schema_version": 1,
    "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "status": status,
    "bundle_id": bundle,
    "profile": profile,
    "actual_march": march,
    "inventory_sha256": inventory_sha,
    "candidate": candidate,
    "restored_release": restored or None,
}
open(path, "w", encoding="utf-8").write(json.dumps(data, indent=2) + "\n")
PY
}
rollback() {
  if [[ -n "${OLD_ACTIVE}" ]]; then
    ln -sfn "${OLD_ACTIVE}" "${PREFIX}/active.rollback"
    mv -Tf "${PREFIX}/active.rollback" "${PREFIX}/active"
    systemctl restart tzcup-j6-runtime tzcup-j6-perception tzcup-j6-autonomy tzcup-j6-health || true
  elif [[ -L "${PREFIX}/active" ]]; then
    unlink "${PREFIX}/active"
  fi
  write_evidence "rolled_back" "${OLD_ACTIVE}" || true
}
trap rollback ERR
(cd "${RELEASE}" && sh -c "${SANITY}")
(cd "${RELEASE}" && sh -c "${WARMUP}")
(cd "${RELEASE}" && sh -c "${PARITY}")
(cd "${RELEASE}" && TZCUP_J6_CANDIDATE="${RELEASE}" sh -c "${HEALTHCHECK}")
install -m 0644 "${RELEASE}"/systemd/*.service /etc/systemd/system/
systemctl daemon-reload
ln -sfn "${RELEASE}" "${PREFIX}/active.new"
mv -Tf "${PREFIX}/active.new" "${PREFIX}/active"
systemctl restart tzcup-j6-runtime tzcup-j6-perception tzcup-j6-autonomy tzcup-j6-health
(cd "${PREFIX}/active" && TZCUP_J6_ROOT="${PREFIX}" sh -c "${HEALTHCHECK}")
trap - ERR
write_evidence "deployed" "${OLD_ACTIVE}"
echo "deployed ${BUNDLE_ID}; rollback point: ${OLD_ACTIVE:-none}"
