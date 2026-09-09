#!/usr/bin/env bash
set -euo pipefail

ROOT="${TZCUP_J6_ROOT:-/opt/tzcup/journey6}"
if [[ -n "${TZCUP_J6_CANDIDATE:-}" ]]; then
  ACTIVE="$(readlink -f "${TZCUP_J6_CANDIDATE}")"
else
  test -L "${ROOT}/active"
  ACTIVE="$(readlink -f "${ROOT}/active")"
fi
test -f "${ACTIVE}/bundle_manifest.json"
test -f "${ACTIVE}/SHA256SUMS"
(cd "${ACTIVE}" && sha256sum -c SHA256SUMS >/dev/null)

if [[ -z "${TZCUP_J6_CANDIDATE:-}" ]]; then
  for service in tzcup-j6-runtime tzcup-j6-perception tzcup-j6-autonomy tzcup-j6-health; do
    systemctl is-active --quiet "${service}.service"
  done
fi
