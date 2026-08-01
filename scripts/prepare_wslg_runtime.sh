#!/usr/bin/env bash
set -Eeuo pipefail

SHARED_MEMORY_DIR="/mnt/shared_memory"
FSTAB_ENTRY="tmpfs ${SHARED_MEMORY_DIR} tmpfs rw,nosuid,nodev,mode=1777 0 0"

if [[ "${EUID}" -ne 0 ]]; then
  echo "prepare_wslg_runtime.sh must run as root." >&2
  exit 2
fi
if [[ ! -d /mnt/wslg ]]; then
  echo "WSLg runtime is unavailable at /mnt/wslg." >&2
  exit 3
fi

mkdir -p "${SHARED_MEMORY_DIR}"
if mountpoint -q "${SHARED_MEMORY_DIR}"; then
  filesystem_type="$(findmnt -n -o FSTYPE --target "${SHARED_MEMORY_DIR}")"
  if [[ "${filesystem_type}" != "tmpfs" ]]; then
    echo "${SHARED_MEMORY_DIR} is already mounted as ${filesystem_type}; refusing to replace it." >&2
    exit 4
  fi
else
  mount -t tmpfs -o rw,nosuid,nodev,mode=1777 tmpfs "${SHARED_MEMORY_DIR}"
fi

if ! awk '$2 == "/mnt/shared_memory" { found=1 } END { exit !found }' /etc/fstab; then
  printf '\n# TZcup WSLg RemoteApp shared-memory recovery\n%s\n' "${FSTAB_ENTRY}" \
    >> /etc/fstab
fi

probe="${SHARED_MEMORY_DIR}/.tzcup_wslg_probe_$$"
: > "${probe}"
rm -f "${probe}"

if grep -q 'rdp_allocate_shared_memory: Failed to open' /mnt/wslg/weston.log 2>/dev/null; then
  printf '{"schema_version":1,"shared_memory":"%s","filesystem":"%s","persistent_fstab":true,"wslg_restart_required":true}\n' \
    "${SHARED_MEMORY_DIR}" \
    "$(findmnt -n -o FSTYPE --target "${SHARED_MEMORY_DIR}")"
  exit 10
fi

printf '{"schema_version":1,"shared_memory":"%s","filesystem":"%s","persistent_fstab":true}\n' \
  "${SHARED_MEMORY_DIR}" \
  "$(findmnt -n -o FSTYPE --target "${SHARED_MEMORY_DIR}")"
