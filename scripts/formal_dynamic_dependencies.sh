#!/usr/bin/env bash
# Print an ldd-compatible dependency list, with a narrowly audited Proot fallback.
set -euo pipefail

[[ "$#" -eq 1 ]] || {
  echo "usage: $0 /absolute/regular-elf" >&2
  exit 2
}
target="$1"
[[ "${target}" = /* ]] || {
  echo "ELF path must be absolute" >&2
  exit 2
}
diagnostic="$(mktemp)"
cleanup() {
  rm -f -- "${diagnostic}"
}
trap cleanup EXIT INT TERM

set +e
/usr/bin/ldd "${target}" 2>"${diagnostic}"
ldd_status=$?
set -e
if (( ldd_status == 0 )); then
  exit 0
fi
if [[ "${FORMAL_NATIVE_LINUX_RUNTIME:-}" != "1" ]] \
  || grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null \
  || ! grep -Fq "you do not have read permission" "${diagnostic}"; then
  cat "${diagnostic}" >&2
  exit "${ldd_status}"
fi
loader=/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2
[[ -f "${loader}" && ! -L "${loader}" && -x "${loader}" ]] || {
  echo "native ELF loader is missing, linked, or not executable" >&2
  exit 125
}
python3 - "${target}" <<'PY'
import os
import stat
import sys

path = sys.argv[1]
try:
    details = os.stat(path, follow_symlinks=False)
except OSError as exc:
    raise SystemExit(f"ELF stat failed: {exc}") from exc
if not stat.S_ISREG(details.st_mode) or os.path.islink(path) or not os.access(path, os.R_OK):
    raise SystemExit("ELF is not a readable regular non-symlink file")
with open(path, "rb") as stream:
    if stream.read(4) != b"\x7fELF":
        raise SystemExit("dependency target is not an ELF file")
PY
exec "${loader}" --list "${target}"
