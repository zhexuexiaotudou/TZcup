#!/usr/bin/env bash
set -euo pipefail

output="${1:?output path required}"
python3 "$(dirname "${BASH_SOURCE[0]}")/auto01_capture_collision_params.py" "${output}"
