#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tag="${1:-local}"
docker build --pull \
  --file "${root}/docker/Dockerfile.perception-product" \
  --tag "tzcup/perception-product:${tag}" "${root}"
