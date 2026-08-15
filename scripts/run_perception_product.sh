#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "usage: $0 /absolute/perception-release" >&2
  exit 2
fi
release="$(realpath "$1")"
test -f "${release}/manifests/perception_pipeline_manifest.yaml"
export TZCUP_PERCEPTION_RELEASE="${release}"
docker compose -f "$(dirname "${BASH_SOURCE[0]}")/../docker/compose.perception-product.yaml" up -d --wait
