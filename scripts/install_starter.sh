#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${SANITATION_WS:-$HOME/sanitation_ws}"

mkdir -p "$WS/src"
rsync -a "$PACK_ROOT/starter_ws/src/" "$WS/src/"
echo "Starter packages installed into $WS/src"
