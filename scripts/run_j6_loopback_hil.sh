#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/docker/compose.journey6-loopback.yaml"
project_name="tzcup-j6-loopback"

require_env() {
  local name="$1"
  if [[ -z "${!name:-}" ]]; then
    echo "Required environment variable is missing: $name" >&2
    exit 2
  fi
}

reject_forbidden_mount() {
  local label="$1"
  local path="$2"
  local lowered="${path,,}"
  case "/$lowered/" in
    *"/ground_truth/"*|*"/world/"*|*"/worlds/"*|*"/sealed/"*|*"/evaluator/"*)
      echo "$label resolves to a forbidden algorithm-container mount: $path" >&2
      exit 2
      ;;
  esac
}

for variable in \
  J6_OE_BASE_IMAGE J6_ROS_SETUP J6_RUNTIME_BUNDLE \
  J6_MODEL_ARTIFACTS J6_ALGORITHM_COMMAND; do
  require_env "$variable"
done

for directory_variable in J6_RUNTIME_BUNDLE J6_MODEL_ARTIFACTS; do
  value="${!directory_variable}"
  if [[ ! -d "$value" ]]; then
    echo "$directory_variable is not an existing directory: $value" >&2
    exit 2
  fi
  reject_forbidden_mount "$directory_variable" "$(realpath "$value")"
done

if [[ -z "${HIL_EVIDENCE_DIR:-}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  HIL_EVIDENCE_DIR="$repo_root/artifacts/j6_loopback_hil_$timestamp"
fi
mkdir -p "$HIL_EVIDENCE_DIR"
HIL_EVIDENCE_DIR="$(realpath "$HIL_EVIDENCE_DIR")"
export HIL_EVIDENCE_DIR

docker compose -f "$compose_file" -p "$project_name" --profile build-only \
  build j6-oe-wrapper
docker compose -f "$compose_file" -p "$project_name" build discovery pc-gateway
docker compose -f "$compose_file" -p "$project_name" build j6-algorithm
docker compose -f "$compose_file" -p "$project_name" up -d \
  discovery pc-gateway j6-algorithm

sleep 2
docker compose -f "$compose_file" -p "$project_name" exec -T j6-algorithm \
  /bin/bash -lc 'ps -ef > /evidence/HIL_J6_PROCESS_LIST.txt'
ps -eo pid,ppid,args > "$HIL_EVIDENCE_DIR/HIL_PC_PROCESS_LIST.txt"

docker compose -f "$compose_file" -p "$project_name" ps

cat > "$HIL_EVIDENCE_DIR/HIL_PC_DDS_ENV.sh" <<EOF
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-66}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF

echo "Journey 6 loopback infrastructure started."
echo "Evidence: $HIL_EVIDENCE_DIR"
echo "Before starting the PC sensor/plant-only graph, run:"
echo "  source '$HIL_EVIDENCE_DIR/HIL_PC_DDS_ENV.sh'"
echo "Then publish a fresh healthy /hil/health frame and an operator resume."
