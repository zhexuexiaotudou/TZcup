#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/docker/compose.journey6-loopback.yaml"
project_name="tzcup-j6-loopback"
runtime_backend="${HIL_RUNTIME_BACKEND:-JOURNEY6_OE}"
duration_s="${HIL_DURATION_S:-30}"
sensor_source="${HIL_SENSOR_SOURCE:-synthetic_transport_probe}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-backend) runtime_backend="$2"; shift 2 ;;
    --duration|--duration-seconds) duration_s="$2"; shift 2 ;;
    --sensor-source) sensor_source="$2"; shift 2 ;;
    --evidence) HIL_EVIDENCE_DIR="$2"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ "$runtime_backend" != "JOURNEY6_OE" && "$runtime_backend" != "PC_ONNX" ]]; then
  echo "--runtime-backend must be JOURNEY6_OE or PC_ONNX" >&2
  exit 2
fi
if [[ "$sensor_source" != "synthetic_transport_probe" && "$sensor_source" != "gazebo" ]]; then
  echo "--sensor-source must be synthetic_transport_probe or gazebo" >&2
  exit 2
fi
if [[ ! "$duration_s" =~ ^[0-9]+$ ]] || (( duration_s < 10 || duration_s > 86400 )); then
  echo "--duration-seconds must be an integer within 10..86400" >&2
  exit 2
fi

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

if [[ "$runtime_backend" == "JOURNEY6_OE" ]]; then
  for variable in J6_OE_BASE_IMAGE J6_ROS_SETUP J6_RUNTIME_BUNDLE \
    J6_MODEL_ARTIFACTS J6_ALGORITHM_COMMAND; do
    require_env "$variable"
  done
  directory_variables=(J6_RUNTIME_BUNDLE J6_MODEL_ARTIFACTS)
else
  for variable in J6_MODEL_ARTIFACTS PC_ONNX_MODEL_FILENAME \
    PC_ONNX_MODEL_ID PC_ONNX_MODEL_SHA256; do
    require_env "$variable"
  done
  directory_variables=(J6_MODEL_ARTIFACTS)
  model_path="$J6_MODEL_ARTIFACTS/$PC_ONNX_MODEL_FILENAME"
  if [[ ! -f "$model_path" ]]; then
    echo "PC_ONNX model is missing: $model_path" >&2
    exit 2
  fi
  actual_sha="$(sha256sum "$model_path" | awk '{print $1}')"
  if [[ "$actual_sha" != "${PC_ONNX_MODEL_SHA256,,}" ]]; then
    echo "PC_ONNX model SHA-256 mismatch" >&2
    exit 2
  fi
  export PC_ONNX_REQUIRED_MODEL_ID="${PC_ONNX_REQUIRED_MODEL_ID:-d1_littercam_yolov9c}"
  export HIL_APPLY_NETWORK_FAULTS="${HIL_APPLY_NETWORK_FAULTS:-true}"
fi

for directory_variable in "${directory_variables[@]}"; do
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
export HIL_DURATION_S="$duration_s"
export HIL_SENSOR_SOURCE="$sensor_source"
export HIL_RUNTIME_BACKEND="$runtime_backend"
HIL_RUN_ID="$(tr -d '\r\n' < /proc/sys/kernel/random/uuid)"
export HIL_RUN_ID
if [[ "$runtime_backend" == "PC_ONNX" ]]; then
  export HIL_NOT_JOURNEY6_RUNTIME=true
else
  export HIL_NOT_JOURNEY6_RUNTIME=false
fi

docker compose -f "$compose_file" -p "$project_name" build discovery pc-gateway
if [[ "$runtime_backend" == "JOURNEY6_OE" ]]; then
  docker compose -f "$compose_file" -p "$project_name" --profile build-only \
    build j6-oe-wrapper
  docker compose -f "$compose_file" -p "$project_name" --profile journey6 \
    build j6-algorithm
  docker compose -f "$compose_file" -p "$project_name" --profile journey6 up -d \
    discovery pc-gateway j6-algorithm
  algorithm_service="j6-algorithm"
else
  docker compose -f "$compose_file" -p "$project_name" --profile pc-onnx \
    build pc-onnx-algorithm pc-harness
  docker compose -f "$compose_file" -p "$project_name" --profile pc-onnx up -d \
    discovery pc-gateway pc-onnx-algorithm pc-harness
  algorithm_service="pc-onnx-algorithm"
fi

sleep 2
docker compose -f "$compose_file" -p "$project_name" exec -T "$algorithm_service" \
  /bin/bash -lc 'ps -ef > /evidence/HIL_J6_PROCESS_LIST.txt'
docker compose -f "$compose_file" -p "$project_name" exec -T \
  -e ROS_SUPER_CLIENT=TRUE pc-gateway /bin/bash -lc '
    source "${TZCUP_ROS_SETUP}"
    source "${TZCUP_WS_SETUP}"
    sleep 5
    {
      echo ROS_SUPER_CLIENT=TRUE
      ros2 topic list --no-daemon -t
      for topic in /hil/camera/color /hil/camera/depth /hil/camera/camera_info /hil/tf /hil/tf_static /hil/vehicle/ackermann_command /hil/vehicle/validated_ackermann_command /hil/health; do
        echo "TOPIC=$topic"
        ros2 topic info -v "$topic"
      done
    } > /evidence/HIL_ROS_QOS_INFO.txt
  '
ps -eo pid,ppid,args > "$HIL_EVIDENCE_DIR/HIL_PC_PROCESS_LIST.txt"

docker compose -f "$compose_file" -p "$project_name" ps

cat > "$HIL_EVIDENCE_DIR/HIL_PC_DDS_ENV.sh" <<EOF
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-66}
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF

echo "Journey 6 loopback infrastructure started."
echo "runtime_backend=$runtime_backend"
echo "run_id=$HIL_RUN_ID"
if [[ "$runtime_backend" == "PC_ONNX" ]]; then
  echo "not_journey6_runtime=true"
  docker compose -f "$compose_file" -p "$project_name" --profile pc-onnx \
    wait pc-harness
  report="$HIL_EVIDENCE_DIR/J6_LOOPBACK_HIL_EMULATION_REPORT.json"
  if [[ ! -f "$report" ]]; then
    echo "PC_ONNX harness exited without its report: $report" >&2
    exit 3
  fi
  echo "PC_ONNX harness report: $report"
fi
echo "Evidence: $HIL_EVIDENCE_DIR"
echo "Before starting the PC sensor/plant-only graph, run:"
echo "  source '$HIL_EVIDENCE_DIR/HIL_PC_DDS_ENV.sh'"
echo "Then publish a fresh healthy /hil/health frame and an operator resume."
