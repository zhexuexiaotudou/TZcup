#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

world="${GZ_LIGHT_REGRESSION_WORLD:?set GZ_LIGHT_REGRESSION_WORLD}"
log="${GZ_LIGHT_REGRESSION_LOG:?set GZ_LIGHT_REGRESSION_LOG}"
camera_log="${GZ_LIGHT_REGRESSION_CAMERA_LOG:-${log%.log}.camera.log}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_gz_light_regression_$$}"

setsid gz sim -r -s "${world}" >"${log}" 2>&1 &
server_pid=$!
cleanup() {
  kill -INT -- "-${server_pid}" 2>/dev/null || true
  wait "${server_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

service_ready=false
for _ in $(seq 1 60); do
  if gz service --list 2>/dev/null | grep -q "/world/lights_command/light_config"; then
    service_ready=true
    break
  fi
  sleep 0.25
done
if [[ "${service_ready}" != true ]]; then
  echo "light_config service did not become ready" >&2
  exit 1
fi

# The subscription starts Sensors' render thread. Updating a light afterwards
# exercises the exact upstream RenderUtil path from Gazebo issue #3862.
timeout 8 gz topic -e -t /camera >"${camera_log}" 2>&1 &
subscriber_pid=$!
sleep 1
gz service \
  -s /world/lights_command/light_config \
  --reqtype gz.msgs.Light \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "point" type: POINT range: 2.6 attenuation_linear: 0.7 attenuation_constant: 0.6 attenuation_quadratic: 0.001 cast_shadows: true is_light_off: false visualize_visual: false'
sleep 2
kill "${subscriber_pid}" 2>/dev/null || true
wait "${subscriber_pid}" 2>/dev/null || true

if grep -q "Could not find visual for entity: 0" "${log}"; then
  count="$(grep -c "Could not find visual for entity: 0" "${log}")"
  echo "Gazebo RenderUtil entity-0 regression reproduced ${count} time(s)" >&2
  exit 1
fi
echo "Gazebo light-command sensor-render regression passed"
