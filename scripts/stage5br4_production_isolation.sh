#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /work/.work/stage1_20260714_154523/install/setup.bash
source /runtime/install/setup.bash
set -u

ROOT=/stage5br4
OUT=${ROOT}/artifacts/stage5br4_20260720_review/production_isolation
mkdir -p "${OUT}"
XACRO=$(ros2 pkg prefix sanitation_vehicle_description)/share/sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro
xacro "${XACRO}" >"${OUT}/production_default.urdf"
world=${ROOT}/artifacts/stage5br3_20260720_review/g2_worlds/world_a_asphalt_campus.sdf
gz sim -r -s --headless-rendering "${world}" >"${OUT}/production.gz.log" 2>&1 & pid=$!
trap 'kill -INT ${pid} 2>/dev/null || true; sleep 2; kill -TERM ${pid} 2>/dev/null || true' EXIT
for _ in $(seq 1 80); do
  gz service -l 2>/dev/null | grep -q '/world/world_a_asphalt_campus/create' && break
  sleep .25
done
ros2 run ros_gz_sim create -world world_a_asphalt_campus -file "${OUT}/production_default.urdf" \
  -name sanitation_vehicle -x -8 -y 0 -z .18 >"${OUT}/spawn.log" 2>&1
sleep 5
gz topic -l | sort >"${OUT}/runtime_topics.txt"
python3 scripts/stage5br3_production_isolation.py --root "${ROOT}" \
  --rendered-urdf "${OUT}/production_default.urdf" --runtime-topics "${OUT}/runtime_topics.txt" \
  --output "${OUT}/production_isolation_report.json"
