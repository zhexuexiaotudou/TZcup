#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_WS="${SANITATION_BASE_WS:?SANITATION_BASE_WS required}"
STAGE4V_WS="${SANITATION_STAGE4V_WS:?SANITATION_STAGE4V_WS required}"
STAGE4W_WS="${SANITATION_STAGE4W_WS:?SANITATION_STAGE4W_WS required}"
WS="${STAGE5A_WS:?STAGE5A_WS required}"
OUT="${STAGE5A_OUT:?STAGE5A_OUT required}"
mkdir -p "${OUT}" "${WS}/src"

set +u
source /opt/ros/jazzy/setup.bash
source "${BASE_WS}/install/setup.bash"
source "${STAGE4V_WS}/install/setup.bash"
source "${STAGE4W_WS}/install/setup.bash"
set -u

packages=(
  sanitation_perception_interfaces
  sanitation_perception
  sanitation_ground_truth
  sanitation_dataset
  sanitation_spot_cleaning
  sanitation_worlds
)
for package in "${packages[@]}"; do
  rm -rf "${WS}/src/${package}"
  cp -a "${PACK_ROOT}/starter_ws/src/${package}" "${WS}/src/${package}"
done

cd "${WS}"
colcon build --merge-install --packages-select "${packages[@]}" \
  > "${OUT}/colcon_build.log" 2>&1
set +u
source "${WS}/install/setup.bash"
set -u
colcon test --merge-install --packages-select "${packages[@]}" --event-handlers console_direct+ \
  > "${OUT}/colcon_test.log" 2>&1
colcon test-result --all --verbose > "${OUT}/colcon_test_results.txt"
grep -Eq '0 errors, 0 failures' "${OUT}/colcon_test_results.txt"

dataset_root="${OUT}/dataset_smoke"
model_path="${OUT}/stage5a_synthetic_color_prototype.onnx"
ros2 run sanitation_dataset stage5a_generate_dataset --output "${dataset_root}" --scene-count 20 \
  > "${OUT}/dataset_generation.log"
ros2 run sanitation_dataset stage5a_build_onnx --output "${model_path}" > "${OUT}/model_build.log"
ros2 run sanitation_dataset stage5a_evaluate_onnx --model "${model_path}" \
  --output "${OUT}/synthetic_perception_report.json" --seeds 16,17,18,19 \
  > "${OUT}/synthetic_evaluation.log"
ros2 run sanitation_spot_cleaning stage5a_spot_clean_evaluator --model "${model_path}" \
  --output "${OUT}/spot_clean_e2e_report.json" --start-seed 100 --trial-count 30 \
  > "${OUT}/spot_clean_evaluation.log"
ros2 run sanitation_perception stage5a_backend_probe --backend horizon_j6 --expect-failure \
  > "${OUT}/j6_fail_closed.json"

python3 - "${dataset_root}" "${model_path}" "${OUT}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

dataset = Path(sys.argv[1])
model = Path(sys.argv[2])
out = Path(sys.argv[3])
manifest = json.loads((dataset / 'dataset_manifest.json').read_text(encoding='utf-8'))
calibration = json.loads((dataset / 'calibration_dataset_manifest.json').read_text(encoding='utf-8'))
perception = json.loads((out / 'synthetic_perception_report.json').read_text(encoding='utf-8'))
spot = json.loads((out / 'spot_clean_e2e_report.json').read_text(encoding='utf-8'))
report = {
    'schema_version': 1,
    'stage': 'Stage5A',
    'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
    'dataset_scene_count': manifest['scene_count'],
    'dataset_split_hash': manifest['split_hash'],
    'duplicate_image_count': manifest['duplicate_image_count'],
    'calibration_scene_count': len(calibration['representative_scene_seeds']),
    'synthetic_perception_pass': perception['synthetic_perception_pass'],
    'spot_clean_e2e_pass': spot['spot_clean_e2e_pass'],
    'competition_perception_pass': False,
    'j6_toolchain_available': False,
    'j6_quantization_pass': False,
    'j6_runtime_pass': False,
    'competition_efficiency_pass': False,
    'theoretical_efficiency_m2_h': 1053.0,
    'target_efficiency_m2_h': 3500.0,
}
(out / 'stage5a_offline_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
assert report['dataset_scene_count'] >= 20
assert report['duplicate_image_count'] == 0
assert report['synthetic_perception_pass']
assert report['spot_clean_e2e_pass']
PY

cd "${PACK_ROOT}"
bash scripts/stage5a_live_smoke_ci.sh "${model_path}" "${OUT}"
