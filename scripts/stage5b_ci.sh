#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${STAGE5B_WS:?STAGE5B_WS required}"
OUT="${STAGE5B_OUT:?STAGE5B_OUT required}"
DATA="${STAGE5B_DATA:?STAGE5B_DATA required}"
FORMAL="${STAGE5B_FORMAL_DATASET:-false}"
mkdir -p "${OUT}" "${DATA}" "${WS}/src"

set +u
source /opt/ros/jazzy/setup.bash
set -u

packages=(
  sanitation_perception_interfaces
  sanitation_perception
  sanitation_ground_truth
  sanitation_dataset
  sanitation_spot_cleaning
  sanitation_learning
  sanitation_worlds
)
for package in "${packages[@]}"; do
  rm -rf "${WS}/src/${package}"
  cp -a "${PACK_ROOT}/starter_ws/src/${package}" "${WS}/src/${package}"
done

cd "${WS}"
colcon build --merge-install --packages-select "${packages[@]}" > "${OUT}/colcon_build.log" 2>&1
set +u
source "${WS}/install/setup.bash"
set -u
colcon test --merge-install --packages-select "${packages[@]}" --event-handlers console_direct+ > "${OUT}/colcon_test.log" 2>&1
colcon test-result --all --verbose > "${OUT}/colcon_test_results.txt"
grep -Eq '0 errors, 0 failures' "${OUT}/colcon_test_results.txt"

share="$(ros2 pkg prefix --share sanitation_learning)"
registry="${share}/config/asset_registry.yaml"
training="${share}/config/training.yaml"
assets="${DATA}/gazebo_assets"
dataset="${DATA}/dataset"
models="${OUT}/models"
rm -rf "${assets}" "${dataset}" "${models}"

ros2 run sanitation_learning stage5b_generate_assets --registry "${registry}" --output "${assets}" > "${OUT}/asset_generation.log"
if [[ "${FORMAL}" == "true" ]]; then
  scene_count=500
else
  scene_count=50
fi
ros2 run sanitation_learning stage5b_generate_dataset --registry "${registry}" --output "${dataset}" --scene-count "${scene_count}" --frames-per-scene 10 > "${OUT}/dataset_generation.log"
cp "${dataset}/dataset_manifest.json" "${OUT}/dataset_manifest.json"
cp "${dataset}/annotation_qa.json" "${OUT}/annotation_qa.json"
cp "${dataset}/calibration_dataset_manifest.json" "${OUT}/calibration_dataset_manifest.json"
cp "${assets}/generated_asset_manifest.json" "${OUT}/generated_asset_manifest.json"

export CUBLAS_WORKSPACE_CONFIG=:4096:8
STAGE5B_CODE_COMMIT="$(git -C "${PACK_ROOT}" rev-parse HEAD 2>/dev/null || echo working_tree_precommit)" \
  ros2 run sanitation_learning stage5b_train_models --registry "${registry}" --config "${training}" --output "${models}" > "${OUT}/training.log"

set +e
ros2 run sanitation_learning stage5b_evaluate_models --model "${models}/stage5b_learned_perception.onnx" --registry "${registry}" --output "${OUT}/d1_perception_report.json" --test-scenes 100 --frames-per-scene 10 > "${OUT}/evaluation.log" 2>&1
evaluation_exit=$?
set -e
printf '%s\n' "${evaluation_exit}" > "${OUT}/evaluation_exit_code.txt"
cp "${models}/model_selection_report.json" "${OUT}/model_selection_report.json"
cp "${models}/model_card.json" "${OUT}/model_card.json"
cp "${models}/environment_lock.json" "${OUT}/environment_lock.json"
cp "${models}/stage5b_learned_perception.onnx" "${OUT}/stage5b_learned_perception.onnx"
cp "${models}"/*_training_report.json "${OUT}/"

ros2 run sanitation_learning stage5b_j6_preflight --model "${OUT}/stage5b_learned_perception.onnx" --calibration-manifest "${OUT}/calibration_dataset_manifest.json" --output "${OUT}/j6_preflight.json" > "${OUT}/j6_preflight.log"

python3 - "${OUT}" "${scene_count}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
expected_scenes = int(sys.argv[2])
dataset = json.loads((out / "dataset_manifest.json").read_text(encoding="utf-8"))
qa = json.loads((out / "annotation_qa.json").read_text(encoding="utf-8"))
selection = json.loads((out / "model_selection_report.json").read_text(encoding="utf-8"))
perception = json.loads((out / "d1_perception_report.json").read_text(encoding="utf-8"))
j6 = json.loads((out / "j6_preflight.json").read_text(encoding="utf-8"))
summary = {
    "schema_version": 1,
    "stage": "Stage5B offline screening",
    "dataset_scene_count": dataset["scene_count"],
    "dataset_frame_count": dataset["frame_count"],
    "formal_dataset_requested": expected_scenes >= 500,
    "dataset_scale_gate": dataset["scene_count"] >= 500 and dataset["frame_count"] >= 5000,
    "annotation_qa_pass": qa["annotation_qa_pass"],
    "learned_model_exists": selection["selected_model"] == "stage5b_learned_perception.onnx",
    "selected_candidate": selection["selected_candidate"],
    "model_sha256": hashlib.sha256((out / "stage5b_learned_perception.onnx").read_bytes()).hexdigest(),
    "rendered_synthetic_perception_pass": perception["rendered_synthetic_perception_pass"],
    "color_shortcut_pass": perception["color_shortcut"]["color_shortcut_pass"],
    "gazebo_camera_rendered": dataset["gazebo_camera_rendered"],
    "live_gazebo_perception_pass": False,
    "real_nav2_spot_clean_pass": False,
    "stage4w_regression_pass": False,
    "real_domain_evaluation_executed": False,
    "competition_perception_pass": False,
    "j6_toolchain_available": j6["j6_toolchain_available"],
    "j6_model_precheck_pass": j6["j6_model_precheck_pass"],
    "j6_runtime_pass": False,
    "j6_board_fps": None,
    "competition_efficiency_pass": False,
    "theoretical_efficiency_m2_h": 1053.0,
    "target_efficiency_m2_h": 3500.0,
    "READY_FOR_GPT_REVIEW_STAGE5B": False,
    "READY_FOR_STAGE5C": False,
    "first_blocking_layer": "D1_gazebo_camera_rendering_and_model_generalization",
}
(out / "stage5b_offline_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
assert dataset["scene_count"] == expected_scenes
assert qa["annotation_error_rate"] <= 0.01
assert summary["learned_model_exists"]
assert not summary["competition_perception_pass"]
assert not summary["j6_runtime_pass"]
PY
