param(
  [string]$OutputName = "autonomous_auto01_g1_runtime",
  [switch]$RebuildWorkspace,
  [switch]$SkipBuild,
  [switch]$SkipCold,
  [switch]$SkipFormal,
  [switch]$RunHeightBand,
  [switch]$RunG2Obstacle,
  [int]$HeightBandTrials = 30,
  [int]$ObstacleTrials = 30,
  [int]$ColdTrialStart = 0,
  [int]$ColdTrialCount = 3,
  [string]$FootprintProfile = "auto01_g1_height_banded",
  [string]$CameraProfile = "V4_engineering",
  [string]$AttemptId = "AUTO-01-G1-C3"
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$runtimeRoot = "F:\Project\TZcup"
$image = "tzcup/sanitation-jazzy:stage5b"
$workspaceVolume = "tzcup-autonomous-auto01-ws"
$workspace = "/tmp/tzcup_autonomous_auto01_ws"
$baseWorkspace = "/work/.work/stage1_20260714_154523"
$stage4vWorkspace = "/work/.work/stage4w_20260716"

docker image inspect $image 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Required image missing: $image" }
if ($RebuildWorkspace) {
  $existingVolumes = docker volume ls --format "{{.Name}}"
  if ($existingVolumes -contains $workspaceVolume) {
    docker volume rm $workspaceVolume | Out-Null
  }
}

if (-not $SkipBuild) {
$build = @'
set -euo pipefail
mkdir -p "${SANITATION_WS}/src"
for package in sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_dataset sanitation_ground_truth sanitation_perception_interfaces sanitation_perception sanitation_spot_cleaning sanitation_learning; do
  rm -rf "${SANITATION_WS}/src/${package}"
  cp -a "/auto01/starter_ws/src/${package}" "${SANITATION_WS}/src/${package}"
done
set +u
source /opt/ros/jazzy/setup.bash
source "${SANITATION_BASE_WS}/install/setup.bash"
source "${SANITATION_STAGE4V_WS}/install/setup.bash"
set -u
cd "${SANITATION_WS}"
colcon build --packages-select sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_dataset sanitation_ground_truth sanitation_perception_interfaces sanitation_perception sanitation_spot_cleaning sanitation_learning
'@
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto01" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto01 $image bash -lc $build
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 overlay build failed: $LASTEXITCODE" }
}

if (-not $SkipCold) {
for ($trial = $ColdTrialStart; $trial -lt ($ColdTrialStart + $ColdTrialCount); $trial++) {
  $domain = 180 + $trial
  $relativeOutput = "$OutputName/cold_start/trial_$trial"
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=$domain --env GZ_PARTITION="auto01_cold_$trial" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env AUTO01_OUT="/auto01/artifacts/$relativeOutput" --env AUTO01_SEED=$trial `
    --env AUTO01_FOOTPRINT_PROFILE=$FootprintProfile --env AUTO01_CAMERA_PROFILE=$CameraProfile `
    --env AUTO01_ATTEMPT_ID="$AttemptId-STARTUP-C$($trial + 1)" `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto01" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto01 $image bash scripts/auto01_cold_start_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 cold start $trial failed: $LASTEXITCODE" }
}
}

if (-not $SkipFormal) {
  $relativeOutput = "$OutputName/formal_seed0"
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=190 --env GZ_PARTITION="auto01_formal_seed0" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env STAGE4W_OUT="/auto01/artifacts/$relativeOutput" --env STAGE4W_SEED=0 `
    --env STAGE5BR6W_FOOTPRINT_PROFILE=$FootprintProfile --env STAGE5BR6W_CAMERA_PROFILE=$CameraProfile `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto01" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto01 $image bash scripts/stage4w_static_coverage_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 formal seed0 failed: $LASTEXITCODE" }

  $trialPath = Join-Path $packRoot "artifacts\$relativeOutput"
  $profile = Join-Path $packRoot "starter_ws\src\sanitation_navigation\config\$FootprintProfile.yaml"
  py -3 "$PSScriptRoot\auto01_runtime_audit.py" --trial $trialPath --profile $profile `
    --output (Join-Path $trialPath "auto01_runtime_geometry_audit.json")
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 runtime geometry audit failed" }
}

if ($RunHeightBand) {
  if ($FootprintProfile -ne "auto01_g1_height_banded") {
    throw "The current height-band harness is specific to G1"
  }
  $relativeOutput = "$OutputName/height_band"
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=195 --env GZ_PARTITION="auto01_height_band" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env AUTO01_OUT="/auto01/artifacts/$relativeOutput" --env AUTO01_HEIGHT_TRIALS=$HeightBandTrials `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto01" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto01 $image bash scripts/auto01_height_band_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 height-band gate failed: $LASTEXITCODE" }
}

if ($RunG2Obstacle) {
  if ($FootprintProfile -ne "auto01_g2_v5_retracted") {
    throw "The G2 obstacle harness requires auto01_g2_v5_retracted"
  }
  $relativeOutput = "$OutputName/g2_obstacles"
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=196 --env GZ_PARTITION="auto01_g2_obstacles" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env AUTO01_OUT="/auto01/artifacts/$relativeOutput" --env AUTO01_OBSTACLE_TRIALS=$ObstacleTrials `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto01" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto01 $image bash scripts/auto01_g2_obstacle_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-01 G2 obstacle gate failed: $LASTEXITCODE" }
}

Write-Output (Join-Path $packRoot "artifacts\$OutputName")
