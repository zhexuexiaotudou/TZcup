param(
  [string]$OutputName = "autonomous_auto03_raw_20260729",
  [switch]$RebuildWorkspace,
  [switch]$SkipBuild,
  [switch]$SkipMatrixGeneration,
  [int]$MaxTrialsPerWorld = 0,
  [int]$TrialOffset = 0,
  [string[]]$WorldIds = @(
    "world_a_asphalt_campus",
    "world_b_concrete_sidewalk",
    "world_c_wet_dark_ground",
    "world_d_mixed_curb_vegetation",
    "world_e_tiled_plaza",
    "world_f_service_road"
  )
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$runtimeRoot = "F:\Project\TZcup"
$image = "tzcup/sanitation-jazzy:stage5b"
$workspaceVolume = "tzcup-autonomous-auto03-ws"
$workspace = "/tmp/tzcup_autonomous_auto03_ws"
$baseWorkspace = "/work/.work/stage1_20260714_154523"
$stage4vWorkspace = "/work/.work/stage4w_20260716"
$matrixRelative = "artifacts/$OutputName/auto03_matrix.json"
$manifestRelative = "artifacts/stage5br3_20260720_review/g2_worlds/g2_world_manifest.json"

docker image inspect $image 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Required image missing: $image" }
if ($RebuildWorkspace) {
  $volumes = docker volume ls --format "{{.Name}}"
  if ($volumes -contains $workspaceVolume) {
    docker volume rm $workspaceVolume | Out-Null
  }
}

if (-not $SkipMatrixGeneration) {
  py -3 "$PSScriptRoot\generate_auto03_matrix.py" `
    --manifest (Join-Path $packRoot ($manifestRelative -replace "/", "\")) `
    --output (Join-Path $packRoot ($matrixRelative -replace "/", "\"))
  if ($LASTEXITCODE -ne 0) { throw "AUTO-03 matrix generation failed" }
}

if (-not $SkipBuild) {
$build = @'
set -euo pipefail
mkdir -p "${SANITATION_WS}/src"
for package in sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_dataset sanitation_ground_truth sanitation_perception_interfaces sanitation_perception sanitation_spot_cleaning sanitation_learning; do
  rm -rf "${SANITATION_WS}/src/${package}"
  cp -a "/auto03/starter_ws/src/${package}" "${SANITATION_WS}/src/${package}"
done
set +u
source /opt/ros/jazzy/setup.bash
source "${SANITATION_BASE_WS}/install/setup.bash"
source "${SANITATION_STAGE4V_WS}/install/setup.bash"
set -u
cd "${SANITATION_WS}"
colcon build --packages-select sanitation_vehicle_description sanitation_bringup sanitation_spot_cleaning
'@
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto03" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto03 $image bash -lc $build
  if ($LASTEXITCODE -ne 0) { throw "AUTO-03 overlay build failed: $LASTEXITCODE" }
}

$allWorldIds = @(
  "world_a_asphalt_campus",
  "world_b_concrete_sidewalk",
  "world_c_wet_dark_ground",
  "world_d_mixed_curb_vegetation",
  "world_e_tiled_plaza",
  "world_f_service_road"
)
foreach ($worldId in $WorldIds) {
  $worldIndex = [array]::IndexOf($allWorldIds, $worldId)
  if ($worldIndex -lt 0) { throw "Unknown AUTO-03 world: $worldId" }
  $domainId = 130 + $worldIndex
  $relative = "$OutputName/$worldId"
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=$domainId --env GZ_PARTITION="auto03_$worldIndex" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env AUTO03_OUT="/auto03/artifacts/$relative" --env AUTO03_WORLD_ID=$worldId `
    --env AUTO03_WORLD_FILE="/auto03/artifacts/stage5br3_20260720_review/g2_worlds/$worldId.sdf" `
    --env AUTO03_MATRIX="/auto03/$matrixRelative" `
    --env AUTO03_MAX_TRIALS=$MaxTrialsPerWorld `
    --env AUTO03_TRIAL_OFFSET=$TrialOffset `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto03" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto03 $image bash scripts/auto03_world_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-03 world $worldId failed: $LASTEXITCODE" }
}

Write-Output (Join-Path $packRoot "artifacts\$OutputName")
