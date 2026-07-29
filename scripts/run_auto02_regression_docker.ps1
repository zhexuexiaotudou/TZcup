param(
  [string]$OutputName = "autonomous_auto02_runtime",
  [switch]$RebuildWorkspace,
  [switch]$SkipBuild,
  [switch]$SkipCold,
  [switch]$SkipStatic,
  [switch]$SkipDynamic,
  [switch]$ReuseCompletedStatic,
  [int]$ColdTrialStart = 0,
  [int]$ColdTrialCount = 5,
  [int[]]$StaticSeeds = @(0, 1, 2, 3, 4)
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$runtimeRoot = "F:\Project\TZcup"
$image = "tzcup/sanitation-jazzy:stage5b"
$workspaceVolume = "tzcup-autonomous-auto02-ws"
$workspace = "/tmp/tzcup_autonomous_auto02_ws"
$baseWorkspace = "/work/.work/stage1_20260714_154523"
$stage4vWorkspace = "/work/.work/stage4w_20260716"
$profile = "auto01_g2_v5_retracted"
$camera = "V5_retracted"

function Assert-RosDomainId {
  param([int]$DomainId)
  if ($DomainId -lt 0 -or $DomainId -gt 232) {
    throw "ROS_DOMAIN_ID must be in the Fast DDS-safe range 0..232: $DomainId"
  }
}

docker image inspect $image 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Required image missing: $image" }
if ($RebuildWorkspace) {
  $volumes = docker volume ls --format "{{.Name}}"
  if ($volumes -contains $workspaceVolume) {
    docker volume rm $workspaceVolume | Out-Null
  }
}

if (-not $SkipBuild) {
$build = @'
set -euo pipefail
mkdir -p "${SANITATION_WS}/src"
for package in sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_dataset sanitation_ground_truth sanitation_perception_interfaces sanitation_perception sanitation_spot_cleaning sanitation_learning; do
  rm -rf "${SANITATION_WS}/src/${package}"
  cp -a "/auto02/starter_ws/src/${package}" "${SANITATION_WS}/src/${package}"
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
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto02" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto02 $image bash -lc $build
  if ($LASTEXITCODE -ne 0) { throw "AUTO-02 overlay build failed: $LASTEXITCODE" }
}

if (-not $SkipCold) {
  for ($trial = $ColdTrialStart; $trial -lt ($ColdTrialStart + $ColdTrialCount); $trial++) {
    $relative = "$OutputName/cold_start/trial_$trial"
    $domainId = $trial + 220
    Assert-RosDomainId $domainId
    docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
      --env ROS_DOMAIN_ID=$domainId --env GZ_PARTITION="auto02_cold_$trial" `
      --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
      --env AUTO01_OUT="/auto02/artifacts/$relative" --env AUTO01_SEED=$trial `
      --env AUTO01_STAGE_ID=AUTO-02 --env AUTO01_FOOTPRINT_PROFILE=$profile --env AUTO01_CAMERA_PROFILE=$camera `
      --env AUTO01_ATTEMPT_ID="AUTO-02-COLD-$($trial + 1)" `
      --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto02" --volume "${workspaceVolume}:${workspace}" `
      --workdir /auto02 $image bash scripts/auto01_cold_start_ci.sh
    if ($LASTEXITCODE -ne 0) { throw "AUTO-02 cold start $trial failed: $LASTEXITCODE" }
  }
}

if (-not $SkipStatic) {
  foreach ($seed in $StaticSeeds) {
    $relative = "$OutputName/static/seed_$seed"
    $domainId = $seed + 180
    Assert-RosDomainId $domainId
    $trialRoot = Join-Path $packRoot "artifacts\$relative"
    if ($ReuseCompletedStatic) {
      $summaryPath = Join-Path $trialRoot "stage4w_static_summary.json"
      $replayPath = Join-Path $trialRoot "auto02_replay_audit.json"
      $geometryPath = Join-Path $trialRoot "auto02_runtime_geometry_audit.json"
      if ((Test-Path $summaryPath) -and (Test-Path $replayPath) -and (Test-Path $geometryPath)) {
        $summary = Get-Content -LiteralPath $summaryPath -Raw | ConvertFrom-Json
        $replay = Get-Content -LiteralPath $replayPath -Raw | ConvertFrom-Json
        $geometry = Get-Content -LiteralPath $geometryPath -Raw | ConvertFrom-Json
        if ($summary.static_gate_pass -and $replay.replay_gate_pass -and $geometry.runtime_geometry_gate_pass) {
          Write-Output "Reusing completed AUTO-02 static seed $seed"
          continue
        }
      }
    }
    docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
      --env ROS_DOMAIN_ID=$domainId --env GZ_PARTITION="auto02_static_$seed" `
      --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
      --env STAGE4W_OUT="/auto02/artifacts/$relative" --env STAGE4W_SEED=$seed `
      --env STAGE5BR6W_FOOTPRINT_PROFILE=$profile --env STAGE5BR6W_CAMERA_PROFILE=$camera `
      --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto02" --volume "${workspaceVolume}:${workspace}" `
      --workdir /auto02 $image bash scripts/stage4w_static_coverage_ci.sh
    if ($LASTEXITCODE -ne 0) { throw "AUTO-02 static seed $seed failed: $LASTEXITCODE" }
    py -3 "$PSScriptRoot\auto01_runtime_audit.py" `
      --trial $trialRoot `
      --profile (Join-Path $packRoot "starter_ws\src\sanitation_navigation\config\auto01_g2_v5_retracted.yaml") `
      --output (Join-Path $trialRoot "auto02_runtime_geometry_audit.json") `
      --stage AUTO-02
    if ($LASTEXITCODE -ne 0) { throw "AUTO-02 runtime geometry audit seed $seed failed" }
  }
  $staticRoot = Join-Path $packRoot "artifacts\$OutputName\static"
  py -3 "$PSScriptRoot\stage4w_static_aggregate.py" $staticRoot `
    (Join-Path $staticRoot "stage4w_static_matrix_report.json") `
    --required-seeds 5
  if ($LASTEXITCODE -ne 0) { throw "AUTO-02 static aggregate failed" }
}

if (-not $SkipDynamic) {
  $relative = "$OutputName/dynamic"
  $domainId = 190
  Assert-RosDomainId $domainId
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=$domainId --env GZ_PARTITION=auto02_dynamic `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env STAGE4W_OUT="/auto02/artifacts/$relative" --env STAGE4W_SEED=10 `
    --env STAGE5BR6W_FOOTPRINT_PROFILE=$profile --env STAGE5BR6W_CAMERA_PROFILE=$camera `
    --volume "${runtimeRoot}:/work" --volume "${packRoot}:/auto02" --volume "${workspaceVolume}:${workspace}" `
    --workdir /auto02 $image bash scripts/stage4w_dynamic_ci.sh
  if ($LASTEXITCODE -ne 0) { throw "AUTO-02 dynamic gate failed: $LASTEXITCODE" }
}

$root = Join-Path $packRoot "artifacts\$OutputName"
if ((Test-Path (Join-Path $root "static\stage4w_static_matrix_report.json")) -and `
    (Test-Path (Join-Path $root "dynamic\stage4w_dynamic_report.json")) -and `
    (Test-Path (Join-Path $root "cold_start\trial_4\cold_start_report.json"))) {
  py -3 "$PSScriptRoot\auto02_acceptance.py" --root $root `
    --output (Join-Path $root "auto02_acceptance_report.json")
  if ($LASTEXITCODE -ne 0) { throw "AUTO-02 acceptance gate failed" }
}
Write-Output $root
