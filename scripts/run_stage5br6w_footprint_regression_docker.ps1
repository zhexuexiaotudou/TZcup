param(
  [string]$OutputName = "stage5br6w_runtime/footprint_regression",
  [switch]$RebuildWorkspace,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$image = "tzcup/sanitation-jazzy:stage5b"
$workspaceVolume = "tzcup-stage5br6w-ws"
$workspace = "/tmp/tzcup_stage5br6w_ws"
$baseWorkspace = "/work/.work/stage1_20260714_154523"
# Stage4W's overlay contains the final hybrid-fuser covariance and safety
# configuration; sourcing the older Stage4V workspace would silently regress it.
$stage4vWorkspace = "/work/.work/stage4w_20260716"

docker image inspect $image 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { throw "Required image missing: $image" }
if ($RebuildWorkspace) { docker volume rm $workspaceVolume 2>$null | Out-Null }

if (-not $SkipBuild) {
$build = @'
set -euo pipefail
mkdir -p "${SANITATION_WS}/src"
for package in sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_spot_cleaning sanitation_learning; do
  rm -rf "${SANITATION_WS}/src/${package}"
  cp -a "/stage5br6w/starter_ws/src/${package}" "${SANITATION_WS}/src/${package}"
done
set +u
source /opt/ros/jazzy/setup.bash
source "${SANITATION_BASE_WS}/install/setup.bash"
source "${SANITATION_STAGE4V_WS}/install/setup.bash"
set -u
cd "${SANITATION_WS}"
colcon build --packages-select sanitation_vehicle_description sanitation_worlds sanitation_bringup sanitation_navigation sanitation_tasks sanitation_coverage sanitation_spot_cleaning sanitation_learning
'@
docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
  --volume "F:\Project\TZcup:/work" --volume "${packRoot}:/stage5br6w" --volume "${workspaceVolume}:${workspace}" `
  --workdir /stage5br6w $image bash -lc $build
if ($LASTEXITCODE -ne 0) { throw "Stage5BR6W overlay build failed: $LASTEXITCODE" }
}

function Invoke-Stage4WTrial([string]$Kind, [int]$Seed, [string]$RelativeOutput) {
  $domain = 160 + $Seed + $(if ($Kind -eq "dynamic") { 10 } else { 0 })
  docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env ROS_DOMAIN_ID=$domain --env GZ_PARTITION="stage5br6w_${Kind}_${Seed}" `
    --env SANITATION_BASE_WS=$baseWorkspace --env SANITATION_STAGE4V_WS=$stage4vWorkspace --env SANITATION_WS=$workspace `
    --env STAGE4W_OUT="/stage5br6w/artifacts/$RelativeOutput" --env STAGE4W_SEED=$Seed `
    --env STAGE5BR6W_FOOTPRINT_PROFILE=stage5br6w_v4 --env STAGE5BR6W_CAMERA_PROFILE=V4_engineering `
    --volume "F:\Project\TZcup:/work" --volume "${packRoot}:/stage5br6w" --volume "${workspaceVolume}:${workspace}" `
    --workdir /stage5br6w $image bash "scripts/stage4w_${Kind}_$(if ($Kind -eq 'static') { 'coverage_' } else { '' })ci.sh"
  if ($LASTEXITCODE -ne 0) { throw "Stage5BR6W $Kind seed $Seed failed: $LASTEXITCODE" }
}

$staticRoot = "$OutputName/static"
foreach ($seed in 0..4) { Invoke-Stage4WTrial "static" $seed "$staticRoot/seed_$seed" }
$staticPath = Join-Path $packRoot "artifacts\$staticRoot"
py "$PSScriptRoot\stage4w_static_aggregate.py" $staticPath (Join-Path $staticPath "stage4w_static_matrix_report.json") --required-seeds 5
if ($LASTEXITCODE -ne 0) { throw "Stage5BR6W static matrix failed" }

$profile = Join-Path $packRoot "starter_ws\src\sanitation_navigation\config\stage5br6w_v4_candidate_footprint.yaml"
py "$PSScriptRoot\stage5br6w_footprint_audit.py" --trial (Join-Path $staticPath "seed_0") --profile $profile --output (Join-Path $staticPath "runtime_footprint_audit.json")
if ($LASTEXITCODE -ne 0) { throw "Stage5BR6W runtime footprint audit failed" }

Invoke-Stage4WTrial "dynamic" 10 "$OutputName/dynamic"
Write-Output (Join-Path $packRoot "artifacts\$OutputName")
