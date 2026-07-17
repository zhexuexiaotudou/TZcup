param(
  [int]$Seed = 0,
  [string]$OutputName = "stage4w_static_seed0"
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null
docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env ROS_DOMAIN_ID=$($Seed + 150) --env GZ_PARTITION=stage4w_static_$Seed `
  --env SANITATION_BASE_WS=/work/.work/stage1_20260714_154523 `
  --env SANITATION_STAGE4V_WS=/work/.work/stage4v_20260716 `
  --env SANITATION_WS=/work/.work/stage4w_20260716 `
  --env STAGE4W_OUT=/stage4w/artifacts/$OutputName --env STAGE4W_SEED=$Seed `
  --volume "F:\Project\TZcup:/work" --volume "${packRoot}:/stage4w" `
  --workdir /stage4w tzcup/sanitation-jazzy:stage0 `
  bash scripts/stage4w_static_coverage_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage4W static coverage failed: $LASTEXITCODE" }
Write-Output $artifact
