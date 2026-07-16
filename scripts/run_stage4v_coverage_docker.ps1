param([string]$OutputName = "stage4v_coverage_20260716")

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null
docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env ROS_DOMAIN_ID=130 --env GZ_PARTITION=stage4v_coverage `
  --env SANITATION_BASE_WS=/work/.work/stage1_20260714_154523 `
  --env SANITATION_WS=/work/.work/stage4v_20260716 `
  --env STAGE4V_OUT=/stage4v/artifacts/$OutputName `
  --volume "F:\Project\TZcup:/work" --volume "${packRoot}:/stage4v" `
  --workdir /stage4v tzcup/sanitation-jazzy:stage0 `
  bash scripts/stage4v_coverage_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage4V coverage gate failed: $LASTEXITCODE" }
Write-Output $artifact
