param(
  [string]$OutputName = "stage5a_formal",
  [switch]$RebuildImage,
  [switch]$RecordBag
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null

$image = "tzcup/sanitation-jazzy:stage5a"
$imageExists = docker image inspect $image 2>$null
if ($RebuildImage -or $LASTEXITCODE -ne 0) {
  docker build --file (Join-Path $packRoot "docker\Dockerfile.stage5a") --tag $image $packRoot
  if ($LASTEXITCODE -ne 0) { throw "Stage5A image build failed: $LASTEXITCODE" }
}

docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env ROS_DOMAIN_ID=205 --env GZ_PARTITION=stage5a_formal `
  --env SANITATION_BASE_WS=/work/.work/stage1_20260714_154523 `
  --env SANITATION_STAGE4V_WS=/work/.work/stage4v_20260716 `
  --env SANITATION_STAGE4W_WS=/work/.work/stage4w_20260716 `
  --env STAGE5A_WS=/tmp/tzcup_stage5a_ws `
  --env STAGE5A_OUT=/stage5a/artifacts/$OutputName `
  --env STAGE5A_RECORD_BAG=$($RecordBag.IsPresent.ToString().ToLowerInvariant()) `
  --volume "F:\Project\TZcup:/work" --volume "${packRoot}:/stage5a" `
  --volume "tzcup-stage5a-ws:/tmp/tzcup_stage5a_ws" `
  --workdir /stage5a $image bash scripts/stage5a_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage5A formal gate failed: $LASTEXITCODE" }
Write-Output $artifact
