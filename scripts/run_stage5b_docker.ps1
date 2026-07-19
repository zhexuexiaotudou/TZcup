param(
  [string]$OutputName = "stage5b_screening",
  [switch]$RebuildImage,
  [switch]$FormalDataset
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null

$image = "tzcup/sanitation-jazzy:stage5b"
docker image inspect $image 2>$null | Out-Null
if ($RebuildImage -or $LASTEXITCODE -ne 0) {
  docker build --file (Join-Path $packRoot "docker\Dockerfile.stage5b") --tag $image $packRoot
  if ($LASTEXITCODE -ne 0) { throw "Stage5B image build failed: $LASTEXITCODE" }
}

docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env ROS_DOMAIN_ID=215 --env GZ_PARTITION=stage5b_gate `
  --env STAGE5B_WS=/tmp/tzcup_stage5b_ws `
  --env STAGE5B_OUT=/stage5b/artifacts/$OutputName `
  --env STAGE5B_DATA=/stage5b_data/$OutputName `
  --env STAGE5B_FORMAL_DATASET=$($FormalDataset.IsPresent.ToString().ToLowerInvariant()) `
  --volume "${packRoot}:/stage5b" `
  --volume "tzcup-stage5b-ws:/tmp/tzcup_stage5b_ws" `
  --volume "tzcup-stage5b-data:/stage5b_data" `
  --workdir /stage5b $image bash scripts/stage5b_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage5B offline gate failed: $LASTEXITCODE" }
Write-Output $artifact
