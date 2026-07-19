param(
  [string]$OutputName = "stage5br_g1_smoke",
  [int]$SceneCount = 50,
  [int]$FramesPerScene = 10,
  [int]$RosDomainId = 219
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null
$image = "tzcup/sanitation-jazzy:stage5b"
$partition = "stage5br_" + ($OutputName -replace '[^A-Za-z0-9_]', '_')
docker image inspect $image 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { throw "missing Docker image: $image" }

docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env ROS_DOMAIN_ID=$RosDomainId --env GZ_PARTITION=$partition `
  --env STAGE5BR_G1_OUT=/stage5br/artifacts/$OutputName `
  --env STAGE5BR_G1_SCENES=$SceneCount `
  --env STAGE5BR_G1_FRAMES_PER_SCENE=$FramesPerScene `
  --volume "${packRoot}:/stage5br" `
  --workdir /stage5br $image bash scripts/stage5br_g1_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage5BR G1 gate failed: $LASTEXITCODE" }
Write-Output $artifact
