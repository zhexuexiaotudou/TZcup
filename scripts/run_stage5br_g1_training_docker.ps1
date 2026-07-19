param(
  [string]$DatasetName = "stage5br_g1_smoke50_clean",
  [string]$OutputName = "stage5br_g1_model_screening"
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$image = "tzcup/sanitation-jazzy:stage5b"
$dataset = "/stage5br/artifacts/$DatasetName"
$output = "/stage5br/artifacts/$OutputName"
New-Item -ItemType Directory -Force -Path (Join-Path $packRoot "artifacts\$OutputName") | Out-Null
docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env PYTHONPATH=/stage5br/starter_ws/src/sanitation_learning `
  --volume "${packRoot}:/stage5br" --workdir /stage5br $image `
  python3 -m sanitation_learning.g1_training `
  --dataset $dataset `
  --config /stage5br/starter_ws/src/sanitation_learning/config/stage5br_g1_training.yaml `
  --output $output
if ($LASTEXITCODE -ne 0) { throw "Stage5BR G1 model screening failed: $LASTEXITCODE" }
Write-Output (Join-Path $packRoot "artifacts\$OutputName")
