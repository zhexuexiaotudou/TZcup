$ErrorActionPreference = "Stop"

$packRoot = Split-Path -Parent $PSScriptRoot
$workspace = Get-ChildItem -LiteralPath (Join-Path $packRoot ".work") -Directory |
    Where-Object Name -Like "stage1_*" |
    Sort-Object Name -Descending |
    Select-Object -First 1

if (-not $workspace) {
    throw "No Stage 1 workspace found"
}

docker run --rm --gpus all `
    --shm-size 1g `
    --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env "SANITATION_WS=/work/.work/$($workspace.Name)" `
    --volume "${packRoot}:/work" `
    --workdir /work `
    tzcup/sanitation-jazzy:stage0 `
    bash scripts/stage4_ci.sh

if ($LASTEXITCODE -ne 0) {
    throw "Stage 4 container gate failed with exit code $LASTEXITCODE"
}
