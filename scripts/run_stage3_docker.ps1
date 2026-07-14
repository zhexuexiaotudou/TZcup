$ErrorActionPreference = "Stop"

$packRoot = Split-Path -Parent $PSScriptRoot
$workRoot = Join-Path $packRoot ".work"
$workspace = Get-ChildItem -LiteralPath $workRoot -Directory |
    Where-Object Name -Like "stage1_*" |
    Sort-Object Name -Descending |
    Select-Object -First 1

if (-not $workspace) {
    throw "No Stage 1 workspace found under $workRoot"
}

$containerWorkspace = "/work/.work/$($workspace.Name)"
docker run --rm --gpus all `
    --shm-size 1g `
    --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env "SANITATION_WS=$containerWorkspace" `
    --volume "${packRoot}:/work" `
    --workdir /work `
    tzcup/sanitation-jazzy:stage0 `
    bash scripts/stage3_ci.sh

if ($LASTEXITCODE -ne 0) {
    throw "Stage 3 container gate failed with exit code $LASTEXITCODE"
}
