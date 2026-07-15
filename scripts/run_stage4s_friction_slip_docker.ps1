$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = if ($env:TZCUP_STAGE1_WORK_ROOT) {
    $env:TZCUP_STAGE1_WORK_ROOT
} else {
    Join-Path (Join-Path (Split-Path -Parent $packRoot) "TZcup") ".work"
}
$workspace = Get-ChildItem -LiteralPath $workspaceRoot -Directory |
    Where-Object Name -Like "stage1_*" | Sort-Object Name -Descending |
    Select-Object -First 1
if (-not $workspace) { throw "No Stage 1 workspace found under $workspaceRoot" }
$mainCheckout = Split-Path -Parent $workspaceRoot
docker run --rm --gpus all `
    --env NVIDIA_DRIVER_CAPABILITIES=all `
    --env "SANITATION_WS=/work/.work/$($workspace.Name)" `
    --volume "${mainCheckout}:/work" `
    --volume "${packRoot}:/stage4s" `
    --workdir /stage4s `
    tzcup/sanitation-jazzy:stage0 `
    bash scripts/stage4s_friction_slip_scan_ci.sh
if ($LASTEXITCODE -ne 0) { throw "Stage4S friction/slip scan failed: $LASTEXITCODE" }
