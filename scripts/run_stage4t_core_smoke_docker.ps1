$ErrorActionPreference = "Stop"

$packRoot = Split-Path -Parent $PSScriptRoot
$mainCheckout = "F:\Project\TZcup"
$workspace = Join-Path $mainCheckout ".work\stage1_20260714_154523"
if (-not (Test-Path -LiteralPath $workspace)) {
    throw "Reusable ROS workspace not found: $workspace"
}

$dockerArgs = @(
    "run", "--rm", "--gpus", "all",
    "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
    "--volume", "${mainCheckout}:/work",
    "--volume", "${packRoot}:/stage4t",
    "--workdir", "/stage4t",
    "tzcup/sanitation-jazzy:stage0",
    "bash", "scripts/stage4t_core_smoke_ci.sh"
)
docker @dockerArgs
if ($LASTEXITCODE -ne 0) {
    throw "Stage4T core smoke gate failed with exit code $LASTEXITCODE"
}
