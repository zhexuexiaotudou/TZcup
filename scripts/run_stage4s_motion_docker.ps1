$ErrorActionPreference = "Stop"

$packRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = if ($env:TZCUP_STAGE1_WORK_ROOT) {
    $env:TZCUP_STAGE1_WORK_ROOT
} else {
    $mainCheckout = Join-Path (Split-Path -Parent $packRoot) "TZcup"
    Join-Path $mainCheckout ".work"
}
$workspace = Get-ChildItem -LiteralPath $workspaceRoot -Directory |
    Where-Object Name -Like "stage1_*" |
    Sort-Object Name -Descending |
    Select-Object -First 1
if (-not $workspace) {
    throw "No Stage 1 workspace found under $workspaceRoot"
}

$mainCheckout = Split-Path -Parent $workspaceRoot
$containerWorkspace = "/work/.work/$($workspace.Name)"
$dockerArgs = @(
    "run", "--rm", "--gpus", "all",
    "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=$containerWorkspace",
    "--volume", "${mainCheckout}:/work",
    "--volume", "${packRoot}:/stage4s",
    "--workdir", "/stage4s"
)
foreach ($name in @("TZCUP_DRIVE_WHEEL_RADIUS", "TZCUP_DRIVE_WHEEL_SEPARATION", "TZCUP_CALIBRATION_LABEL")) {
    if (Test-Path "Env:$name") {
        $dockerArgs += @("--env", "$name=$((Get-Item "Env:$name").Value)")
    }
}
$dockerArgs += @("tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4s_motion_calibration_ci.sh")

docker @dockerArgs

if ($LASTEXITCODE -ne 0) {
    throw "Stage4S motion calibration experiment failed with exit code $LASTEXITCODE"
}
