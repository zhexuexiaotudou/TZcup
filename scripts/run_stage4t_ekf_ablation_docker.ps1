$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$mainCheckout = "F:\Project\TZcup"
$repeats = if ($env:TZCUP_EKF_REPEATS) { $env:TZCUP_EKF_REPEATS } else { "1" }
$argsList = @(
    "run", "--rm", "--gpus", "all",
    "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
    "--env", "TZCUP_EKF_REPEATS=$repeats",
    "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
    "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0",
    "bash", "scripts/stage4t_ekf_ablation_ci.sh"
)
if ($env:TZCUP_STAGE4T_ABLATION_DIR) {
    $argsList = $argsList[0..9] + @(
        "--env", "STAGE4T_OUT=/stage4t/artifacts/$($env:TZCUP_STAGE4T_ABLATION_DIR)"
    ) + $argsList[10..($argsList.Count - 1)]
}
docker @argsList
if ($LASTEXITCODE -ne 0) { throw "Stage4T EKF ablation failed: $LASTEXITCODE" }
