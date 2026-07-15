$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot; $mainCheckout = "F:\Project\TZcup"
if (-not $env:TZCUP_STAGE4T_ABLATION_DIR) { throw "Set TZCUP_STAGE4T_ABLATION_DIR to the reusable artifact directory name" }
$artifact = Join-Path $packRoot "artifacts\$($env:TZCUP_STAGE4T_ABLATION_DIR)"
$processes = @(); $candidates = @("A", "B", "C", "D")
for ($index = 0; $index -lt $candidates.Count; $index++) {
  $candidate = $candidates[$index]
  $stdout = Join-Path $artifact "worker_${candidate}.stdout.log"; $stderr = Join-Path $artifact "worker_${candidate}.stderr.log"
  $dockerArgs = @(
    "run", "--rm", "--gpus", "all", "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
    "--env", "STAGE4T_OUT=/stage4t/artifacts/$($env:TZCUP_STAGE4T_ABLATION_DIR)",
    "--env", "TZCUP_EKF_REPEATS=5", "--env", "CANDIDATE_ONLY=$candidate", "--env", "SKIP_AGGREGATE=true", "--env", "SKIP_BUILD=true",
    "--env", "ROS_DOMAIN_ID=$($index + 60)", "--env", "GZ_PARTITION=stage4t_ekf_$candidate",
    "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
    "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4t_ekf_ablation_ci.sh"
  )
  $processes += Start-Process -FilePath "docker" -ArgumentList $dockerArgs -PassThru -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr
}
$processes | Wait-Process
$failed = $processes | Where-Object ExitCode -ne 0
if ($failed) { throw "EKF worker failure: $($failed.ExitCode -join ',')" }
py scripts/stage4t_ekf_ablation.py $artifact --required-repeats 5 --config-dir starter_ws/src/sanitation_bringup/config
if ($LASTEXITCODE -ne 0) { throw "EKF aggregation failed" }
Write-Output $artifact
