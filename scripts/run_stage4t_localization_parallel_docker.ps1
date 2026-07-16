$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot; $mainCheckout = "F:\Project\TZcup"
if (-not $env:TZCUP_MAP_DIR) { throw "Set TZCUP_MAP_DIR first" }
$lane = if ($env:TZCUP_LOCALIZATION_LANE) { $env:TZCUP_LOCALIZATION_LANE } else { "realistic" }
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"; $artifactName = "stage4t_${lane}_localization_$stamp"; $artifact = Join-Path $packRoot "artifacts\$artifactName"; New-Item -ItemType Directory -Force -Path $artifact | Out-Null
$processes = @()
for ($shard = 0; $shard -lt 4; $shard++) {
  $stdout = Join-Path $artifact "worker_${shard}.stdout.log"; $stderr = Join-Path $artifact "worker_${shard}.stderr.log"
  $dockerArgs = @(
    "run", "--rm", "--gpus", "all", "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
    "--env", "TZCUP_MAP_ROOT=/stage4t/artifacts/$($env:TZCUP_MAP_DIR)",
    "--env", "TZCUP_LOCALIZATION_SEEDS=10", "--env", "TZCUP_LOCALIZATION_LANE=$lane",
    "--env", "STAGE4T_OUT=/stage4t/artifacts/$artifactName",
    "--env", "SEED_START=$shard", "--env", "SEED_STEP=4", "--env", "SKIP_BUILD=true", "--env", "SKIP_AGGREGATE=true",
    "--env", "ROS_DOMAIN_ID=$($shard + 80)", "--env", "GZ_PARTITION=stage4t_${lane}_$shard",
    "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
    "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4t_localization_ci.sh"
  )
  foreach ($name in @("TZCUP_MAP_YAML", "TZCUP_MAP_CALIBRATION", "TZCUP_FILTER_ROOT", "TZCUP_LOCALIZATION_BACKEND", "TZCUP_SLAM_PARAMS", "TZCUP_POSEGRAPH", "TZCUP_LIDAR_SAMPLES", "TZCUP_LIDAR_UPDATE_RATE", "TZCUP_NAV2_PARAMS", "TZCUP_WORLD_FILE")) {
    $value = [Environment]::GetEnvironmentVariable($name)
    if ($value) {
      $insertAt = $dockerArgs.IndexOf("--volume")
      $dockerArgs = $dockerArgs[0..($insertAt - 1)] + @("--env", "$name=$value") + $dockerArgs[$insertAt..($dockerArgs.Count - 1)]
    }
  }
  $processes += Start-Process -FilePath "docker" -ArgumentList $dockerArgs -PassThru -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr
}
$processes | Wait-Process; $failed = $processes | Where-Object ExitCode -ne 0
if ($failed) { throw "Localization worker failure: $($failed.ExitCode -join ',')" }
$report = if ($lane -eq "oracle") { "oracle_localization_report.json" } else { "realistic_localization_report.json" }
py scripts/stage4t_localization_aggregate.py (Join-Path $artifact "localization_trials") (Join-Path $artifact $report) --lane $lane --required-seeds 10
if ($LASTEXITCODE -ne 0) { throw "Localization aggregation failed" }
Write-Output $artifact
