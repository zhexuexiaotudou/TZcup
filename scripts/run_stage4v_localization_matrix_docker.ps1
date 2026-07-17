param(
  [int]$SeedStart = 0,
  [int]$SeedCount = 10,
  [int]$MaxParallel = 2,
  [string]$Lane = "hybrid_rtk_scan_imu_wheel",
  [string]$OutputName = "",
  [string]$RuntimeWorkspace = "/work/.work/stage4v_20260716"
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
$mainCheckout = "F:\Project\TZcup"
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
if (-not $OutputName) {
  $OutputName = "stage4v_localization_${Lane}_$stamp"
}
$artifact = Join-Path $packRoot "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $artifact | Out-Null

$pending = [System.Collections.Generic.Queue[int]]::new()
foreach ($seed in $SeedStart..($SeedStart + $SeedCount - 1)) {
  $pending.Enqueue($seed)
}
$active = @()
$failures = @()

while ($pending.Count -gt 0 -or $active.Count -gt 0) {
  while ($pending.Count -gt 0 -and $active.Count -lt $MaxParallel) {
    $seed = $pending.Dequeue()
    $stdout = Join-Path $artifact "seed_${seed}_docker.stdout.log"
    $stderr = Join-Path $artifact "seed_${seed}_docker.stderr.log"
    $dockerArgs = @(
      "run", "--rm", "--gpus", "all",
      "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
      "--env", "ROS_DOMAIN_ID=$($seed + 100)",
      "--env", "GZ_PARTITION=stage4v_${seed}",
      "--env", "SANITATION_BASE_WS=/work/.work/stage1_20260714_154523",
      "--env", "SANITATION_WS=$RuntimeWorkspace",
      "--env", "STAGE4V_OUT=/stage4v/artifacts/$OutputName",
      "--env", "STAGE4V_SEED=$seed",
      "--env", "STAGE4V_LANE=$Lane",
      "--volume", "${mainCheckout}:/work",
      "--volume", "${packRoot}:/stage4v",
      "--workdir", "/stage4v",
      "tzcup/sanitation-jazzy:stage0",
      "bash", "scripts/stage4v_localization_trial_ci.sh"
    )
    $process = Start-Process -FilePath "docker" -ArgumentList $dockerArgs `
      -PassThru -WindowStyle Hidden `
      -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    $active += [pscustomobject]@{ Seed = $seed; Process = $process }
    Write-Output "started seed $seed (PID $($process.Id))"
  }

  if ($active.Count -gt 0) {
    Start-Sleep -Seconds 2
    $remaining = @()
    foreach ($job in $active) {
      if ($job.Process.HasExited) {
        $job.Process.WaitForExit()
        $job.Process.Refresh()
        $exitCode = $job.Process.ExitCode
        if ($null -eq $exitCode -or "$exitCode" -eq "") {
          $summaryPath = Join-Path $artifact "seed_$($job.Seed)\trial_summary.json"
          if (Test-Path -LiteralPath $summaryPath) {
            $trial = Get-Content -LiteralPath $summaryPath -Raw | ConvertFrom-Json
            $internalCodes = @(
              $trial.exit_codes.navigation,
              $trial.exit_codes.evaluator,
              $trial.exit_codes.tf_audit
            )
            if (($internalCodes | Where-Object { $_ -ne 0 }).Count -eq 0) {
              $exitCode = 0
            }
          }
        }
        if ($exitCode -ne 0) {
          $failures += "seed $($job.Seed): exit $exitCode"
        }
        Write-Output "finished seed $($job.Seed) (validated exit $exitCode)"
      } else {
        $remaining += $job
      }
    }
    $active = $remaining
  }
}

if ($failures.Count -gt 0) {
  throw "Stage4V trials failed: $($failures -join '; ')"
}

py (Join-Path $packRoot "scripts\stage4v_localization_aggregate.py") `
  $artifact --lane $Lane --required-seeds $SeedCount
if ($LASTEXITCODE -ne 0) {
  throw "Stage4V localization aggregate gate failed"
}
Write-Output $artifact
