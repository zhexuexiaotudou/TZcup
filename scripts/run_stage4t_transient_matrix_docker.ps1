$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot; $mainCheckout = "F:\Project\TZcup"
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$artifactName = if ($env:TZCUP_STAGE4T_TRANSIENT_DIR) { $env:TZCUP_STAGE4T_TRANSIENT_DIR } else { "stage4t_transient_$stamp" }
$artifact = Join-Path $packRoot "artifacts\$artifactName"; New-Item -ItemType Directory -Force -Path $artifact | Out-Null

docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all `
  --env SANITATION_WS=/work/.work/stage1_20260714_154523 `
  --volume "${mainCheckout}:/work" --volume "${packRoot}:/stage4t" --workdir /stage4t `
  tzcup/sanitation-jazzy:stage0 bash -lc "set +u; source /opt/ros/jazzy/setup.bash; source /work/.work/stage1_20260714_154523/install/setup.bash; set -u; rsync -a starter_ws/src/ /work/.work/stage1_20260714_154523/src/; cd /work/.work/stage1_20260714_154523; colcon build --packages-select-regex '^sanitation_' --symlink-install --event-handlers console_direct+" `
  *> (Join-Path $artifact "build.log")
if ($LASTEXITCODE -ne 0) { throw "Stage4T transient preparation build failed" }

$processes = @()
for ($shard = 0; $shard -lt 4; $shard++) {
  $stdout = Join-Path $artifact "worker_${shard}.stdout.log"; $stderr = Join-Path $artifact "worker_${shard}.stderr.log"
  $dockerArgs = @(
    "run", "--rm", "--gpus", "all", "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
    "--env", "STAGE4T_OUT=/stage4t/artifacts/$artifactName",
    "--env", "SHARD_INDEX=$shard", "--env", "SHARD_COUNT=4",
    "--env", "ROS_DOMAIN_ID=$($shard + 40)", "--env", "GZ_PARTITION=stage4t_$shard",
    "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
    "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4t_transient_worker_ci.sh"
  )
  $processes += Start-Process -FilePath "docker" -ArgumentList $dockerArgs -PassThru -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr
}
$processes | Wait-Process
$failed = $processes | Where-Object ExitCode -ne 0
if ($failed) { throw "One or more Stage4T transient workers failed: $($failed.ExitCode -join ',')" }
py scripts/stage4t_aggregate.py $artifact --required-repeats 10
if ($LASTEXITCODE -ne 0) { throw "Stage4T transient aggregation failed" }
Write-Output $artifact
