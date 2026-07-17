param(
  [int[]]$Seeds = @(0, 1, 2, 3, 4),
  [string]$OutputName = "stage4w_static_formal"
)

$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot
foreach ($seed in $Seeds) {
  & "$PSScriptRoot\run_stage4w_static_coverage_docker.ps1" `
    -Seed $seed -OutputName "$OutputName/seed_$seed"
}
$root = Join-Path $packRoot "artifacts\$OutputName"
$report = Join-Path $root "stage4w_static_matrix_report.json"
py "$PSScriptRoot\stage4w_static_aggregate.py" $root $report `
  --required-seeds $Seeds.Count
if ($LASTEXITCODE -ne 0) { throw "Stage4W static matrix failed: $LASTEXITCODE" }
Write-Output $report
