[CmdletBinding()]
param(
    [ValidateSet("Validate", "Build", "Simulation", "Matrix", "Package")]
    [string]$Mode = "Validate",
    [string]$OutputDir = "",
    [string]$Commit = "HEAD"
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
. (Join-Path $PSScriptRoot "formal_wsl_entry_memory_guard.ps1")
Push-Location $root
try {
    switch ($Mode) {
        "Validate" {
            & py -3 scripts/ci_fast.py
            if ($LASTEXITCODE -ne 0) { throw "ci_fast failed" }
            & py -3 scripts/verify_state_invariants.py
            if ($LASTEXITCODE -ne 0) { throw "state invariant check failed" }
            & py -3 scripts/scan_secrets.py
            if ($LASTEXITCODE -ne 0) { throw "secret scan failed" }
        }
        "Build" {
            & powershell.exe -NoProfile -ExecutionPolicy Bypass `
                -File scripts/run_stage1_docker.ps1 `
                -Image "tzcup/sanitation-jazzy:stage5b"
            if ($LASTEXITCODE -ne 0) { throw "clean Docker build gate failed" }
        }
        "Simulation" {
            Write-Host "Launching ROS 2/Gazebo baseline; close it with Ctrl+C."
            $memoryEvidenceRoot = Join-Path $root ".work\wsl_entry_memory_guard"
            $memoryRunId = [DateTime]::UtcNow.ToString("auto16_yyyyMMddTHHmmssfffZ")
            Invoke-FormalWslEntryMemoryGuard -EvidencePath (
                Join-Path $memoryEvidenceRoot "$memoryRunId.json"
            )
            $wslRoot = (& wsl.exe -d TZcup-Ubuntu-24.04 -- `
                wslpath -a $root).Trim()
            if ($LASTEXITCODE -ne 0) { throw "Cannot translate repository path" }
            Invoke-FormalWslEntryMemoryGuard -EvidencePath (
                Join-Path $memoryEvidenceRoot "$memoryRunId.launch.json"
            )
            & wsl.exe -d TZcup-Ubuntu-24.04 -- bash -lc `
                "export SANITATION_WS='$wslRoot/starter_ws'; cd '$wslRoot'; bash scripts/run_baseline.sh"
            if ($LASTEXITCODE -ne 0) { throw "baseline simulation failed" }
        }
        "Matrix" {
            $matrix = Get-Content -LiteralPath "reports/release/FINAL_COMPETITION_MATRIX.json" `
                -Raw -Encoding UTF8 | ConvertFrom-Json
            $matrix | ConvertTo-Json -Depth 4
            if (-not $matrix.simulation_competition_matrix_pass) {
                throw "Formal matrix is fail-closed: $($matrix.first_blocking_layer)"
            }
        }
        "Package" {
            if ([string]::IsNullOrWhiteSpace($OutputDir)) {
                throw "-OutputDir is required for Package mode"
            }
            $resolvedCommit = (& git rev-parse $Commit).Trim()
            if ($LASTEXITCODE -ne 0) { throw "Cannot resolve commit $Commit" }
            & py -3 scripts/auto16_release.py --package `
                --commit $resolvedCommit --output-dir $OutputDir
            if ($LASTEXITCODE -ne 0) { throw "release package failed" }
        }
    }
}
finally {
    Pop-Location
}
