function Invoke-FormalWslEntryMemoryGuard {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$EvidencePath,
        [switch]$RequireCold
    )

    $probe = Join-Path $PSScriptRoot "formal_windows_memory_probe.py"
    if (-not (Test-Path -LiteralPath $probe -PathType Leaf)) {
        throw "Formal Windows memory probe is missing: $probe"
    }
    if (Test-Path -LiteralPath $EvidencePath) {
        throw "Refusing stale WSL-entry memory evidence: $EvidencePath"
    }
    $parent = Split-Path -Parent $EvidencePath
    if ($parent) {
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
    }

    if ($RequireCold) {
        $minimumCommit = [UInt64]13421772800
        $requireWslStopped = "1"
        $requireWslRunning = "0"
        $reportedCold = $true
    } else {
        $wslVmRunning = $null -ne (
            Get-Process -Name vmmemWSL -ErrorAction SilentlyContinue |
                Select-Object -First 1
        )
        $minimumCommit = if ($wslVmRunning) {
            [UInt64]10737418240
        } else {
            [UInt64]13421772800
        }
        $requireWslStopped = if ($wslVmRunning) { "0" } else { "1" }
        $requireWslRunning = if ($wslVmRunning) { "1" } else { "0" }
        $reportedCold = -not $wslVmRunning
    }

    $oldCommitFloor = $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES
    $oldDockerCeiling = $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES
    $oldRequireWslStopped = $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED
    $oldRequireWslRunning = $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING
    try {
        $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES = [string]$minimumCommit
        $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES = "4294967296"
        $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED = $requireWslStopped
        $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING = $requireWslRunning
        & py -3 $probe --check-start --output $EvidencePath
        $probeRc = $LASTEXITCODE
    }
    finally {
        $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES = $oldCommitFloor
        $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES = $oldDockerCeiling
        $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED = $oldRequireWslStopped
        $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING = $oldRequireWslRunning
    }
    if ($probeRc -ne 0) {
        $probeRefusal = [System.InvalidOperationException]::new(
            "WSL entry refused by the formal memory gate (rc=$probeRc, " +
            "cold=$reportedCold). Evidence: $EvidencePath"
        )
        $probeRefusal.Data["ExitCode"] = [int]$probeRc
        throw $probeRefusal
    }
    Write-Host (
        "WSL entry memory gate passed: cold=$reportedCold, " +
        "minimum_commit_bytes=$minimumCommit, evidence=$EvidencePath"
    )
}
