[CmdletBinding()]
param(
    [string]$Distro = "TZcup-Ubuntu-24.04",
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^/[^\r\n]*$')]
    [string]$RuntimeWs,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')]
    [string]$RunId = ([DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffZ")),
    [ValidateRange(0, 231)]
    [int]$DomainId = 81,
    [string]$EvidenceRoot,
    [string]$DiagnosticLog
)

# This wrapper owns only Windows-side admission and evidence.  The existing
# Linux sensor runner remains the sole implementation of the full-rate sensor
# acceptance, its exact-PGID watchdog, and its Gazebo cleanup.
$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
if ($repoRoot -notmatch '^[A-Za-z]:\\') {
    throw "The Windows sensor transport probe requires a drive-letter repository path"
}
if ($RuntimeWs -eq "/") {
    throw "RuntimeWs must not be /"
}
if (($DomainId -gt 101) -and ($DomainId -lt 215)) {
    throw "DomainId intersects the Linux ephemeral UDP port range"
}
if (-not $EvidenceRoot) {
    $EvidenceRoot = Join-Path $repoRoot ".work\formal_sensor_transport_windows_guard\$RunId"
}
if (Test-Path -LiteralPath $EvidenceRoot) {
    throw "Refusing to reuse sensor transport probe evidence: $EvidenceRoot"
}
New-Item -ItemType Directory -Path $EvidenceRoot -ErrorAction Stop | Out-Null

if (-not $DiagnosticLog) {
    $DiagnosticLog = Join-Path $EvidenceRoot "sensor_transport_probe.log"
}
if (Test-Path -LiteralPath $DiagnosticLog) {
    throw "Refusing to overwrite sensor transport probe log: $DiagnosticLog"
}
$logParent = Split-Path -Parent $DiagnosticLog
if ($logParent) {
    New-Item -ItemType Directory -Path $logParent -Force | Out-Null
}

function Convert-ToWslDrivePath([string]$Path) {
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "Path cannot be represented as a WSL drive mount: $resolved"
    }
    $pathDrive = $Matches[1].ToLowerInvariant()
    $pathTail = $Matches[2].Replace('\', '/')
    return "/mnt/$pathDrive/$pathTail"
}

function Get-FormalExceptionExitCode([System.Exception]$Exception) {
    $candidate = $Exception.Data["ExitCode"]
    if ($null -ne $candidate) {
        return [int]$candidate
    }
    # An unobservable guard is never evidence that the host is safe.
    return 125
}

function Get-FormalEvidenceFile([string]$Path) {
    $row = [ordered]@{ path = $Path; exists = $false; sha256 = $null; bytes = $null }
    if (Test-Path -LiteralPath $Path -PathType Leaf) {
        $item = Get-Item -LiteralPath $Path -Force
        $row.exists = $true
        $row.sha256 = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
        $row.bytes = [Int64]$item.Length
    }
    return $row
}

function Test-StrictPoolEvidence([object]$Evidence, [string]$Label) {
    $sample = $Evidence.sample
    $checks = $Evidence.checks
    $pool = if ($null -ne $sample) { $sample.pool_tag_diagnostics } else { $null }
    if (
        $Evidence.passed -ne $true -or
        $null -eq $checks -or
        $checks.no_suspected_ndis_nonpaged_pool_leak -ne $true -or
        $null -eq $pool -or
        $pool.status -ne "available" -or
        $pool.suspected_ndis_nonpaged_pool_leak -ne $false
    ) {
        throw "$Label pool evidence is unavailable, suspected, or did not pass"
    }
}

$wslRepo = Convert-ToWslDrivePath $repoRoot
$attemptRoot = "$wslRepo/.work/formal_sensor_transport_probe/$RunId"
$attemptRootWindows = Join-Path $repoRoot ".work\formal_sensor_transport_probe\$RunId"
$attemptParentWindows = Split-Path -Parent $attemptRootWindows
New-Item -ItemType Directory -Path $attemptParentWindows -Force | Out-Null
$preflightJson = Join-Path $EvidenceRoot "cold_start_preflight.json"
$postRunPoolJson = Join-Path $EvidenceRoot "post_run_pool.json"
$summaryPath = Join-Path $EvidenceRoot "attempt_summary_windows.json"
$coldRc = $null
$childRc = $null
$postRc = $null
$wrapperError = $null
$coldPoolVerified = $false
$postPoolVerified = $false
$finalizerRc = $null
$finalReport = Join-Path $EvidenceRoot "formal_sensor_transport_probe.json"

. (Join-Path $PSScriptRoot "formal_wsl_entry_memory_guard.ps1")

try {
    try {
        Invoke-FormalWslEntryMemoryGuard -EvidencePath $preflightJson -RequireCold
        $coldRc = 0
        $preflight = Get-Content -Raw -LiteralPath $preflightJson | ConvertFrom-Json
        if (
            $preflight.require_wsl_stopped -ne $true -or
            $preflight.require_wsl_running -ne $false -or
            [UInt64]$preflight.thresholds_bytes.min_commit_available -ne [UInt64]13421772800 -or
            [UInt64]$preflight.thresholds_bytes.max_docker_private -ne [UInt64]4294967296 -or
            [UInt64]$preflight.sample.vmmem_wsl_private_bytes -ne [UInt64]0
        ) {
            throw "Cold-start evidence does not satisfy the fixed sensor transport entry contract"
        }
        Test-StrictPoolEvidence $preflight "Cold-start"
        $coldPoolVerified = $true
    }
    catch {
        $coldRc = Get-FormalExceptionExitCode $_.Exception
        throw
    }

    $wslArgs = @(
        "-d", $Distro,
        "--cd", $wslRepo,
        "--", "bash", "scripts/run_formal_sensor_transport_probe.sh",
        "--runtime-ws", $RuntimeWs,
        "--attempt-root", $attemptRoot,
        "--domain", [string]$DomainId
    )
    # Native stderr can be surfaced as PowerShell ErrorRecords; only wsl.exe's
    # exit code decides the child result while its combined output stays logged.
    $savedErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        & wsl.exe @wslArgs 2>&1 | Tee-Object -FilePath $DiagnosticLog
        $childRc = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $savedErrorActionPreference
    }
}
catch {
    $wrapperError = $_.Exception.Message
    if ($null -eq $coldRc) {
        $coldRc = Get-FormalExceptionExitCode $_.Exception
    }
}
finally {
    # This intentionally does not require a cold VM: it captures the pool after
    # the child has exited, whether WSL remains resident or not.  The helper is
    # read-only and does not stop WSL, Docker, processes, or network adapters.
    try {
        Invoke-FormalWslEntryMemoryGuard -EvidencePath $postRunPoolJson
        $postRc = 0
        $postEvidence = Get-Content -Raw -LiteralPath $postRunPoolJson | ConvertFrom-Json
        Test-StrictPoolEvidence $postEvidence "Post-run"
        $postPoolVerified = $true
    }
    catch {
        $postRc = Get-FormalExceptionExitCode $_.Exception
        if (-not $wrapperError) {
            $wrapperError = $_.Exception.Message
        }
    }

    if (
        (Test-Path -LiteralPath $attemptRootWindows -PathType Container) -and
        (Test-Path -LiteralPath $preflightJson -PathType Leaf) -and
        (Test-Path -LiteralPath $postRunPoolJson -PathType Leaf)
    ) {
        & py -3 (Join-Path $PSScriptRoot "finalize_formal_sensor_transport_probe.py") `
            --attempt-root $attemptRootWindows `
            --windows-before $preflightJson `
            --windows-after $postRunPoolJson `
            --output $finalReport
        $finalizerRc = $LASTEXITCODE
    }
    else {
        $finalizerRc = 2
    }

    $summary = [ordered]@{
        report_id = "tzcup_formal_sensor_transport_probe_windows_v1"
        run_id = $RunId
        runtime_ws = $RuntimeWs
        attempt_root_wsl = $attemptRoot
        attempt_root_windows = $attemptRootWindows
        domain_id = $DomainId
        child_rc = $childRc
        cold_gate_rc = $coldRc
        post_run_pool_rc = $postRc
        finalizer_rc = $finalizerRc
        cold_pool_tags_available_and_not_suspect = $coldPoolVerified
        post_pool_tags_available_and_not_suspect = $postPoolVerified
        wrapper_error = $wrapperError
        evidence = [ordered]@{
            cold_start_preflight = Get-FormalEvidenceFile $preflightJson
            post_run_pool = Get-FormalEvidenceFile $postRunPoolJson
            diagnostic_log = Get-FormalEvidenceFile $DiagnosticLog
            acceptance_session = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_sensor_probe_session.json")
            sensor_runtime = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_runtime_report.json")
            sensor_fov = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_fov_occlusion_report.json")
            runtime_binding = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_runtime_report.json.runtime_binding.json")
            preembedded_world = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "preembedded_sensor_world.sdf")
            preembedded_report = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "preembedded_sensor_world.json")
            wsl_memory_preflight = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_runtime_report.windows_memory_preflight.json")
            wsl_memory_watchdog = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_runtime_report.memory_watchdog.json")
            loopback_attestation = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "formal_vehicle_runtime_report.loopback_attestation.json")
            cleanup_attestation = Get-FormalEvidenceFile (Join-Path $attemptRootWindows "cleanup_attestation.json")
            final_probe_report = Get-FormalEvidenceFile $finalReport
        }
        host_actions = [ordered]@{
            wsl_shutdown_invoked = $false
            processes_stopped = $false
            network_configuration_changed = $false
        }
    }
    [System.IO.File]::WriteAllText(
        $summaryPath,
        ($summary | ConvertTo-Json -Depth 8) + [Environment]::NewLine,
        [System.Text.UTF8Encoding]::new($false)
    )
}

if (
    $coldRc -eq 0 -and
    $childRc -eq 0 -and
    $postRc -eq 0 -and
    $finalizerRc -eq 0 -and
    $coldPoolVerified -and
    $postPoolVerified
) {
    Write-Output "FORMAL_SENSOR_TRANSPORT_PROBE_WINDOWS_PASSED"
    Write-Output "attempt_root=$attemptRoot"
    Write-Output "summary=$summaryPath"
    exit 0
}

$exitCode = if ($null -ne $childRc -and $childRc -ne 0) {
    [int]$childRc
} elseif ($null -ne $postRc -and $postRc -ne 0) {
    [int]$postRc
} elseif ($null -ne $coldRc -and $coldRc -ne 0) {
    [int]$coldRc
} else {
    125
}
[Console]::Error.WriteLine("Formal sensor transport probe failed (rc=$exitCode). Summary: $summaryPath")
exit $exitCode
