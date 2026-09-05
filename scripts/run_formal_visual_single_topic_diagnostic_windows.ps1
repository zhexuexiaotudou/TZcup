[CmdletBinding()]
param(
    [string]$Distro = "TZcup-Ubuntu-24.04",
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^/[^\r\n]*$')]
    [string]$RuntimeWs,
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^/[^\r\n]*$')]
    [string]$OutputRoot,
    [ValidateRange(0, 231)]
    [int]$DomainId = 225,
    [ValidatePattern('^/formal_visual/[^\r\n]+$')]
    [string]$Topic = "/formal_visual/front_left",
    [string]$EvidenceRoot,
    [string]$DiagnosticLog
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
if ($repoRoot -notmatch '^[A-Za-z]:\\') {
    throw "The Windows diagnostic wrapper requires a drive-letter repository path"
}
if ($RuntimeWs -eq "/" -or $OutputRoot -eq "/") {
    throw "RuntimeWs and OutputRoot must not be /"
}
if (($DomainId -gt 101) -and ($DomainId -lt 215)) {
    throw "DomainId intersects the Linux ephemeral UDP port range"
}
if (-not $EvidenceRoot) {
    $runId = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffZ")
    $EvidenceRoot = Join-Path $repoRoot ".work\formal_visual_single_topic_windows_guard\$runId"
}
if (Test-Path -LiteralPath $EvidenceRoot) {
    throw "Refusing to reuse diagnostic guard evidence: $EvidenceRoot"
}
New-Item -ItemType Directory -Path $EvidenceRoot -ErrorAction Stop | Out-Null

$preflightJson = Join-Path $EvidenceRoot "cold_start_preflight.json"
. (Join-Path $PSScriptRoot "formal_wsl_entry_memory_guard.ps1")
try {
    Invoke-FormalWslEntryMemoryGuard -EvidencePath $preflightJson -RequireCold
}
catch {
    $guardExitCode = $_.Exception.Data["ExitCode"]
    if ($null -ne $guardExitCode) {
        [Console]::Error.WriteLine($_.Exception.Message)
        exit [int]$guardExitCode
    }
    throw
}
$preflight = Get-Content -Raw -LiteralPath $preflightJson | ConvertFrom-Json
$preflightSample = $preflight.sample
if (
    $preflight.passed -ne $true -or
    $preflight.require_wsl_stopped -ne $true -or
    $preflight.require_wsl_running -ne $false -or
    [UInt64]$preflight.thresholds_bytes.min_commit_available -ne [UInt64]13421772800 -or
    [UInt64]$preflight.thresholds_bytes.max_docker_private -ne [UInt64]4294967296 -or
    $null -eq $preflightSample -or
    [UInt64]$preflightSample.commit_available_bytes -lt [UInt64]13421772800 -or
    [UInt64]$preflightSample.docker_private_bytes -gt [UInt64]4294967296 -or
    [UInt64]$preflightSample.vmmem_wsl_private_bytes -ne [UInt64]0
) {
    throw "Cold-start evidence does not satisfy the fixed diagnostic entry contract"
}

if (-not $DiagnosticLog) {
    $DiagnosticLog = Join-Path $EvidenceRoot "diagnostic.log"
}
if (Test-Path -LiteralPath $DiagnosticLog) {
    throw "Refusing to overwrite diagnostic log: $DiagnosticLog"
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

$wslRepo = Convert-ToWslDrivePath $repoRoot
$runtimeSetup = "$RuntimeWs/install/setup.bash"
$wslArgs = @(
    "-d", $Distro,
    "--cd", $wslRepo,
    "--", "bash", "scripts/run_formal_visual_single_topic_diagnostic.sh",
    "--runtime-setup", $runtimeSetup,
    "--output-root", $OutputRoot,
    "--domain-id", [string]$DomainId,
    "--topic", $Topic
)
& wsl.exe @wslArgs 2>&1 | Tee-Object -FilePath $DiagnosticLog
$diagnosticRc = $LASTEXITCODE
if ($diagnosticRc -ne 0) {
    [Console]::Error.WriteLine(
        "Single-topic diagnostic failed (rc=$diagnosticRc). Log: $DiagnosticLog"
    )
    exit $diagnosticRc
}

Write-Output "FORMAL_VISUAL_SINGLE_TOPIC_WINDOWS_GUARD_PASSED"
Write-Output "runtime_ws=$RuntimeWs"
Write-Output "output_root=$OutputRoot"
Write-Output "preflight=$preflightJson"
Write-Output "diagnostic_log=$DiagnosticLog"
