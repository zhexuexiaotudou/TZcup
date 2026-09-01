[CmdletBinding()]
param(
    [string]$Distro = "TZcup-Ubuntu-24.04",
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^/[^\r\n]*$')]
    [string]$RuntimeWs,
    [ValidateRange(1, 1)]
    [int]$Workers = 1,
    [ValidateRange(13421772800, [UInt64]::MaxValue)]
    [UInt64]$ColdMinCommitAvailableBytes = 13421772800,
    [ValidateRange(0, 4294967296)]
    [UInt64]$MaxDockerPrivateBytes = 4294967296,
    [string]$EvidenceRoot,
    [string]$BuildLog
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
if ($repoRoot -notmatch '^[A-Za-z]:\\') {
    throw "The Windows guarded builder requires a drive-letter repository path"
}
if ($RuntimeWs -eq "/") {
    throw "RuntimeWs must not be /"
}

if (-not $EvidenceRoot) {
    $runId = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffZ")
    $EvidenceRoot = Join-Path $repoRoot ".work\formal_final_runtime_windows_guard\$runId"
}
if (Test-Path -LiteralPath $EvidenceRoot) {
    throw "Refusing to reuse Windows build-guard evidence: $EvidenceRoot"
}
New-Item -ItemType Directory -Path $EvidenceRoot -ErrorAction Stop | Out-Null

$preflightJson = Join-Path $EvidenceRoot "cold_start_preflight.json"
$preflightLog = Join-Path $EvidenceRoot "cold_start_preflight.log"
$probe = Join-Path $repoRoot "scripts\formal_windows_memory_probe.py"
$oldCommitFloor = $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES
$oldDockerCeiling = $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES
$oldRequireWslStopped = $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED
$oldRequireWslRunning = $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING
try {
    $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES = [string]$ColdMinCommitAvailableBytes
    $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES = [string]$MaxDockerPrivateBytes
    $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED = "1"
    $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING = "0"
    & py -3 $probe --check-start --output $preflightJson *> $preflightLog
    $preflightRc = $LASTEXITCODE
}
finally {
    $env:FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES = $oldCommitFloor
    $env:FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES = $oldDockerCeiling
    $env:FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED = $oldRequireWslStopped
    $env:FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING = $oldRequireWslRunning
}
if ($preflightRc -ne 0) {
    [Console]::Error.WriteLine(
        "Cold Windows memory gate refused WSL startup (rc=$preflightRc). " +
        "Evidence: $preflightJson"
    )
    exit $preflightRc
}

if (-not $BuildLog) {
    $BuildLog = Join-Path $EvidenceRoot "build.log"
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
if (Test-Path -LiteralPath $BuildLog) {
    throw "Refusing to overwrite build log: $BuildLog"
}
$buildLogParent = Split-Path -Parent $BuildLog
if ($buildLogParent) {
    New-Item -ItemType Directory -Path $buildLogParent -Force | Out-Null
}

$wslRepo = Convert-ToWslDrivePath $repoRoot
$wslPreflightJson = Convert-ToWslDrivePath $preflightJson

# The shell build performs the second, inside-WSL 10 GiB start gate before it
# creates or compiles the frozen workspace.  The Windows gate above reserves
# 12.5 GiB by default so normal WSL VM startup cannot consume that margin.
$wslArgs = @(
    "-d", $Distro,
    "--cd", $wslRepo,
    "--", "env",
    "FORMAL_FINAL_RUNTIME_WS=$RuntimeWs",
    "FORMAL_COLCON_PARALLEL_WORKERS=$Workers",
    "FORMAL_WINDOWS_COLD_GATE_EVIDENCE=$wslPreflightJson",
    "bash", "scripts/build_formal_final_runtime.sh"
)
# Windows PowerShell 5 materializes native stderr records as ErrorRecord
# objects.  With the script-wide Stop preference, harmless progress output
# (for example Git's "Cloning into ...") used to abort this wrapper before the
# native process exit code could be observed.  Preserve the combined log but
# judge the build only by wsl.exe's actual exit code.
$savedErrorActionPreference = $ErrorActionPreference
try {
    $ErrorActionPreference = "Continue"
    & wsl.exe @wslArgs 2>&1 |
        Tee-Object -FilePath $BuildLog
    $buildRc = $LASTEXITCODE
}
finally {
    $ErrorActionPreference = $savedErrorActionPreference
}
if ($buildRc -ne 0) {
    [Console]::Error.WriteLine(
        "Frozen runtime build failed (rc=$buildRc). Log: $BuildLog"
    )
    exit $buildRc
}

Write-Output "FORMAL_FINAL_RUNTIME_WINDOWS_GUARD_PASSED"
Write-Output "runtime_ws=$RuntimeWs"
Write-Output "preflight=$preflightJson"
Write-Output "build_log=$BuildLog"
