[CmdletBinding()]
param(
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [string]$Workspace = "",
    [string]$BaseWorkspace = "",
    [string]$OutputDirectory = "",
    [ValidateSet("auto", "on", "off")]
    [string]$Video = "auto",
    [int]$DashboardPort = 8877,
    [int]$TimeoutSeconds = 1800,
    [int]$Seed = 0,
    [ValidateSet("auto", "d3d12", "software")]
    [string]$GazeboGuiRenderer = "auto",
    [ValidateSet("normal", "fast", "turbo")]
    [string]$SimulationSpeed = "fast",
    [ValidateSet("optimized", "legacy")]
    [string]$CoverageProfile = "optimized",
    [ValidateRange(0, 1000)]
    [int]$DynamicObstacleTrials = 0,
    [ValidateSet("ogre2", "ogre")]
    [string]$SimulationRenderEngine = "ogre2",
    [switch]$SkipBuild,
    [switch]$NoGui,
    [switch]$NoRviz,
    [switch]$NoMcap,
    [switch]$GazeboOnly,
    [switch]$Showcase,
    [ValidateSet("small", "medium", "large")]
    [string]$MapSize = "medium",
    [switch]$ManualControl,
    [switch]$CompetitionProfile,
    [switch]$NoBrowser,
    [switch]$NoGazeboTrail,
    [switch]$KeepOpen
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$distributions = ((& wsl.exe --list --quiet) -replace "`0", "") |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ }
if ($distributions -notcontains $WslDistribution) {
    throw "WSL distribution '$WslDistribution' is not installed."
}

function ConvertTo-WslPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "Only local Windows drive paths can be translated: $fullPath"
    }
    $drive = $Matches[1].ToLowerInvariant()
    $tail = $Matches[2].Replace('\', '/')
    return "/mnt/$drive/$tail"
}

$guardProcess = $null
$guardStopFile = ""
$guardEvidencePath = ""
$guardFailureFile = ""
$guardScript = Join-Path $PSScriptRoot "wslg_window_guard.ps1"

if (-not $OutputDirectory) {
    $timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ")
    $OutputDirectory = Join-Path $repoRoot "artifacts\auto17_visual_demo_$timestamp"
}
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
[System.IO.Directory]::CreateDirectory($resolvedOutput) | Out-Null

$wslRoot = ConvertTo-WslPath -Path $repoRoot
if (-not $NoGui) {
    $wslgPrepareScript = "$wslRoot/scripts/prepare_wslg_runtime.sh"
    Write-Host "Preparing the WSLg shared-memory transport..."
    & wsl.exe -d $WslDistribution -u root -- bash $wslgPrepareScript
    $prepareExitCode = $LASTEXITCODE
    if ($prepareExitCode -eq 10) {
        $otherRunningDistributions = ((& wsl.exe --list --running --quiet) -replace "`0", "") |
            ForEach-Object { $_.Trim() } |
            Where-Object { $_ -and $_ -ne $WslDistribution }
        if ($otherRunningDistributions) {
            $names = $otherRunningDistributions -join ", "
            throw "WSLg needs one full restart, but other WSL distributions are running: $names"
        }
        Write-Host "Recovering the WSLg shared-memory session once..."
        & wsl.exe --shutdown
        if ($LASTEXITCODE -ne 0) {
            throw "WSLg recovery shutdown failed with exit code $LASTEXITCODE."
        }
        Start-Sleep -Seconds 5
        & wsl.exe -d $WslDistribution -u root -- bash $wslgPrepareScript
        $prepareExitCode = $LASTEXITCODE
    }
    if ($prepareExitCode -ne 0) {
        throw "WSLg shared-memory preparation failed with exit code $prepareExitCode."
    }
    Start-Sleep -Seconds 2
}
$arguments = @(
    "$wslRoot/scripts/run_visual_demo.sh",
    "--dashboard-port", "$DashboardPort",
    "--video", $Video,
    "--timeout", "$TimeoutSeconds",
    "--seed", "$Seed",
    "--gazebo-gui-renderer", $GazeboGuiRenderer,
    "--simulation-speed", $SimulationSpeed,
    "--coverage-profile", $CoverageProfile,
    "--dynamic-obstacle-trials", "$DynamicObstacleTrials",
    "--simulation-render-engine", $SimulationRenderEngine,
    "--map-size", $MapSize
)
if ($Workspace) {
    $arguments += @("--workspace", $Workspace)
}
if ($BaseWorkspace) {
    $arguments += @("--base-workspace", $BaseWorkspace)
}
$wslOutput = ConvertTo-WslPath -Path $resolvedOutput
$arguments += @("--output", $wslOutput)
if ($SkipBuild) { $arguments += "--skip-build" }
if ($NoGui) { $arguments += "--no-gui" }
if ($NoRviz) { $arguments += "--no-rviz" }
if ($NoMcap) { $arguments += "--no-mcap" }
if ($GazeboOnly) { $arguments += "--gazebo-only" }
if ($Showcase) { $arguments += "--showcase" }
if ($ManualControl) { $arguments += "--manual-control" }
if ($CompetitionProfile) { $arguments += "--competition-profile" }
if ($NoBrowser) { $arguments += "--no-browser" }
if ($NoGazeboTrail) { $arguments += "--no-gazebo-trail" }
if ($KeepOpen) { $arguments += "--keep-open" }

Write-Host "Launching AUTO-17 visual demo in $WslDistribution..."
Write-Host "Dashboard: http://127.0.0.1:$DashboardPort"

if (-not $NoGui) {
    if (-not (Test-Path -LiteralPath $guardScript)) {
        throw "WSLg window guard is missing: $guardScript"
    }
    $guardStopFile = Join-Path $resolvedOutput "wslg_window_guard.stop"
    $guardEvidencePath = Join-Path $resolvedOutput "wslg_window_guard.jsonl"
    $guardFailureFile = Join-Path $resolvedOutput "wslg_window_guard.failed"
    $quotedGuardScript = '"' + $guardScript.Replace('"', '\"') + '"'
    $quotedStopFile = '"' + $guardStopFile.Replace('"', '\"') + '"'
    $quotedEvidence = '"' + $guardEvidencePath.Replace('"', '\"') + '"'
    $quotedFailure = '"' + $guardFailureFile.Replace('"', '\"') + '"'
    $guardArguments = @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", $quotedGuardScript,
        "-WindowTitle", '"Gazebo Sim"',
        "-StopFile", $quotedStopFile,
        "-EvidencePath", $quotedEvidence,
        "-FailureFile", $quotedFailure,
        "-CloseWindowOnStop",
        "-Monitor"
    )
}

$wslExitCode = 1
$wslgRecoveryAttempted = $false
while ($true) {
    $guardProcess = $null
    if (-not $NoGui) {
        Remove-Item -LiteralPath $guardStopFile -Force -ErrorAction SilentlyContinue
        Remove-Item -LiteralPath $guardFailureFile -Force -ErrorAction SilentlyContinue
        $guardProcess = Start-Process -FilePath "powershell.exe" `
            -ArgumentList $guardArguments -WindowStyle Hidden -PassThru
    }
    try {
        & wsl.exe -d $WslDistribution -- bash @arguments
        $wslExitCode = $LASTEXITCODE
    } finally {
        if ($guardProcess) {
            New-Item -ItemType File -Path $guardStopFile -Force | Out-Null
            if (-not $guardProcess.WaitForExit(5000)) {
                Stop-Process -Id $guardProcess.Id -Force -ErrorAction SilentlyContinue
            }
        }
    }
    $recoverableWslgExit = $wslExitCode -in @(4, 7)
    if (-not $recoverableWslgExit -or $wslgRecoveryAttempted -or $NoGui) {
        break
    }
    $otherRunningDistributions = ((& wsl.exe --list --running --quiet) -replace "`0", "") |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ -and $_ -ne $WslDistribution }
    if ($otherRunningDistributions) {
        $names = $otherRunningDistributions -join ", "
        throw "WSLg recovery needs one full WSL restart, but other distributions are running: $names"
    }
    $recoveryReason = if ($wslExitCode -eq 7) { "COPY MODE detected" } else { "Gazebo GUI exited before native controls loaded" }
    Write-Host "$recoveryReason; restarting WSLg once and retrying the demo..."
    $terminationPath = Join-Path $resolvedOutput "launcher_termination.json"
    if (Test-Path -LiteralPath $terminationPath) {
        $attemptTerminationName = if ($wslExitCode -eq 7) {
            "launcher_termination_copy_mode_attempt.json"
        } else {
            "launcher_termination_early_gui_exit_attempt.json"
        }
        Move-Item -LiteralPath $terminationPath `
            -Destination (Join-Path $resolvedOutput $attemptTerminationName) `
            -Force
    }
    & wsl.exe --shutdown
    if ($LASTEXITCODE -ne 0) {
        throw "WSLg recovery shutdown failed with exit code $LASTEXITCODE."
    }
    Start-Sleep -Seconds 5
    & wsl.exe -d $WslDistribution -u root -- bash $wslgPrepareScript
    if ($LASTEXITCODE -ne 0) {
        throw "WSLg recovery preflight failed with exit code $LASTEXITCODE."
    }
    Start-Sleep -Seconds 2
    $wslgRecoveryAttempted = $true
}
if ($wslExitCode -ne 0) {
    throw "AUTO-17 visual demo failed with exit code $wslExitCode."
}
