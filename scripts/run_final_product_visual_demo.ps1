[CmdletBinding()]
param(
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [Parameter(Mandatory = $true)][string]$RunRoot,
    [Parameter(Mandatory = $true)][string]$RuntimeWorkspace,
    [Parameter(Mandatory = $true)][string]$RuntimeClosure,
    [Parameter(Mandatory = $true)][string]$VehicleSnapshot,
    [Parameter(Mandatory = $true)][string]$AcceptanceSession,
    [Parameter(Mandatory = $true)][string]$EpisodeRoot,
    [Parameter(Mandatory = $true)][string]$MapRoot,
    [Parameter(Mandatory = $true)][string]$TerminalOutput,
    [ValidateSet("unavailable", "pc", "s100p")]
    [Parameter(Mandatory = $true)][string]$PerceptionMode,
    [ValidateSet("full_coverage", "rl_dirt_priority")]
    [Parameter(Mandatory = $true)][string]$CleaningPlanner,
    [string]$PerceptionArtifactRoot = "",
    [string]$PolicyCheckpoint = "",
    [string]$S100pBoardBridge = "",
    [string]$S100pReceipt = "",
    [ValidateRange(0, 232)][int]$MappingRosDomain = 99,
    [ValidateRange(0, 232)][int]$CleaningRosDomain = 60,
    [ValidateRange(1, 86400)][int]$MappingTimeoutSeconds = 21600,
    [ValidateRange(1, 172800)][int]$CleaningTimeoutSeconds = 86400,
    [double]$FullCoverageDistanceM = 0.0,
    [ValidateSet("dry_cleaning_competition_candidate")][string]$OperationSpeedProfile = "dry_cleaning_competition_candidate",
    [ValidateSet("true", "false")][string]$FormalVisualGui = "false",
    [ValidateRange(1, 65535)][int]$DashboardPort = 8878,
    [string]$DashboardOutput = "",
    [switch]$Preflight
)

$ErrorActionPreference = "Stop"

function ConvertTo-WslPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    if ($Path -match '^/') { return $Path }
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "Only local Windows drive paths or absolute WSL paths are accepted: $fullPath"
    }
    return "/mnt/$($Matches[1].ToLowerInvariant())/$($Matches[2].Replace('\\', '/'))"
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if ($MappingRosDomain -eq $CleaningRosDomain) {
    throw "MappingRosDomain and CleaningRosDomain must differ for the hard restart."
}
$distributions = ((& wsl.exe --list --quiet) -replace "`0", "") |
    ForEach-Object { $_.Trim() } | Where-Object { $_ }
if ($distributions -notcontains $WslDistribution) {
    throw "WSL distribution '$WslDistribution' is not installed."
}

$wslRoot = ConvertTo-WslPath -Path $repoRoot
$arguments = @(
    "$wslRoot/scripts/run_final_product_visual_demo.sh",
    "--run-root", (ConvertTo-WslPath -Path $RunRoot),
    "--runtime-ws", (ConvertTo-WslPath -Path $RuntimeWorkspace),
    "--runtime-closure", (ConvertTo-WslPath -Path $RuntimeClosure),
    "--vehicle-snapshot", (ConvertTo-WslPath -Path $VehicleSnapshot),
    "--acceptance-session", (ConvertTo-WslPath -Path $AcceptanceSession),
    "--episode-root", (ConvertTo-WslPath -Path $EpisodeRoot),
    "--map-root", (ConvertTo-WslPath -Path $MapRoot),
    "--terminal-output", (ConvertTo-WslPath -Path $TerminalOutput),
    "--perception-mode", $PerceptionMode,
    "--cleaning-planner", $CleaningPlanner,
    "--mapping-ros-domain", "$MappingRosDomain",
    "--cleaning-ros-domain", "$CleaningRosDomain",
    "--mapping-timeout-sec", "$MappingTimeoutSeconds",
    "--cleaning-timeout-sec", "$CleaningTimeoutSeconds",
    "--full-coverage-distance-m", "$FullCoverageDistanceM",
    "--operation-speed-profile", $OperationSpeedProfile,
    "--formal-visual-gui", $FormalVisualGui,
    "--dashboard-port", "$DashboardPort"
)
if ($DashboardOutput) { $arguments += @("--dashboard-output", (ConvertTo-WslPath -Path $DashboardOutput)) }
if ($PerceptionArtifactRoot) { $arguments += @("--perception-artifact-root", (ConvertTo-WslPath -Path $PerceptionArtifactRoot)) }
if ($PolicyCheckpoint) { $arguments += @("--policy-checkpoint", (ConvertTo-WslPath -Path $PolicyCheckpoint)) }
if ($S100pBoardBridge) { $arguments += @("--s100p-board-bridge", (ConvertTo-WslPath -Path $S100pBoardBridge)) }
if ($S100pReceipt) { $arguments += @("--s100p-receipt", (ConvertTo-WslPath -Path $S100pReceipt)) }
if ($Preflight) { $arguments += "--preflight" }

& wsl.exe -d $WslDistribution -- bash @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Formal final-product visual demo failed with exit code $LASTEXITCODE."
}
