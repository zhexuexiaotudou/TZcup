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

$wslRoot = ConvertTo-WslPath -Path $repoRoot
$arguments = @(
    "$wslRoot/scripts/run_visual_demo.sh",
    "--dashboard-port", "$DashboardPort",
    "--video", $Video,
    "--timeout", "$TimeoutSeconds",
    "--seed", "$Seed",
    "--map-size", $MapSize
)
if ($Workspace) {
    $arguments += @("--workspace", $Workspace)
}
if ($BaseWorkspace) {
    $arguments += @("--base-workspace", $BaseWorkspace)
}
if ($OutputDirectory) {
    $resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
    [System.IO.Directory]::CreateDirectory($resolvedOutput) | Out-Null
    $wslOutput = ConvertTo-WslPath -Path $resolvedOutput
    $arguments += @("--output", $wslOutput)
}
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
& wsl.exe -d $WslDistribution -- bash @arguments
if ($LASTEXITCODE -ne 0) {
    throw "AUTO-17 visual demo failed with exit code $LASTEXITCODE."
}
