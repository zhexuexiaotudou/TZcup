[CmdletBinding()]
param(
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [string]$Workspace = "",
    [string]$BaseWorkspace = "",
    [string]$OutputDirectory = "",
    [int]$TimeoutSeconds = 1800,
    [int]$Seed = 0,
    [switch]$SkipBuild,
    [switch]$NoMcap,
    [switch]$FullArea,
    [ValidateSet("small", "medium", "large")]
    [string]$MapSize = "small",
    [switch]$CloseOnComplete
)

$ErrorActionPreference = "Stop"
$launcher = Join-Path $PSScriptRoot "run_visual_demo.ps1"
$launchParameters = @{
    WslDistribution = $WslDistribution
    DashboardPort = 8877
    TimeoutSeconds = $TimeoutSeconds
    Seed = $Seed
    Video = "off"
    GazeboOnly = $true
    MapSize = $MapSize
    ManualControl = $true
}
if ($FullArea) { $launchParameters["MapSize"] = "medium" }
if ($Workspace) { $launchParameters["Workspace"] = $Workspace }
if ($BaseWorkspace) { $launchParameters["BaseWorkspace"] = $BaseWorkspace }
if ($OutputDirectory) { $launchParameters["OutputDirectory"] = $OutputDirectory }
if ($SkipBuild) { $launchParameters["SkipBuild"] = $true }
if ($NoMcap) { $launchParameters["NoMcap"] = $true }
if (-not $CloseOnComplete) { $launchParameters["KeepOpen"] = $true }

if ($FullArea) {
    Write-Host "Launching the medium 80 m x 50 m scene with the full 17-component mission..."
} else {
    Write-Host "Launching the $MapSize Gazebo scene with native mission controls..."
}
Write-Host "Use the right-side Gazebo panel: Start, Pause, Resume, Stop, or Close Gazebo."
Write-Host "Gray is the assigned area, teal is cleaned ground, and amber is the active path."
& $launcher @launchParameters
