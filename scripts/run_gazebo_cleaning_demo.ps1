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
}
if (-not $FullArea) { $launchParameters["Showcase"] = $true }
if ($Workspace) { $launchParameters["Workspace"] = $Workspace }
if ($BaseWorkspace) { $launchParameters["BaseWorkspace"] = $BaseWorkspace }
if ($OutputDirectory) { $launchParameters["OutputDirectory"] = $OutputDirectory }
if ($SkipBuild) { $launchParameters["SkipBuild"] = $true }
if ($NoMcap) { $launchParameters["NoMcap"] = $true }
if (-not $CloseOnComplete) { $launchParameters["KeepOpen"] = $true }

if ($FullArea) {
    Write-Host "Launching the full-area cleaning mission in Gazebo only..."
} else {
    Write-Host "Launching the bounded 6 m x 5 m showcase mission in Gazebo only..."
}
Write-Host "Gray is the assigned area, teal is cleaned ground, and amber is the active path."
& $launcher @launchParameters
