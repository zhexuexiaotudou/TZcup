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
if ($Workspace) { $launchParameters["Workspace"] = $Workspace }
if ($BaseWorkspace) { $launchParameters["BaseWorkspace"] = $BaseWorkspace }
if ($OutputDirectory) { $launchParameters["OutputDirectory"] = $OutputDirectory }
if ($SkipBuild) { $launchParameters["SkipBuild"] = $true }
if ($NoMcap) { $launchParameters["NoMcap"] = $true }
if (-not $CloseOnComplete) { $launchParameters["KeepOpen"] = $true }

Write-Host "Launching the full cleaning mission in Gazebo only..."
Write-Host "The teal band is cleaned ground; the amber line is the active path."
& $launcher @launchParameters
