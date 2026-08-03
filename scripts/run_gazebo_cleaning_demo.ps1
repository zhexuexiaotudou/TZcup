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
    [ValidateSet("normal", "fast", "turbo")]
    [string]$SimulationSpeed = "fast",
    [ValidateSet("optimized", "legacy")]
    [string]$CoverageProfile = "optimized",
    [ValidateRange(0, 1000)]
    [int]$DynamicObstacleTrials = 0,
    [ValidateSet("ogre2", "ogre")]
    [string]$SimulationRenderEngine = "ogre2",
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
    SimulationSpeed = $SimulationSpeed
    CoverageProfile = $CoverageProfile
    DynamicObstacleTrials = $DynamicObstacleTrials
    SimulationRenderEngine = $SimulationRenderEngine
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
    Write-Host "Launching the independent 16 m x 12 m competition demo with native mission controls at $SimulationSpeed speed..."
}
Write-Host "Use the right-side Gazebo panel: Start, Pause, Resume, Stop, or Close Gazebo."
Write-Host "Orange is the outer task area, cyan is the actual cleaning area, green is cleaned ground, and purple is the active path."
& $launcher @launchParameters
