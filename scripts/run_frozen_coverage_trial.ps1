[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateRange(0, 2147483647)]
    [int]$Seed,
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9_-]+$')]
    [string]$Tag,
    [ValidateSet("fast", "turbo")]
    [string]$SimulationSpeed = "fast",
    [ValidateSet("optimized", "legacy")]
    [string]$CoverageProfile = "optimized",
    [ValidateSet("ogre2", "ogre")]
    [string]$SimulationRenderEngine = "ogre2",
    [switch]$Trace,
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [int]$DashboardPort = 8899
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "formal_wsl_entry_memory_guard.ps1")
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(3).Replace('\', '/')
$wslRoot = "/mnt/$drive/$tail"
$outputDirectory = Join-Path $repoRoot "artifacts\coverage_optimizer_$Tag"
[System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
$memoryEvidence = Join-Path $outputDirectory "wsl_entry_memory_preflight.json"
Invoke-FormalWslEntryMemoryGuard -EvidencePath $memoryEvidence
$launcherLog = Join-Path $repoRoot "artifacts\coverage_optimizer_${Tag}_launcher.log"
$launcherErrorLog = Join-Path $repoRoot "artifacts\coverage_optimizer_${Tag}_launcher.err.log"
$partition = "tzcup_cov_$Tag"

$arguments = @(
    "-d", $WslDistribution,
    "--", "env",
    "ROS_DOMAIN_ID=42",
    "GZ_PARTITION=$partition",
    "bash"
)
if ($Trace) {
    $arguments += "-x"
}
$arguments += @(
    "$wslRoot/scripts/run_visual_demo.sh",
    "--workspace", "/home/zhexu/tzcup_coverage_optimizer_ws",
    "--base-workspace", "/home/zhexu/sanitation_ws",
    "--output", "$wslRoot/artifacts/coverage_optimizer_$Tag",
    "--dashboard-port", "$DashboardPort",
    "--skip-build", "--gazebo-only", "--no-gui", "--no-mcap",
    "--simulation-speed", $SimulationSpeed,
    "--coverage-profile", $CoverageProfile,
    "--simulation-render-engine", $SimulationRenderEngine,
    "--map-size", "small", "--timeout", "300", "--seed", "$Seed"
)

$process = Start-Process -FilePath "wsl.exe" -ArgumentList $arguments `
    -RedirectStandardOutput $launcherLog `
    -RedirectStandardError $launcherErrorLog `
    -NoNewWindow -Wait -PassThru
exit $process.ExitCode
