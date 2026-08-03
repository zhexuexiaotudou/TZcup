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
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [int]$DashboardPort = 8899
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$drive = $repoRoot.Substring(0, 1).ToLowerInvariant()
$tail = $repoRoot.Substring(3).Replace('\', '/')
$wslRoot = "/mnt/$drive/$tail"
$outputDirectory = Join-Path $repoRoot "artifacts\coverage_optimizer_$Tag"
[System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
$launcherLog = Join-Path $repoRoot "artifacts\coverage_optimizer_${Tag}_launcher.log"
$partition = "tzcup_cov_$Tag"

$arguments = @(
    "-d", $WslDistribution,
    "--", "env",
    "ROS_DOMAIN_ID=42",
    "GZ_PARTITION=$partition",
    "bash", "$wslRoot/scripts/run_visual_demo.sh",
    "--workspace", "/home/zhexu/tzcup_coverage_optimizer_ws",
    "--base-workspace", "/home/zhexu/sanitation_ws",
    "--output", "$wslRoot/artifacts/coverage_optimizer_$Tag",
    "--dashboard-port", "$DashboardPort",
    "--skip-build", "--gazebo-only", "--no-gui", "--no-mcap",
    "--simulation-speed", $SimulationSpeed,
    "--map-size", "small", "--timeout", "300", "--seed", "$Seed"
)

& wsl.exe @arguments *> $launcherLog
exit $LASTEXITCODE
