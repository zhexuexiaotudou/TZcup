[CmdletBinding()]
param(
    [string]$WslDistribution = "TZcup-Ubuntu-24.04",
    [Parameter(Mandatory = $true)][string]$RunRoot,
    [Parameter(Mandatory = $true)][string]$RuntimeWorkspace,
    [string]$EpisodeRoot = "",
    [ValidateRange(1, 86400)][int]$MappingTimeoutSeconds = 21600,
    [ValidateRange(1, 172800)][int]$CleaningTimeoutSeconds = 86400,
    [ValidateRange(0, 231)][int]$MappingRosDomain = 99,
    [ValidateRange(0, 231)][int]$CleaningRosDomain = 60,
    [ValidateRange(1, 65535)][int]$DashboardPort = 8879,
    [switch]$Preflight
)

$ErrorActionPreference = "Stop"
function ConvertTo-WslPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    if ($Path -match '^/') { return $Path }
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if ($fullPath -notmatch '^([A-Za-z]):\\(.*)$') { throw "Expected a local drive path or absolute WSL path: $fullPath" }
    return "/mnt/$($Matches[1].ToLowerInvariant())/$($Matches[2].Replace('\', '/'))"
}
function Test-LinuxSafeRosDomain {
    param([int]$Domain)
    return (($Domain -ge 0 -and $Domain -le 101) -or ($Domain -ge 215 -and $Domain -le 231))
}
if ($MappingRosDomain -eq $CleaningRosDomain) { throw "MappingRosDomain and CleaningRosDomain must differ for hard restart." }
if (-not (Test-LinuxSafeRosDomain $MappingRosDomain) -or -not (Test-LinuxSafeRosDomain $CleaningRosDomain)) {
    throw "MappingRosDomain and CleaningRosDomain must be Linux-safe ROS domains: 0..101 or 215..231."
}
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$wslRoot = ConvertTo-WslPath $repoRoot
$arguments = @(
    "$wslRoot/scripts/run_final_product_visual_preview.sh",
    "--run-root", (ConvertTo-WslPath $RunRoot),
    "--runtime-ws", (ConvertTo-WslPath $RuntimeWorkspace),
    "--mapping-timeout-sec", "$MappingTimeoutSeconds",
    "--cleaning-timeout-sec", "$CleaningTimeoutSeconds",
    "--mapping-ros-domain", "$MappingRosDomain",
    "--cleaning-ros-domain", "$CleaningRosDomain",
    "--dashboard-port", "$DashboardPort"
)
if ($EpisodeRoot) { $arguments += @("--episode-root", (ConvertTo-WslPath $EpisodeRoot)) }
if ($Preflight) { $arguments += "--preflight" }
& wsl.exe -d $WslDistribution -- bash @arguments
if ($LASTEXITCODE -ne 0) { throw "Final visual preview failed with exit code $LASTEXITCODE." }
