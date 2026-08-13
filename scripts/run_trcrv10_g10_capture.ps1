param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('train', 'val')]
    [string]$Split,
    [Parameter(Mandatory = $true)]
    [string]$ArtifactRoot,
    [string]$Image = 'tzcup/sanitation-jazzy:stage5b'
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$artifact = [System.IO.Path]::GetFullPath($ArtifactRoot)
$domain = Join-Path $artifact 'g10\domain_route_v4'
$runtime = Join-Path $artifact 'g10\runtime'
$capture = Join-Path $artifact ("g10\route_v5\capture_{0}" -f $Split)
$logRoot = Join-Path $artifact 'g10\route_v5\capture_logs'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null

$config = if ($Split -eq 'train') {
    @{ Start = 0; Worlds = 6; Scenes = 8; Seed = 30000; Domain = 190 }
} else {
    @{ Start = 6; Worlds = 3; Scenes = 6; Seed = 40000; Domain = 200 }
}

for ($local = 0; $local -lt $config.Worlds; $local++) {
    $worldIndex = $config.Start + $local
    $rosDomain = $config.Domain + $local
    $partition = "trcrv10_g10_{0}_{1}" -f $Split, $worldIndex
    $log = Join-Path $logRoot ("{0}_world_{1:D2}.log" -f $Split, $worldIndex)
    # Wet-surface worlds can run below 0.08 RTF on the acceptance host.
    # This is an infrastructure bound only: frame count, route, camera,
    # motion gates, seeds, and all product acceptance semantics stay fixed.
    & (Join-Path $repo 'scripts\run_auto05r_g4_capture_docker.ps1') `
        -DataRoot $capture `
        -ResourceRoot $domain `
        -ModelResourceRoot $domain `
        -RuntimeWorkspaceRoot $runtime `
        -Image $Image `
        -ScenesPerWorld $config.Scenes `
        -MaxWorlds 1 `
        -StartWorldIndex $worldIndex `
        -RosDomainId $rosDomain `
        -GzPartition $partition `
        -AssetSourceSplit $Split `
        -NegativeSourceSplit $Split `
        -SceneSeedOffset $config.Seed `
        -CaptureFrameCount 125 `
        -CaptureTimeoutSeconds 1200 `
        -CaptureSpeedMps 0.20 `
        -CaptureMinTranslationM 0.02 `
        -CaptureMaxAttempts 2 `
        -DetectorInstancesPerClass 1 `
        -G10ApproachSequence `
        -SkipWorldGeneration *> $log
    if ($LASTEXITCODE -ne 0) {
        throw "TRCRV10 G10 $Split capture failed at world index $worldIndex"
    }
}

Write-Output "TRCRV10 G10 $Split capture complete: $($config.Worlds * $config.Scenes) missions"
