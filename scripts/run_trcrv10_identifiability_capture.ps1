param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('train_diag', 'holdout_diag')]
    [string]$Split,
    [Parameter(Mandatory = $true)]
    [string]$ArtifactRoot,
    [string]$Image = 'tzcup/sanitation-jazzy:stage5b'
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$artifact = [System.IO.Path]::GetFullPath($ArtifactRoot)
$domain = Join-Path $artifact 'g10\domain_route_v4'
$runtime = Join-Path $artifact 'identifiability\diag_runtime'
$capture = Join-Path $artifact ("identifiability\capture_{0}_v1" -f $Split)
$logRoot = Join-Path $artifact 'identifiability\capture_logs'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null

$config = if ($Split -eq 'train_diag') {
    @{ Start = 0; Worlds = 6; Asset = 'train'; Seed = 50000; Domain = 210 }
} else {
    @{ Start = 6; Worlds = 3; Asset = 'val'; Seed = 60000; Domain = 220 }
}

for ($local = 0; $local -lt $config.Worlds; $local++) {
    $worldIndex = $config.Start + $local
    $rosDomain = $config.Domain + $local
    $partition = "trcrv10_{0}_{1}" -f $Split, $worldIndex
    $log = Join-Path $logRoot ("{0}_world_{1:D2}.log" -f $Split, $worldIndex)
    & (Join-Path $repo 'scripts\run_auto05r_g4_capture_docker.ps1') `
        -DataRoot $capture `
        -ResourceRoot $domain `
        -ModelResourceRoot $domain `
        -RuntimeWorkspaceRoot $runtime `
        -Image $Image `
        -CameraProfileId 'trcrv10_diag_v1_low_oblique_evaluator_only' `
        -CameraX 0.67 -CameraY 0.0 -CameraZ 0.30 -CameraPitchRad 0.610865238 `
        -ScenesPerWorld 24 `
        -MaxWorlds 1 `
        -StartWorldIndex $worldIndex `
        -RosDomainId $rosDomain `
        -GzPartition $partition `
        -AssetSourceSplit $config.Asset `
        -NegativeSourceSplit $config.Asset `
        -SceneSeedOffset $config.Seed `
        -CaptureFrameCount 34 `
        -CaptureTimeoutSeconds 300 `
        -CaptureSpeedMps 0.001 `
        -CaptureMinTranslationM 0 `
        -CaptureMaxAttempts 2 `
        -DetectorInstancesPerClass 1 `
        -G10IdentifiabilityDiagnostic `
        -SkipWorldGeneration *> $log
    if ($LASTEXITCODE -ne 0) {
        throw "TRCRV10 $Split capture failed at world index $worldIndex"
    }
}

Write-Output "TRCRV10 $Split capture complete: $($config.Worlds * 24) scenes"
