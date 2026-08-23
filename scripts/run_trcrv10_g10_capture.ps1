param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('train', 'val')]
    [string]$Split,
    [Parameter(Mandatory = $true)]
    [string]$ArtifactRoot,
    [Parameter(Mandatory = $true)]
    [string]$DomainRoot,
    [Parameter(Mandatory = $true)]
    [string]$UpstreamRoot,
    [string]$TrainQa = '',
    [string]$Image = 'tzcup/sanitation-jazzy@sha256:418550f48916d794bc0aff144c60a3b1353d0bb0bb1dcf086cda0ec8e2a5aadc'
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$artifact = [System.IO.Path]::GetFullPath($ArtifactRoot)
$domain = (Resolve-Path -LiteralPath $DomainRoot).Path
$domainManifest = Join-Path $domain 'worlds\g4_world_manifest.json'
$expectedDomainSha256 = '3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208'
if (-not (Test-Path -LiteralPath $domainManifest -PathType Leaf)) {
    throw "fixed G10 domain manifest missing: $domainManifest"
}
$actualDomainSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $domainManifest).Hash.ToLowerInvariant()
if ($actualDomainSha256 -ne $expectedDomainSha256) {
    throw "fixed G10 domain manifest SHA-256 mismatch: $actualDomainSha256"
}
$upstream = (Resolve-Path -LiteralPath $UpstreamRoot).Path
$expectedUpstreamRevision = 'b96aa42fbfa4390a77e0aab90935fe55d66d04ba'
if (-not (Test-Path -LiteralPath (Join-Path $upstream 'linorobot2_description\package.xml') -PathType Leaf)) {
    throw "pinned linorobot2_description package missing: $upstream"
}
$actualUpstreamRevision = (git -C $upstream rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $actualUpstreamRevision -ne $expectedUpstreamRevision) {
    throw "linorobot2 revision mismatch: $actualUpstreamRevision"
}
if (git -C $upstream status --porcelain) {
    throw "linorobot2 checkout must be clean: $upstream"
}
$runtime = Join-Path $artifact 'g10\runtime'
$capture = Join-Path $artifact ("g10\route_v21_reverse_xneg1p95\capture_{0}" -f $Split)
$logRoot = Join-Path $artifact 'g10\route_v21_reverse_xneg1p95\capture_logs'
if ((Test-Path -LiteralPath $capture) -and (Get-ChildItem -LiteralPath $capture -Force | Select-Object -First 1)) {
    throw "route v21 capture output must be fresh and empty: $capture"
}
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
if ($Split -eq 'val') {
    if ([string]::IsNullOrWhiteSpace($TrainQa) -or -not (Test-Path -LiteralPath $TrainQa -PathType Leaf)) {
        throw 'val capture requires the completed G10 TRAIN route QA'
    }
    $trainQaPath = (Resolve-Path -LiteralPath $TrainQa).Path
    $trainQaPayload = Get-Content -Raw -LiteralPath $trainQaPath | ConvertFrom-Json
    if (
        $trainQaPayload.G10_TRAIN_ROUTE_QA_PASS -ne $true -or
        $trainQaPayload.route_id -ne 'g10_route_v21_reverse_xneg1p95' -or
        $trainQaPayload.route_config_sha256 -ne 'ad6cb131739626827296cb37bcd365f58e3a0e4455f9c11f11dfe2b8111b4e36' -or
        $trainQaPayload.g10_domain_manifest_sha256 -ne $expectedDomainSha256 -or
        [int]$trainQaPayload.mission_counts.G10_TRAIN -lt 45
    ) {
        throw 'G10 TRAIN route QA does not authorize HOLDOUT capture'
    }
    [pscustomobject]@{
        schema_version = 1
        train_qa_path = $trainQaPath
        train_qa_sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $trainQaPath).Hash.ToLowerInvariant()
        route_id = $trainQaPayload.route_id
        route_config_sha256 = $trainQaPayload.route_config_sha256
        g10_domain_manifest_sha256 = $trainQaPayload.g10_domain_manifest_sha256
        holdout_capture_authorized = $true
    } | ConvertTo-Json | Set-Content -Encoding utf8 -LiteralPath (Join-Path $logRoot 'G10_HOLDOUT_AUTHORIZATION.json')
}

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
        -UpstreamRoot $upstream `
        -Image $Image `
        -ScenesPerWorld $config.Scenes `
        -MaxWorlds 1 `
        -StartWorldIndex $worldIndex `
        -RosDomainId $rosDomain `
        -GzPartition $partition `
        -AssetSourceSplit $Split `
        -NegativeSourceSplit $Split `
        -SceneSeedOffset $config.Seed `
        -CaptureFrameCount 360 `
        -CaptureTimeoutSeconds 2400 `
        -CaptureSpeedMps 0.05 `
        -CaptureMinTranslationM 0.02 `
        -CaptureMinRotationRad 0.03 `
        -CaptureMaxAttempts 2 `
        -DetectorInstancesPerClass 1 `
        -G10ApproachSequence `
        -SkipWorldGeneration *> $log
    if ($LASTEXITCODE -ne 0) {
        throw "TRCRV10 G10 $Split capture failed at world index $worldIndex"
    }
}

Write-Output "TRCRV10 G10 $Split capture complete: $($config.Worlds * $config.Scenes) missions"
