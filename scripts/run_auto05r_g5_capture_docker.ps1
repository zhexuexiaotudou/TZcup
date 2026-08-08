param(
    [string]$DataRoot = "F:\Project\TZcup-perception-product-runtime\g5-sealed-v1",
    [string]$UpstreamRoot = "F:\Project\TZcup-coverage-docker-src\linorobot2",
    [string]$Image = "tzcup/sanitation-jazzy:stage5b",
    [int]$MaxWorlds = 0,
    [int]$StartWorldIndex = 0,
    [int]$RosDomainId = -1,
    [string]$GzPartition = ""
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = [System.IO.Path]::GetFullPath($DataRoot)
New-Item -ItemType Directory -Force -Path $data | Out-Null
if ($RosDomainId -lt 0) { $RosDomainId = 180 + $StartWorldIndex }
if ($RosDomainId -lt 0 -or $RosDomainId -gt 232) {
    throw "ROS_DOMAIN_ID must be in [0,232], got $RosDomainId"
}
if ([string]::IsNullOrWhiteSpace($GzPartition)) {
    $GzPartition = "tzcup_g5_$StartWorldIndex"
}
$volumes = @("-v", "${repo}:/repo:ro", "-v", "${data}:/data")
if (Test-Path -LiteralPath $UpstreamRoot) {
    $upstream = (Resolve-Path $UpstreamRoot).Path
    $volumes += @("-v", "${upstream}:/upstream/linorobot2:ro")
}
docker run --rm --gpus all --shm-size 2g `
    -e AUTO05R_DATA_ROOT=/data/g5_sealed_native `
    -e AUTO05R_MAX_WORLDS=$MaxWorlds `
    -e AUTO05R_START_WORLD_INDEX=$StartWorldIndex `
    -e ROS_DOMAIN_ID=$RosDomainId `
    -e GZ_PARTITION=$GzPartition `
    -e IGN_PARTITION=$GzPartition `
    @volumes $Image bash /repo/scripts/auto05r_g5_capture_all.sh
if ($LASTEXITCODE -ne 0) {
    throw "AUTO-05R G5 capture failed with exit code $LASTEXITCODE"
}
