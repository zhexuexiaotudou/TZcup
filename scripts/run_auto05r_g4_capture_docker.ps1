param(
    [string]$DataRoot = "F:\Project\TZcup-autonomous-auto05r-g4-data",
    [string]$BaselineRoot = "",
    [string]$UpstreamRoot = "F:\Project\TZcup-coverage-docker-src\linorobot2",
    [string]$Image = "tzcup/sanitation-jazzy:stage5b",
    [int]$ScenesPerWorld = 25
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = [System.IO.Path]::GetFullPath($DataRoot)
New-Item -ItemType Directory -Force -Path $data | Out-Null

$volumeArgs = @(
    "-v", "${repo}:/repo:ro",
    "-v", "${data}:/data"
)
if (-not [string]::IsNullOrWhiteSpace($BaselineRoot) -and (Test-Path -LiteralPath $BaselineRoot)) {
    $baseline = (Resolve-Path $BaselineRoot).Path
    $volumeArgs += @("-v", "${baseline}:/work/.work/stage1_20260714_154523:ro")
}
if (-not [string]::IsNullOrWhiteSpace($UpstreamRoot) -and (Test-Path -LiteralPath $UpstreamRoot)) {
    $upstream = (Resolve-Path $UpstreamRoot).Path
    $volumeArgs += @("-v", "${upstream}:/upstream/linorobot2:ro")
}

docker run --rm --gpus all --shm-size 2g `
    -e AUTO05R_DATA_ROOT=/data/g4_screening_native `
    -e AUTO05R_SCENES_PER_WORLD=$ScenesPerWorld `
    @volumeArgs `
    $Image `
    bash /repo/scripts/auto05r_g4_capture_all.sh

if ($LASTEXITCODE -ne 0) {
    throw "AUTO-05R G4 capture failed with exit code $LASTEXITCODE"
}
