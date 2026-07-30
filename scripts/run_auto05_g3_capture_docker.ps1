param(
    [string]$DataRoot = "F:\Project\TZcup-autonomous-auto05-data",
    [string]$BaselineRoot = "F:\Project\TZcup\.work\stage1_20260714_154523",
    [string]$Image = "tzcup/sanitation-jazzy:stage5b"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = [System.IO.Path]::GetFullPath($DataRoot)
$baseline = (Resolve-Path $BaselineRoot).Path
New-Item -ItemType Directory -Force -Path $data | Out-Null

docker run --rm --gpus all --shm-size 2g `
    -e AUTO05_DATA_ROOT=/data/g3_screening_native `
    -v "${repo}:/repo:ro" `
    -v "${baseline}:/work/.work/stage1_20260714_154523:ro" `
    -v "${data}:/data" `
    $Image `
    bash /repo/scripts/auto05_capture_all.sh

if ($LASTEXITCODE -ne 0) {
    throw "AUTO-05 G3 capture failed with exit code $LASTEXITCODE"
}
