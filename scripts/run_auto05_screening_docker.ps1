param(
    [string]$DataRoot = "F:\Project\TZcup-autonomous-auto05-data\g3_screening_native",
    [string]$DatasetEvidence = "F:\Project\TZcup-autonomous-auto05-data\dataset_evidence",
    [string]$OutputName = "autonomous_auto05_attempt1_raw",
    [string]$ImplementationCommit = "WORKTREE",
    [int]$Attempt = 1,
    [int]$DetectorEpochs = 45,
    [int]$AreaEpochs = 35,
    [string]$Image = "tzcup/sanitation-jazzy:stage5b"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = (Resolve-Path $DataRoot).Path
$dataset = (Resolve-Path $DatasetEvidence).Path
$output = Join-Path $repo "artifacts\$OutputName"
New-Item -ItemType Directory -Force -Path $output | Out-Null

docker run --rm --gpus all --shm-size 4g `
    -v "${repo}:/repo" `
    -v "${data}:/data:ro" `
    -v "${dataset}:/dataset:ro" `
    $Image `
    python3 /repo/scripts/auto05_screening.py `
      --data-root /data `
      --dataset-evidence /dataset `
      --output "/repo/artifacts/$OutputName" `
      --implementation-commit $ImplementationCommit `
      --attempt $Attempt `
      --detector-epochs $DetectorEpochs `
      --area-epochs $AreaEpochs

if ($LASTEXITCODE -notin @(0, 2)) {
    throw "AUTO-05 screening crashed with exit code $LASTEXITCODE"
}
exit $LASTEXITCODE
