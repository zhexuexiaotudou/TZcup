param(
    [string]$DataRoot = "F:\Project\TZcup-stage5br3-data\g2_screening_native",
    [string]$OutputName = "autonomous_auto04_20260730_evidence",
    [string]$ImplementationCommit = "",
    [int]$Attempt = 1,
    [int]$DetectorEpochs = 180,
    [int]$AreaEpochs = 160
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = (Resolve-Path $DataRoot).Path
$output = Join-Path $repo "artifacts\$OutputName"
if ([string]::IsNullOrWhiteSpace($ImplementationCommit)) {
    $ImplementationCommit = (git -C $repo rev-parse HEAD).Trim()
}
if (Test-Path -LiteralPath $output) {
    throw "Output already exists: $output"
}

docker run --rm --gpus all `
    -v "${repo}:/repo" `
    -v "${data}:/data:ro" `
    -w /repo `
    tzcup/sanitation-jazzy:stage5b `
    python3 scripts/auto04_micro_overfit.py `
        --data-root /data `
        --output "/repo/artifacts/$OutputName" `
        --implementation-commit $ImplementationCommit `
        --attempt $Attempt `
        --detector-epochs $DetectorEpochs `
        --area-epochs $AreaEpochs

if ($LASTEXITCODE -ne 0) {
    throw "AUTO-04 micro-overfit failed with exit code $LASTEXITCODE"
}
