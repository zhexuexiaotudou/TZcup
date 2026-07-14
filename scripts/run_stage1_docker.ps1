[CmdletBinding()]
param(
    [string]$Image = "tzcup/sanitation-jazzy:stage0"
)

$ErrorActionPreference = "Stop"
$packRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path

& docker image inspect $Image *> $null
if ($LASTEXITCODE -ne 0) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "run_docker_preflight.ps1") -Image $Image
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to prepare Docker image $Image"
    }
}

$mount = "${packRoot}:/work"
& docker run --rm --volume $mount --workdir /work --env SANITATION_PACK_ROOT=/work $Image bash scripts/stage1_ci.sh
if ($LASTEXITCODE -ne 0) {
    throw "Stage 1 container build failed with exit code $LASTEXITCODE"
}
