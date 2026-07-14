[CmdletBinding()]
param(
    [string]$Image = "tzcup/sanitation-jazzy:stage0",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$packRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path

if (-not $SkipBuild) {
    & docker build --file (Join-Path $packRoot "docker\Dockerfile.jazzy") --tag $Image $packRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Docker image build failed with exit code $LASTEXITCODE"
    }
}

$mount = "${packRoot}:/work"
& docker run --rm --volume $mount --workdir /work --env SANITATION_PACK_ROOT=/work $Image bash scripts/check_env.sh

if ($LASTEXITCODE -ne 0) {
    throw "Container preflight failed with exit code $LASTEXITCODE"
}
