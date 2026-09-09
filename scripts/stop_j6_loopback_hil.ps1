[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$composeFile = Join-Path $repoRoot "docker\compose.journey6-loopback.yaml"

# Compose requires interpolation even for down; these values are not mounted
# or executed because no service is created by this operation.
if (-not $env:J6_OE_BASE_IMAGE) { $env:J6_OE_BASE_IMAGE = "scratch" }
if (-not $env:J6_ROS_SETUP) { $env:J6_ROS_SETUP = "/dev/null" }
if (-not $env:J6_RUNTIME_BUNDLE) { $env:J6_RUNTIME_BUNDLE = $repoRoot }
if (-not $env:J6_MODEL_ARTIFACTS) { $env:J6_MODEL_ARTIFACTS = $repoRoot }
if (-not $env:J6_ALGORITHM_COMMAND) { $env:J6_ALGORITHM_COMMAND = "/bin/false" }
if (-not $env:HIL_EVIDENCE_DIR) { $env:HIL_EVIDENCE_DIR = $repoRoot }
if (-not $env:HIL_RUN_ID) { $env:HIL_RUN_ID = "00000000-0000-4000-8000-000000000000" }

& docker compose -f $composeFile -p tzcup-j6-loopback `
    --profile build-only --profile journey6 --profile pc-onnx `
    down --remove-orphans
if ($LASTEXITCODE -ne 0) { throw "Journey 6 loopback shutdown failed." }
Write-Host "Journey 6 loopback containers stopped; images and evidence were retained."
