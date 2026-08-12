param(
    [string]$DataRoot = "F:\Project\TZcup-autonomous-auto05r-g4-data",
    [string]$BaselineRoot = "",
    [string]$ResourceRoot = "",
    [string]$ModelResourceRoot = "",
    [string]$RuntimeWorkspaceRoot = "",
    [string]$UpstreamRoot = "F:\Project\TZcup-coverage-docker-src\linorobot2",
    [string]$Image = "tzcup/sanitation-jazzy:stage5b",
    [int]$ScenesPerWorld = 25,
    [int]$MaxWorlds = 0,
    [int]$StartWorldIndex = 0,
    [int]$RosDomainId = -1,
    [string]$GzPartition = "",
    [ValidateSet("", "D1", "D2", "D3", "D4", "D5")]
    [string]$DiagnosticRole = "",
    [ValidateSet("", "train", "val", "test")]
    [string]$AssetSourceSplit = "",
    [ValidateSet("", "train", "val", "test")]
    [string]$NegativeSourceSplit = "",
    [int]$SceneSeedOffset = 0,
    [string]$OnlyScenes = "",
    [int]$CaptureFrameCount = 10,
    [double]$CaptureTimeoutSeconds = 90.0,
    [double]$CaptureSpeedMps = 0.35,
    [double]$CaptureMinTranslationM = 0.25,
    [double]$CaptureMinRotationRad = 0.0,
    [ValidateRange(1, 8)]
    [int]$DetectorInstancesPerClass = 1,
    [switch]$G8AutoDomainMatrix,
    [ValidateRange(1, 10)]
    [int]$CaptureMaxAttempts = 3,
    [ValidateSet("", "turn_entry", "occlusion", "reflection", "dynamic_removal", "dynamic_insertion")]
    [string]$Oprv3CoverageProfile = "",
    [switch]$SkipWorldGeneration,
    [switch]$ForceNegativeOnly
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$data = [System.IO.Path]::GetFullPath($DataRoot)
New-Item -ItemType Directory -Force -Path $data | Out-Null

if ($RosDomainId -lt 0) {
    $RosDomainId = 100 + $StartWorldIndex
}
if ($RosDomainId -lt 0 -or $RosDomainId -gt 232) {
    throw "ROS_DOMAIN_ID must be in [0,232], got $RosDomainId"
}
if ([string]::IsNullOrWhiteSpace($GzPartition)) {
    $GzPartition = "tzcup_g4_$StartWorldIndex"
}

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
if (-not [string]::IsNullOrWhiteSpace($ResourceRoot)) {
    $resource = (Resolve-Path $ResourceRoot).Path
    $volumeArgs += @("-v", "${resource}:/resource:ro")
}
if (-not [string]::IsNullOrWhiteSpace($ModelResourceRoot)) {
    $modelResource = (Resolve-Path $ModelResourceRoot).Path
    $volumeArgs += @("-v", "${modelResource}:/model-resource:ro")
}
if (-not [string]::IsNullOrWhiteSpace($RuntimeWorkspaceRoot)) {
    $runtimeWorkspace = [System.IO.Path]::GetFullPath($RuntimeWorkspaceRoot)
    New-Item -ItemType Directory -Force -Path $runtimeWorkspace | Out-Null
    $volumeArgs += @("-v", "${runtimeWorkspace}:/runtime")
}

$resourceRootInContainer = if ([string]::IsNullOrWhiteSpace($ResourceRoot)) { "/data/g4_screening_native" } else { "/resource" }
$modelResourceRootInContainer = if ([string]::IsNullOrWhiteSpace($ModelResourceRoot)) { $resourceRootInContainer } else { "/model-resource" }
$worldManifestInContainer = "$resourceRootInContainer/worlds/g4_world_manifest.json"
$runtimeWorkspaceInContainer = if ([string]::IsNullOrWhiteSpace($RuntimeWorkspaceRoot)) { "/data/runtime_ws_g4" } else { "/runtime" }

docker run --rm --gpus all --shm-size 2g `
    -e AUTO05R_DATA_ROOT=/data/g4_screening_native `
    -e AUTO05R_SCENES_PER_WORLD=$ScenesPerWorld `
    -e AUTO05R_MAX_WORLDS=$MaxWorlds `
    -e AUTO05R_START_WORLD_INDEX=$StartWorldIndex `
    -e ROS_DOMAIN_ID=$RosDomainId `
    -e GZ_PARTITION=$GzPartition `
    -e IGN_PARTITION=$GzPartition `
    -e AUTO05R_DIAGNOSTIC_ROLE=$DiagnosticRole `
    -e AUTO05R_ASSET_SOURCE_SPLIT=$AssetSourceSplit `
    -e AUTO05R_NEGATIVE_SOURCE_SPLIT=$NegativeSourceSplit `
    -e AUTO05R_SCENE_SEED_OFFSET=$SceneSeedOffset `
    -e AUTO05R_SKIP_WORLD_GENERATION=$([int]$SkipWorldGeneration.IsPresent) `
    -e AUTO05R_FORCE_NEGATIVE_ONLY=$([int]$ForceNegativeOnly.IsPresent) `
    -e AUTO05R_RESOURCE_ROOT=$resourceRootInContainer `
    -e AUTO05R_MODEL_RESOURCE_ROOT=$modelResourceRootInContainer `
    -e AUTO05R_WORLD_MANIFEST=$worldManifestInContainer `
    -e AUTO05R_ONLY_SCENES=$OnlyScenes `
    -e AUTO05R_CAPTURE_FRAME_COUNT=$CaptureFrameCount `
    -e AUTO05R_CAPTURE_TIMEOUT_SECONDS=$CaptureTimeoutSeconds `
    -e AUTO05R_CAPTURE_SPEED_MPS=$CaptureSpeedMps `
    -e AUTO05R_CAPTURE_MIN_TRANSLATION_M=$CaptureMinTranslationM `
    -e AUTO05R_CAPTURE_MIN_ROTATION_RAD=$CaptureMinRotationRad `
    -e AUTO05R_CAPTURE_MAX_ATTEMPTS=$CaptureMaxAttempts `
    -e AUTO05R_OPRV3_COVERAGE_PROFILE=$Oprv3CoverageProfile `
    -e AUTO05R_DETECTOR_INSTANCES_PER_CLASS=$DetectorInstancesPerClass `
    -e AUTO05R_G8_AUTO_DOMAIN_MATRIX=$([int]$G8AutoDomainMatrix.IsPresent) `
    -e AUTO05R_RUNTIME_WS=$runtimeWorkspaceInContainer `
    @volumeArgs `
    $Image `
    bash /repo/scripts/auto05r_g4_capture_all.sh

if ($LASTEXITCODE -ne 0) {
    throw "AUTO-05R G4 capture failed with exit code $LASTEXITCODE"
}
