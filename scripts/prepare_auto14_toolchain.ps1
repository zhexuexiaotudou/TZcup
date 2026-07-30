param(
    [string]$Archive = "F:\Project\TZcup-auto14-toolchain\oe-package-3.7.0-s100-s600.tgz",
    [string]$ToolchainVolume = "tzcup_auto14_toolchain_370",
    [string]$EnvironmentVolume = "tzcup_auto14_env_370",
    [string]$CudaImage = "nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04"
)

$ErrorActionPreference = "Stop"
$expected = "DE90DA5CF58879A0883BB47856232514C3CC30E368D8864911BD05E267229C5B"
if (-not (Test-Path -LiteralPath $Archive)) {
    throw "Official OE 3.7.0 archive not found: $Archive"
}
$actual = (Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash
if ($actual -ne $expected) {
    throw "Official OE archive SHA-256 mismatch: $actual"
}

docker volume create $ToolchainVolume | Out-Null
docker volume create $EnvironmentVolume | Out-Null
$archiveDirectory = Split-Path -Parent $Archive
$archiveName = Split-Path -Leaf $Archive
$probe = docker run --rm -v "${ToolchainVolume}:/toolchain" alpine:3.21 `
    sh -lc "test -f /toolchain/drobotics_s100_s600_open_explorer_v3.7.0/README-EN"
if ($LASTEXITCODE -ne 0) {
    docker run --rm `
        -v "${archiveDirectory}:/input:ro" `
        -v "${ToolchainVolume}:/toolchain" `
        alpine:3.21 `
        sh -lc "tar -xzf /input/$archiveName -C /toolchain"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to extract official OE archive"
    }
}

$bootstrap = @'
set -e
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  python3 python3-venv python3-dev build-essential libgomp1 libgl1 libglib2.0-0
if [ ! -x /venv/bin/python ]; then
  python3 -m venv /venv
fi
/venv/bin/python -m pip install --disable-pip-version-check --upgrade pip wheel
root=/toolchain/drobotics_s100_s600_open_explorer_v3.7.0/package/host/ai_toolchain
/venv/bin/python -m pip install --disable-pip-version-check \
  numpy==1.23.0 protobuf==3.20.3 onnx==1.15.0 onnxruntime==1.19.0 \
  matplotlib==3.5.3 numba==0.56.4 opencv-python==4.6.0.66 PyYAML==5.3.1
/venv/bin/python -m pip install --disable-pip-version-check \
  "$root"/hbdk4_march-4.7.5-*.whl \
  "$root"/hbdk4_runtime_x86_64_unknown_linux_gnu_nash-4.7.5-*.whl \
  "$root"/hbdk4_runtime_aarch64_unknown_linux_gnu_nash-4.7.5-*.whl \
  "$root"/hbdnn-1.0.3-*.whl \
  "$root"/hbm_infer-3.13.6-*.whl \
  "$root"/hbdk4_compiler-4.7.5-cp310-*.whl \
  "$root"/hmct-2.6.5-cp310-*.whl \
  "$root"/horizon_tc_ui-3.5.3-*.whl
/venv/bin/hb_compile --help >/tmp/hb_compile_help.txt
'@
docker run --rm `
    -v "${ToolchainVolume}:/toolchain:ro" `
    -v "${EnvironmentVolume}:/venv" `
    $CudaImage bash -lc $bootstrap
if ($LASTEXITCODE -ne 0) {
    throw "Official AUTO-14 toolchain bootstrap failed"
}
Write-Output "AUTO-14 official OE 3.7.0 toolchain environment ready"
