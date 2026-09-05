[CmdletBinding()]
param(
    [switch]$ReuseExistingVenv,
    [switch]$FormalExport
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$WorkRoot = Join-Path $RepositoryRoot ".work"
$VenvRoot = Join-Path $WorkRoot "cadquery-venv"
$Preflight = Join-Path $RepositoryRoot "scripts\cadquery_windows_preflight.py"
$LockFile = Join-Path $RepositoryRoot "config\cadquery-windows-cp313.lock"
$Roundtrip = Join-Path $RepositoryRoot "scripts\run_cadquery_step_roundtrip.py"
$SerialExport = Join-Path $RepositoryRoot "scripts\run_native_cadquery_serial_export.py"
$PreflightReport = Join-Path $WorkRoot "cadquery-windows-preflight.json"
$RoundtripDir = Join-Path $WorkRoot "cadquery-step-roundtrip"
$RoundtripReport = Join-Path $RoundtripDir "roundtrip-report.json"
$SerialExportDir = Join-Path $WorkRoot "native-cadquery-serial-release"

# This lock is resolved only for CPython 3.13 on Windows amd64.  Do not accept
# a caller-supplied interpreter or launcher arguments here: an accidental
# ``py -3`` currently selects CPython 3.14 on this host.  Every preflight,
# venv, and pre-venv contract invocation must therefore go through this exact
# launcher selector.  The Python preflight independently verifies the selected
# interpreter before this script can create a venv or invoke pip.
$LockedPythonLauncher = "py"
$LockedPythonArguments = @("-3.13")

function Invoke-LockedCadQueryPython {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)

    & $LockedPythonLauncher @LockedPythonArguments @Arguments
}

New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null
Invoke-LockedCadQueryPython $Preflight --root $RepositoryRoot --output $PreflightReport --strict
if ($LASTEXITCODE -ne 0) {
    Write-Host "CadQuery bootstrap was not started. Read $PreflightReport for the fail-closed resource diagnosis."
    exit $LASTEXITCODE
}

if ($FormalExport) {
    # This is deliberately before venv creation/install and CadQuery import.  A
    # current pending contract exits here without attempting a formal export.
    Invoke-LockedCadQueryPython $SerialExport --repo-root $RepositoryRoot --preflight-only
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if ((Test-Path -LiteralPath $VenvRoot) -and -not $ReuseExistingVenv) {
    throw "Refusing to overwrite an existing local venv: $VenvRoot. Inspect it or rerun with -ReuseExistingVenv."
}
if (-not (Test-Path -LiteralPath $VenvRoot)) {
    Invoke-LockedCadQueryPython -m venv $VenvRoot
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

$VenvPython = Join-Path $VenvRoot "Scripts\python.exe"
if (-not (Test-Path -LiteralPath $VenvPython)) {
    throw "The project-local virtual environment is missing its interpreter: $VenvPython"
}
& $VenvPython -m pip install --disable-pip-version-check --only-binary=:all: --require-hashes -r $LockFile
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $VenvPython $Roundtrip --repo-root $RepositoryRoot --output-dir $RoundtripDir --report $RoundtripReport
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($FormalExport) {
    & $VenvPython $SerialExport --repo-root $RepositoryRoot --roundtrip-report $RoundtripReport --output-dir $SerialExportDir
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

& $VenvPython -m pip freeze --all | Set-Content -LiteralPath (Join-Path $WorkRoot "cadquery-pip-freeze.txt") -Encoding utf8
Get-FileHash -LiteralPath $LockFile -Algorithm SHA256 | Format-List
Write-Host "Windows-native CadQuery B-rep smoke test passed: $RoundtripReport"
if ($FormalExport) { Write-Host "Released serial component export receipt: $(Join-Path $SerialExportDir 'sha256-receipt.json')" }
Write-Host "This result is only a local toolchain check; it does not change formal vehicle readiness."
