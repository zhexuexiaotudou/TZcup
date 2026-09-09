[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$BoardIp,
    [Parameter(Mandatory = $true)][string]$User,
    [Parameter(Mandatory = $true)][string]$Bundle,
    [string]$Profile = "auto",
    [switch]$Execute
)

$ErrorActionPreference = "Stop"
$bundlePath = (Resolve-Path -LiteralPath $Bundle).Path
if ($BoardIp -notmatch '^[A-Za-z0-9_.:-]+$' -or $User -notmatch '^[A-Za-z0-9._-]+$' -or $Profile -notmatch '^(auto|journey6_[a-z0-9_]+)$') {
    throw "Invalid board host, user, or profile syntax"
}
$manifestPath = Join-Path $bundlePath "bundle_manifest.json"
$sumsPath = Join-Path $bundlePath "SHA256SUMS"
if (-not (Test-Path -LiteralPath $manifestPath -PathType Leaf) -or -not (Test-Path -LiteralPath $sumsPath -PathType Leaf)) {
    throw "Invalid Journey 6 bundle directory: $bundlePath"
}

foreach ($line in Get-Content -LiteralPath $sumsPath) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line -notmatch '^([0-9a-f]{64})  (.+)$') { throw "Invalid SHA256SUMS row: $line" }
    $expected = $Matches[1]
    $relative = $Matches[2].Replace('/', [IO.Path]::DirectorySeparatorChar)
    $candidate = Join-Path $bundlePath $relative
    if (-not (Test-Path -LiteralPath $candidate -PathType Leaf)) { throw "Bundle file missing: $relative" }
    $actual = (Get-FileHash -LiteralPath $candidate -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $expected) { throw "Bundle checksum mismatch: $relative" }
}

if (-not $Execute) {
    Write-Output "dry-run: checksum verified locally; no board connection or mutation performed"
    Write-Output "target=$User@$BoardIp profile=$Profile bundle=$bundlePath"
    exit 0
}

$manifest = Get-Content -Raw -LiteralPath $manifestPath | ConvertFrom-Json
if ($manifest.target_family -ne "journey6") { throw "Bundle target is not journey6" }
if ($manifest.status -eq "skeleton" -or $manifest.external_blockers.Count -gt 0) {
    throw "blocked_external: bundle is still a skeleton"
}
$target = "$User@$BoardIp"
$remoteRoot = "/var/tmp/tzcup-j6-$($manifest.bundle_id)-$PID"
& ssh -o StrictHostKeyChecking=yes $target "mkdir -p '$remoteRoot'"
if ($LASTEXITCODE -ne 0) { throw "Failed to create remote staging directory" }
& scp -o StrictHostKeyChecking=yes (Join-Path $bundlePath "scripts/j6_board_inventory.py") "${target}:${remoteRoot}/j6_board_inventory.py"
if ($LASTEXITCODE -ne 0) { throw "Failed to stage read-only inventory tool" }
& ssh -o StrictHostKeyChecking=yes $target "python3 '$remoteRoot/j6_board_inventory.py' --output '$remoteRoot/J6_BOARD_INVENTORY.json'"
if ($LASTEXITCODE -ne 0) { throw "blocked_external: Journey 6 board inventory was not ready" }
& scp -o StrictHostKeyChecking=yes -r $bundlePath "${target}:${remoteRoot}/bundle"
if ($LASTEXITCODE -ne 0) { throw "Failed to upload bundle" }
& ssh -o StrictHostKeyChecking=yes -t $target "sudo bash '$remoteRoot/bundle/scripts/install_candidate.sh' --source '$remoteRoot/bundle' --inventory '$remoteRoot/J6_BOARD_INVENTORY.json' --profile '$Profile' --execute"
if ($LASTEXITCODE -ne 0) { throw "Board installer failed; inspect retained remote staging and rollback evidence" }
Write-Output "deployment completed; board-side installer reported its rollback point"
