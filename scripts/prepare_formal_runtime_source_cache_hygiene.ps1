[CmdletBinding()]
param(
    [switch]$Execute,
    [string]$OutputPath = (Join-Path $PSScriptRoot "..\reports\engineering\formal_runtime_source_cache_hygiene_dry_run.json")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Test-ReparsePoint {
    param([Parameter(Mandatory = $true)]$Item)

    return (($Item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)
}

function Assert-FixedRepositoryRoot {
    param([Parameter(Mandatory = $true)][string]$ExpectedRoot)

    $gitRootLines = @(& git -C $ExpectedRoot rev-parse --show-toplevel)
    if ($LASTEXITCODE -ne 0 -or $gitRootLines.Count -ne 1) {
        throw "Unable to resolve exactly one Git repository root from '$ExpectedRoot'."
    }
    $gitRoot = [System.IO.Path]::GetFullPath($gitRootLines[0].Trim())
    if (-not $gitRoot.Equals($ExpectedRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing cache hygiene outside this repository: Git root '$gitRoot' differs from script repository '$ExpectedRoot'."
    }
    return $gitRoot
}

function Assert-PathWithinSourceRoot {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$SourceRoot,
        [switch]$AllowSourceRoot
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $sourceWithSeparator = $SourceRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    $isSourceRoot = $fullPath.Equals($SourceRoot, [System.StringComparison]::OrdinalIgnoreCase)
    $isChild = $fullPath.StartsWith($sourceWithSeparator, [System.StringComparison]::OrdinalIgnoreCase)
    if ((-not $isChild) -and (-not ($AllowSourceRoot -and $isSourceRoot))) {
        throw "Refusing path outside fixed starter_ws/src root: '$fullPath'."
    }
    return $fullPath
}

function Assert-ItemSafe {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$SourceRoot,
        [switch]$AllowSourceRoot
    )

    $item = Get-Item -LiteralPath $Path -Force
    if (Test-ReparsePoint -Item $item) {
        throw "Refusing reparse point or symlink: '$($item.FullName)'."
    }
    [void](Assert-PathWithinSourceRoot -Path $item.FullName -SourceRoot $SourceRoot -AllowSourceRoot:$AllowSourceRoot)
    return $item
}

function Assert-NoReparseDescendants {
    param(
        [Parameter(Mandatory = $true)][string]$StartPath,
        [Parameter(Mandatory = $true)][string]$SourceRoot
    )

    $pending = New-Object 'System.Collections.Generic.Queue[string]'
    $pending.Enqueue($StartPath)
    while ($pending.Count -gt 0) {
        $current = Assert-ItemSafe -Path $pending.Dequeue() -SourceRoot $SourceRoot
        if (-not $current.PSIsContainer) {
            continue
        }
        foreach ($entry in @(Get-ChildItem -LiteralPath $current.FullName -Force)) {
            [void](Assert-ItemSafe -Path $entry.FullName -SourceRoot $SourceRoot)
            if ($entry.PSIsContainer) {
                $pending.Enqueue($entry.FullName)
            }
        }
    }
}

function Get-RelativeRepositoryPath {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$RepositoryRoot
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $rootWithSeparator = $RepositoryRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if (-not $fullPath.StartsWith($rootWithSeparator, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing path outside repository while building manifest: '$fullPath'."
    }
    return $fullPath.Substring($rootWithSeparator.Length).Replace('\', '/')
}

function Get-TrackedConflicts {
    param(
        [Parameter(Mandatory = $true)][string]$RepositoryRoot,
        [Parameter(Mandatory = $true)][AllowEmptyCollection()][object[]]$Candidates
    )

    $trackedPaths = @(& git -C $RepositoryRoot ls-files --full-name)
    if ($LASTEXITCODE -ne 0) {
        throw "git ls-files failed; refusing cleanup without tracked-file verification."
    }
    $conflicts = New-Object 'System.Collections.Generic.List[object]'
    foreach ($candidate in $Candidates) {
        foreach ($trackedPath in $trackedPaths) {
            $matches = if ($candidate.kind -ceq "cache_directory") {
                $trackedPath -ceq $candidate.relative_path -or $trackedPath.StartsWith("$($candidate.relative_path)/", [System.StringComparison]::Ordinal)
            }
            else {
                $trackedPath -ceq $candidate.relative_path
            }
            if ($matches) {
                $conflicts.Add([pscustomobject]@{
                        candidate_relative_path = $candidate.relative_path
                        tracked_path = $trackedPath
                    })
            }
        }
    }
    return $conflicts.ToArray()
}

function Get-CandidateSizeBytes {
    param([Parameter(Mandatory = $true)]$Item)

    if (-not $Item.PSIsContainer) {
        return [Int64]$Item.Length
    }
    $total = [Int64]0
    $pending = New-Object 'System.Collections.Generic.Queue[string]'
    $pending.Enqueue($Item.FullName)
    while ($pending.Count -gt 0) {
        $current = Get-Item -LiteralPath $pending.Dequeue() -Force
        foreach ($entry in @(Get-ChildItem -LiteralPath $current.FullName -Force)) {
            if (Test-ReparsePoint -Item $entry) {
                throw "Refusing reparse point or symlink while sizing candidate: '$($entry.FullName)'."
            }
            if ($entry.PSIsContainer) {
                $pending.Enqueue($entry.FullName)
            }
            else {
                $total += [Int64]$entry.Length
            }
        }
    }
    return $total
}

$repositoryRoot = $null
$sourceRoot = $null
$candidates = New-Object 'System.Collections.Generic.List[object]'
$deletedManifest = New-Object 'System.Collections.Generic.List[object]'
$trackedConflicts = @()
$errors = New-Object 'System.Collections.Generic.List[string]'
$deletionPerformed = $false
$exitCode = 0

try {
    $repositoryRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
    $repositoryRoot = Assert-FixedRepositoryRoot -ExpectedRoot $repositoryRoot
    $sourceRoot = Join-Path $repositoryRoot "starter_ws\src"
    $sourceItem = Assert-ItemSafe -Path $sourceRoot -SourceRoot $sourceRoot -AllowSourceRoot
    if (-not $sourceItem.PSIsContainer) {
        throw "Fixed source root is not a directory: '$sourceRoot'."
    }
    $sourceRoot = [System.IO.Path]::GetFullPath($sourceItem.FullName)

    $pendingDirectories = New-Object 'System.Collections.Generic.Queue[string]'
    $pendingDirectories.Enqueue($sourceRoot)
    while ($pendingDirectories.Count -gt 0) {
        $directory = Assert-ItemSafe -Path $pendingDirectories.Dequeue() -SourceRoot $sourceRoot -AllowSourceRoot
        foreach ($entry in @(Get-ChildItem -LiteralPath $directory.FullName -Force)) {
            $safeEntry = Assert-ItemSafe -Path $entry.FullName -SourceRoot $sourceRoot
            if ($safeEntry.PSIsContainer) {
                if ($safeEntry.Name -ceq "__pycache__" -or $safeEntry.Name -ceq ".pytest_cache") {
                    Assert-NoReparseDescendants -StartPath $safeEntry.FullName -SourceRoot $sourceRoot
                    $candidates.Add([pscustomobject]@{
                            kind = "cache_directory"
                            absolute_path = $safeEntry.FullName
                            relative_path = Get-RelativeRepositoryPath -Path $safeEntry.FullName -RepositoryRoot $repositoryRoot
                            size_bytes = Get-CandidateSizeBytes -Item $safeEntry
                        })
                }
                else {
                    $pendingDirectories.Enqueue($safeEntry.FullName)
                }
            }
            elseif ($safeEntry.Extension -ceq ".pyc") {
                $parentName = Split-Path -Path $safeEntry.DirectoryName -Leaf
                if ($parentName -cne "__pycache__") {
                    $candidates.Add([pscustomobject]@{
                            kind = "orphan_pyc"
                            absolute_path = $safeEntry.FullName
                            relative_path = Get-RelativeRepositoryPath -Path $safeEntry.FullName -RepositoryRoot $repositoryRoot
                            size_bytes = Get-CandidateSizeBytes -Item $safeEntry
                        })
                }
            }
        }
    }

    $trackedConflicts = @(Get-TrackedConflicts -RepositoryRoot $repositoryRoot -Candidates $candidates.ToArray())
    if ($trackedConflicts.Count -gt 0) {
        throw "Refusing cleanup because git ls-files found $($trackedConflicts.Count) tracked candidate path(s)."
    }

    if ($Execute) {
        foreach ($candidate in $candidates) {
            $item = Assert-ItemSafe -Path $candidate.absolute_path -SourceRoot $sourceRoot
            Assert-NoReparseDescendants -StartPath $item.FullName -SourceRoot $sourceRoot
            $currentConflict = @(Get-TrackedConflicts -RepositoryRoot $repositoryRoot -Candidates @($candidate))
            if ($currentConflict.Count -gt 0) {
                throw "Refusing deletion because git ls-files now tracks '$($candidate.relative_path)'."
            }
            if ($candidate.kind -ceq "cache_directory") {
                Remove-Item -LiteralPath $item.FullName -Recurse -Force -ErrorAction Stop
            }
            else {
                Remove-Item -LiteralPath $item.FullName -Force -ErrorAction Stop
            }
            $deletedManifest.Add([pscustomobject]@{
                    kind = $candidate.kind
                    relative_path = $candidate.relative_path
                    size_bytes = $candidate.size_bytes
                })
        }
        $deletionPerformed = $true
    }
}
catch {
    $errors.Add($_.Exception.Message)
    $exitCode = 1
}
finally {
    $resolvedOutputPath = [System.IO.Path]::GetFullPath($OutputPath)
    $outputDirectory = Split-Path -Path $resolvedOutputPath -Parent
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
    $status = if ($errors.Count -gt 0) {
        "FORMAL_RUNTIME_SOURCE_CACHE_HYGIENE_REFUSED"
    }
    elseif ($Execute) {
        "FORMAL_RUNTIME_SOURCE_CACHE_HYGIENE_EXECUTED"
    }
    else {
        "FORMAL_RUNTIME_SOURCE_CACHE_HYGIENE_DRY_RUN_READY"
    }
    $report = [pscustomobject]@{
        schema_version = 1
        generated_at_utc = (Get-Date).ToUniversalTime().ToString("o")
        status = $status
        mode = if ($Execute) { "execute" } else { "dry_run" }
        execute_requested = [bool]$Execute
        fixed_repository_root = $repositoryRoot
        fixed_source_root = $sourceRoot
        deletion_scope = [pscustomobject]@{
            allowed_directory_names = @("__pycache__", ".pytest_cache")
            allowed_file_rule = "orphan .pyc outside __pycache__"
            reparse_points_refused = $true
            tracked_paths_refused = $true
            out_of_root_paths_refused = $true
        }
        candidate_count = $candidates.Count
        planned_deletions = @($candidates.ToArray() | ForEach-Object {
                [pscustomobject]@{
                    kind = $_.kind
                    relative_path = $_.relative_path
                    size_bytes = $_.size_bytes
                }
            })
        git_tracked_check = [pscustomobject]@{
            completed = $null -ne $repositoryRoot
            tracked_conflicts = @($trackedConflicts)
        }
        deleted_manifest = @($deletedManifest.ToArray())
        deletion_performed = $deletionPerformed
        errors = @($errors)
    }
    $report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $resolvedOutputPath -Encoding utf8
    Write-Output ($report | ConvertTo-Json -Depth 8)
}

exit $exitCode
