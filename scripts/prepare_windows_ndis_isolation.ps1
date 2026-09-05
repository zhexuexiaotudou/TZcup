[CmdletBinding()]
param(
    [ValidateSet("Tailscale", "FSE")]
    [string]$Target,
    [switch]$Execute,
    [ValidateRange(1, 60)]
    [int]$IsolationSeconds = 15,
    [string]$OutputPath = (Join-Path $PSScriptRoot "..\reports\engineering\windows_ndis_isolation_harness_dry_run.json")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# These are deliberately exact identities, not discovery patterns.  The
# harness must never select a general VPN, Wi-Fi, Intel, Realtek, or Ethernet
# adapter merely because its display name happens to look related.
$TargetCatalog = @{
    Tailscale = [pscustomobject]@{
        target = "Tailscale"
        adapter_names = @("Tailscale")
        service_names = @("Tailscale")
    }
    FSE = [pscustomobject]@{
        target = "FSE"
        adapter_names = @("vEthernet (FSE HostVnic)", "vSwitch (FSE Switch)")
        service_names = @()
    }
}
$ProtectedAdapterNames = @("WLAN")
$ProtectedInterfaceDescriptions = @(
    "Intel(R) Wi-Fi 6E AX211 160MHz",
    "Realtek Gaming 2.5GbE Family Controller"
)

function Get-ExactNetAdapter {
    param([Parameter(Mandatory = $true)][string]$Name)

    $matches = @(Get-NetAdapter -IncludeHidden | Where-Object { $_.Name -ceq $Name })
    if ($matches.Count -ne 1) {
        throw "Expected exactly one adapter named '$Name'; found $($matches.Count)."
    }
    return $matches[0]
}

function Get-ExactService {
    param([Parameter(Mandatory = $true)][string]$Name)

    # $Name is one of the fixed catalog values above.  CIM equality avoids
    # Get-Service's wildcard-capable name parameter.
    $matches = @(Get-CimInstance -ClassName Win32_Service -Filter "Name='$Name'")
    if ($matches.Count -ne 1) {
        throw "Expected exactly one service named '$Name'; found $($matches.Count)."
    }
    return $matches[0]
}

function Get-DefaultRouteInterfaceIndexes {
    $routes = @(Get-NetRoute -AddressFamily IPv4 | Where-Object { $_.DestinationPrefix -eq "0.0.0.0/0" })
    if ($routes.Count -eq 0) {
        throw "No IPv4 default route was found; refusing any state-changing isolation."
    }
    return @($routes | ForEach-Object { [int]$_.InterfaceIndex } | Sort-Object -Unique)
}

function Assert-SafeIsolationTarget {
    param(
        [Parameter(Mandatory = $true)]$Plan,
        [Parameter(Mandatory = $true)][int[]]$DefaultRouteInterfaceIndexes
    )

    foreach ($adapterName in $Plan.adapter_names) {
        $adapter = Get-ExactNetAdapter -Name $adapterName
        if ($ProtectedAdapterNames -ccontains $adapter.Name) {
            throw "Refusing protected WLAN adapter '$($adapter.Name)'."
        }
        if ($ProtectedInterfaceDescriptions -ccontains $adapter.InterfaceDescription) {
            throw "Refusing protected physical adapter '$($adapter.InterfaceDescription)'."
        }
        if ($DefaultRouteInterfaceIndexes -contains [int]$adapter.ifIndex) {
            throw "Refusing '$($adapter.Name)' because it owns an active IPv4 default route."
        }
    }
}

function Get-MemorySnapshot {
    $memory = Get-CimInstance -ClassName Win32_PerfRawData_PerfOS_Memory
    return [pscustomobject]@{
        captured_at_utc = (Get-Date).ToUniversalTime().ToString("o")
        pool_nonpaged_bytes = [Int64]$memory.PoolNonpagedBytes
        pool_nonpaged_gib = [math]::Round(([double]$memory.PoolNonpagedBytes / 1GB), 2)
        pool_paged_bytes = [Int64]$memory.PoolPagedBytes
        available_bytes = [Int64]$memory.AvailableBytes
        committed_bytes = [Int64]$memory.CommittedBytes
    }
}

function Get-TargetSnapshot {
    param([AllowNull()]$Plan)

    $adapters = @()
    $services = @()
    if ($null -ne $Plan) {
        $adapters = @($Plan.adapter_names | ForEach-Object {
                $adapter = Get-ExactNetAdapter -Name $_
                [pscustomobject]@{
                    name = $adapter.Name
                    interface_description = $adapter.InterfaceDescription
                    interface_index = [int]$adapter.ifIndex
                    status = "$($adapter.Status)"
                    admin_status = "$($adapter.AdminStatus)"
                    mac_address = $adapter.MacAddress
                    link_speed = $adapter.LinkSpeed
                }
            })
        $services = @($Plan.service_names | ForEach-Object {
                $service = Get-ExactService -Name $_
                [pscustomobject]@{
                    name = $service.Name
                    display_name = $service.DisplayName
                    state = "$($service.State)"
                    start_mode = "$($service.StartMode)"
                    process_id = $service.ProcessId
                }
            })
    }
    return [pscustomobject]@{
        memory = Get-MemorySnapshot
        adapters = $adapters
        services = $services
    }
}

function Invoke-TargetIsolation {
    param(
        [Parameter(Mandatory = $true)]$Plan,
        [Parameter(Mandatory = $true)]$Before,
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[object]]$AppliedActions,
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[string]]$StoppedServiceNames,
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[string]]$DisabledAdapterNames,
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[string]]$RecoveryCommands
    )

    if ($Plan.target -ceq "Tailscale") {
        foreach ($service in $Before.services) {
            if ($service.state -ceq "Running") {
                Stop-Service -Name $service.name -ErrorAction Stop
                $StoppedServiceNames.Add($service.name)
                $RecoveryCommands.Add("Start-Service -Name '$($service.name)' -ErrorAction Stop")
                $AppliedActions.Add([pscustomobject]@{ action = "stop_service"; exact_name = $service.name })
            }
        }
        return
    }

    if ($Plan.target -ceq "FSE") {
        foreach ($adapter in $Before.adapters) {
            if ($adapter.status -cne "Disabled") {
                Disable-NetAdapter -Name $adapter.name -Confirm:$false -ErrorAction Stop
                $DisabledAdapterNames.Add($adapter.name)
                $RecoveryCommands.Add("Enable-NetAdapter -Name '$($adapter.name)' -Confirm:`$false -ErrorAction Stop")
                $AppliedActions.Add([pscustomobject]@{ action = "disable_adapter"; exact_name = $adapter.name })
            }
        }
        return
    }

    throw "Unsupported target '$($Plan.target)'."
}

function Restore-TargetState {
    param(
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[string]]$StoppedServiceNames,
        [Parameter(Mandatory = $true)][System.Collections.Generic.List[string]]$DisabledAdapterNames
    )

    # Re-enable in reverse dependency order: FSE isolation disables the host
    # vNIC before the vSwitch adapter, so restoration must unwind that stack.
    for ($index = $DisabledAdapterNames.Count - 1; $index -ge 0; $index--) {
        Enable-NetAdapter -Name $DisabledAdapterNames[$index] -Confirm:$false -ErrorAction Stop
    }
    foreach ($serviceName in $StoppedServiceNames) {
        Start-Service -Name $serviceName -ErrorAction Stop
    }
}

function Test-RestoredState {
    param(
        [Parameter(Mandatory = $true)]$Before,
        [Parameter(Mandatory = $true)]$Restored,
        [Parameter(Mandatory = $true)][AllowEmptyCollection()][System.Collections.Generic.List[object]]$Drift
    )

    foreach ($baselineService in $Before.services) {
        if ($baselineService.state -ceq "Running") {
            $current = @($Restored.services | Where-Object { $_.name -ceq $baselineService.name })
            if ($current.Count -ne 1 -or $current[0].state -cne "Running") {
                $Drift.Add([pscustomobject]@{
                        resource_type = "service"
                        exact_name = $baselineService.name
                        required_state = "Running"
                        observed_state = if ($current.Count -eq 1) { $current[0].state } else { "missing_or_ambiguous" }
                    })
            }
        }
    }
    foreach ($baselineAdapter in $Before.adapters) {
        if ($baselineAdapter.status -ceq "Up") {
            $current = @($Restored.adapters | Where-Object { $_.name -ceq $baselineAdapter.name })
            if ($current.Count -ne 1 -or $current[0].status -cne "Up") {
                $Drift.Add([pscustomobject]@{
                        resource_type = "adapter"
                        exact_name = $baselineAdapter.name
                        required_state = "Up"
                        observed_state = if ($current.Count -eq 1) { $current[0].status } else { "missing_or_ambiguous" }
                    })
            }
        }
    }
}

$errors = [System.Collections.Generic.List[object]]::new()
$recoveryErrors = [System.Collections.Generic.List[object]]::new()
$appliedActions = [System.Collections.Generic.List[object]]::new()
$stoppedServiceNames = [System.Collections.Generic.List[string]]::new()
$disabledAdapterNames = [System.Collections.Generic.List[string]]::new()
$recoveryCommands = [System.Collections.Generic.List[string]]::new()
$stateDrift = [System.Collections.Generic.List[object]]::new()
$exitCode = 0
$plan = $null
$defaultRouteInterfaceIndexes = @()
$before = $null
$after = $null
$restored = $null
$isolationSecondsElapsed = 0.0

try {
    # This is intentionally performed even in dry-run mode so the report
    # records the route safety boundary before anyone considers execution.
    $defaultRouteInterfaceIndexes = Get-DefaultRouteInterfaceIndexes
    if ($Execute -and [string]::IsNullOrWhiteSpace($Target)) {
        throw "-Execute requires an explicit -Target Tailscale or -Target FSE."
    }
    if (-not [string]::IsNullOrWhiteSpace($Target)) {
        $plan = $TargetCatalog[$Target]
        if ($null -eq $plan) {
            throw "Target '$Target' is not in the exact allowlist."
        }
        Assert-SafeIsolationTarget -Plan $plan -DefaultRouteInterfaceIndexes $defaultRouteInterfaceIndexes
    }

    $before = Get-TargetSnapshot -Plan $plan
    if (-not $Execute) {
        $after = $before
    }
    else {
        Invoke-TargetIsolation -Plan $plan -Before $before -AppliedActions $appliedActions -StoppedServiceNames $stoppedServiceNames -DisabledAdapterNames $disabledAdapterNames -RecoveryCommands $recoveryCommands
        # Give kernel allocations a bounded, recorded observation interval.
        # This command is reachable only after explicit -Execute and target
        # validation, never during dry-run or a rejected execution request.
        $isolationStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
        Start-Sleep -Seconds $IsolationSeconds
        $isolationStopwatch.Stop()
        $isolationSecondsElapsed = [math]::Round($isolationStopwatch.Elapsed.TotalSeconds, 3)
        $after = Get-TargetSnapshot -Plan $plan
    }
}
catch {
    $errors.Add([pscustomobject]@{ phase = "prepare_or_execute"; error = $_.Exception.Message })
    $exitCode = 1
}
finally {
    if ($Execute -and ($StoppedServiceNames.Count -gt 0 -or $DisabledAdapterNames.Count -gt 0)) {
        try {
            Restore-TargetState -StoppedServiceNames $stoppedServiceNames -DisabledAdapterNames $disabledAdapterNames
        }
        catch {
            $recoveryErrors.Add([pscustomobject]@{ phase = "restore"; error = $_.Exception.Message })
            # Recovery failure takes precedence: callers must not treat a
            # failed restore as a successful isolation experiment.
            $exitCode = 2
        }
    }
    try {
        $restored = Get-TargetSnapshot -Plan $plan
        if ($null -ne $before -and $null -ne $restored) {
            Test-RestoredState -Before $before -Restored $restored -Drift $stateDrift
            if ($stateDrift.Count -gt 0) {
                $exitCode = 2
            }
        }
    }
    catch {
        $recoveryErrors.Add([pscustomobject]@{ phase = "post_restore_snapshot"; error = $_.Exception.Message })
        $exitCode = 2
    }

    $report = [ordered]@{
        schema_version = 1
        report_kind = "windows_ndis_isolation_harness"
        captured_at_utc = (Get-Date).ToUniversalTime().ToString("o")
        mode = if ($Execute) { "execute" } else { "dry_run" }
        execute_requested = [bool]$Execute
        selected_target = if ($null -ne $plan) { $plan.target } else { $null }
        isolation_wait = [ordered]@{
            requested_seconds = $IsolationSeconds
            actual_elapsed_seconds = $isolationSecondsElapsed
            sleep_performed = [bool]($Execute -and $isolationSecondsElapsed -gt 0)
        }
        exact_target_catalog = [ordered]@{
            Tailscale = $TargetCatalog.Tailscale
            FSE = $TargetCatalog.FSE
        }
        default_route_interface_indexes = $defaultRouteInterfaceIndexes
        protections = [ordered]@{
            protected_adapter_names = $ProtectedAdapterNames
            protected_interface_descriptions = $ProtectedInterfaceDescriptions
            default_route_rejected = $true
        }
        before = $before
        after_isolation = $after
        after_restore = $restored
        applied_actions = @($appliedActions)
        recovery = [ordered]@{
            attempted = [bool]$Execute
            commands = @($recoveryCommands)
            errors = @($recoveryErrors)
            state_drift = @($stateDrift)
            success = $recoveryErrors.Count -eq 0 -and $stateDrift.Count -eq 0
        }
        errors = @($errors)
    }
    $resolvedOutput = [System.IO.Path]::GetFullPath($OutputPath)
    $outputDirectory = Split-Path -Parent $resolvedOutput
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $resolvedOutput -Encoding utf8
}

if ($exitCode -ne 0) {
    exit $exitCode
}
