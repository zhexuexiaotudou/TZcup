[CmdletBinding()]
param(
    [string]$OutputPath = (Join-Path $PSScriptRoot "..\reports\engineering\windows_ndis_nonpaged_pool_diagnostic.json"),
    [ValidateRange(1, 30)]
    [int]$EventLookbackDays = 7
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Invoke-ReadOnlyQuery {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][scriptblock]$Query,
        [Parameter(Mandatory = $true)][AllowEmptyCollection()][System.Collections.Generic.List[object]]$Errors
    )

    try {
        return & $Query
    }
    catch {
        $Errors.Add([pscustomobject]@{
                query = $Name
                error = $_.Exception.Message
            })
        return @()
    }
}

function Select-NetworkAdapterRecord {
    param([Parameter(Mandatory = $true)]$Adapter)

    [pscustomobject]@{
        name = $Adapter.Name
        interface_description = $Adapter.InterfaceDescription
        status = "$($Adapter.Status)"
        link_speed = $Adapter.LinkSpeed
        mac_address = $Adapter.MacAddress
        # Get-NetAdapter exposes DriverDate as a localized string on some
        # Windows builds, unlike the CIM DateTime property used below.
        driver_date = if ($Adapter.DriverDate) { "$($Adapter.DriverDate)" } else { $null }
        driver_version = $Adapter.DriverVersion
        ndis_version = $Adapter.NdisVersion
    }
}

function Select-DriverRecord {
    param([Parameter(Mandatory = $true)]$Driver)

    [pscustomobject]@{
        name = $Driver.Name
        display_name = $Driver.DisplayName
        state = "$($Driver.State)"
        start_mode = "$($Driver.StartMode)"
        path = $Driver.PathName
    }
}

function Select-ServiceRecord {
    param([Parameter(Mandatory = $true)]$Service)

    [pscustomobject]@{
        name = $Service.Name
        display_name = $Service.DisplayName
        state = "$($Service.State)"
        start_mode = "$($Service.StartMode)"
        path = $Service.PathName
    }
}

function Test-ContainsAny {
    param([AllowNull()][string]$Value, [Parameter(Mandatory = $true)][string[]]$Tokens)
    if ([string]::IsNullOrWhiteSpace($Value)) { return $false }
    foreach ($token in $Tokens) {
        if ($Value -match $token) { return $true }
    }
    return $false
}

$errors = [System.Collections.Generic.List[object]]::new()
$candidateTokens = @{
    tailscale = @("Tailscale", "Wintun", "WireGuard")
    fse = @("FSE", "Flow steering", "Hyper-V", "vms", "VMS")
    ikuuu_vpn = @("iKuuu", "ikuuu")
    realtek = @("Realtek", "rt25cx")
    intel_wlan = @("Intel.*Wi-Fi", "Netwtw")
}

$computer = Invoke-ReadOnlyQuery -Name "computer-system" -Errors $errors -Query {
    Get-CimInstance -ClassName Win32_ComputerSystem | Select-Object Manufacturer, Model, TotalPhysicalMemory
}
$os = Invoke-ReadOnlyQuery -Name "operating-system" -Errors $errors -Query {
    Get-CimInstance -ClassName Win32_OperatingSystem | Select-Object Caption, Version, BuildNumber, LastBootUpTime
}
$memory = Invoke-ReadOnlyQuery -Name "memory-counters" -Errors $errors -Query {
    Get-CimInstance -ClassName Win32_PerfRawData_PerfOS_Memory |
        Select-Object PoolNonpagedBytes, PoolPagedBytes, CommittedBytes, AvailableBytes
}
$adapters = @(Invoke-ReadOnlyQuery -Name "network-adapters" -Errors $errors -Query {
        Get-NetAdapter -IncludeHidden | ForEach-Object { Select-NetworkAdapterRecord $_ }
    })
$activeAdapters = @($adapters | Where-Object { $_.status -eq "Up" })
$bindings = @(Invoke-ReadOnlyQuery -Name "network-bindings" -Errors $errors -Query {
        Get-NetAdapterBinding -IncludeHidden | ForEach-Object {
            [pscustomobject]@{
                adapter_name = $_.Name
                display_name = $_.DisplayName
                component_id = $_.ComponentID
                enabled = [bool]$_.Enabled
            }
        }
    })
$systemDrivers = @(Invoke-ReadOnlyQuery -Name "system-drivers" -Errors $errors -Query {
        Get-CimInstance -ClassName Win32_SystemDriver | ForEach-Object { Select-DriverRecord $_ }
    })
$services = @(Invoke-ReadOnlyQuery -Name "services" -Errors $errors -Query {
        Get-CimInstance -ClassName Win32_Service | ForEach-Object { Select-ServiceRecord $_ }
    })
$pnpDrivers = @(Invoke-ReadOnlyQuery -Name "pnp-network-drivers" -Errors $errors -Query {
        Get-CimInstance -ClassName Win32_PnPSignedDriver |
            Where-Object { $_.DeviceClass -eq "NET" } |
            ForEach-Object {
                [pscustomobject]@{
                    device_name = $_.DeviceName
                    manufacturer = $_.Manufacturer
                    driver_version = $_.DriverVersion
                    driver_date = if ($_.DriverDate) { $_.DriverDate.ToUniversalTime().ToString("o") } else { $null }
                    inf_name = $_.InfName
                    device_id = $_.DeviceID
                }
            }
    })

$eventStart = (Get-Date).AddDays(-$EventLookbackDays)
$eventPattern = "NDIS|network adapter|network|Tailscale|FSE|iKuuu|Realtek|Intel|Hyper-V|Netwtw|rt25cx"
$systemEvents = @(Invoke-ReadOnlyQuery -Name "system-network-events" -Errors $errors -Query {
        Get-WinEvent -FilterHashtable @{ LogName = "System"; StartTime = $eventStart } -MaxEvents 4000 |
            Where-Object {
                $_.LevelDisplayName -in @("Error", "Warning") -and
                ("$($_.ProviderName) $($_.Message)" -match $eventPattern)
            } |
            Select-Object -First 100 |
            ForEach-Object {
                [pscustomobject]@{
                    time_utc = $_.TimeCreated.ToUniversalTime().ToString("o")
                    provider = $_.ProviderName
                    id = $_.Id
                    level = $_.LevelDisplayName
                    message = ($_.Message -replace "\r?\n", " ")
                }
            }
    })
$ndisLog = @(Invoke-ReadOnlyQuery -Name "ndis-operational-log" -Errors $errors -Query {
        Get-WinEvent -ListLog "Microsoft-Windows-NDIS/Operational" |
            Select-Object LogName, IsEnabled, RecordCount, LastWriteTime
    })

$poolmon = Get-Command -Name "poolmon.exe" -ErrorAction SilentlyContinue
$poolTagEvidence = if ($null -eq $poolmon) {
    [pscustomobject]@{
        status = "unavailable_poolmon_not_installed"
        direct_driver_attribution = $false
        note = "No PoolMon binary was found on PATH. This diagnostic does not install or run third-party tooling."
    }
}
else {
    [pscustomobject]@{
        status = "available_not_captured"
        direct_driver_attribution = $false
        command = $poolmon.Source
        note = "PoolMon is available but was intentionally not parsed by this report; collect a controlled before/after tag capture before assigning causality."
    }
}

$candidateAssessments = foreach ($candidateName in $candidateTokens.Keys) {
    $tokens = $candidateTokens[$candidateName]
    $adapterEvidence = @($activeAdapters | Where-Object { Test-ContainsAny -Value ("$($_.name) $($_.interface_description)") -Tokens $tokens })
    $driverEvidence = @($systemDrivers | Where-Object { Test-ContainsAny -Value ("$($_.name) $($_.display_name) $($_.path)") -Tokens $tokens })
    $serviceEvidence = @($services | Where-Object { Test-ContainsAny -Value ("$($_.name) $($_.display_name) $($_.path)") -Tokens $tokens })
    $pnpEvidence = @($pnpDrivers | Where-Object { Test-ContainsAny -Value ("$($_.device_name) $($_.manufacturer) $($_.inf_name)") -Tokens $tokens })
    $bindingEvidence = @($bindings | Where-Object { Test-ContainsAny -Value ("$($_.adapter_name) $($_.display_name) $($_.component_id)") -Tokens $tokens })
    $hasKernelOrAdapterEvidence = $adapterEvidence.Count -gt 0 -or $driverEvidence.Count -gt 0 -or $pnpEvidence.Count -gt 0 -or $bindingEvidence.Count -gt 0
    $classification = if ($hasKernelOrAdapterEvidence) { "possible" } else { "unproven" }

    [pscustomobject]@{
        candidate = $candidateName
        classification = $classification
        direct_pool_tag_evidence = $false
        reasoning = if ($classification -eq "possible") {
            "Observed active network/kernel evidence, but no pool-tag-to-driver mapping; this is not causal attribution."
        } else {
            "No observed active NDIS adapter, driver, PnP, or binding evidence for this candidate; service-only evidence is not enough to attribute nonpaged pool."
        }
        active_adapter_evidence = $adapterEvidence
        system_driver_evidence = $driverEvidence
        service_evidence = $serviceEvidence
        pnp_network_driver_evidence = $pnpEvidence
        binding_evidence = $bindingEvidence
    }
}

$report = [ordered]@{
    schema_version = 1
    report_kind = "windows_ndis_nonpaged_pool_read_only_diagnostic"
    collected_at_utc = (Get-Date).ToUniversalTime().ToString("o")
    collection_policy = [ordered]@{
        read_only = $true
        no_service_stop = $true
        no_adapter_disable_or_restart = $true
        no_registry_change = $true
        no_tool_install = $true
    }
    system = [ordered]@{
        computer = $computer
        operating_system = $os
    }
    memory_snapshot = [ordered]@{
        pool_nonpaged_bytes = if ($memory) { [Int64]$memory.PoolNonpagedBytes } else { $null }
        pool_nonpaged_gib = if ($memory) { [math]::Round(([double]$memory.PoolNonpagedBytes / 1GB), 2) } else { $null }
        pool_paged_bytes = if ($memory) { [Int64]$memory.PoolPagedBytes } else { $null }
        committed_bytes = if ($memory) { [Int64]$memory.CommittedBytes } else { $null }
        available_bytes = if ($memory) { [Int64]$memory.AvailableBytes } else { $null }
    }
    pool_tag_evidence = $poolTagEvidence
    active_network_adapters = $activeAdapters
    candidate_assessments = @($candidateAssessments)
    event_evidence = [ordered]@{
        lookback_days = $EventLookbackDays
        ndis_operational_log = $ndisLog
        relevant_system_warning_or_error_events = $systemEvents
    }
    next_steps_require_user_confirmation = @(
        "Capture two read-only snapshots after the same uptime interval; if PoolMon is available, retain tag counts before attributing any driver.",
        "If controlled isolation is approved, test non-default overlay components first: Tailscale, then FSE/Hyper-V virtual switching, then iKuuuVPN only if a kernel network component is discovered.",
        "Keep the default Intel WLAN enabled throughout the initial overlay tests; test WLAN or Realtek only after the overlay results and a vendor-driver review.",
        "After any approved change, compare nonpaged-pool trend across the same workload and reboot cycle; a single low sample is not proof of cause."
    )
    collection_errors = @($errors)
}

$resolvedOutput = [System.IO.Path]::GetFullPath($OutputPath)
$outputDirectory = Split-Path -Parent $resolvedOutput
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $resolvedOutput -Encoding utf8
Write-Output $resolvedOutput
