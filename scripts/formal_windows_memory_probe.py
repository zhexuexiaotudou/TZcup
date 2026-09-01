#!/usr/bin/env python3
"""Read Windows commit, kernel-pool, Docker and vmmemWSL bytes read-only."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable


START_MIN_COMMIT_AVAILABLE_BYTES = 10 * 1024**3
START_MAX_DOCKER_PRIVATE_BYTES = 4 * 1024**3
# A short-lived Windows allocation can leave commit below the formal floor just
# before a Gazebo launch even though the owner is already unwinding.  Retry only
# that condition: Docker and WSL-state violations remain immediate refusals.
# These are constants rather than caller-controlled thresholds so the formal
# start gate cannot be weakened or made unbounded through its environment.
START_COMMIT_RECOVERY_TIMEOUT_S = 60.0
START_COMMIT_RECOVERY_INTERVAL_S = 5.0
READ_ONCE_MAX_ATTEMPTS = 2
READ_ONCE_RETRY_DELAY_S = 0.25
UINT64_MAX = (1 << 64) - 1
NDIS_NONPAGED_POOL_SUSPECT_FLOOR_BYTES = 2 * 1024**3
NDIS_TRACKED_POOL_TAG_SUSPECT_FLOOR_BYTES = 1024**3
NDIS_POOL_TAGS = ("Nbuf", "Nnbl", "Nnbf")
NATIVE_LINUX_RUNTIME_ENV = "FORMAL_NATIVE_LINUX_RUNTIME"


class ProbeError(RuntimeError):
    pass


def native_linux_runtime_requested() -> bool:
    """Allow an explicit native-Linux caller to mark Windows gates N/A."""

    raw = os.environ.get(NATIVE_LINUX_RUNTIME_ENV, "0")
    if raw not in {"0", "1"}:
        raise ProbeError(f"{NATIVE_LINUX_RUNTIME_ENV} must be 0 or 1")
    if raw == "0":
        return False
    if not sys.platform.startswith("linux"):
        raise ProbeError(f"{NATIVE_LINUX_RUNTIME_ENV}=1 requires native Linux")
    try:
        kernel_release = Path("/proc/sys/kernel/osrelease").read_text(
            encoding="utf-8"
        )
    except OSError as exc:
        raise ProbeError("cannot prove the native Linux kernel boundary") from exc
    if "microsoft" in kernel_release.casefold():
        raise ProbeError(f"{NATIVE_LINUX_RUNTIME_ENV}=1 is forbidden under WSL")
    return True


def _uint64(payload: dict[str, Any], key: str) -> int:
    value = payload.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 0
        or value > UINT64_MAX
    ):
        raise ProbeError(f"PowerShell memory sample has invalid {key}")
    return value


def _parse_pool_tag_diagnostics(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate optional, read-only pool-tag evidence without gating runtime."""

    diagnostics = payload.get("pool_tag_diagnostics")
    if not isinstance(diagnostics, dict):
        raise ProbeError("PowerShell memory sample has invalid pool_tag_diagnostics")
    status = diagnostics.get("status")
    if status not in ("available", "unavailable"):
        raise ProbeError("PowerShell memory sample has invalid pool tag status")
    result: dict[str, Any] = {"status": status}
    if status == "unavailable":
        failure = diagnostics.get("failure")
        if not isinstance(failure, dict) or not isinstance(failure.get("kind"), str):
            raise ProbeError("PowerShell memory sample has invalid pool tag failure")
        result["failure"] = {
            "kind": failure["kind"],
            "nt_status": failure.get("nt_status"),
        }
        if result["failure"]["nt_status"] is not None and not isinstance(
            result["failure"]["nt_status"], str
        ):
            raise ProbeError("PowerShell memory sample has invalid pool tag nt_status")
        return result

    tags = diagnostics.get("tracked_nonpaged_bytes")
    if not isinstance(tags, dict):
        raise ProbeError("PowerShell memory sample has invalid tracked_nonpaged_bytes")
    parsed_tags = {tag: _uint64(tags, tag) for tag in NDIS_POOL_TAGS}
    total = _uint64(diagnostics, "tracked_nonpaged_bytes_total")
    if total != sum(parsed_tags.values()):
        raise ProbeError("PowerShell memory sample has inconsistent tracked pool tags")
    suspected = diagnostics.get("suspected_ndis_nonpaged_pool_leak")
    if not isinstance(suspected, bool):
        raise ProbeError("PowerShell memory sample has invalid NDIS pool suspicion")
    result.update(
        {
            "tracked_nonpaged_bytes": parsed_tags,
            "tracked_nonpaged_bytes_total": total,
            "suspected_ndis_nonpaged_pool_leak": suspected,
        }
    )
    return result


def parse_powershell_sample(line: str) -> dict[str, Any]:
    """Validate one compact JSON object emitted by the PowerShell fixture."""

    try:
        payload: Any = json.loads(line.strip().lstrip("\ufeff"))
    except json.JSONDecodeError as exc:
        raise ProbeError(f"PowerShell memory sample is not JSON: {exc}") from exc
    required = (
        "epoch_ns",
        "commit_limit_bytes",
        "commit_charge_bytes",
        "commit_available_bytes",
        "docker_private_bytes",
        "vmmem_wsl_private_bytes",
    )
    if not isinstance(payload, dict):
        raise ProbeError("PowerShell memory sample is not an object")
    result: dict[str, Any] = {}
    for key in required:
        result[key] = _uint64(payload, key)
    if result["commit_charge_bytes"] > result["commit_limit_bytes"]:
        raise ProbeError("Windows commit charge exceeds the reported commit limit")
    expected_available = (
        result["commit_limit_bytes"] - result["commit_charge_bytes"]
    )
    if result["commit_available_bytes"] != expected_available:
        raise ProbeError("Windows commit available is inconsistent")
    # Older evidence fixtures intentionally omit these diagnostic-only fields.
    # Their absence must not alter the formal memory threshold contract.
    if "nonpaged_pool_bytes" in payload or "pool_tag_diagnostics" in payload:
        result["nonpaged_pool_bytes"] = _uint64(payload, "nonpaged_pool_bytes")
        if payload.get("nonpaged_pool_status") not in ("available", "unavailable"):
            raise ProbeError("PowerShell memory sample has invalid nonpaged pool status")
        result["nonpaged_pool_status"] = payload["nonpaged_pool_status"]
        result["pool_tag_diagnostics"] = _parse_pool_tag_diagnostics(payload)
    return result


def powershell_command(*, stream: bool, interval_s: float) -> list[str]:
    mode = "while ($true) { Emit-Sample; Start-Sleep -Milliseconds %d }" % max(
        100, round(interval_s * 1000)
    ) if stream else "Emit-Sample"
    # GlobalMemoryStatusEx is a cheap in-process kernel query.  In particular,
    # do not replace it with Win32_PerfRawData_PerfOS_Memory: asking CIM for
    # that class once per watchdog tick has enough latency and allocation
    # overhead to perturb a real-time Gazebo run.  One PowerShell process owns
    # this native type for the entire stream and only the small native query is
    # repeated.
    script = rf"""
$ErrorActionPreference = 'Stop'
Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;

public static class TzcupGlobalMemoryStatus
{{
    [StructLayout(LayoutKind.Sequential)]
    public struct MEMORYSTATUSEX
    {{
        public UInt32 dwLength;
        public UInt32 dwMemoryLoad;
        public UInt64 ullTotalPhys;
        public UInt64 ullAvailPhys;
        public UInt64 ullTotalPageFile;
        public UInt64 ullAvailPageFile;
        public UInt64 ullTotalVirtual;
        public UInt64 ullAvailVirtual;
        public UInt64 ullAvailExtendedVirtual;
    }}

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GlobalMemoryStatusEx(ref MEMORYSTATUSEX state);

    public static MEMORYSTATUSEX Read()
    {{
        MEMORYSTATUSEX state = new MEMORYSTATUSEX();
        state.dwLength = checked((UInt32)Marshal.SizeOf(typeof(MEMORYSTATUSEX)));
        if (!GlobalMemoryStatusEx(ref state))
        {{
            throw new Win32Exception(Marshal.GetLastWin32Error());
        }}
        return state;
    }}
}}

// All calls below are local kernel queries.  They neither enumerate adapters
// nor change driver, network, process, WSL, or Docker state.
public static class TzcupReadonlyKernelPool
{{
    private const int SystemPoolTagInformation = 22;
    private const int StatusInfoLengthMismatch = unchecked((int)0xC0000004);
    private const int MaxPoolTagBufferBytes = 16 * 1024 * 1024;

    [StructLayout(LayoutKind.Sequential)]
    private struct PERFORMANCE_INFORMATION
    {{
        public UInt32 cb;
        public UIntPtr CommitTotal;
        public UIntPtr CommitLimit;
        public UIntPtr CommitPeak;
        public UIntPtr PhysicalTotal;
        public UIntPtr PhysicalAvailable;
        public UIntPtr SystemCache;
        public UIntPtr KernelTotal;
        public UIntPtr KernelPaged;
        public UIntPtr KernelNonpaged;
        public UIntPtr PageSize;
        public UInt32 HandleCount;
        public UInt32 ProcessCount;
        public UInt32 ThreadCount;
    }}

    [StructLayout(LayoutKind.Sequential)]
    private struct SYSTEM_POOLTAG
    {{
        public UInt32 TagUlong;
        public UInt32 PagedAllocs;
        public UInt32 PagedFrees;
        public UIntPtr PagedUsed;
        public UInt32 NonPagedAllocs;
        public UInt32 NonPagedFrees;
        public UIntPtr NonPagedUsed;
    }}

    [StructLayout(LayoutKind.Sequential)]
    private struct SYSTEM_POOLTAG_INFORMATION_HEAD
    {{
        public UInt32 Count;
        public SYSTEM_POOLTAG FirstTag;
    }}

    [DllImport("psapi.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetPerformanceInfo(
        out PERFORMANCE_INFORMATION info, UInt32 cb);

    [DllImport("ntdll.dll")]
    private static extern int NtQuerySystemInformation(
        int systemInformationClass, IntPtr systemInformation,
        UInt32 systemInformationLength, out UInt32 returnLength);

    public sealed class PoolTagResult
    {{
        public bool Available {{ get; set; }}
        public string FailureKind {{ get; set; }}
        public string NtStatus {{ get; set; }}
        public UInt64 Nbuf {{ get; set; }}
        public UInt64 Nnbl {{ get; set; }}
        public UInt64 Nnbf {{ get; set; }}

        public PoolTagResult()
        {{
            FailureKind = "unavailable";
            NtStatus = "";
        }}
    }}

    public static UInt64 ReadNonPagedPoolBytes()
    {{
        PERFORMANCE_INFORMATION info;
        UInt32 size = checked((UInt32)Marshal.SizeOf(typeof(PERFORMANCE_INFORMATION)));
        if (!GetPerformanceInfo(out info, size))
        {{
            throw new Win32Exception(Marshal.GetLastWin32Error());
        }}
        UInt64 pages = info.KernelNonpaged.ToUInt64();
        UInt64 pageSize = info.PageSize.ToUInt64();
        if (pageSize == 0 || pages > UInt64.MaxValue / pageSize)
        {{
            throw new InvalidOperationException("Kernel nonpaged pool byte count overflowed UInt64");
        }}
        return pages * pageSize;
    }}

    private static string TagText(UInt32 tag)
    {{
        char[] chars = new char[4];
        for (int index = 0; index < chars.Length; index++)
        {{
            byte value = (byte)(tag >> (index * 8));
            chars[index] = value >= 32 && value <= 126 ? (char)value : '?';
        }}
        return new string(chars);
    }}

    public static PoolTagResult ReadTrackedPoolTags()
    {{
        PoolTagResult result = new PoolTagResult();
        IntPtr buffer = IntPtr.Zero;
        try
        {{
            UInt32 required = 0;
            int status = NtQuerySystemInformation(
                SystemPoolTagInformation, IntPtr.Zero, 0, out required);
            if (status != StatusInfoLengthMismatch || required == 0 || required > MaxPoolTagBufferBytes)
            {{
                result.FailureKind = required > MaxPoolTagBufferBytes ? "buffer_limit" : "ntquery_failed";
                result.NtStatus = "0x" + unchecked((UInt32)status).ToString("X8");
                return result;
            }}
            for (int attempt = 0; attempt < 3; attempt++)
            {{
                if (required == 0 || required > MaxPoolTagBufferBytes)
                {{
                    result.FailureKind = "buffer_limit";
                    return result;
                }}
                buffer = Marshal.AllocHGlobal(checked((int)required));
                UInt32 nextRequired;
                status = NtQuerySystemInformation(
                    SystemPoolTagInformation, buffer, required, out nextRequired);
                if (status == 0)
                {{
                    UInt32 count = unchecked((UInt32)Marshal.ReadInt32(buffer));
                    int headerSize = checked((int)Marshal.OffsetOf(
                        typeof(SYSTEM_POOLTAG_INFORMATION_HEAD), "FirstTag").ToInt64());
                    int entrySize = Marshal.SizeOf(typeof(SYSTEM_POOLTAG));
                    if (required < (UInt32)headerSize || count > (UInt32)(
                        ((int)required - headerSize) / entrySize))
                    {{
                        result.FailureKind = "malformed_count";
                        return result;
                    }}
                    for (UInt32 index = 0; index < count; index++)
                    {{
                        IntPtr entryPointer = IntPtr.Add(
                            buffer, checked(headerSize + (int)index * entrySize));
                        SYSTEM_POOLTAG entry = (SYSTEM_POOLTAG)Marshal.PtrToStructure(
                            entryPointer, typeof(SYSTEM_POOLTAG));
                        UInt64 used = entry.NonPagedUsed.ToUInt64();
                        switch (TagText(entry.TagUlong))
                        {{
                            case "Nbuf": result.Nbuf = used; break;
                            case "Nnbl": result.Nnbl = used; break;
                            case "Nnbf": result.Nnbf = used; break;
                        }}
                    }}
                    result.Available = true;
                    return result;
                }}
                Marshal.FreeHGlobal(buffer);
                buffer = IntPtr.Zero;
                if (status != StatusInfoLengthMismatch)
                {{
                    result.FailureKind = "ntquery_failed";
                    result.NtStatus = "0x" + unchecked((UInt32)status).ToString("X8");
                    return result;
                }}
                required = nextRequired;
            }}
            result.FailureKind = "retry_exhausted";
            return result;
        }}
        catch (Exception error)
        {{
            result.FailureKind = "managed_exception";
            result.NtStatus = error.GetType().Name;
            return result;
        }}
        finally
        {{
            if (buffer != IntPtr.Zero) {{ Marshal.FreeHGlobal(buffer); }}
        }}
    }}
}}
'@

function Get-DockerPrivateBytes {{
  [uint64]$sum = 0
  $rows = [System.Diagnostics.Process]::GetProcessesByName('com.docker.backend')
  try {{
    foreach ($row in $rows) {{
      try {{
        [uint64]$private = $row.PrivateMemorySize64
      }} catch [System.InvalidOperationException] {{
        # A backend that exited after enumeration no longer contributes to
        # current private commit.  Other access failures remain fatal.
        continue
      }}
      if ($private -gt ([uint64]::MaxValue - $sum)) {{
        throw 'Docker private byte sum overflowed UInt64'
      }}
      $sum = [uint64]($sum + $private)
    }}
  }} finally {{
    foreach ($row in $rows) {{ $row.Dispose() }}
  }}
  return [uint64]$sum
}}

function Get-VmmemWslPrivateBytes {{
  [uint64]$sum = 0
  $rows = [System.Diagnostics.Process]::GetProcessesByName('vmmemWSL')
  try {{
    foreach ($row in $rows) {{
      try {{
        [uint64]$private = $row.PrivateMemorySize64
      }} catch [System.InvalidOperationException] {{
        continue
      }}
      if ($private -gt ([uint64]::MaxValue - $sum)) {{
        throw 'vmmemWSL private byte sum overflowed UInt64'
      }}
      $sum = [uint64]($sum + $private)
    }}
  }} finally {{
    foreach ($row in $rows) {{ $row.Dispose() }}
  }}
  return [uint64]$sum
}}

function Get-ReadonlyKernelPoolDiagnostics {{
  [uint64]$nonpaged = 0
  $nonpagedStatus = 'available'
  try {{
    $nonpaged = [TzcupReadonlyKernelPool]::ReadNonPagedPoolBytes()
  }} catch {{
    # Diagnostic evidence must not weaken or fail the established commit gate.
    $nonpagedStatus = 'unavailable'
  }}
  $tags = [TzcupReadonlyKernelPool]::ReadTrackedPoolTags()
  if (-not $tags.Available) {{
    return [ordered]@{{
      nonpaged_pool_bytes = $nonpaged
      nonpaged_pool_status = $nonpagedStatus
      pool_tag_diagnostics = [ordered]@{{
        status = 'unavailable'
        failure = [ordered]@{{ kind = $tags.FailureKind; nt_status = $tags.NtStatus }}
      }}
    }}
  }}
  [uint64]$tracked = $tags.Nbuf + $tags.Nnbl + $tags.Nnbf
  $suspected = $nonpagedStatus -eq 'available' -and
    $nonpaged -ge {NDIS_NONPAGED_POOL_SUSPECT_FLOOR_BYTES} -and
    $tracked -ge {NDIS_TRACKED_POOL_TAG_SUSPECT_FLOOR_BYTES}
  return [ordered]@{{
    nonpaged_pool_bytes = $nonpaged
    nonpaged_pool_status = $nonpagedStatus
    pool_tag_diagnostics = [ordered]@{{
      status = 'available'
      tracked_nonpaged_bytes = [ordered]@{{ Nbuf = $tags.Nbuf; Nnbl = $tags.Nnbl; Nnbf = $tags.Nnbf }}
      tracked_nonpaged_bytes_total = $tracked
      suspected_ndis_nonpaged_pool_leak = [bool]$suspected
    }}
  }}
}}

function Emit-Sample {{
  $memory = [TzcupGlobalMemoryStatus]::Read()
  [uint64]$limit = $memory.ullTotalPageFile
  [uint64]$available = $memory.ullAvailPageFile
  if ($available -gt $limit) {{
    throw 'GlobalMemoryStatusEx reported available page file above total page file'
  }}
  [uint64]$charge = $limit - $available
  [uint64]$dockerPrivate = Get-DockerPrivateBytes
  [uint64]$vmmemWslPrivate = Get-VmmemWslPrivateBytes
  $kernelPool = Get-ReadonlyKernelPoolDiagnostics
  [ordered]@{{
    epoch_ns = [int64]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()) * 1000000
    commit_limit_bytes = $limit
    commit_charge_bytes = $charge
    commit_available_bytes = $available
    docker_private_bytes = $dockerPrivate
    vmmem_wsl_private_bytes = $vmmemWslPrivate
    nonpaged_pool_bytes = $kernelPool.nonpaged_pool_bytes
    nonpaged_pool_status = $kernelPool.nonpaged_pool_status
    pool_tag_diagnostics = $kernelPool.pool_tag_diagnostics
  }} | ConvertTo-Json -Compress
  [Console]::Out.Flush()
}}
{mode}
"""
    return [
        os.environ.get("FORMAL_WINDOWS_POWERSHELL", "powershell.exe"),
        "-NoLogo",
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-Command",
        script,
    ]


def read_once() -> dict[str, Any]:
    """Read one validated sample with exactly one bounded query retry."""

    last_error: BaseException | None = None
    for attempt in range(1, READ_ONCE_MAX_ATTEMPTS + 1):
        try:
            result = subprocess.run(
                powershell_command(stream=False, interval_s=1.0),
                check=False,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=15.0,
            )
            if result.returncode != 0:
                diagnostics = " ".join(
                    part
                    for part in (
                        f"stderr={result.stderr.strip()[:300]}",
                        f"stdout={result.stdout.strip()[:300]}",
                    )
                    if not part.endswith("=")
                )
                raise ProbeError(
                    f"PowerShell memory probe failed rc={result.returncode}: "
                    f"{diagnostics}"
                )
            lines = [line for line in result.stdout.splitlines() if line.strip()]
            if len(lines) != 1:
                raise ProbeError(
                    "PowerShell memory probe did not emit exactly one sample"
                )
            return parse_powershell_sample(lines[0])
        except (OSError, ProbeError, subprocess.SubprocessError) as exc:
            last_error = exc
            if attempt >= READ_ONCE_MAX_ATTEMPTS:
                raise
            print(
                "formal Windows one-shot query failed transiently; "
                f"bounded retry {attempt}/{READ_ONCE_MAX_ATTEMPTS - 1}: {exc}",
                file=sys.stderr,
            )
            time.sleep(READ_ONCE_RETRY_DELAY_S)
    raise ProbeError(f"unreachable one-shot probe state: {last_error}")


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise ProbeError(f"refusing stale Windows memory evidence: {path}")
    pending = path.with_name(f"{path.name}.pending.{os.getpid()}")
    pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def env_uint(name: str, default: int) -> int:
    raw = os.environ.get(name, str(default))
    if not raw.isdigit():
        raise ProbeError(f"{name} must be an unsigned byte integer")
    return int(raw)


def env_bool01(name: str, default: bool) -> bool:
    value = env_uint(name, int(default))
    if value not in (0, 1):
        raise ProbeError(f"{name} must be 0 or 1")
    return bool(value)


def _start_checks(
    sample: dict[str, Any],
    *,
    min_commit_available: int,
    max_docker_private: int,
    require_wsl_stopped: bool,
    require_wsl_running: bool,
) -> dict[str, bool]:
    pool_diagnostics = sample.get("pool_tag_diagnostics", {})
    suspected_ndis_leak = bool(
        isinstance(pool_diagnostics, dict)
        and pool_diagnostics.get("status") == "available"
        and pool_diagnostics.get("suspected_ndis_nonpaged_pool_leak") is True
    )
    return {
        "windows_commit_available_at_least_configured_minimum": (
            sample["commit_available_bytes"] >= min_commit_available
        ),
        "docker_private_at_most_configured_maximum": (
            sample["docker_private_bytes"] <= max_docker_private
        ),
        "wsl_vm_stopped_when_required": (
            not require_wsl_stopped or sample["vmmem_wsl_private_bytes"] == 0
        ),
        "wsl_vm_running_when_required": (
            not require_wsl_running or sample["vmmem_wsl_private_bytes"] > 0
        ),
        "no_suspected_ndis_nonpaged_pool_leak": not suspected_ndis_leak,
    }


def _commit_only_shortfall(checks: dict[str, bool]) -> bool:
    """Return whether retrying can safely help without relaxing another gate."""

    return (
        not checks["windows_commit_available_at_least_configured_minimum"]
        and checks["docker_private_at_most_configured_maximum"]
        and checks["wsl_vm_stopped_when_required"]
        and checks["wsl_vm_running_when_required"]
        and checks["no_suspected_ndis_nonpaged_pool_leak"]
    )


def check_start(
    output: Path,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    min_commit_available = env_uint(
        "FORMAL_WINDOWS_START_MIN_COMMIT_AVAILABLE_BYTES",
        START_MIN_COMMIT_AVAILABLE_BYTES,
    )
    max_docker_private = env_uint(
        "FORMAL_WINDOWS_START_MAX_DOCKER_PRIVATE_BYTES",
        START_MAX_DOCKER_PRIVATE_BYTES,
    )
    require_wsl_stopped = env_bool01(
        "FORMAL_WINDOWS_START_REQUIRE_WSL_STOPPED",
        False,
    )
    require_wsl_running = env_bool01(
        "FORMAL_WINDOWS_START_REQUIRE_WSL_RUNNING",
        False,
    )
    if require_wsl_stopped and require_wsl_running:
        raise ProbeError("WSL cannot be required both stopped and running")
    sample = read_once()
    checks = _start_checks(
        sample,
        min_commit_available=min_commit_available,
        max_docker_private=max_docker_private,
        require_wsl_stopped=require_wsl_stopped,
        require_wsl_running=require_wsl_running,
    )
    recovery_deadline = monotonic() + START_COMMIT_RECOVERY_TIMEOUT_S
    sample_count = 1
    while _commit_only_shortfall(checks) and monotonic() < recovery_deadline:
        remaining_s = recovery_deadline - monotonic()
        if remaining_s <= 0:
            break
        sleep(
            min(
                START_COMMIT_RECOVERY_INTERVAL_S,
                remaining_s,
            )
        )
        sample = read_once()
        sample_count += 1
        checks = _start_checks(
            sample,
            min_commit_available=min_commit_available,
            max_docker_private=max_docker_private,
            require_wsl_stopped=require_wsl_stopped,
            require_wsl_running=require_wsl_running,
        )
    passed = all(checks.values())
    atomic_json(
        output,
        {
            "report_id": "tzcup_formal_windows_memory_start_gate_v1",
            "status": (
                "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED"
                if passed
                else "FORMAL_WINDOWS_MEMORY_START_REFUSED"
            ),
            "passed": passed,
            "recorded_epoch_ns": time.time_ns(),
            "thresholds_bytes": {
                "min_commit_available": min_commit_available,
                "max_docker_private": max_docker_private,
            },
            "require_wsl_stopped": require_wsl_stopped,
            "require_wsl_running": require_wsl_running,
            "sample": sample,
            "checks": checks,
            "commit_recovery": {
                "timeout_s": START_COMMIT_RECOVERY_TIMEOUT_S,
                "interval_s": START_COMMIT_RECOVERY_INTERVAL_S,
                "sample_count": sample_count,
                "attempted": sample_count > 1,
                "recovered": sample_count > 1 and passed,
            },
            "docker_was_signalled_or_stopped": False,
        },
    )
    return 0 if passed else 86


def record_native_linux_not_applicable(output: Path) -> int:
    """Record why the Windows-only gate is not applicable on native Linux."""

    atomic_json(
        output,
        {
            "report_id": "tzcup_formal_windows_memory_start_gate_v1",
            "status": "FORMAL_WINDOWS_MEMORY_START_NOT_APPLICABLE_NATIVE_LINUX",
            "passed": True,
            "recorded_epoch_ns": time.time_ns(),
            "native_linux_runtime": True,
            "windows_probe_performed": False,
            "checks": {
                "explicit_native_linux_runtime_requested": True,
                "kernel_is_not_wsl": True,
                "windows_gate_not_applicable": True,
            },
            "claim_boundary": (
                "Windows commit, Docker Desktop, vmmemWSL and NDIS pool data are "
                "not applicable on this explicitly selected native-Linux runtime; "
                "the Linux memory floor and exact-process-group watchdog remain required."
            ),
        },
    )
    return 0


def _format_stream_record(sample: dict[str, Any]) -> bytes:
    """Return one complete, ASCII-only watchdog record.

    The shell reader has a bounded timeout.  ``print`` writes each argument and
    separator independently, so a timeout can consume half a record and make
    the next read look malformed.  Keep this record below POSIX ``PIPE_BUF``
    and emit it with exactly one ``os.write`` call instead.
    """

    fields = (
        sample["epoch_ns"],
        sample["commit_limit_bytes"],
        sample["commit_charge_bytes"],
        sample["commit_available_bytes"],
        sample["docker_private_bytes"],
        sample["vmmem_wsl_private_bytes"],
        sample.get("nonpaged_pool_bytes", 0),
        int(sample.get("pool_tag_diagnostics", {}).get("status") == "available"),
        sample.get("pool_tag_diagnostics", {}).get(
            "tracked_nonpaged_bytes_total", 0
        ),
        int(
            sample.get("pool_tag_diagnostics", {}).get(
                "suspected_ndis_nonpaged_pool_leak", False
            )
        ),
    )
    payload = (" ".join(str(value) for value in fields) + "\n").encode("ascii")
    if len(payload) >= 4096:
        raise ProbeError("formal Windows stream record exceeds atomic pipe limit")
    return payload


def _write_stream_record(sample: dict[str, Any]) -> None:
    payload = _format_stream_record(sample)
    written = os.write(sys.stdout.fileno(), payload)
    if written != len(payload):
        raise ProbeError("formal Windows stream record write was short")


def _with_next_stream_sequence(
    sample: dict[str, Any],
    previous_sequence: int,
) -> tuple[dict[str, Any], int]:
    """Replace wall-clock time with a process-local strictly increasing token.

    Windows wall time and WSL CLOCK_MONOTONIC can both be rebased during VM
    time synchronisation.  The first pipe field is used only for stale or
    duplicate detection, so an in-process sequence is the robust contract.
    The original UTC epoch remains in parsed one-shot/start evidence.
    """

    if previous_sequence >= UINT64_MAX:
        raise ProbeError("formal Windows stream sequence exhausted uint64")
    sequence = previous_sequence + 1
    return {**sample, "epoch_ns": sequence}, sequence


def stream(interval_s: float) -> int:
    max_restarts = env_uint("FORMAL_WINDOWS_STREAM_MAX_RESTARTS", 1)
    process: subprocess.Popen[str] | None = None
    stop_requested = False
    previous_handlers: dict[int, Any] = {}

    def request_stop(signum: int, unused_frame: Any) -> None:
        nonlocal stop_requested, process
        stop_requested = True
        if process is not None and process.poll() is None:
            process.terminate()

    for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if signum is None:
            continue
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    restart_count = 0
    result = 125
    previous_stream_sequence = 0
    try:
        while not stop_requested:
            process = subprocess.Popen(
                powershell_command(stream=True, interval_s=interval_s),
                stdout=subprocess.PIPE,
                # The caller already redirects this process' stderr into the
                # watchdog log.  Inheriting it avoids an unread PIPE filling
                # and stalling a still-alive PowerShell query process.
                stderr=None,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
            )
            assert process.stdout is not None
            for line in process.stdout:
                if not line.strip():
                    continue
                sample = parse_powershell_sample(line)
                stream_sample, previous_stream_sequence = _with_next_stream_sequence(
                    sample,
                    previous_stream_sequence,
                )
                _write_stream_record(stream_sample)
            returncode = process.wait()
            if stop_requested:
                result = 0
                break
            if restart_count >= max_restarts:
                print(
                    "formal Windows streaming query exhausted bounded restarts "
                    f"after rc={returncode}",
                    file=sys.stderr,
                )
                result = 125
                break
            restart_count += 1
            print(
                "formal Windows streaming query ended unexpectedly; "
                f"bounded restart {restart_count}/{max_restarts}, rc={returncode}",
                file=sys.stderr,
            )
            time.sleep(min(0.25, interval_s))
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check-start", action="store_true")
    mode.add_argument("--stream", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--interval-s", type=float, default=1.0)
    args = parser.parse_args()
    try:
        if args.check_start:
            if args.output is None:
                raise ProbeError("--check-start requires --output")
            if native_linux_runtime_requested():
                return record_native_linux_not_applicable(args.output)
            return check_start(args.output)
        if args.interval_s < 0.1:
            raise ProbeError("--interval-s must be at least 0.1")
        return stream(args.interval_s)
    except (OSError, ProbeError, subprocess.SubprocessError) as exc:
        print(f"formal Windows memory probe failed closed: {exc}", file=sys.stderr)
        return 125


if __name__ == "__main__":
    raise SystemExit(main())
