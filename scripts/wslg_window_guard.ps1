[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$WindowTitle,
    [string]$StopFile = "",
    [string]$EvidencePath = "",
    [string]$FailureFile = "",
    [int]$StartupTimeoutSeconds = 180,
    [int]$PollMilliseconds = 750,
    [switch]$CloseWindowOnStop,
    [switch]$Monitor
)

$ErrorActionPreference = "Stop"

if (-not ("TZcup.WslgWindow" -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace TZcup
{
    public sealed class WindowState
    {
        public long Handle { get; set; }
        public string Title { get; set; }
        public bool Visible { get; set; }
        public bool Minimized { get; set; }
    }

    public static class WslgWindow
    {
        private delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

        [DllImport("user32.dll")]
        private static extern bool EnumWindows(EnumWindowsProc callback, IntPtr lParam);

        [DllImport("user32.dll", CharSet = CharSet.Unicode)]
        private static extern int GetWindowText(IntPtr hWnd, StringBuilder text, int count);

        [DllImport("user32.dll")]
        private static extern int GetWindowTextLength(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern bool IsWindowVisible(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern bool IsIconic(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern bool ShowWindowAsync(IntPtr hWnd, int command);

        [DllImport("user32.dll")]
        private static extern bool SetForegroundWindow(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern bool IsWindow(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern bool PostMessage(IntPtr hWnd, uint message, IntPtr wParam, IntPtr lParam);

        [DllImport("user32.dll")]
        private static extern IntPtr GetForegroundWindow();

        [DllImport("user32.dll")]
        private static extern uint GetWindowThreadProcessId(IntPtr hWnd, IntPtr processId);

        [DllImport("kernel32.dll")]
        private static extern uint GetCurrentThreadId();

        [DllImport("user32.dll")]
        private static extern bool AttachThreadInput(uint attach, uint attachTo, bool value);

        [DllImport("user32.dll")]
        private static extern bool BringWindowToTop(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern IntPtr SetFocus(IntPtr hWnd);

        private const int SW_RESTORE = 9;
        private const int SW_SHOW = 5;
        private const uint WM_CLOSE = 0x0010;

        public static WindowState[] Find(string titleFragment)
        {
            var matches = new List<WindowState>();
            EnumWindows(delegate(IntPtr hWnd, IntPtr lParam)
            {
                int length = GetWindowTextLength(hWnd);
                if (length <= 0)
                    return true;
                var text = new StringBuilder(length + 1);
                GetWindowText(hWnd, text, text.Capacity);
                string title = text.ToString();
                if (title.IndexOf(titleFragment, StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    matches.Add(new WindowState
                    {
                        Handle = hWnd.ToInt64(),
                        Title = title,
                        Visible = IsWindowVisible(hWnd),
                        Minimized = IsIconic(hWnd)
                    });
                }
                return true;
            }, IntPtr.Zero);
            return matches.ToArray();
        }

        public static bool Restore(long handle)
        {
            var hWnd = new IntPtr(handle);
            if (!IsWindowVisible(hWnd))
                ShowWindowAsync(hWnd, SW_SHOW);
            if (IsIconic(hWnd))
                ShowWindowAsync(hWnd, SW_RESTORE);
            IntPtr foreground = GetForegroundWindow();
            uint currentThread = GetCurrentThreadId();
            uint targetThread = GetWindowThreadProcessId(hWnd, IntPtr.Zero);
            uint foregroundThread = GetWindowThreadProcessId(foreground, IntPtr.Zero);
            bool attachedForeground = foregroundThread != 0 && foregroundThread != currentThread &&
                AttachThreadInput(currentThread, foregroundThread, true);
            bool attachedTarget = targetThread != 0 && targetThread != currentThread &&
                targetThread != foregroundThread && AttachThreadInput(currentThread, targetThread, true);
            try
            {
                BringWindowToTop(hWnd);
                SetForegroundWindow(hWnd);
                SetFocus(hWnd);
                return GetForegroundWindow() == hWnd;
            }
            finally
            {
                if (attachedTarget)
                    AttachThreadInput(currentThread, targetThread, false);
                if (attachedForeground)
                    AttachThreadInput(currentThread, foregroundThread, false);
            }
        }

        public static bool Close(long handle)
        {
            var hWnd = new IntPtr(handle);
            return IsWindow(hWnd) && PostMessage(hWnd, WM_CLOSE, IntPtr.Zero, IntPtr.Zero);
        }

    }
}
'@
}

function Write-WindowEvidence {
    param(
        [string]$Event,
        [object]$Window = $null,
        [hashtable]$Details = @{}
    )
    if (-not $EvidencePath) {
        return
    }
    $parent = Split-Path -Parent $EvidencePath
    if ($parent) {
        [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    }
    $record = [ordered]@{
        timestamp_utc = [DateTime]::UtcNow.ToString("o")
        event = $Event
        title = if ($null -ne $Window) { $Window.Title } else { $null }
        handle = if ($null -ne $Window) { $Window.Handle } else { $null }
        visible = if ($null -ne $Window) { $Window.Visible } else { $null }
        minimized = if ($null -ne $Window) { $Window.Minimized } else { $null }
        details = $Details
    }
    Add-Content -LiteralPath $EvidencePath -Value ($record | ConvertTo-Json -Compress -Depth 4) -Encoding utf8
}

function Write-GuardFailure {
    param([string]$Reason)
    if (-not $FailureFile) {
        return
    }
    $parent = Split-Path -Parent $FailureFile
    if ($parent) {
        [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    }
    Set-Content -LiteralPath $FailureFile -Value $Reason -Encoding utf8
}

$deadline = [DateTime]::UtcNow.AddSeconds($StartupTimeoutSeconds)
$seenWindow = $false
$lastRecoveryHandle = 0L
$healthyOnce = $false
$copyModeDeadline = $null
$windowRecoveryDeadline = $null
$lastSeenWindow = $null

while ($true) {
    if ($StopFile -and (Test-Path -LiteralPath $StopFile)) {
        if ($CloseWindowOnStop -and $null -ne $lastSeenWindow) {
            $closeRequested = [TZcup.WslgWindow]::Close($lastSeenWindow.Handle)
            Write-WindowEvidence -Event "window_close_requested" -Window $lastSeenWindow -Details @{
                reason = "guard_stop"
                request_accepted = $closeRequested
            }
        }
        Write-WindowEvidence -Event "guard_stopped"
        exit 0
    }

    $windows = @([TZcup.WslgWindow]::Find($WindowTitle))
    if ($windows.Count -gt 1) {
        Write-WindowEvidence -Event "ambiguous_windows" -Details @{ count = $windows.Count }
        Write-GuardFailure -Reason "ambiguous_windows"
        throw "Found multiple WSLg windows matching '$WindowTitle'; refusing to target an ambiguous window."
    }

    if ($windows.Count -eq 1) {
        $seenWindow = $true
        $window = $windows[0]
        $lastSeenWindow = $window
        $copyMode = $window.Title.StartsWith("[WARN:COPY MODE]", [StringComparison]::OrdinalIgnoreCase)
        $needsRecovery = -not $window.Visible -or $window.Minimized

        if ($copyMode) {
            if ($null -eq $copyModeDeadline) {
                $copyModeDeadline = [DateTime]::UtcNow.AddSeconds(10)
                Write-WindowEvidence -Event "copy_mode_detected" -Window $window
            } elseif ([DateTime]::UtcNow -ge $copyModeDeadline) {
                Write-WindowEvidence -Event "copy_mode_timeout" -Window $window
                Write-GuardFailure -Reason "copy_mode_timeout"
                throw "Gazebo entered WSLg COPY MODE. Verify the /mnt/shared_memory preflight."
            }
        } elseif ($needsRecovery) {
            if ($null -eq $windowRecoveryDeadline) {
                $windowRecoveryDeadline = [DateTime]::UtcNow.AddSeconds(10)
            } elseif ([DateTime]::UtcNow -ge $windowRecoveryDeadline) {
                Write-WindowEvidence -Event "recovery_timeout" -Window $window
                Write-GuardFailure -Reason "window_recovery_timeout"
                throw "Gazebo WSLg window appeared but remained hidden or minimized."
            }
            $focusAcquired = [TZcup.WslgWindow]::Restore($window.Handle)
            if ($lastRecoveryHandle -ne $window.Handle) {
                Write-WindowEvidence -Event "window_recovered" -Window $window -Details @{
                    focus_acquired = $focusAcquired
                }
                $lastRecoveryHandle = $window.Handle
            }
        } else {
            $copyModeDeadline = $null
            $windowRecoveryDeadline = $null
            if (-not $healthyOnce) {
                Write-WindowEvidence -Event "window_ready" -Window $window
                $healthyOnce = $true
            }
            if (-not $Monitor) {
                exit 0
            }
        }
    } elseif ($seenWindow) {
        $closeRequested = $false
        if ($CloseWindowOnStop -and $null -ne $lastSeenWindow) {
            $closeRequested = [TZcup.WslgWindow]::Close($lastSeenWindow.Handle)
        }
        Write-WindowEvidence -Event "window_closed" -Window $lastSeenWindow -Details @{
            close_request_accepted = $closeRequested
        }
        exit 5
    } elseif ([DateTime]::UtcNow -ge $deadline) {
        Write-WindowEvidence -Event "startup_timeout"
        Write-GuardFailure -Reason "startup_timeout"
        throw "Gazebo WSLg window '$WindowTitle' did not appear within $StartupTimeoutSeconds seconds."
    }

    Start-Sleep -Milliseconds $PollMilliseconds
}
