"""Product latency, throughput, drop, memory, and soak acceptance accounting."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import math
import os
import statistics
import subprocess
import sys
import time

try:
    import resource
except ImportError:  # pragma: no cover - Windows uses psutil in CI.
    resource = None


STAGES = (
    "preprocess", "discovery", "classifier_batch", "leaf", "puddle",
    "projection", "tracking", "inference_pipeline", "end_to_end",
)


@dataclass(frozen=True)
class PerformanceConfig:
    inference_p95_ms: float
    end_to_end_p95_ms: float
    minimum_effective_hz: float
    maximum_drop_rate: float
    soak_duration_s: float
    maximum_memory_growth_ratio: float

    @classmethod
    def from_pipeline_manifest(cls, manifest: dict) -> "PerformanceConfig":
        try:
            config = cls(**manifest["runtime"]["performance"])
        except (KeyError, TypeError) as exc:
            raise ValueError("pipeline performance configuration is incomplete") from exc
        if min(
            config.inference_p95_ms, config.end_to_end_p95_ms,
            config.minimum_effective_hz, config.soak_duration_s,
            config.maximum_memory_growth_ratio,
        ) <= 0 or not 0 <= config.maximum_drop_rate <= 1:
            raise ValueError("pipeline performance thresholds are invalid")
        return config


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = (len(ordered) - 1) * percent / 100.0
    low, high = math.floor(rank), math.ceil(rank)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - rank) + ordered[high] * (rank - low)


def current_rss_bytes() -> int:
    try:
        import psutil

        return int(psutil.Process().memory_info().rss)
    except ImportError:
        if resource is not None and hasattr(resource, "getrusage"):
            maximum = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
            return maximum if sys.platform == "darwin" else maximum * 1024
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes

            class ProcessMemoryCounters(ctypes.Structure):
                _fields_ = [
                    ("cb", wintypes.DWORD),
                    ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t),
                    ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t),
                    ("PeakPagefileUsage", ctypes.c_size_t),
                ]

            counters = ProcessMemoryCounters()
            counters.cb = ctypes.sizeof(counters)
            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            psapi = ctypes.WinDLL("psapi", use_last_error=True)
            kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            psapi.GetProcessMemoryInfo.argtypes = (
                wintypes.HANDLE,
                ctypes.POINTER(ProcessMemoryCounters),
                wintypes.DWORD,
            )
            psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
            process = kernel32.GetCurrentProcess()
            if not psapi.GetProcessMemoryInfo(
                process, ctypes.byref(counters), counters.cb
            ):
                raise OSError(ctypes.get_last_error(), "GetProcessMemoryInfo failed")
            return int(counters.WorkingSetSize)
        raise RuntimeError("RSS sampling is unsupported on this platform")


def current_gpu_memory_bytes() -> int | None:
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,used_memory",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2, check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    total_mib = 0
    for line in result.stdout.splitlines():
        fields = [item.strip() for item in line.split(",")]
        if len(fields) == 2 and fields[0] == str(os.getpid()):
            total_mib += int(fields[1])
    return total_mib * 1024 * 1024


class PerformanceMonitor:
    def __init__(self, config: PerformanceConfig, *, history: int = 4096):
        self.config = config
        self.started_s = time.monotonic()
        self.stage_ms = {name: deque(maxlen=history) for name in STAGES}
        self.submitted = 0
        self.processed = 0
        self.dropped = 0
        self.candidate_count = deque(maxlen=history)
        self.reject_count = 0
        self.track_count = 0

    def record_submission(self, *, dropped: int = 0) -> None:
        self.submitted += 1
        self.dropped += int(dropped)

    def record_frame(
        self, latencies_ms: dict[str, float], *, candidate_count: int = 0,
        reject_count: int = 0, track_count: int = 0,
    ) -> None:
        unknown = set(latencies_ms) - set(STAGES)
        if unknown:
            raise ValueError(f"unknown product latency stages: {sorted(unknown)}")
        for name, value in latencies_ms.items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid {name} latency: {value}")
            self.stage_ms[name].append(float(value))
        self.processed += 1
        self.candidate_count.append(int(candidate_count))
        self.reject_count += int(reject_count)
        self.track_count = int(track_count)

    def snapshot(self, now_s: float | None = None) -> dict:
        now_s = time.monotonic() if now_s is None else float(now_s)
        elapsed = max(now_s - self.started_s, 1e-9)
        drop_rate = self.dropped / max(self.submitted, 1)
        latencies = {
            name: {"p50_ms": percentile(list(values), 50),
                   "p95_ms": percentile(list(values), 95),
                   "sample_count": len(values)}
            for name, values in self.stage_ms.items()
        }
        inference_p95 = latencies["inference_pipeline"]["p95_ms"]
        end_to_end_p95 = latencies["end_to_end"]["p95_ms"]
        effective_hz = self.processed / elapsed
        gates = {
            "inference_pipeline_p95": inference_p95 is not None
            and inference_p95 <= self.config.inference_p95_ms,
            "end_to_end_p95": end_to_end_p95 is not None
            and end_to_end_p95 <= self.config.end_to_end_p95_ms,
            "effective_hz": effective_hz >= self.config.minimum_effective_hz,
            "drop_rate": self.submitted > 0
            and drop_rate <= self.config.maximum_drop_rate,
        }
        return {
            "latencies": latencies, "submitted": self.submitted,
            "processed": self.processed, "dropped": self.dropped,
            "drop_rate": drop_rate, "effective_hz": effective_hz,
            "candidate_count_mean": statistics.fmean(self.candidate_count)
            if self.candidate_count else None,
            "reject_count": self.reject_count, "track_count": self.track_count,
            "cpu_rss_bytes": current_rss_bytes(),
            "gpu_memory_bytes": current_gpu_memory_bytes(),
            "gates": gates, "performance_gate_pass": all(gates.values()),
        }


@dataclass
class SoakAudit:
    config: PerformanceConfig
    started_s: float
    initial_rss_bytes: int
    crash_count: int = 0
    deadlock_count: int = 0
    unintended_model_reload_count: int = 0
    maximum_queue_depth: int = 0
    tf_stale_storm_count: int = 0

    def finish(self, *, ended_s: float, final_rss_bytes: int) -> dict:
        duration = float(ended_s) - self.started_s
        growth = (int(final_rss_bytes) - self.initial_rss_bytes) / max(
            self.initial_rss_bytes, 1
        )
        gates = {
            "duration_at_least_two_hours": duration >= self.config.soak_duration_s,
            "crash_zero": self.crash_count == 0,
            "deadlock_zero": self.deadlock_count == 0,
            "unintended_model_reload_zero": self.unintended_model_reload_count == 0,
            "memory_growth_at_most_limit": growth
            <= self.config.maximum_memory_growth_ratio,
            "queue_growth_zero": self.maximum_queue_depth <= 2,
            "tf_stale_storm_zero": self.tf_stale_storm_count == 0,
        }
        return {"duration_s": duration, "initial_rss_bytes": self.initial_rss_bytes,
                "final_rss_bytes": int(final_rss_bytes), "memory_growth_ratio": growth,
                "gates": gates, "soak_gate_pass": all(gates.values())}
