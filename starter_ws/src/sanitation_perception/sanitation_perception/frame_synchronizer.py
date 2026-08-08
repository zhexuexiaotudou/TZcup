"""Strict RGB-D-CameraInfo synchronization and latest-frame scheduling."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock


@dataclass(frozen=True)
class StampedPayload:
    stamp_ns: int
    payload: object


@dataclass(frozen=True)
class SynchronizedFrame:
    rgb: StampedPayload
    depth: StampedPayload
    camera_info: StampedPayload

    @property
    def rgb_stamp_ns(self) -> int:
        return self.rgb.stamp_ns

    @property
    def maximum_delta_ns(self) -> int:
        stamps = (
            self.rgb.stamp_ns,
            self.depth.stamp_ns,
            self.camera_info.stamp_ns,
        )
        return max(stamps) - min(stamps)


class StrictFrameSynchronizer:
    """Bounded three-stream synchronizer with a hard timestamp tolerance."""

    STREAMS = ("rgb", "depth", "camera_info")

    def __init__(self, tolerance_ms: float = 20.0, queue_depth: int = 2):
        if tolerance_ms <= 0.0 or queue_depth not in (1, 2):
            raise ValueError("sync tolerance must be positive and queue_depth <= 2")
        self.tolerance_ns = int(tolerance_ms * 1_000_000)
        self.queue_depth = queue_depth
        self.queues = {
            name: deque(maxlen=queue_depth) for name in self.STREAMS
        }
        self.received = {name: 0 for name in self.STREAMS}
        self.dropped = {name: 0 for name in self.STREAMS}
        self.sync_reject_count = 0
        self.sync_count = 0

    def add(self, stream: str, stamp_ns: int, payload: object) -> SynchronizedFrame | None:
        if stream not in self.queues:
            raise ValueError(f"unknown synchronized stream: {stream}")
        queue = self.queues[stream]
        if len(queue) == queue.maxlen:
            self.dropped[stream] += 1
        queue.append(StampedPayload(int(stamp_ns), payload))
        self.received[stream] += 1
        return self._match_latest_rgb()

    def _match_latest_rgb(self) -> SynchronizedFrame | None:
        if any(not self.queues[name] for name in self.STREAMS):
            return None
        rgb = self.queues["rgb"][-1]
        depth = min(
            self.queues["depth"], key=lambda item: abs(item.stamp_ns - rgb.stamp_ns)
        )
        camera = min(
            self.queues["camera_info"],
            key=lambda item: abs(item.stamp_ns - rgb.stamp_ns),
        )
        frame = SynchronizedFrame(rgb=rgb, depth=depth, camera_info=camera)
        if frame.maximum_delta_ns > self.tolerance_ns:
            oldest_stream = min(
                self.STREAMS, key=lambda name: self.queues[name][0].stamp_ns
            )
            self.queues[oldest_stream].popleft()
            self.dropped[oldest_stream] += 1
            self.sync_reject_count += 1
            return None
        for name, selected in (
            ("rgb", rgb),
            ("depth", depth),
            ("camera_info", camera),
        ):
            queue = self.queues[name]
            while queue:
                item = queue.popleft()
                if item is selected:
                    break
                self.dropped[name] += 1
        self.sync_count += 1
        return frame


class LatestFrameScheduler:
    """Thread-safe latest-frame-wins queue; inference can never accumulate."""

    def __init__(self, queue_depth: int = 2):
        if queue_depth not in (1, 2):
            raise ValueError("product frame queue depth must be 1 or 2")
        self._frames = deque(maxlen=queue_depth)
        self._lock = Lock()
        self.submitted = 0
        self.dropped = 0
        self.consumed = 0

    def submit(self, frame: SynchronizedFrame) -> None:
        with self._lock:
            if len(self._frames) == self._frames.maxlen:
                self.dropped += 1
            self._frames.append(frame)
            self.submitted += 1

    def pop_latest(self) -> SynchronizedFrame | None:
        with self._lock:
            if not self._frames:
                return None
            newest = self._frames.pop()
            self.dropped += len(self._frames)
            self._frames.clear()
            self.consumed += 1
            return newest

    @property
    def depth(self) -> int:
        with self._lock:
            return len(self._frames)
