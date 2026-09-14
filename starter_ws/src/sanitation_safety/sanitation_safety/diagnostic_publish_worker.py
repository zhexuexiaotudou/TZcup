"""Bounded best-effort transport for non-control diagnostic payloads."""

from __future__ import annotations

import json
import queue
import threading
from collections.abc import Callable


class DiagnosticPublishWorker:
    """Keep diagnostic encoding and publication out of control-output threads."""

    def __init__(self, publish_encoded: Callable[[str], None], *, capacity: int = 16):
        if capacity <= 0:
            raise ValueError("diagnostic worker capacity must be positive")
        self._publish_encoded = publish_encoded
        self._queue: queue.Queue[dict[str, object] | None] = queue.Queue(capacity)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._dropped = 0
        self._published = 0
        self._errors = 0
        self._last_error: str | None = None
        self._shutdown_timed_out = False
        self._thread = threading.Thread(
            target=self._run,
            name="best_effort_diagnostic_publish",
            daemon=True,
        )
        self._thread.start()

    def enqueue(self, payload: dict[str, object]) -> bool:
        """Copy and enqueue without waiting; a full queue drops diagnostics."""

        payload_copy = dict(payload)
        payload_copy["diagnostic_transport"] = self.health()
        try:
            self._queue.put_nowait(payload_copy)
            return True
        except queue.Full:
            with self._lock:
                self._dropped += 1
            return False

    def health(self) -> dict[str, int | bool | str | None]:
        with self._lock:
            return {
                "capacity": self._queue.maxsize,
                "depth": self._queue.qsize(),
                "dropped": self._dropped,
                "published": self._published,
                "errors": self._errors,
                "last_error": self._last_error,
                "worker_alive": self._thread.is_alive(),
                "shutdown_requested": self._stop_event.is_set(),
                "shutdown_timed_out": self._shutdown_timed_out,
            }

    def shutdown(self, timeout_sec: float = 0.1) -> bool:
        """Bound shutdown wait; a blocked diagnostic publisher cannot stall safety."""

        self._stop_event.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=max(0.0, timeout_sec))
        stopped = not self._thread.is_alive()
        with self._lock:
            self._shutdown_timed_out = not stopped
        return stopped

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                payload = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if payload is None:
                return
            try:
                self._publish_encoded(
                    json.dumps(payload, sort_keys=True, separators=(",", ":"))
                )
                with self._lock:
                    self._published += 1
            except BaseException as error:
                with self._lock:
                    self._errors += 1
                    self._last_error = type(error).__name__
            finally:
                self._queue.task_done()
