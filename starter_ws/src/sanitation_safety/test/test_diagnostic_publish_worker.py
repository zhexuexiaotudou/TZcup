import threading
import time

from sanitation_safety.diagnostic_publish_worker import DiagnosticPublishWorker


def _wait_until(predicate, timeout_sec=1.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    assert predicate()


def test_blocked_publisher_never_blocks_enqueue_or_shutdown():
    entered = threading.Event()
    release = threading.Event()

    def blocking_publish(_encoded):
        entered.set()
        release.wait()

    worker = DiagnosticPublishWorker(blocking_publish, capacity=1)
    try:
        started = time.monotonic()
        assert worker.enqueue({"event": "first"})
        assert time.monotonic() - started < 0.1
        assert entered.wait(0.5)
        assert worker.enqueue({"event": "queued"})
        assert not worker.enqueue({"event": "dropped"})
        assert worker.health()["dropped"] == 1

        started = time.monotonic()
        assert not worker.shutdown(0.01)
        assert time.monotonic() - started < 0.2
        assert worker.health()["shutdown_timed_out"] is True
    finally:
        release.set()
        assert worker.shutdown(0.5)


def test_publish_exception_is_recorded_and_worker_continues():
    calls = []

    def failing_publish(_encoded):
        calls.append(1)
        raise RuntimeError("injected_diagnostic_failure")

    worker = DiagnosticPublishWorker(failing_publish)
    try:
        assert worker.enqueue({"event": "exception"})
        _wait_until(lambda: worker.health()["errors"] == 1)
        health = worker.health()
        assert health["last_error"] == "RuntimeError"
        assert health["worker_alive"] is True
        assert calls == [1]
    finally:
        assert worker.shutdown(0.5)
