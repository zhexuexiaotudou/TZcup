from sanitation_perception.frame_synchronizer import (
    LatestFrameScheduler,
    StrictFrameSynchronizer,
)


def synchronized(sync, base_ns):
    assert sync.add("depth", base_ns + 5_000_000, "depth") is None
    assert sync.add("camera_info", base_ns - 3_000_000, "camera") is None
    return sync.add("rgb", base_ns, "rgb")


def test_strict_sync_accepts_20ms_and_rejects_older_samples():
    sync = StrictFrameSynchronizer(tolerance_ms=20, queue_depth=2)
    frame = synchronized(sync, 1_000_000_000)
    assert frame is not None
    assert frame.maximum_delta_ns == 8_000_000
    assert sync.sync_count == 1
    assert sync.add("depth", 2_100_000_000, "late") is None
    assert sync.add("camera_info", 2_000_000_000, "camera") is None
    assert sync.add("rgb", 2_000_000_000, "rgb") is None
    assert sync.sync_reject_count == 1


def test_latest_frame_wins_and_queue_never_exceeds_two():
    sync = StrictFrameSynchronizer()
    scheduler = LatestFrameScheduler(queue_depth=2)
    frames = [synchronized(sync, index * 100_000_000) for index in range(1, 4)]
    for frame in frames:
        scheduler.submit(frame)
    assert scheduler.depth == 2
    assert scheduler.dropped == 1
    assert scheduler.pop_latest().rgb.payload == "rgb"
    assert scheduler.pop_latest() is None
    assert scheduler.dropped == 2
