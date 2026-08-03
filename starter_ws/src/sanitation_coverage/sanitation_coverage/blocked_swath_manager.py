"""Auditable, bounded blocked-swath retry and defer policy."""

from dataclasses import asdict, dataclass, field
from enum import Enum
import time


class BlockedSwathState(str, Enum):
    ACTIVE = "ACTIVE"
    PARTIALLY_BLOCKED = "PARTIALLY_BLOCKED"
    DEFERRED = "DEFERRED"
    RETRY_PENDING = "RETRY_PENDING"
    COMPLETED = "COMPLETED"
    UNREACHABLE = "UNREACHABLE"


@dataclass
class BlockedSwathRecord:
    swath_id: str
    state: BlockedSwathState = BlockedSwathState.ACTIVE
    blocked_intervals: list[tuple[float, float]] = field(default_factory=list)
    first_blocked_time_sec: float | None = None
    retry_count: int = 0
    next_retry_time_sec: float | None = None
    last_obstacle_state: str = "UNKNOWN"
    terminal_reason: str | None = None

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["state"] = self.state.value
        payload["blocked_intervals"] = [list(item) for item in self.blocked_intervals]
        return payload


@dataclass
class BlockedSwathManager:
    max_retries: int = 2
    minimum_retry_delay_sec: float = 10.0
    records: dict[str, BlockedSwathRecord] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.max_retries < 0:
            raise ValueError("max_retries must be non-negative")
        if self.minimum_retry_delay_sec < 0.0:
            raise ValueError("minimum_retry_delay_sec must be non-negative")

    def _record(self, swath_id: str) -> BlockedSwathRecord:
        return self.records.setdefault(swath_id, BlockedSwathRecord(swath_id))

    def report_blocked(
        self,
        swath_id: str,
        blocked_interval: tuple[float, float] = (0.0, 1.0),
        obstacle_state: str = "PRESENT",
        now_sec: float | None = None,
    ) -> str:
        start, end = (float(value) for value in blocked_interval)
        if not 0.0 <= start <= end <= 1.0:
            raise ValueError("blocked_interval must be ordered within 0..1")
        now = time.monotonic() if now_sec is None else float(now_sec)
        record = self._record(swath_id)
        if record.state in {
            BlockedSwathState.COMPLETED,
            BlockedSwathState.UNREACHABLE,
        }:
            return record.state.value
        if record.first_blocked_time_sec is None:
            record.first_blocked_time_sec = now
        interval = (start, end)
        if interval not in record.blocked_intervals:
            record.blocked_intervals.append(interval)
        record.last_obstacle_state = str(obstacle_state)
        record.state = BlockedSwathState.PARTIALLY_BLOCKED
        if record.retry_count < self.max_retries:
            record.retry_count += 1
            record.next_retry_time_sec = now + self.minimum_retry_delay_sec
            record.state = BlockedSwathState.RETRY_PENDING
        else:
            record.next_retry_time_sec = None
            record.state = BlockedSwathState.DEFERRED
        return record.state.value

    def retry_ready(self, swath_id: str, now_sec: float | None = None) -> bool:
        record = self._record(swath_id)
        now = time.monotonic() if now_sec is None else float(now_sec)
        return bool(
            record.state is BlockedSwathState.RETRY_PENDING
            and record.next_retry_time_sec is not None
            and now >= record.next_retry_time_sec
        )

    def activate_retry(self, swath_id: str, now_sec: float | None = None) -> bool:
        if not self.retry_ready(swath_id, now_sec):
            return False
        record = self._record(swath_id)
        record.state = BlockedSwathState.ACTIVE
        record.next_retry_time_sec = None
        return True

    def report_clear(self, swath_id: str, obstacle_state: str = "CLEARED") -> None:
        record = self._record(swath_id)
        record.state = BlockedSwathState.COMPLETED
        record.next_retry_time_sec = None
        record.last_obstacle_state = str(obstacle_state)
        record.terminal_reason = "completed_after_obstacle_clearance"

    def mark_unreachable(self, swath_id: str, reason: str) -> None:
        record = self._record(swath_id)
        record.state = BlockedSwathState.UNREACHABLE
        record.next_retry_time_sec = None
        record.terminal_reason = str(reason)

    def repair_queue(self) -> tuple[str, ...]:
        return tuple(
            swath_id for swath_id, record in self.records.items()
            if record.state is BlockedSwathState.DEFERRED
        )

    def snapshot(self) -> list[dict]:
        return [record.to_dict() for record in self.records.values()]
