"""Bounded blocked-swath retry and defer policy."""

from dataclasses import dataclass, field


@dataclass
class BlockedSwathManager:
    max_retries: int = 1
    attempts: dict[str, int] = field(default_factory=dict)
    deferred: list[str] = field(default_factory=list)

    def report_blocked(self, component_id: str) -> str:
        count = self.attempts.get(component_id, 0) + 1
        self.attempts[component_id] = count
        if count <= self.max_retries:
            return "RETRY"
        if component_id not in self.deferred:
            self.deferred.append(component_id)
        return "DEFER_TO_REPAIR"

    def report_clear(self, component_id: str) -> None:
        self.attempts.pop(component_id, None)
        if component_id in self.deferred:
            self.deferred.remove(component_id)

    def repair_queue(self) -> tuple[str, ...]:
        return tuple(self.deferred)
