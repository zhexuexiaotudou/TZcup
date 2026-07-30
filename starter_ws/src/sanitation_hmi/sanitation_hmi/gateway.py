"""Authentication, authorization, validation, and idempotency for APP commands."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import threading
import time
from typing import Any

from .dsl import parse_command, validate_dsl


ROLE_PERMISSIONS = {
    "viewer": {"status"},
    "operator": {
        "status",
        "pause",
        "resume",
        "return_home",
        "start_coverage",
        "spot_clean",
        "schedule",
        "emergency_stop",
    },
}


@dataclass
class CommandGateway:
    tokens: dict[str, str]
    responses: dict[str, tuple[str, dict[str, Any]]] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def submit(
        self, token: str, idempotency_key: str, payload: dict[str, Any]
    ) -> tuple[int, dict[str, Any]]:
        started = time.perf_counter_ns()
        if token not in self.tokens:
            return 401, self._response("REJECTED", "authentication_failed", started)
        if not idempotency_key or len(idempotency_key) > 128:
            return 400, self._response("REJECTED", "invalid_idempotency_key", started)
        if set(payload) != {"command"} or not isinstance(payload["command"], str):
            return 400, self._response("REJECTED", "invalid_request_schema", started)
        fingerprint = hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode()
        ).hexdigest()
        with self.lock:
            previous = self.responses.get(idempotency_key)
            if previous:
                if previous[0] != fingerprint:
                    return 409, self._response(
                        "REJECTED", "idempotency_conflict", started
                    )
                replay = dict(previous[1])
                replay["idempotent_replay"] = True
                replay["latency_ms"] = (time.perf_counter_ns() - started) / 1e6
                return 200, replay
        result = parse_command(payload["command"])
        if result.status != "ACCEPTED":
            return 422, self._response("REJECTED", result.reason, started)
        errors = validate_dsl(result.dsl)
        if errors:
            return 500, self._response("REJECTED", errors[0], started)
        role = self.tokens[token]
        if result.dsl["intent"] not in ROLE_PERMISSIONS.get(role, set()):
            return 403, self._response("REJECTED", "authorization_failed", started)
        response = {
            "status": "ACCEPTED",
            "reason": None,
            "dsl": result.dsl,
            "idempotent_replay": False,
            "execution_dispatched": False,
            "latency_ms": (time.perf_counter_ns() - started) / 1e6,
        }
        with self.lock:
            self.responses[idempotency_key] = (fingerprint, dict(response))
        return 202, response

    @staticmethod
    def _response(status: str, reason: str, started: int) -> dict[str, Any]:
        return {
            "status": status,
            "reason": reason,
            "dsl": None,
            "idempotent_replay": False,
            "execution_dispatched": False,
            "latency_ms": (time.perf_counter_ns() - started) / 1e6,
        }
