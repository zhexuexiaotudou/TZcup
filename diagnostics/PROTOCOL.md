# Short09 lifecycle health protocol — offline design

Base: b430. No remote build or runtime authorized.

The existing business status stays intact. A separate health envelope binds producer boot ID, PID, process start ticks, run token, acceptance-session path/hash, health sequence and monotonic publication time. Heartbeats run independently at 0.5 seconds. Only an actual business evaluation/state publication increments evaluation sequence/time and changes the business hash. A heartbeat never refreshes business evaluation age.

Readers validate the actual producer process, including its environment session/run token, and lock the identity for the run. They retain the last health/evaluation sequences and byte hashes. Same sequence/same bytes is a repeated read and cannot refresh the original timestamp. Same sequence/different bytes, rollback, changed owner, restart, wrong session, future time and expired health fail closed. Health freshness remains at most 2 seconds. Other topic checks are unchanged.

Business validity uses a newly selected 15-second nonterminal evaluation lease, distinct from existing sensor/map input freshness. The lease, kind, evaluation timestamp and business hash are immutable for the same eval_seq. Terminal-ready requires exact session-relative map/episode paths and hashes, and calls the original saved-map artifact validator with the original loaded episode contract, while continuing to require live health. No terminal state alone proves a mission succeeded. Failure terminal denies admission.

Fixtures first: sparse legitimate business updates with fresh unique health; identical repeated read without lease renewal; terminal-ready and terminal-failure; producer dead, zombie or restart; cross-session/replayed packet; health/evaluation rollback; same-sequence mutation; future timestamp; expired health and expired business evaluation; normal writer close.

All 2 m / 100 ms / 95 percent / 570 seconds / 125 seconds / safety gates remain unchanged. The implementation is locally tested and awaiting independent review; this is not a formal runtime PASS.
