# Writer timing audit

Verdict: **EVIDENCE_WRITER_CAUSE_UNRESOLVED**.

The freeze03 source creates the database and topics, samples/writes the open marker, creates subscriptions and then calls spin_once on one thread. Thus an ordinary callback before that marker is not supported by source order. The old test failed before invoking its finalizer, so the old finalizer did not cause that assertion. The exact harness start variable and per-callback realtime/monotonic pairs were not retained in 03; no clock-adjustment or storage-layer cause can be proven retrospectively.

New evidence records main-entry (not kernel birth) epoch/monotonic, actual writer-ready after topic creation, open-marker seal completion, subscription creation, spin-start and each callback timestamp plus serialized bytes hash. Kernel process start ticks remain in owner identity. The writer-ready lower bound is sealed before subscriptions/spin. Callback timestamps before it fail closed without clamping or rewriting. The modified finalizer verifies startup order and exact actual MCAP `(topic, timestamp, serialized SHA)` counts against completed callback trace, with no window slack for this timing contract. Legacy markers remain explicitly reported as timing-contract unverified.

The single writer-only run used a synthetic publisher and the minimal real lifecycle producer needed for its health contract; no observer or Gazebo ran. Transient-local publisher cache existed before subscription; the production reader is volatile, and the same cached object was explicitly republished. Its old source stamp remained unchanged while its storage time belonged to the new writer window. This is not proof of automatic historical DDS delivery.

The new writer/finalizer chain passed for 194 records, but does not erase or explain the older 03 failure. Synthetic clock-regression fixture proves refusal before write; it is not evidence that 03 experienced a clock regression.

Strict re-verification: finalizer and verifier default to requiring the timing schema; the real runner and minimal harness explicitly pass --require-writer-timing. Missing or downgraded markers reject. Only explicit legacy opt-out permits historical fixtures, which do not satisfy this candidate gate. Callback sequences must be contiguous, monotonic receive times ordered and bounded by spin/close, and close follows spin on both clocks. The frozen finalizer was re-run offline on the unchanged closed 194-record bag with a new receipt; no ROS producer, observer or Gazebo was restarted. Focused recorder/finalizer tests: 19 passed.
