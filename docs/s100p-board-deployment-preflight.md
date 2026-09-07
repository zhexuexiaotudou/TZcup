# S100P controlled staging read-only admission

`scripts/validate_s100p_board_deployment_preflight.py` is a board-local,
read-only admission gate. It verifies the exact S100P/aarch64 identity,
`/dev/bpu_core0` as a non-link character device, allowlisted TROS setup, and
observed `bpu_cores`/`bpu_framework` modules. It neither copies, renames,
deletes, installs, starts nodes, commands actuators, nor creates receipts.

The input is a non-link board-local handoff manifest. Its entries bind the
board-local final-predeploy report, model-payload receipt, acceptance session,
runtime closure, and the four exact payload roles by relative path, SHA-256 and
byte size. Every bound item must be a regular non-link file and match before
the report is considered. The final-predeploy report must have the exact
report id and operation boundary, `PREDEPLOY_READY_NOT_DEPLOYED`, all checks
true, no blockers, and an embedded model-receipt digest/size that matches the
board-local receipt. PC paths recorded in that report are never followed.

The model receipt must use the four formal payload roles and exact final target
relative paths. A handoff payload source may be in `handoff/payloads` or a
staging area: its role, SHA-256, and byte size must match the receipt, but it
need not equal the receipt's final target path. The report's
`pc_session_runtime_identity` binds the board-local session SHA/size and frozen
runtime-closure binding. The compatibility copy in `board_handoff_binding` must
match it exactly; the gate recomputes the closure binding and requires it to
match both the report and session.

The three fixed roles are `active=/opt/tzcup/s100p` (an existing non-link
directory), `candidate=/opt/tzcup/stages/<release>` (fresh and absent), and
`retained_old=/opt/tzcup/rollback/<old-id>` (fresh and absent). All three
parents must be existing non-link directories on the same `st_dev`; free space
is checked at the candidate parent against total verified payload bytes plus a
strictly positive explicit safety margin. All supplied board paths and handoff
relative paths reject `.`/`..`, and non-link checks include every intermediate
component to prevent escapes through a symlink.
`READY_FOR_CONTROLLED_STAGE` is not copied, deployed, started, or accepted.

## Current observed-board boundary

The retained read-only S100P observation records `/opt/tzcup/s100p` as a
directory, `/opt/tzcup/rollback` as a directory, and `/opt/tzcup/stages` as
absent, with approximately `27,037,792 KiB` free on the root filesystem. The
preflight must therefore currently return `BLOCKED`: the required candidate
parent does not exist. This is expected evidence, not an invitation to weaken
the check. A future separately authorized controlled bootstrap may create only
the empty non-link `stages` parent; it must not create a candidate, copy a
payload, or otherwise alter this read-only admission result today.

After separate authorization and graph stop, an operator may create and verify
a versioned staging directory on that same filesystem. Activation is two
controlled directory renames: first move `active` to the fresh `retained_old`
destination, then rename the verified `candidate` to `active`. A later rollback
uses a separate fresh failed-candidate destination before moving
`retained_old` back to `active`; that procedure is a second implementation
stage. This is not an atomic directory exchange. Preserve the old root until
user confirmation; later atomic-exchange work is separate.

`thermal_power_receipt.json` remains an external-measurement hard gate. Its
1800-second duration, temperature, memory, and input-power evidence is only
validated by `validate_s100p_final_predeploy.py`; this preflight never
synthesizes it.
