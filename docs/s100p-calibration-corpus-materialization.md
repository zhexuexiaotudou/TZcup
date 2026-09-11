# S100P DOSOD real-input calibration corpus materialization

`scripts/materialize_s100p_calibration_corpus.py` consumes only an already
committed `ProductIntermediateCapture` root.  It verifies each frame bundle's
two-file manifest before reading `arrays.npz`, derives the source SHA-256 from
the recorded `rgb` bytes, writes a unique `[1,3,640,640] float32` tensor, and
binds timestamp, camera frame, scenario, observed class labels, capture-manifest
hash and preprocessing contract to per-frame provenance.

The output state is intentionally resumable only when its capture root,
scenario, frozen compile contract, oracle identity and holdout set match
exactly.  Both duplicate RGB source hashes and duplicate generated tensor hashes
are skipped; a hash in the independent holdout set is rejected.  It never
creates inputs, replays MCAP, launches ROS/Gazebo, or uses public COCO/Gazebo
calibration data.

On each resume it re-hashes every already registered tensor and checks the
provenance source binding before admitting another frame.  Once a frozen
manifest exists, the output is immutable and cannot be resumed.

Example for a previously retained product-capture root:

```powershell
py -3 scripts/materialize_s100p_calibration_corpus.py `
  --capture-root <ProductIntermediateCapture root> `
  --output <fresh private corpus root> `
  --scenario-id <immutable scene identifier>
```

Run the same command again with the same three values after the next retained
product capture; it resumes only after revalidating every prior output.  New
captures must come from the already-running product adapter's
`intermediate_capture_root` output.  Do not point this command at a Gazebo
pilot, a rosbag replay, COCO, or a copied frame directory.

## Current local input inventory (2026-09-11)

A read-only filename inventory of the authorized
`F:/Project/TZcup/.workspace` found **0** committed
`frames/frame-NNNN/arrays.npz` ProductIntermediateCapture bundles and **0**
MCAP files.  Therefore no true-input materialization smoke was run, the current
real tensor count is **0/500**, and the remaining quantity blocker is **500
unique real camera frames**.  The independent validation-holdout and canonical
official preprocessing-oracle finalizer are also absent.  These are blockers,
not permission to create substitutes.

Without the canonical outer preprocessing-oracle receipt, the resulting
`materialization_receipt.json` is `BLOCKED`; it is a true-input smoke corpus,
not a frozen compiler input.  The only route that creates
`calibration_manifest.json` is: a verified official oracle, at least 500
unique source and tensor hashes, and a separately bound non-empty validation
holdout.  That status is still only compile-preflight eligible and does not
produce an HBM or claim board acceptance.
