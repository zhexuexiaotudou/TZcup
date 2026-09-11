# Competition demo delivery manifest

Use this offline tool only after one live runtime has produced its own MCAP directory, A12 MP4 observation manifest, and raw collector report. All inputs and the output directory must be under one run root. It hashes every regular file, rejects links and cross-run identities, verifies the MP4 through `ffprobe`, and writes an immutable `manifest.json`, `checksums.sha256`, and `timeline.csv`.

```powershell
py -3 scripts/generate_demo_artifact_manifest.py --run-root <run-root> --raw-collection <run-root>/raw_collection.json --video-manifest <run-root>/a12_execution.mp4.json --mcap <run-root>/a12_execution.mcap --output-dir <run-root>/demo_delivery
```

The timeline always contains map creation, hard restart, cleaning, grasp/drop, obstacle avoidance, and return home. The tool derives cleaning from the recorded A12 window, and derives grasp/drop and return only from timestamped raw-collector observations. For map creation, hard restart, and obstacle avoidance, pass one same-run observation per event using `--event-report`; the report must contain the exact `run_identity`, `event`, positive `event_epoch_ns`, and `status: "OBSERVED"`. Any missing observation remains `missing`; no historical artifact or animation can fill it.

`READY_FOR_REVIEW` means all six event rows are present and the inventory inputs match one source/runtime. It is not a competition acceptance result: semantic ROS replay and the actual runtime gates remain separate. With incomplete footage or reports the command succeeds with `BLOCKED_MISSING_REQUIRED_EVIDENCE`, producing the reviewable gap record rather than a misleading package.
