# PERCEPTION-ONLINE third-party status

The current branch ships no third-party reference code, checkpoint, or dataset in the ROS
product runtime or release. The authoritative machine-readable inventory is
`third_party/perception/`.

- Grounding DINO: official repository pinned at
  `856dde20aee659246248e20734ef9ba5214f5e44`, Apache-2.0 repository license. Its exact
  checkpoint is not downloaded or redistributed; artifact-level provenance remains pending.
- SAM 2: official repository pinned at
  `2b90b9f5ceec907a1c18123530e92e794ad901a4`. The official repository states code and
  checkpoints use Apache-2.0. It is currently an adapter-only reference lane.
- Grounded SAM 2: official repository pinned at
  `b7a9c29f196edff0eb54dbe14588d7ae5e3dde28`; reference-only until all transitive component
  obligations are audited.
- YOLO-World: official repository pinned at
  `4f70adbaacf5685bd9ec5bea85f1f91057f6fc0b`, GPL-3.0. It is benchmark-only and excluded
  from the default product bundle.
- TACO: toolkit repository pinned at
  `29de1a9ba05a647b83a90f18d7772e20bb23d846`, MIT toolkit license. No image is ingested
  until its source and image-level rights pass audit; any accepted data is TRAIN-only.
