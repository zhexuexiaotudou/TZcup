# Perception reference third-party notices

This directory is an audit registry, not a vendored dependency bundle. No third-party code,
checkpoint, or dataset is committed here or authorized for product redistribution.

- Grounding DINO is pinned to the official IDEA-Research repository under Apache-2.0 for
  reference evaluation. Checkpoint redistribution remains blocked until the exact downloaded
  artifact and its license/provenance are recorded.
- SAM 2 is pinned to the official Meta repository. Its official README states that model
  checkpoints, demo code, and training code use Apache-2.0; optional bundled components keep
  their own notices.
- Grounded SAM 2 remains reference-only until all transitive component and checkpoint licenses
  are audited.
- YOLO-World is GPL-3.0 and benchmark-only by default. It is not part of the product image or
  default release.
- The TACO toolkit repository is MIT. Dataset images are not ingested until source and
  image-level rights are audited; any accepted data is training-only and excluded from sealed
  final evaluation.
