# S100P development artifact mode

`development_s100p_open_vocab.launch.py` is the sole non-formal product
adapter entry. It passes the exact node parameter `artifact_mode=development`.
The node defaults to `formal`; the normal `formal_s100p_open_vocab.launch.py`
does not expose a development argument, and its frozen validator rejects
development manifests.

Development requires a separate absolute-path manifest classified as
`NON_FORMAL_ABI_DEVELOPMENT`, with `status=NON_FORMAL_DEVELOPMENT`,
`formal=false`, and `board_acceptance=false`. The adapter recomputes the
regular, non-link artifact and evidence-file hashes/sizes and checks them
against version-controlled A2 HBM, custom4 vocabulary, ABI-gate, disassembly,
and source-revision anchors. Only the A2 DOSOD HBM uses the development source
revision; the vocabulary and EdgeSAM rows retain their frozen formal roles,
revisions, sizes, and hashes. It never substitutes formal expected hashes.

This path may demonstrate only `CHAIN_LIVENESS_ONLY_NON_FORMAL_NON_SEMANTIC`.
It is not a formal artifact acceptance, semantic evaluation, board acceptance,
or production authorization. Its diagnostics carry the non-formal
classification at WARN level. B0 uses an exact prepared 848x480
`DERIVED_LIVENESS_INPUT` RGB-D tuple with CameraInfo, map, and static
map-to-camera TF; it is non-semantic and cannot establish a product result.
