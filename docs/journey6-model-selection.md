# Journey 6 pretrained model selection

## Fixed candidates

| ID | Role | Source revision | Available format at audit | Current status |
|---|---|---|---|---|
| D1 | YOLOv9-C litter detector | `aryanshh/littercamv3@861363597e109f9f0840f537f48d890cef5b5461` | PT weights only | ONNX/J6 preflight blocked |
| D2 | YOLO11n material detector | `SUHAN-I/YOLO11@d7a78128455ef607a922f50681187f8b32b2af53` | ONNX and PT | SHA/checker pass; semantic/J6 preflight blocked |
| C1 | YOLOv8n material classifier | `SriramRokkam/wastewise-garbage-cls@a30c36c6b181ac0d2eb387bbd4f6d4a5b88ee078` | ONNX | SHA/checker pass; release license blocked |
| C2 | EfficientNet-B0 material classifier | `nabaouladyahich/ecodetect-waste-classifier@9719e6fc9a352d62209529e0e0573fff3bb7dc3d` | H5 only | reproducible ONNX export pending |

These revision locks identify source snapshots; they are not artifact hashes.
D2 and C1 were downloaded outside Git and verified as
`bce5b56cc825efaf4912a7137f74ad147634ac99ef6585662d003d39329ab100`
and `2b46d491091dbc0ed98a0f1eaee7fe5739c8fd3eb5bd5935396c3b2712e1f7a6`.
Both pass ONNX checker, have static batch-1 inputs, no custom operators, and no
embedded NMS.

D2 nevertheless fails closed: both advertised ONNX filenames resolve to the
same artifact, whose graph exposes eight anonymous `class_0...class_7` outputs
instead of the model card's six named material classes. Its IR version is 10,
above the current J6 preflight maximum of 9. C1's eight embedded names match the
card, but the graph is opset 12 rather than the card's opset 17, and ONNX
metadata identifies the Ultralytics implementation as AGPL-3.0 while the model
card declares Apache-2.0. C1 remains development-audit-only until the layered
license is resolved. The current `pretrained_pc_integration_dev` manifest is
intentionally disabled.

## Class semantics

D1 can directly propose `plastic_bottle`, `drinks_can -> metal_can`, and
`paper_waste -> paper_litter`. D2 supplies material evidence; `plastic` and
`metal` alone do not prove bottle or can shape. C1/C2 material classes support
the detector decision, while cardboard, glass, trash, battery, and biological
map to `background_or_unknown`.

The pipeline requires a stable class-agnostic track, valid RGB-D projection,
detector/classifier agreement, and ActionVerifier. It can emit
`READY_FOR_ACTION_VERIFIER`, `OBSERVE_AGAIN`, or `DEFER`; it never emits
`CONFIRMED` or `CLEAN_NOW` directly.

## Selection order

Candidates are compared in this order:

1. source and redistribution license;
2. class coverage and semantic fit;
3. fixed development detector/classifier/combined metrics;
4. static ONNX and Journey 6 operator compatibility;
5. accuracy and safety gates;
6. 10 Hz full-chain capacity;
7. smallest resource profile among qualifying candidates.

If no pretrained combination passes V1, the best verified combination may be
used only for the `integration_dev` lane. It remains
`competition_claim_allowed=false`; sealed data stays unread and no new training
starts automatically.

The prior CRCRV11 stop condition remains a product fact. This pretrained route
does not rewrite its failed `0.6311` macro-F1 result and cannot activate the V1
classifier gate without new fixed-development evidence.
