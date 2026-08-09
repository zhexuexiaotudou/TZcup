# PERCEPTION PRODUCT FINAL THIRD-PARTY NOTICES

No third-party model checkpoint is shipped because no product model qualified.

| Dependency | Pinned/audited version | License | Product disposition |
|---|---|---|---|
| Torchvision FCOS ResNet50-FPN | COCO_V1, SHA-256 `99b0c9b7...b9e7` | BSD-3-Clause code; reference-weight dataset terms also apply | Used for X1/X3 development only; trained checkpoints external and not shipped |
| Grounding DINO | commit `856dde20aee659246248e20734ef9ba5214f5e44` | Apache-2.0 | X2 checkpoint unavailable; not loaded or shipped |
| SAM 2 | commit `2b90b9f5ceec907a1c18123530e92e794ad901a4` | Apache-2.0 | Reference tooling only; no checkpoint shipped |
| YOLO-World | commit `4f70adbaacf5685bd9ec5bea85f1f91057f6fc0b` | GPL-3.0 | Rejected as the product X3 route; not shipped |

Exact dependency commit and URL records are in
`prod00_resources/reference_dependency_inventory.json`. Runtime redistribution requires a
fresh legal review if a future qualified model changes this no-shipment state.
