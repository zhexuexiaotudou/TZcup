# PERCEPTION MRV2 third-party notices

## Grounding DINO

- Upstream: IDEA-Research/GroundingDINO
- Source commit: `856dde20aee659246248e20734ef9ba5214f5e44`
- Source license: Apache-2.0
- Checkpoint: `groundingdino_swint_ogc.pth`
- Checkpoint SHA256: `3b3ca2563c77c69f651d7bd133e97139c186df06231157a64c507099c52bc799`
- Role: reference benchmark only
- Shipped in product: no
- Redistribution: not attempted; exact checkpoint-artifact license remains a release blocker.
- Local modification: recorded CUDA use of the official PyTorch deformable-attention fallback because the reference container has no nvcc/custom op.

## Torchvision

- Role: FCOS/ResNet-FPN training and benchmark runtime
- Distribution status: no product bundle was created because the static gate failed.
