# Journey 6 pretrained model selection report

No detector/classifier combination was selected or activated.

- D1 is blocked because the pinned revision contains PT weights but no ONNX.
- D2 downloads and passes SHA, ONNX checker, static batch-one shape, custom-op,
  and embedded-NMS audits. Both advertised ONNX filenames resolve to the same
  SHA. The graph contains eight anonymous classes, not the model card's six
  named material classes, and IR version 10 exceeds the current J6 preflight
  maximum of 9.
- C1 downloads and passes the same static graph checks. Its eight class names
  agree with the model card, but the card says opset 17 while the graph is
  opset 12. More importantly, embedded Ultralytics metadata declares AGPL-3.0
  while the card declares Apache-2.0, so release and competition use remain
  blocked pending a resolved layered license.
- C2 is blocked because the pinned revision contains H5 but no ONNX.

The fixed calibration gate also fails with `0 < 1000` records. Because no
usable detector remains, offline accuracy, Gazebo online function,
ActionVerifier/Map/Spot/Post-Clean, performance, soak, fault, and replay runs
were not started. Sealed data was not accessed and no training, QAT,
distillation, or fine-tuning was started.
