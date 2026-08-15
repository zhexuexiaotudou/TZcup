from reference_vision.adapters.base import CallableDetectorAdapter


class YoloWorldBenchmarkAdapter(CallableDetectorAdapter):
    benchmark_only = True
    shipped_in_product = False

    def __init__(self, predictor):
        super().__init__(model_id="yolo-world-online-x3-benchmark", predictor=predictor)
