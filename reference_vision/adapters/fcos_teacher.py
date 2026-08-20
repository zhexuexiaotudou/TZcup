from reference_vision.adapters.base import CallableDetectorAdapter


class FcosTeacherAdapter(CallableDetectorAdapter):
    def __init__(self, predictor):
        super().__init__(model_id="fcos-r50-online-x1", predictor=predictor)
