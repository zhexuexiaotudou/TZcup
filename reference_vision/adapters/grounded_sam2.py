from reference_vision.adapters.base import CallableTrackerAdapter


class GroundedSam2Adapter(CallableTrackerAdapter):
    reference_only = True
    shipped_in_product = False

    def __init__(self, predictor):
        super().__init__(model_id="grounded-sam2-reference", predictor=predictor)
