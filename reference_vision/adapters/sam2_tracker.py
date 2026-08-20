from reference_vision.adapters.base import CallableTrackerAdapter


class Sam2TrackerAdapter(CallableTrackerAdapter):
    shipped_in_product = False

    def __init__(self, predictor):
        super().__init__(model_id="sam2.1-reference-tracker", predictor=predictor)
