from reference_vision.adapters.base import CallableDetectorAdapter


PROMPT_SETS = {
    "closed": ("plastic bottle", "beverage can", "metal can", "paper litter", "cardboard litter"),
    "litter": ("trash", "litter"),
    "ensemble": (
        "plastic bottle",
        "beverage can",
        "metal can",
        "paper litter",
        "cardboard litter",
        "trash",
        "litter",
    ),
}


class GroundingDinoAdapter(CallableDetectorAdapter):
    def __init__(self, predictor):
        super().__init__(
            model_id="grounding-dino-online-x2",
            predictor=predictor,
            label_map={
                "plastic bottle": "plastic_bottle",
                "beverage can": "metal_can",
                "metal can": "metal_can",
                "paper litter": "paper_litter",
                "cardboard litter": "paper_litter",
                "trash": "litter_candidate",
                "litter": "litter_candidate",
            },
        )
