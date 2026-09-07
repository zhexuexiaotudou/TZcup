import numpy as np
import pytest
import public_gazebo_dosod_metrics as m
def test_raw_metrics_zero_and_nonfinite_edges():
 assert m.raw_metrics(np.zeros((2,)),np.zeros((2,)))["cosine"]==1
 assert m.raw_metrics(np.zeros((2,)),np.ones((2,)))["cosine"]==0
 with pytest.raises(ValueError):m.raw_metrics(np.asarray([np.nan]),np.zeros((1,)))
def test_iou_matching_empty_and_truth_tie_are_deterministic():
 rows=[{"truth":[{"class_id":"litter_cube","xyxy":[0,0,2,2]},{"class_id":"litter_cube","xyxy":[0,0,2,2]}],"predictions":[{"class_id":"litter_cube","score":.9,"xyxy":[0,0,2,2]}]}]
 result=m.evaluate(rows); assert result["classes"]["litter_cube"]["tp"]==1 and result["classes"]["litter_cube"]["fn"]==1
 empty=m.evaluate([{"truth":[],"predictions":[]}]); assert empty["fp_per_frame"]==0 and empty["classes"]["puddle"]["tp"]==0
