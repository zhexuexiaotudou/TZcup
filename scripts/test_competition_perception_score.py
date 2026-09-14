from competition_perception_score import score
import pytest

def test_false_positive_and_zero_recall_are_visible():
    rows=[{'frame_id':'one','predictions':[{'class_id':'plastic_bottle','confidence':.9,'xyxy':[0,0,10,10]}],'truth':[{'object_id':'leaf','class_id':'leaf_pile','xyxy':[0,0,10,10]}]},
          {'frame_id':'background','predictions':[],'truth':[]}]
    r=score(rows);assert r['class_metrics']['leaf_pile']['recall']==0
    assert r['class_metrics']['plastic_bottle']['fp']==1
    assert len(r['false_positive_samples'])==1;assert not r['competition_perception_pass']
    assert r['false_negative_samples'][0]['truth']['object_id']=='leaf'

def test_duplicate_frame_cannot_inflate_score():
    r={'frame_id':'same','predictions':[],'truth':[]}
    with pytest.raises(ValueError):score([r,r])

def test_multiple_same_class_predictions_match_once():
    p={'class_id':'leaf_pile','confidence':.9,'xyxy':[0,0,10,10]}
    r=score([{'frame_id':'a','predictions':[p,p],'truth':[{'object_id':'x','class_id':'leaf_pile','xyxy':[0,0,10,10]}]}])
    assert r['class_metrics']['leaf_pile']['precision']==.5
