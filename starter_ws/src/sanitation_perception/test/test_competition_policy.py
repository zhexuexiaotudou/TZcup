import json
from types import SimpleNamespace as NS
import pytest
from sanitation_perception.competition_policy import thresholds,validate_context,in_ground_roi,publishable

def message():
    return NS(header=NS(frame_id='camera',stamp=NS(sec=1,nanosec=0)),width=4,height=4,k=[2.,0,2,0,2,2,0,0,1],encoding='32FC1')

def test_tentative_is_not_target():
    assert not publishable('TENTATIVE');assert publishable('CONFIRMED')
    assert publishable('TENTATIVE',True);assert not publishable('LOST',True)

@pytest.mark.parametrize('change',[
    lambda r,d,i:setattr(d.header.stamp,'nanosec',100_000_000),
    lambda r,d,i:setattr(d.header,'frame_id','other'),
    lambda r,d,i:setattr(d,'width',3),
    lambda r,d,i:setattr(d,'encoding','mono16'),
    lambda r,d,i:setattr(r.header.stamp,'sec',0),
    lambda r,d,i:setattr(i,'k',[float('nan')]*9)])
def test_context_rejects(change):
    r,d,i=message(),message(),message();change(r,d,i)
    with pytest.raises(ValueError):validate_context(r,d,i,.03)

def test_registered_context_and_latched_info():
    r,d,i=message(),message(),message();i.header.stamp.sec=0
    validate_context(r,d,i,.03)

def test_roi_and_thresholds_fail_closed():
    assert in_ground_roi([1,0,.01],[.4,3,-.8,.8,-.08,.15])
    assert not in_ground_roi([1,0,1],[.4,3,-.8,.8,-.08,.15])
    assert thresholds('{"leaf":0.8}',['leaf'])=={'leaf':.8}
    with pytest.raises(ValueError):thresholds('{"leaf":NaN}',['leaf'])
    with pytest.raises(ValueError):thresholds('{}',['leaf'])

def test_same_frame_components_do_not_confirm_each_other():
    from sanitation_perception.tracking import TargetTracker
    tracker=TargetTracker(confirmation_observations=3)
    base={'class_id':'plastic_bottle','target_type':'discrete','cleaning_policy':'spot_clean',
          'y_m':0.,'confidence':.9,'covariance_trace':.001,'source_backend':'onnxruntime'}
    tracks=tracker.update([{**base,'x_m':x} for x in (1.,1.05,1.1)],now=1.)
    assert len(tracks)==3
    assert all(t.state=='TENTATIVE' and t.observation_count==1 for t in tracks)
