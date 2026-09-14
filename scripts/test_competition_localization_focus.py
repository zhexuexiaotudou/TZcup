import copy
import importlib.util
import json
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('focus',Path(__file__).with_name('competition_localization_focus.py'))
focus=importlib.util.module_from_spec(spec);spec.loader.exec_module(focus)

def sample():
    rows=[[i*.02,i*.006,0.,0.,.3,0.] for i in range(501)]
    d={'gt':copy.deepcopy(rows),'fused':copy.deepcopy(rows),'odom':copy.deepcopy(rows),'tf':[[r[0],0.,0.,0.] for r in rows],'local_tf':[r[:4] for r in rows],
       'counts':{k:500 for k in ('/odom/unfiltered','/imu/data','/amcl_pose','/odometry/gps','/gnss/fix')},'tf_future_max_s':0.,'tf_past_max_s':0.}
    for r in d['gt']:r[1]-=98
    a={'graph_nodes':['/global_ekf'],'tf_edges':{'map->odom':{'message_count':500,'messages_by_gid':{'one':500}}},'endpoint_registry':{'one':{'node':'/global_ekf'}},'topics':{'/localization/fused_odom':{'message_count':500,'publishers':[{'node':'/global_ekf'}]},'/odometry/gps':{'subscriptions':[{'node':'/global_ekf'}]}}}
    m={'vehicle_start_pose_source_world':{'x_m':-98.,'y_m':0.,'yaw_rad':0.},'vehicle_start_pose_localization_map':{'x_m':0.,'y_m':0.,'yaw_rad':0.}}
    return d,a,m

def test_valid_dynamic_map():
    r=focus.assess(*sample());assert r['status']=='PASS';assert r['accuracy']['samples']==501

def test_static_is_rejected():
    d,a,m=sample()
    for row in d['gt']:row[1]=-98.;row[4]=0
    r=focus.assess(d,a,m);assert r['status']=='FAIL';assert r['accuracy']['samples']==0

def test_two_tf_gids_rejected_even_if_same_node():
    d,a,m=sample();a['tf_edges']['map->odom']['messages_by_gid']['two']=4;a['endpoint_registry']['two']={'node':'/global_ekf'}
    assert focus.assess(d,a,m)['status']=='FAIL'

def test_static_contract_attributes_single_cyclone_gid_without_registry_match():
    d,a,m=sample();a['endpoint_registry']={}
    contract={'owner':'/global_ekf'}
    result=focus.assess(d,a,m,contract)
    assert result['status']=='PASS'
    assert result['authority']['basis']=='single_gid_static_contract'

def test_static_contract_never_overrides_missing_runtime_global_ekf():
    d,a,m=sample();a['endpoint_registry']={}
    a['topics']['/localization/fused_odom']['publishers']=[]
    result=focus.assess(d,a,m,{'owner':'/global_ekf'})
    assert result['status']=='FAIL'
    assert result['authority']['basis']=='unproven'

def test_source_static_authority_contract_is_complete(tmp_path):
    effective={
        'schema_version':1,
        'all_expected':True,
        'nodes':{
            name:{'fixture':{'expected':True,'actual':True,'matched':True}}
            for name in ('/local_ekf','/global_ekf','/amcl','/navsat_transform')
        },
    }
    path=tmp_path/'effective-parameters-fixture.json'
    path.write_text(json.dumps(effective),encoding='utf-8')
    contract=focus.static_authority_contract(Path(__file__).resolve().parents[1],path)
    assert contract['owner']=='/global_ekf'
    assert all(contract['checks'].values())

def test_source_static_authority_contract_rejects_unmatched_parameters(tmp_path):
    effective={
        'schema_version':1,
        'all_expected':True,
        'nodes':{
            name:{'fixture':{'expected':True,'actual':False,'matched':False}}
            for name in ('/local_ekf','/global_ekf','/amcl','/navsat_transform')
        },
    }
    path=tmp_path/'effective-parameters-fixture.json'
    path.write_text(json.dumps(effective),encoding='utf-8')
    with pytest.raises(ValueError,match='effective_parameters_expected'):
        focus.static_authority_contract(Path(__file__).resolve().parents[1],path)

@pytest.mark.parametrize('field,value',[('tf_future_max_s',.5),('tf_past_max_s',1),('fused',[]),('frame_errors',['wrong_frame']),('counts',{})])
def test_fail_closed_missing_or_stale(field,value):
    d,a,m=sample();d[field]=value;assert focus.assess(d,a,m)['status']=='FAIL'

def test_no_fitted_offset():
    d,a,m=sample()
    for r in d['fused']:r[1]+=.23
    result=focus.assess(d,a,m);assert result['status']=='FAIL';assert result['fused_accuracy']['max_m']==pytest.approx(.23)

def test_missing_spans_remain_in_denominator():
    d,a,m=sample();d['fused']=d['fused'][:200]+d['fused'][350:]
    r=focus.assess(d,a,m);assert r['pair_coverage']<.75;assert r['status']=='FAIL'

def test_no_interpolation_extrapolation():
    assert focus.interpolate([[1,0,0,0],[2,0,0,0]],.9,.03) is None
    assert focus.interpolate([[1,0,0,0],[2,0,0,0]],1.5,.03) is None

def test_public_rotated_origin():
    import math
    d,a,m=sample();m['vehicle_start_pose_source_world']['yaw_rad']=math.pi/2
    for r in d['gt']:r[2]=r[1]+98;r[1]=-98
    assert focus.assess(d,a,m)['status']=='PASS'

def test_nonmonotonic_and_nonfinite():
    for val in (-1,float('nan')):
        d,a,m=sample();d['fused'][10][0]=val;assert focus.assess(d,a,m)['status']=='FAIL'


def test_navigation_tf_is_scored_even_when_fused_odometry_is_perfect():
    d,a,m=sample()
    for r in d['tf']:r[1]=.06
    result=focus.assess(d,a,m)
    assert result['status']=='FAIL'
    assert result['accuracy']['max_m']==pytest.approx(.06)
    assert result['fused_accuracy']['max_m']==pytest.approx(0)

def test_no_local_tf_cannot_fall_back_to_odometry():
    d,a,m=sample();d['local_tf']=[]
    assert 'missing_local_tf' in focus.assess(d,a,m)['failures']
