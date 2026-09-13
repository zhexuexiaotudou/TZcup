#!/usr/bin/env python3
"""Offline time budget and strict observed-cell area gate; never runs ROS."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'starter_ws/src/sanitation_formal_campus_integration'))


def area_gate(observed_area):
    return math.isfinite(observed_area) and observed_area >= 20000.0


def budget(config,rtf,wall_minutes):
    if not math.isfinite(rtf) or rtf<=0 or not math.isfinite(wall_minutes) or wall_minutes<=0:raise ValueError('positive measured RTF and wall budget required')
    r=config['mapping_range_m'];physical=config['sensor_physical_range_m'];v=config['speed_mps']
    if not 0<r<physical or not 0<v<=.45:raise ValueError('physical range or mapping speed violated')
    route=config['source_world_route_m']
    length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
    theoretical=max(0,(20000-math.pi*r*r)/(2*r))
    expected=[x/rtf for x in config['expected_sim_minutes']]
    return {'status':'BUDGET_ONLY_NOT_MAP_ACCEPTANCE','route_length_m':length,'ideal_motion_sim_minutes':length/v/60,
            'unoccluded_lower_bound_length_m':theoretical,'expected_wall_minutes':expected,
            'wall_budget_minutes':wall_minutes,'budget_sufficient_for_upper_estimate':wall_minutes>=expected[1],
            'rtf_input':rtf,'rtf_must_be_measured_in_next_run':True,'target_observed_m2':20000,
            'ninety_five_percent_m2':19000,'map_gate':'NOT_MEASURED'}


def observed_grid(bag,manifest):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from sanitation_formal_campus_integration.map_lifecycle_core import load_campus_map_contract,assess_grid_observation
    latest=None
    with bag.open('rb') as f:
        reader=make_reader(f,decoder_factories=[DecoderFactory()],validate_crcs=True)
        if reader.get_summary() is None:raise ValueError('unsealed map bag')
        for _,_,_,message in reader.iter_decoded_messages(topics=['/map']):latest=message
    if latest is None:raise ValueError('no measured map in bag')
    if latest.header.frame_id.lstrip('/')!='map':raise ValueError('map frame mismatch')
    contract=load_campus_map_contract(manifest);p=latest.info.origin.position;q=latest.info.origin.orientation
    angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    result=asdict(assess_grid_observation(latest.data,width=latest.info.width,height=latest.info.height,
        resolution=latest.info.resolution,origin_x=p.x,origin_y=p.y,origin_yaw=angle,geofence=contract.geofence,threshold=1.0))
    result['strict_area_gate']=area_gate(result['observed_area_m2'])
    result['mcap_sha256']=hashlib.sha256(bag.read_bytes()).hexdigest()
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path);p.add_argument('--rtf',required=True,type=float);p.add_argument('--wall-minutes',required=True,type=float);p.add_argument('--map-bag',type=Path);p.add_argument('--manifest',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    if a.output.exists():raise SystemExit('fresh output required')
    result=budget(json.loads(a.config.read_text()),a.rtf,a.wall_minutes)
    if a.map_bag:
        if not a.manifest:raise ValueError('--manifest required with map bag')
        result['measured_map']=observed_grid(a.map_bag,a.manifest)
        result['map_gate']='PASS' if result['measured_map']['strict_area_gate'] else 'FAIL'
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    return 2 if result['map_gate']=='FAIL' else 0


if __name__=='__main__':raise SystemExit(main())
