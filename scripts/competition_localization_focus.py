#!/usr/bin/env python3
"""Read-only, truth-isolated offline scoring of a sealed dynamic run.

No pose fitting, timestamp shifting or odom-as-map fallback. Raw fusion inputs
are diagnosed, never edited. Requires the existing GID-aware runtime collector.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics
import yaml


def metrics(values):
    if not values:
        return {'samples': 0, 'rmse_m': None, 'p95_m': None, 'max_m': None}
    ordered = sorted(values)
    rank = (len(ordered) - 1) * .95
    lo = int(rank); hi = min(lo + 1, len(ordered) - 1)
    return {'samples': len(values), 'rmse_m': math.sqrt(sum(x*x for x in values)/len(values)),
            'p95_m': ordered[lo] + (ordered[hi]-ordered[lo])*(rank-lo), 'max_m': max(values)}


def interpolate(rows, t, gap):
    """Rows [stamp,x,y,yaw,vx,vy]; no extrapolation or bridging missing spans."""
    times = [r[0] for r in rows]
    i = bisect.bisect_left(times, t)
    if i < len(rows) and abs(times[i]-t) < 1e-9:
        return rows[i][1:]
    if i == 0 or i == len(rows) or times[i]-times[i-1] > gap:
        return None
    a, b = rows[i-1], rows[i]; ratio = (t-a[0])/(b[0]-a[0])
    v = [x+(y-x)*ratio for x,y in zip(a[1:],b[1:])]
    if len(v) >= 3:
        delta = math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))
        v[2] = a[3] + ratio*delta
    return v


def static_authority_contract(
    source_root, effective_parameters, map_odom_owner='/global_ekf'
):
    """Prove the configured cleaning authority without relying on DDS GIDs."""
    if map_odom_owner not in {'/global_ekf', '/map_odom_stabilizer'}:
        raise ValueError(f'unsupported map->odom owner: {map_odom_owner}')
    fusion_path = (
        source_root
        / 'starter_ws/src/sanitation_localization/config/formal_fusion.yaml'
    )
    fusion_launch_path = (
        source_root
        / 'starter_ws/src/sanitation_localization/launch/formal_localization_fusion.launch.py'
    )
    probe_path = (
        source_root / 'scripts/run_day1_localization_stabilizer_live.sh'
        if map_odom_owner == '/map_odom_stabilizer'
        else source_root / 'scripts/run_competition_motion_cleaning_probe.sh'
    )
    capture_path = (
        source_root / 'scripts/capture_competition_localization_parameters.py'
    )
    lifecycle_path = (
        source_root
        / 'starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py'
    )
    fusion = yaml.safe_load(fusion_path.read_text())
    local = fusion['local_ekf']['ros__parameters']
    global_ = fusion['global_ekf']['ros__parameters']
    navsat = fusion['navsat_transform']['ros__parameters']
    probe_text = probe_path.read_text()
    effective = json.loads(effective_parameters.read_text())
    effective_nodes = effective.get('nodes', {})
    expected_nodes = {'/local_ekf', '/global_ekf', '/amcl', '/navsat_transform'}
    if map_odom_owner == '/map_odom_stabilizer':
        expected_nodes.add(map_odom_owner)
    effective_matched = all(
        isinstance(effective_nodes.get(node), dict)
        and bool(effective_nodes[node])
        and all(
            value.get('matched') is True
            and value.get('actual') == value.get('expected')
            for value in effective_nodes[node].values()
        )
        for node in expected_nodes
    )
    checks = {
        'local_ekf_publishes_odom_tf': (
            local.get('publish_tf') is True
            and local.get('world_frame') == 'odom'
            and local.get('odom_frame') == 'odom'
            and local.get('base_link_frame') == 'base_footprint'
        ),
        'global_ekf_publishes_map_tf': (
            global_.get('publish_tf') is True
            and global_.get('world_frame') == 'map'
            and global_.get('map_frame') == 'map'
            and global_.get('odom_frame') == 'odom'
            and global_.get('base_link_frame') == 'base_footprint'
        ),
        'navsat_publishes_no_tf': (
            navsat.get('broadcast_utm_transform') is False
            and navsat.get('broadcast_cartesian_transform') is False
        ),
        'probe_amcl_tf_default_false': (
            'export PROBE_AMCL_TF_BROADCAST="${PROBE_AMCL_TF_BROADCAST:-false}"'
            in probe_text
            or 'export PROBE_AMCL_TF_BROADCAST=false' in probe_text
        ),
        'probe_amcl_tf_is_explicit': (
            "nav['amcl']['ros__parameters']['tf_broadcast']="
            "os.environ['PROBE_AMCL_TF_BROADCAST']=='true'"
            in probe_text
            or "nav['amcl']['ros__parameters']['tf_broadcast']=False"
            in probe_text
        ),
        'lifecycle_amcl_tf_false': (
            'nav2["amcl"]["ros__parameters"]["tf_broadcast"] = False'
            in lifecycle_path.read_text()
        ),
        'effective_parameters_expected': (
            effective.get('schema_version') == 1
            and effective.get('all_expected') is True
            and set(effective_nodes) == expected_nodes
            and effective_matched
        ),
        'stabilizer_harness_contract': (
            map_odom_owner != '/map_odom_stabilizer'
            or (
                'map_odom_stabilizer:=true' in probe_text
                and '--map-odom-owner /map_odom_stabilizer'
                in probe_text
            )
        ),
        'stabilizer_runtime_parameters': (
            map_odom_owner != '/map_odom_stabilizer'
            or (
                effective_nodes.get('/map_odom_stabilizer', {})
                .get('tau_sec', {})
                .get('actual')
                == 1.5
                and effective_nodes.get('/map_odom_stabilizer', {})
                .get('max_filter_dt_sec', {})
                .get('actual')
                == 0.1
                and effective_nodes.get('/map_odom_stabilizer', {})
                .get('input_tf_topic', {})
                .get('actual')
                == '/localization/raw_map_odom'
            )
        ),
        'stabilizer_launch_contract': (
            map_odom_owner != '/map_odom_stabilizer'
            or (
                '("/tf", raw_map_odom_topic)'
                in fusion_launch_path.read_text()
                and 'executable="map_odom_stabilizer"'
                in fusion_launch_path.read_text()
            )
        ),
    }
    if not all(checks.values()):
        failed = ','.join(name for name, passed in checks.items() if not passed)
        raise ValueError(f'static map->odom authority contract failed: {failed}')
    return {
        'owner': map_odom_owner,
        'checks': checks,
        'source_files': [
            str(fusion_path),
            str(fusion_launch_path),
            str(probe_path),
            str(capture_path),
            str(lifecycle_path),
            str(effective_parameters),
        ],
    }


def authority_check(report, contract=None):
    expected_owner = (contract or {}).get('owner', '/global_ekf')
    gids = {g for g,n in report.get('tf_edges',{}).get('map->odom',{}).get('messages_by_gid',{}).items() if n > 0}
    registry = report.get('endpoint_registry',{})
    nodes = [registry.get(g,{}).get('node') for g in gids]
    count = report.get('tf_edges',{}).get('map->odom',{}).get('message_count',0)
    exact = len(gids) == 1 and nodes == [expected_owner] and count >= 3
    fused = report.get('topics',{}).get('/localization/fused_odom',{})
    fused_publishers = {x.get('node') for x in fused.get('publishers',[])}
    gps_subscribers = {
        x.get('node')
        for x in report.get('topics',{}).get('/odometry/gps',{}).get('subscriptions',[])
    }
    configured = expected_owner in {'/global_ekf', '/map_odom_stabilizer'}
    runtime_proves_global = (
        '/global_ekf' in set(report.get('graph_nodes',[]))
        and fused_publishers == {'/global_ekf'}
        and '/global_ekf' in gps_subscribers
        and int(fused.get('message_count',0)) >= 3
    )
    raw_topic = report.get('topics',{}).get('/localization/raw_map_odom',{})
    raw_publishers = {x.get('node') for x in raw_topic.get('publishers',[])}
    raw_subscribers = {
        x.get('node') for x in raw_topic.get('subscriptions',[])
    }
    non_recorder_raw_subscribers = {
        node
        for node in raw_subscribers
        if node and not node.rstrip('/').endswith('/rosbag2_recorder')
        and node.rstrip('/') != '/rosbag2_recorder'
    }
    runtime_proves_stabilizer = (
        expected_owner == '/map_odom_stabilizer'
        and '/map_odom_stabilizer' in set(report.get('graph_nodes',[]))
        and raw_publishers == {'/global_ekf'}
        and non_recorder_raw_subscribers == {'/map_odom_stabilizer'}
        and int(raw_topic.get('message_count',0)) >= 3
    )
    runtime_proves_owner = (
        runtime_proves_stabilizer
        if expected_owner == '/map_odom_stabilizer'
        else runtime_proves_global
    )
    attributed = len(gids) == 1 and count >= 3 and configured and runtime_proves_owner
    return (exact or attributed), {
        'gids': sorted(gids),
        'nodes': nodes,
        'count': count,
        'basis': (
            'exact_endpoint_gid'
            if exact
            else ('single_gid_static_contract' if attributed else 'unproven')
        ),
        'configured_owner': (contract or {}).get('owner'),
        'runtime_global_ekf': runtime_proves_global,
        'runtime_stabilizer': runtime_proves_stabilizer,
        'fused_publishers': sorted(x for x in fused_publishers if x),
        'gps_subscribers': sorted(x for x in gps_subscribers if x),
    }


def assess(data, authority, manifest, authority_contract=None):
    failures=[]
    unique, owners=authority_check(authority, authority_contract)
    if not unique: failures.append('map_to_odom_authority_not_expected_unique')
    for name in ('gt','fused','odom','tf','local_tf'):
        rows=data.get(name,[])
        if not rows: failures.append('missing_'+name)
        if any(not all(math.isfinite(v) for v in row) for row in rows): failures.append('nonfinite_'+name)
        if any(b[0] <= a[0] for a,b in zip(rows,rows[1:])): failures.append('nonmonotonic_'+name)
    if any(f != 'map_to_odom_authority_not_expected_unique' for f in failures):
        return {'status':'FAIL','failures':failures,'authority':owners,'accuracy':metrics([])}
    gt=data['gt']; fused=data['fused']; odom=data['odom']; tf=data['tf']
    source=manifest['vehicle_start_pose_source_world']; dest=manifest['vehicle_start_pose_localization_map']
    angle=float(dest['yaw_rad'])-float(source['yaw_rad']);c,s=math.cos(angle),math.sin(angle)
    def reference(row):
        x,y=row[0]-source['x_m'],row[1]-source['y_m']
        return dest['x_m']+c*x-s*y,dest['y_m']+s*x+c*y
    displacement=max(math.hypot(r[1]-gt[0][1],r[2]-gt[0][2]) for r in gt)
    path=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(gt,gt[1:]))
    # Score on every dynamic GT epoch, so missing estimator spans cannot vanish
    # from the denominator. Linear interpolation uses bounded adjacent samples.
    dynamic=[r for r in gt if math.hypot(r[4],r[5]) > .05 or (len(r)>6 and abs(r[6])>.05)]
    errors=[];cross=[];fused_errors=[];pairs=[]
    for r in dynamic:
        f=interpolate(fused,r[0],.05); o=interpolate(data['local_tf'],r[0],.03);tr=interpolate(tf,r[0],.05)
        if f is None or o is None or tr is None: continue
        gx,gy=reference(r[1:]); fused_errors.append(math.hypot(f[0]-gx,f[1]-gy))
        ct,st=math.cos(tr[2]),math.sin(tr[2])
        nx,ny=tr[0]+ct*o[0]-st*o[1],tr[1]+st*o[0]+ct*o[1]
        errors.append(math.hypot(nx-gx,ny-gy));cross.append(math.hypot(nx-f[0],ny-f[1]))
        pairs.append({'sim_s':r[0],'gt_map_xy':[gx,gy],'navigation_map_xy':[nx,ny],'fused_xy':f[:2],'navigation_error_m':errors[-1],'fused_error_m':fused_errors[-1]})
    coverage=len(errors)/len(dynamic) if dynamic else 0
    accuracy=metrics(errors);consistency=metrics(cross)
    if displacement<=2:failures.append('dynamic_displacement_not_over_2m')
    if len(errors)<100:failures.append('insufficient_dynamic_pairs')
    if coverage<.95:failures.append('dynamic_pair_coverage_below_95_percent')
    if accuracy['max_m'] is None or accuracy['max_m']>.05:failures.append('map_max_error_over_50mm_or_missing')
    if consistency['max_m'] is None or consistency['max_m']>.02:failures.append('fused_vs_tf_chain_inconsistent')
    if data.get('frame_errors'):failures.extend(data['frame_errors'])
    if data.get('tf_future_max_s',0)>.05:failures.append('future_tf_over_50ms')
    if data.get('tf_past_max_s',0)>.10:failures.append('stale_tf_over_100ms')
    for topic in ('/odom/unfiltered','/imu/data','/amcl_pose','/odometry/gps','/gnss/fix'):
        if data.get('counts',{}).get(topic,0)<3:failures.append('missing_diagnostic_input:'+topic)
    return {'status':'FAIL' if failures else 'PASS','scope':'one sealed dynamic simulation run only',
            'failures':failures,'authority':owners,'displacement_m':displacement,'path_m':path,
            'dynamic_reference_samples':len(dynamic),'pair_coverage':coverage,'accuracy':accuracy,
            'fused_accuracy':metrics(fused_errors),'paired_samples':pairs,'scored_pose':'Nav2 map->odom TF composed with odom->base_footprint TF at the GT source epoch','fused_vs_tf_chain':consistency,'tf_future_max_s':data.get('tf_future_max_s'),
            'tf_past_max_s':data.get('tf_past_max_s'),'raw_input_diagnostics':data.get('input_diagnostics',[]),
            'alignment':'public source-world start to public localization-map start; no fitted offset',
            'reference_point_contract':'formal model origin and base_footprint have coincident planar origin; verify if vehicle changes'}


def decode(paths):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    data={n:[] for n in ('gt','fused','odom','tf','local_tf')};data.update(counts={},frame_errors=[],input_diagnostics=[])
    topics={'/ground_truth/model_odom_raw':'gt','/localization/fused_odom':'fused','/odom':'odom'}
    clock=None; future=[];past=[]
    def stamp(s):return s.sec+s.nanosec*1e-9
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    for path in paths:
        with path.open('rb') as stream:
            reader=make_reader(stream,decoder_factories=[DecoderFactory()],validate_crcs=True)
            if reader.get_summary() is None:raise ValueError('unsealed MCAP: no summary')
            for _,ch,record,m in reader.iter_decoded_messages():
                topic=ch.topic;data['counts'][topic]=data['counts'].get(topic,0)+1
                if topic=='/clock':clock=stamp(m.clock)
                if topic in topics:
                    p=m.pose.pose.position;v=m.twist.twist.linear
                    data[topics[topic]].append([stamp(m.header.stamp),p.x,p.y,yaw(m.pose.pose.orientation),v.x,v.y,m.twist.twist.angular.z])
                    if topic=='/localization/fused_odom' and (m.header.frame_id.lstrip('/')!='map' or m.child_frame_id.lstrip('/')!='base_footprint'):
                        data['frame_errors'].append('fused_frame_not_map_base_footprint')
                    if topic=='/odom' and (m.header.frame_id.lstrip('/')!='odom' or m.child_frame_id.lstrip('/')!='base_footprint'):
                        data['frame_errors'].append('local_frame_not_odom_base_footprint')
                    if topic=='/ground_truth/model_odom_raw' and (m.header.frame_id.lstrip('/')!='world' or m.child_frame_id.lstrip('/')!='base_footprint'):
                        data['frame_errors'].append('reference_frame_not_world_base_footprint')
                if topic=='/tf':
                    for tr in m.transforms:
                        if tr.header.frame_id.lstrip('/')=='odom' and tr.child_frame_id.lstrip('/')=='base_footprint':
                            v=tr.transform.translation;data['local_tf'].append([stamp(tr.header.stamp),v.x,v.y,yaw(tr.transform.rotation)])
                        if tr.header.frame_id.lstrip('/')=='map' and tr.child_frame_id.lstrip('/')=='odom':
                            v=tr.transform.translation;t=stamp(tr.header.stamp)
                            data['tf'].append([t,v.x,v.y,yaw(tr.transform.rotation)])
                            if clock is None:data['frame_errors'].append('tf_before_clock')
                            else:future.append(t-clock);past.append(clock-t)
                if topic in ('/imu/data','/odom/unfiltered'):
                    # Diagnostics only: no truth, no rejection from fusion or score.
                    values=([m.linear_acceleration.x,m.linear_acceleration.y,m.angular_velocity.z]
                            if topic=='/imu/data' else [m.twist.twist.linear.x,m.twist.twist.angular.z])
                    if any(not math.isfinite(v) for v in values) or (topic=='/imu/data' and math.hypot(*values[:2])>4.0):
                        data['input_diagnostics'].append({'topic':topic,'stamp':stamp(m.header.stamp),'values':[v if math.isfinite(v) else str(v) for v in values],'reason':'nonfinite_or_planar_acceleration_over_4mps2','action':'diagnostic_only_no_input_or_score_mutation'})
    # Equal repeated TF epochs with equal transforms are harmless. Conflicting
    # values at an epoch stay present so assess rejects them; never choose one.
    for name in ('gt','fused','odom','tf','local_tf'):
        data[name]=[r for i,r in enumerate(data[name]) if i==0 or r!=data[name][i-1]]
    data['tf_future_max_s']=max(future,default=1e9);data['tf_past_max_s']=max(past,default=1e9)
    data['frame_errors']=sorted(set(data['frame_errors']))
    return data


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag-dir',type=Path,required=True)
    p.add_argument('--authority',type=Path,required=True)
    p.add_argument('--effective-parameters',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--revision',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument(
        '--map-odom-owner',
        choices=('/global_ekf','/map_odom_stabilizer'),
        default='/global_ekf',
    )
    a=p.parse_args()
    if a.output.exists():raise SystemExit('fresh output required')
    paths=sorted(a.bag_dir.glob('*.mcap'))
    if not paths:raise SystemExit('no MCAP')
    try:
        source_root=Path(__file__).resolve().parents[1]
        contract=static_authority_contract(
            source_root,a.effective_parameters,a.map_odom_owner
        )
        result=assess(
            decode(paths),
            json.loads(a.authority.read_text()),
            json.loads(a.manifest.read_text()),
            contract,
        )
    except Exception as e:result={'status':'FAIL','failures':[type(e).__name__+': '+str(e)]}
    contract_files=[
        Path(x)
        for x in ((contract if 'contract' in locals() else {}).get('source_files',[]))
    ]
    result.update(
        revision=a.revision,
        run_id=a.bag_dir.parent.name,
        files={
            str(x): hashlib.sha256(x.read_bytes()).hexdigest()
            for x in paths + [a.authority, a.manifest] + contract_files
        },
    )
    if 'contract' in locals():result['authority_contract']=contract
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return 0 if result['status']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
