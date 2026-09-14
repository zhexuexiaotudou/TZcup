"""Bounded local ROS-only recording-arm regression; never starts Gazebo."""
import argparse,ast,hashlib,json,os,pathlib,signal,subprocess,sys,time
from pathlib import Path
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def actual_arm_consumer(observer_path, run_out, env):
    source=Path(observer_path).read_text(encoding='utf-8');tree=ast.parse(source)
    names={'integer','process_starttime_ticks','arm_owner_alive','sha256_regular_file','writer_topics_ready','validate_recording_role_contract','validate_formal_recording_arm','formal_recording_arm_runtime_failure'}
    selected=[]
    for item in tree.body:
        if isinstance(item,ast.FunctionDef) and item.name in names:selected.append(item)
        if isinstance(item,ast.Assign) and any(isinstance(t,ast.Name) and t.id in {'ARM_FRESH_NS','ARM_REQUIRED_TOPICS','LOCALIZATION_ARM_REQUIRED_TOPICS'} for t in item.targets):selected.append(item)
    ns={'pathlib':pathlib,'os':os,'time':time,'json':json,'hashlib':hashlib,'EXPECTED_SESSION':env['FORMAL_ACCEPTANCE_SESSION'],'EXPECTED_RUN_TOKEN':env['FORMAL_RECORDING_RUN_TOKEN']}
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(observer_path),'exec'),ns)
    payload,error=ns['validate_formal_recording_arm'](run_out)
    assert error is None,error
    assert ns['formal_recording_arm_runtime_failure'](payload,run_out) is None
    return {'observer_sha256':hashlib.sha256(source.encode()).hexdigest(),'actual_proc_liveness_checked':True,'accepted':True}


def run_case(root, name, mode, observer_path):
    out=root/name/'map'
    out.mkdir(parents=True)
    env=dict(os.environ,FORMAL_ACCEPTANCE_SESSION=str(out.parent/'formal_acceptance_session.json'),FORMAL_RECORDING_RUN_TOKEN='TEST_ONLY_'+name,FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS=str(time.monotonic_ns()+30_000_000_000))
    (out.parent/'formal_acceptance_session.json').write_text(json.dumps({'test_only':True}),encoding='utf-8')
    helper=Path(__file__).with_name('capture_formal_first_map_early_recording_audit.py')
    workers=[]
    for role in ['localization','early']:
        log=(out/(role+'.log')).open('xb')
        cmd=[sys.executable,str(helper),'--output',str(out),'--timeout','10','--role',role]
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        stat=Path(f'/proc/{proc.pid}/stat').read_text(encoding='utf-8'); fields=stat[stat.rfind(')')+2:].split()
        owner={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text(encoding='utf-8').strip(),'pid':proc.pid,'pgid':int(fields[2]),'session':int(fields[3]),'starttime_ticks':int(fields[19])}
        workers.append((role,proc,log,owner,cmd))
    node=rclpy.create_node('recording_arm_test_'+name)
    pubs={}
    for topic,typ in [('/clock',Clock),('/safety/status_json',String),('/base_controller/cmd_vel',TwistStamped),('/safety/relay_cycle_diagnostic_json',String),('/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json',String),('/odom',Odometry),('/odom/unfiltered',Odometry),('/odometry/gps',Odometry),('/gnss/fix',NavSatFix),('/formal_mapping/lifecycle_status',String)]:
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ['/safety/status_json','/formal_mapping/lifecycle_status'] else 10
        pubs[topic]=(node.create_publisher(typ,topic,qos),typ)
    started=time.monotonic(); tick=0; observed_open=False; observed_arm=None; sent_required=False
    failure=None; consumer_result=None
    try:
        while time.monotonic()-started<15:
            elapsed=time.monotonic()-started
            opened=(out/'formal_recording_writer_open.json').exists() and (out/'formal_localization_writer_open.json').exists()
            if opened and not observed_open:
                observed_open=True;open_at=elapsed
            if opened and mode!='empty':
                tick+=1
                c=Clock();c.clock.sec=1;c.clock.nanosec=tick*1000000;pubs['/clock'][0].publish(c)
                fault=String();safe=mode=='delayed' and elapsed-open_at>2
                fault.data=json.dumps({'state':'ENABLED' if safe else 'INHIBITED','active_reasons':'' if safe else 'TEST_EARLY_FAULT','test_only':True,'status_publish_count':1000+tick,'safety_inputs_permit_actuators':safe,'actuators_enabled':safe,'managed_controllers_active':safe});pubs['/safety/status_json'][0].publish(fault)
                for topic in ['/base_controller/cmd_vel','/safety/relay_cycle_diagnostic_json','/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json']:
                    pub,typ=pubs[topic];msg=typ()
                    if typ is String:msg.data=json.dumps({'test_only':True,'status_publish_count':tick,'cycle_id':tick,'state':'TEST_EARLY_FAULT'})
                    else:msg.header.stamp=c.clock;msg.header.frame_id='base_footprint'
                    pub.publish(msg)
                if elapsed-open_at>2:
                    sent_required=True
                    for topic,(pub,typ) in pubs.items():
                        if topic in ['/clock','/safety/status_json','/base_controller/cmd_vel','/safety/relay_cycle_diagnostic_json','/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json'] or (mode=='missing' and topic=='/formal_mapping/lifecycle_status'):continue
                        msg=typ()
                        if hasattr(msg,'header'):msg.header.stamp=c.clock
                        if typ is String:msg.data=json.dumps({'test_only':True,'state':'mapping'})
                        pub.publish(msg)
            arm_path=out/'formal_recording_arm.json'
            if arm_path.exists():
                if observed_arm is None:
                    consumer_result=actual_arm_consumer(observer_path,out.parent,env)
                observed_arm=json.loads(arm_path.read_text(encoding='utf-8'))
                assert mode=='delayed' and sent_required,'armed before required data'
                assert observed_arm['armed'] and observed_arm['clock_advances']>=2
                assert observed_arm['pre_arm_status_count']==observed_arm['pre_arm_latest_safety']['status_publish_count']
                assert observed_arm['pre_arm_status_count']>observed_arm['pre_arm_status_messages_received']
                assert observed_arm['pre_arm_status_messages_received']==observed_arm['writer_message_counts']['/safety/status_json']
                assert observed_arm['pre_arm_unsafe_count']>0
                assert all(observed_arm['writer_message_counts'].get(t,0)>0 for t in pubs)
                if elapsed>11:
                    assert all(w[1].poll() is None for w in workers),'ready writer exited at admission timeout'
                    latest_ready=json.loads((out/'formal_recording_ready.json').read_text(encoding='utf-8'))
                    assert latest_ready['pre_arm_unsafe_count']==observed_arm['pre_arm_unsafe_count'],'normal ENABLED counted as unsafe'
                    break
            if any(w[1].poll() is not None for w in workers):break
            rclpy.spin_once(node,timeout_sec=.03)
            time.sleep(.02)
        if mode=='delayed':
            assert observed_open and observed_arm,'ready never armed'
            assert time.monotonic()-started>11,'ready writer exited before post-admission liveness check'
        else:assert observed_open and observed_arm is None,'missing/empty unexpectedly armed'
    except BaseException as exc:
        failure=repr(exc)
    finally:
        terminal=[]
        for role,proc,log,owner,cmd in reversed(workers):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:rc=proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3);raise AssertionError('helper did not close on SIGINT')
            log.close()
            survivors=[]
            for entry in Path('/proc').iterdir():
                if not entry.name.isdigit():continue
                try:
                    txt=(entry/'stat').read_text(encoding='utf-8'); f=txt[txt.rfind(')')+2:].split()
                    if int(f[2])==owner['pgid'] or int(f[3])==owner['session']:survivors.append(int(entry.name))
                except (FileNotFoundError,ProcessLookupError):pass
            terminal.append({'role':role,'initial':owner,'argv':cmd,'rc':rc,'survivors':survivors})
            assert not survivors,survivors
        node.destroy_node()
        (out/'owner-terminal.json').write_text(json.dumps(terminal,indent=2),encoding='utf-8')
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(out/'early_recording_audit'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    counts={};fault_count=0
    while reader.has_next():
        topic,data,stamp=reader.read_next();counts[topic]=counts.get(topic,0)+1
        if topic=='/safety/status_json' and 'TEST_EARLY_FAULT' in deserialize_message(data,String).data:fault_count+=1
    del reader
    main_counts={}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(out/'mapping_localization_diagnostic'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    while reader.has_next():
        topic,data,stamp=reader.read_next();main_counts[topic]=main_counts.get(topic,0)+1
        typ=String if topic=='/formal_mapping/lifecycle_status' else (NavSatFix if topic=='/gnss/fix' else Odometry)
        deserialize_message(data,typ)
    del reader
    required=['/odom','/odom/unfiltered','/odometry/gps','/gnss/fix','/formal_mapping/lifecycle_status']
    if mode=='delayed':assert all(main_counts.get(t,0)>0 for t in required),'main bag not actually written'
    if mode=='empty':assert not main_counts and not counts
    if mode=='missing':assert main_counts.get('/formal_mapping/lifecycle_status',0)==0
    if mode!='empty':assert fault_count>0,'early fault absent from actual bag'
    if mode=='delayed':assert all(w['rc']==0 for w in terminal)
    else:assert all(w['rc']!=0 for w in terminal) and not (out/'formal_recording_arm.json').exists()
    if failure:raise AssertionError(failure)
    return {'case':name,'mode':mode,'workers':terminal,'arm_consumer_result':consumer_result,'writer_open_observed':observed_open,'armed':observed_arm is not None,'actual_early_mcap_message_counts':counts,'actual_main_mcap_message_counts':main_counts,'actual_early_fault_messages':fault_count,'passed':True}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--observer',required=True);p.add_argument('--case',choices=['delayed','missing','empty']);args=p.parse_args();root=Path(args.output).resolve();root.mkdir(exist_ok=False)
    assert os.environ.get('ROS_DOMAIN_ID')=='171','run only in isolated local test domain 171'
    rclpy.init();results=[]
    try:
        for name,mode in [('delayed_required','delayed'),('missing_required','missing'),('no_messages','empty')]:
            if args.case is None or mode==args.case:results.append(run_case(root,name,mode,args.observer))
    finally:
        rclpy.shutdown();(root/'results.json').write_text(json.dumps({'test_only':True,'gazebo_started':False,'cases':results,'all_selected_passed':len(results)==(1 if args.case else 3),'all_three_passed':len(results)==3},indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results))

if __name__=='__main__':main()