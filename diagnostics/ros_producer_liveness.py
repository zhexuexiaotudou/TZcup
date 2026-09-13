import os,sys,json,time,signal,subprocess,pathlib,hashlib,shutil
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from sanitation_formal_campus_integration.lifecycle_health_protocol import validate_health,process_identity
root=pathlib.Path(sys.argv[1]);root.mkdir();(root/'map').mkdir();(root/'episode/public').mkdir(parents=True)
source=pathlib.Path('/mnt/f/Project/TZcup/.workspace/evidence/short08-b430dda-failure-closed-01/original/frontier-short-readiness-diagnostic-08')
for name in ['geofence_keepout.yaml','geofence_keepout.pgm','neutral_speed.yaml','neutral_speed.pgm','mission_geometry.yaml','materialization_contract.yaml']:
 shutil.copyfile(source/'map'/name,root/'map'/name)
shutil.copyfile(source/'episode/public/episode_manifest.json',root/'episode/public/episode_manifest.json')
session=root/'formal_acceptance_session.json';session.write_text('{"test_only":true}');session_sha=hashlib.sha256(session.read_bytes()).hexdigest();env=dict(os.environ,FORMAL_ACCEPTANCE_SESSION=str(session),FORMAL_RECORDING_RUN_TOKEN='short09_ros_test')
workers=[];messages=[];receipts=[];failure=None
rclpy.init();node=rclpy.create_node('short09_health_reader_test');qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);subscription=node.create_subscription(String,'/formal_mapping/lifecycle_status',lambda m:messages.append((m.data,time.monotonic_ns())),qos)
def launch(label):
 command=[sys.executable,'-c','from sanitation_formal_campus_integration.map_lifecycle_manager import main; main()', '--ros-args','-p','episode_manifest:='+str(root/'episode/public/episode_manifest.json'),'-p','artifact_directory:='+str(root/'map'),'-p','support_artifacts_prepared:=true']
 log=(root/(label+'.log')).open('xb');p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);workers.append((p,log));identity=process_identity(p.pid);receipts.append({'label':label,'argv':command,'initial_identity':identity,'pgid':os.getpgid(p.pid),'session':os.getsid(p.pid)});return p
try:
 first=launch('producer01');deadline=time.monotonic()+12;state=None;packet=None;wire=None
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if messages:
   wire,_=messages.pop(0);packet=json.loads(wire);state,error=validate_health(packet,state,time.monotonic_ns(),str(session),session_sha,'short09_ros_test',wire_sha256=hashlib.sha256(wire.encode()).hexdigest(),wire_text=wire)
   assert error is None,error
   if packet['lifecycle_health']['health_seq']>=4:break
 assert state is not None and first.poll() is None
 os.kill(first.pid,signal.SIGSTOP);time.sleep(2.2)
 _,error=validate_health(packet,state,time.monotonic_ns(),str(session),session_sha,'short09_ros_test',wire_sha256=hashlib.sha256(wire.encode()).hexdigest(),wire_text=wire);assert error=='health_expired',error
 receipts.append({'executor_stopped_health_expired':True})
 os.kill(first.pid,signal.SIGCONT);first.terminate();first.wait(timeout=8)
 _,error=validate_health(packet,state,time.monotonic_ns(),str(session),session_sha,'short09_ros_test',wire_sha256=hashlib.sha256(wire.encode()).hexdigest(),wire_text=wire);assert error=='producer_dead_or_identity_mismatch',error
 receipts.append({'producer_death_rejected':True});messages.clear();second=launch('producer02');deadline=time.monotonic()+12;rejected=False
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if messages:
   raw,_=messages.pop(0);value=json.loads(raw)
   if value['lifecycle_health']['producer_identity']['pid']!=second.pid:continue
   _,error=validate_health(value,state,time.monotonic_ns(),str(session),session_sha,'short09_ros_test',wire_sha256=hashlib.sha256(raw.encode()).hexdigest(),wire_text=raw);assert error=='producer_restart',error;rejected=True;break
 assert rejected;receipts.append({'producer_restart_rejected':True})
except BaseException as exc:
 import traceback
 failure=traceback.format_exc()
finally:
 for p,log in workers:
  if p.poll() is None:
   os.kill(p.pid,signal.SIGCONT);p.terminate()
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();failure=failure or 'forced_kill'
  log.close();receipts.append({'terminal_pid':p.pid,'rc':p.returncode,'proc_absent':not pathlib.Path('/proc',str(p.pid)).exists()})
 node.destroy_node();rclpy.shutdown()
 report={'test_only':True,'actual_product_manager':True,'passed':failure is None,'failure':failure,'receipts':receipts,'formal_acceptance':False};(root/'results.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));sys.exit(0 if failure is None else 1)
