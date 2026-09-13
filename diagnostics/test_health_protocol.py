import copy,hashlib,json,pathlib,sys,tempfile,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'starter_ws/src/sanitation_formal_campus_integration'))
from sanitation_formal_campus_integration.lifecycle_health_protocol import validate_health,digest
class HealthTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=pathlib.Path(self.temp.name);self.proc=self.root/'proc';(self.proc/'sys/kernel/random').mkdir(parents=True);(self.proc/'sys/kernel/random/boot_id').write_text('boot');(self.proc/'123').mkdir()
  fields=['0']*20;fields[0]='S';fields[19]='100';(self.proc/'123/stat').write_text('123 (producer) '+' '.join(fields))
  self.session=self.root/'session.json';self.session.write_text('{}');self.sha=hashlib.sha256(self.session.read_bytes()).hexdigest();(self.proc/'123/environ').write_bytes(('FORMAL_ACCEPTANCE_SESSION='+str(self.session)+'\0FORMAL_RECORDING_RUN_TOKEN=token\0').encode())
  (self.proc/'123/cmdline').write_bytes(b'/usr/bin/python3\0-c\0from sanitation_formal_campus_integration.map_lifecycle_manager import main; main()\0')
  self.now=10_000_000_000;business={'status':'exploring_until_observed_fraction_gate','ready':False};self.packet=dict(business,lifecycle_health={'version':1,'session':str(self.session),'session_sha256':self.sha,'run_token':'token','producer_identity':{'boot_id':'boot','pid':123,'process_starttime_ticks':100},'health_seq':1,'health_monotonic_ns':self.now,'eval_seq':1,'eval_monotonic_ns':self.now,'business_sha256':digest(business),'validity_kind':'evaluation_lease','business_valid_until_monotonic_ns':self.now+15_000_000_000})
 def check(self,p=None,prior=None,now=None):
  packet=p or self.packet;raw=json.dumps(packet,sort_keys=True);return validate_health(packet,prior,self.now if now is None else now,str(self.session),self.sha,'token',self.proc,hashlib.sha256(raw.encode()).hexdigest(),raw)
 def test_sparse_health_does_not_renew_evaluation(self):
  state,error=self.check();self.assertIsNone(error);p=copy.deepcopy(self.packet);p['lifecycle_health'].update(health_seq=2,health_monotonic_ns=self.now+5_000_000_000);new,error=self.check(p,state,self.now+5_000_000_000);self.assertIsNone(error);self.assertEqual(new['eval_monotonic_ns'],state['eval_monotonic_ns'])
 def test_duplicate_expires_without_renewal(self):
  state,error=self.check();self.assertIsNone(error);self.assertIsNone(self.check(prior=state,now=self.now+1_000_000_000)[1]);self.assertEqual(self.check(prior=state,now=self.now+2_000_000_001)[1],'health_expired')
 def test_changed_same_health_sequence(self):
  state,_=self.check();p=copy.deepcopy(self.packet);p['lifecycle_health']['health_monotonic_ns']+=1;self.assertEqual(self.check(p,state,self.now+1)[1],'same_health_sequence_changed_bytes')
 def test_business_lease_not_renewable_by_heartbeat(self):
  state,_=self.check();p=copy.deepcopy(self.packet);p['lifecycle_health'].update(health_seq=2,health_monotonic_ns=self.now+1,business_valid_until_monotonic_ns=self.now+14_000_000_000);self.assertEqual(self.check(p,state,self.now+1)[1],'same_evaluation_sequence_changed_business')
 def test_wrong_session(self):
  self.packet['lifecycle_health']['run_token']='other';self.assertEqual(self.check()[1],'session_mismatch')
 def test_producer_dead(self):
  (self.proc/'123/stat').unlink();self.assertEqual(self.check()[1],'producer_dead_or_identity_mismatch')
 def test_producer_restart(self):
  (self.proc/'123/stat').write_text('123 (producer) '+' '.join(['S']+['0']*18+['101']));self.assertEqual(self.check()[1],'producer_dead_or_identity_mismatch')
 def test_wrong_role_same_environment(self):
  (self.proc/'123/cmdline').write_bytes(b'/usr/bin/python3\0recorder.py\0');self.assertEqual(self.check()[1],'producer_dead_or_identity_mismatch')
 def test_future_health(self):
  self.packet['lifecycle_health']['health_monotonic_ns']+=1;self.assertEqual(self.check()[1],'future_time')
 def test_expired_business(self):
  self.packet['lifecycle_health'].update(health_monotonic_ns=self.now+16_000_000_000);self.assertEqual(self.check(now=self.now+16_000_000_000)[1],'business_expired_or_invalid')
 def test_terminal_not_implicitly_valid(self):
  self.packet['lifecycle_health']['validity_kind']='sealed_artifact';self.assertEqual(self.check()[1],'terminal_ready_mismatch')
 def test_same_sequence_wire_bytes_change(self):
  state,_=self.check();raw=json.dumps(self.packet,indent=4)
  _,error=validate_health(self.packet,state,self.now,str(self.session),self.sha,'token',self.proc,hashlib.sha256(raw.encode()).hexdigest(),raw);self.assertEqual(error,'same_health_sequence_changed_bytes')
 def test_health_and_eval_rollback(self):
  state,_=self.check();state=dict(state,health_seq=2);self.assertEqual(self.check(prior=state)[1],'sequence_or_time_rollback')
 def test_unknown_or_failed_state(self):
  for status in ['anything','map_save_or_integrity_gate_failed']:
   p=copy.deepcopy(self.packet);p['status']=status;p['lifecycle_health']['business_sha256']=digest({k:v for k,v in p.items() if k!='lifecycle_health'});self.assertEqual(self.check(p)[1],'nonterminal_ready_mismatch')
 def test_wrong_boot_and_zombie(self):
  (self.proc/'sys/kernel/random/boot_id').write_text('newboot');self.assertEqual(self.check()[1],'producer_dead_or_identity_mismatch');(self.proc/'sys/kernel/random/boot_id').write_text('boot')
  path=self.proc/'123/stat';path.write_text(path.read_text().replace(') S ',') Z '));self.assertEqual(self.check()[1],'producer_dead_or_identity_mismatch')
 def test_terminal_existing_artifact_validator(self):
  import ast
  from unittest.mock import patch
  sys.path.insert(0,str(pathlib.Path('/mnt/f/Project/TZcup/.workspace/runtime-dependencies/first-map-mcap-analysis-01')))
  from sanitation_formal_campus_integration import map_lifecycle_core as core
  import yaml
  source=pathlib.Path(__file__).resolve().parents[1]/'starter_ws/src/sanitation_formal_campus_integration/test/test_map_lifecycle_core.py'
  func=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_materialized_saved_map')
  ns=dict(vars(core),Path=pathlib.Path,json=json,hashlib=hashlib,yaml=yaml);exec(compile(ast.Module(body=[func],type_ignores=[]),str(source),'exec'),ns)
  root=self.root/'map';contract,_=ns['_materialized_saved_map'](root);episode=self.root/'episode/public/episode_manifest.json';episode.parent.mkdir(parents=True);episode.write_text('{}')
  p=copy.deepcopy(self.packet);p.update(status='ready_for_localization_cleaning',ready=True,map_id=contract.map_id);h=p['lifecycle_health'];h.update(validity_kind='sealed_artifact',business_sha256=digest({k:v for k,v in p.items() if k!='lifecycle_health'}),sealed_artifact={'artifact_root':str(root),'episode_manifest':str(episode),'episode_sha256':hashlib.sha256(episode.read_bytes()).hexdigest(),'manifest_sha256':hashlib.sha256((root/'map_lifecycle_manifest.json').read_bytes()).hexdigest()})
  # A small pre-existing core fixture supplies the episode contract; the actual
  # original artifact validator runs unchanged against every sealed fixture file.
  with patch.object(core,'load_campus_map_contract',return_value=contract):
   self.assertIsNone(self.check(p)[1]);(root/'occupancy.pgm').write_bytes(b'corrupted');self.assertIsNotNone(self.check(p)[1])
 def test_manager_no_new_map_does_not_renew_business(self):
  import ast,time
  from types import SimpleNamespace
  from unittest.mock import patch
  from sanitation_formal_campus_integration.lifecycle_health_protocol import HealthProducer
  with patch('sanitation_formal_campus_integration.lifecycle_health_protocol.process_identity',return_value={'boot_id':'boot','pid':123,'process_starttime_ticks':100}):
   producer=HealthProducer(str(self.session),'token')
  producer.evaluate({'status':'exploring_until_observed_fraction_gate','ready':False});before=dict(producer.evaluation);seq=producer.eval_seq
  path=pathlib.Path(__file__).resolve().parents[1]/'starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/map_lifecycle_manager.py';tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FormalMapLifecycleManager');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_evaluate');ns={'time':time};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
  fake=SimpleNamespace(_finished=False,_saving=False,_latest_map=object(),_map_stamp_ns=1,_consumed_map_stamp_ns=1,_inputs_fresh=lambda now:True,get_parameter=lambda name:SimpleNamespace(value=0.95 if name=='observation_threshold' else 3),_publish=lambda *a:producer.evaluate({'status':'exploring_until_observed_fraction_gate','ready':False}))
  ns['_evaluate'](fake)
  for i in range(4):producer.heartbeat()
  self.assertEqual(producer.evaluation,before);self.assertEqual(producer.eval_seq,seq)
  time.sleep(.001);producer.evaluate({'status':'exploring_until_observed_fraction_gate','ready':False});self.assertEqual(producer.eval_seq,seq+1);self.assertGreater(producer.evaluation['eval_monotonic_ns'],before['eval_monotonic_ns'])
 def test_same_eval_changed_validity_kind(self):
  state,_=self.check();p=copy.deepcopy(self.packet);p['lifecycle_health'].update(health_seq=2,health_monotonic_ns=self.now+1,validity_kind='failed');self.assertIsNotNone(self.check(p,state,self.now+1)[1])
if __name__=='__main__':unittest.main()
