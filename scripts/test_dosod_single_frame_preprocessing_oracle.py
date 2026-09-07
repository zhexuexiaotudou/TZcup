from __future__ import annotations
import hashlib, importlib.util, json, sys
from pathlib import Path
import pytest
HERE=Path(__file__).parent
def mod(n):
 s=importlib.util.spec_from_file_location(n,HERE/f'{n}.py'); m=importlib.util.module_from_spec(s); assert s.loader; sys.modules[n]=m; s.loader.exec_module(m); return m
validator=mod('validate_dosod_single_frame_preprocessing_oracle')
candidate=mod('execute_dosod_nonformal_oracle_candidate_compile')
parity=mod('run_dosod_hbm_x86_parity')
metric=mod('validate_dosod_quantized_metric_regression')
capture=mod('capture_dosod_official_preprocess')
predeploy=mod('validate_s100p_final_predeploy')


def _sha(path: Path) -> str:
 return hashlib.sha256(path.read_bytes()).hexdigest()
def test_handwritten_capture_never_validates(tmp_path):
 p=tmp_path/'capture.json'; p.write_text(json.dumps({}))
 with pytest.raises(ValueError): validator._capture(p)
def test_fixture_oracle_is_not_verified(tmp_path):
 p=tmp_path/'oracle.json'; p.write_text(json.dumps({'receipt_id':validator.RECEIPT_ID,'status':'TEST_FIXTURE_BLOCKED'}))
 with pytest.raises(ValueError,match='not_verified'): validator.validate(p)
def test_capture_producer_is_explicit_ci_surface():
 assert (HERE/'capture_dosod_official_preprocess.py').is_file()


@pytest.mark.parametrize('fault', ['argv','rc','yuv','missing_log','missing_dpkg'])
def test_capture_refuses_incomplete_or_unbound_official_evidence(tmp_path,monkeypatch,fault):
 monkeypatch.setattr(capture,'_pilot_binding',lambda *_:{'path':str(tmp_path/'raw'),'sha256':'a'*64,'byte_size':1221120,'width':848,'height':480,'step':2544,'encoding':'rgb8','frame_id':'camera','stamp_ns':1})
 raw=tmp_path/'raw'; raw.write_bytes(b'r'*1221120)
 paths={name:tmp_path/name for name in ('y','uv','bin','source','dpkg','stdout','stderr')}
 paths['y'].write_bytes(b'y'*409600); paths['uv'].write_bytes(b'u'*204800)
 for name in ('bin','source','dpkg','stdout','stderr'): paths[name].write_text(name)
 if fault=='yuv': paths['y'].write_bytes(b'y')
 if fault=='missing_log': paths['stdout'].unlink()
 if fault=='missing_dpkg': paths['dpkg'].unlink()
 command=[str(paths['bin'].resolve())] if fault!='argv' else ['wrong']
 rc=1 if fault=='rc' else 0
 receipt=capture.capture(pilot_manifest=tmp_path/'pilot',pilot_record_index=0,images_y=paths['y'],images_uv=paths['uv'],adapter_binary=paths['bin'],adapter_source=paths['source'],dpkg_output=paths['dpkg'],stdout=paths['stdout'],stderr=paths['stderr'],command=command,returncode=rc,output=tmp_path/'out',test_fixture=False)
 assert receipt['status']=='BLOCKED'


def test_capture_fixture_is_never_verified(tmp_path):
 receipt=capture.capture(pilot_manifest=tmp_path/'missing',pilot_record_index=0,images_y=tmp_path/'y',images_uv=tmp_path/'uv',adapter_binary=tmp_path/'bin',adapter_source=tmp_path/'source',dpkg_output=tmp_path/'dpkg',stdout=tmp_path/'out',stderr=tmp_path/'err',command=[],returncode=0,output=tmp_path/'capture',test_fixture=True)
 assert receipt['status']=='TEST_FIXTURE_BLOCKED'


def test_capture_rejects_ancestor_or_leaf_symlink(tmp_path):
 target=tmp_path/'target'; target.write_text('x'); link=tmp_path/'link'
 try: link.symlink_to(target)
 except OSError: pytest.skip('symlink creation unavailable on this host')
 with pytest.raises(ValueError,match='symlink_forbidden'):
  capture.b(link)


@pytest.mark.parametrize('mutation', ['package','version','role','binary_sha','source_sha','dpkg_rc'])
def test_official_identity_helper_requires_every_frozen_field(tmp_path,monkeypatch,mutation):
 root=tmp_path/'root'; (root/'config').mkdir(parents=True); binary=tmp_path/'bin'; source=tmp_path/'source'; dpkg=tmp_path/'dpkg'
 binary.write_text('bin'); source.write_text('source'); dpkg.write_text('pkg\t1.0\t/usr/bin/official')
 identity={'status':'VERIFIED','binary_path':str(binary.resolve()),'binary_sha256':_sha(binary),'source_path':str(source.resolve()),'source_sha256':_sha(source),'source_revision':'abc123','dpkg_package':'pkg','dpkg_version':'1.0','dpkg_path_role':'/usr/bin/official'}
 if mutation=='package': identity['dpkg_package']='other'
 if mutation=='version': identity['dpkg_version']='2.0'
 if mutation=='role': identity['dpkg_path_role']='other'
 if mutation=='binary_sha': identity['binary_sha256']='0'*64
 if mutation=='source_sha': identity['source_sha256']='0'*64
 (root/'config'/'dosod_single_frame_preprocessing_oracle_contract.json').write_text(json.dumps({'official_preprocess_identity':identity}))
 monkeypatch.setattr(capture,'ROOT',root)
 with pytest.raises(ValueError): capture._official_identity(binary,source,dpkg,1 if mutation=='dpkg_rc' else 0)


def test_official_identity_helper_schema_only_positive(tmp_path,monkeypatch):
 root=tmp_path/'root'; (root/'config').mkdir(parents=True); binary=tmp_path/'bin'; source=tmp_path/'source'; dpkg=tmp_path/'dpkg'
 binary.write_text('bin'); source.write_text('source'); dpkg.write_text('pkg\t1.0\t/usr/bin/official')
 identity={'status':'VERIFIED','binary_path':str(binary.resolve()),'binary_sha256':_sha(binary),'source_path':str(source.resolve()),'source_sha256':_sha(source),'source_revision':'abc123','dpkg_package':'pkg','dpkg_version':'1.0','dpkg_path_role':'/usr/bin/official'}
 (root/'config'/'dosod_single_frame_preprocessing_oracle_contract.json').write_text(json.dumps({'official_preprocess_identity':identity})); monkeypatch.setattr(capture,'ROOT',root)
 assert capture._official_identity(binary,source,dpkg,0)['package']=='pkg'


def test_candidate_manifest_rejects_handwritten_or_drifted_25_set(tmp_path):
 pilot=tmp_path/'pilot.json'; pilot.write_text(json.dumps({'records':[{'source_sha256':str(i)} for i in range(25)]}))
 with pytest.raises(ValueError):
  candidate._candidate_calibration(pilot,{'candidate_routes':[{'route_id':'BOOTSTRAP_SYMMETRIC_BLACK_V1'}]})


def test_candidate_success_stays_nonformal_and_uses_exact_recipe(tmp_path,monkeypatch):
 import yaml
 model=tmp_path/'model.onnx'; vocab=tmp_path/'vocab.json'; compiler=tmp_path/'hb_compile'; identity=tmp_path/'identity.json'; pilot=tmp_path/'pilot.json'
 for p,data in ((model,b'model'),(vocab,b'vocab'),(compiler,b'compiler'),(pilot,b'{}')): p.write_bytes(data)
 recipe={'march':'nash-m','input_name':'images','input_type_train':'rgb','input_layout_train':'NCHW','input_shape':'1x3x640x640','input_batch':1,'norm_type':'data_scale','scale_value':0.003921568627451,'input_layout_rt':'NHWC','input_type_rt':'nv12','cal_data_type':'float32','preprocess_on':False,'calibration_type':'max','max_percentile':0.99995,'optimization':'set_all_nodes_int16;','compile_mode':'latency','optimize_level':'O2','jobs_default':1,'output_model_file_prefix':'dosod_mlp3x_s_tzcup_rep-int16'}
 contract={'model':{'sha256':_sha(model)},'vocabulary':{'sha256':_sha(vocab)},'toolchain':{'required_versions':{'x':'1'}},'compile_recipe':recipe}
 identity.write_text('{}'); output=tmp_path/'out'; work=output/'candidate_work'; config=tmp_path/'config.yaml'; config.write_text(yaml.safe_dump(candidate._expected_compile_config(model=model,work=work,calibration=pilot.parent/'samples',recipe=recipe)))
 def fake_load(path):
  if path==candidate.CANONICAL_ORACLE_CONTRACT: return {'candidate_routes':[{'route_id':'BOOTSTRAP_SYMMETRIC_BLACK_V1'}]}
  if path==candidate.CANONICAL_COMPILE_CONTRACT: return contract
  if path==identity: return {'identity_verified':True,'hb_compile_probe_returncode':0,'required_versions':{'x':'1'},'hb_compile_executable':str(compiler.resolve()),'hb_compile_executable_sha256':_sha(compiler)}
  if path==pilot: return {'records':[{} for _ in range(25)]}
  return json.loads(path.read_text())
 monkeypatch.setattr(candidate,'load_object',fake_load); monkeypatch.setattr(candidate,'_pilot_binding',lambda *_:{'sha256':'a'*64}); monkeypatch.setattr(candidate,'_candidate_calibration',lambda *_:{'candidate_route':'BOOTSTRAP_SYMMETRIC_BLACK_V1','records_sha256':'b'*64}); monkeypatch.setattr(candidate.shutil,'which',lambda _:str(compiler)); monkeypatch.setattr(candidate,'validate_candidate_receipt',lambda *_ ,**__: {})
 def fake_run(command,timeout_seconds):
  expected=work/'dosod_mlp3x_s_tzcup_rep-int16.hbm'; expected.parent.mkdir(); expected.write_bytes(b'hbm'); return 0,'ok','',{'timed_out':False,'zero_survivor':True}
 monkeypatch.setattr(candidate,'run_owned_process',fake_run)
 receipt=candidate.execute(pilot_manifest=pilot,pilot_record_index=0,compiler_identity=identity,compile_config=config,model=model,vocabulary=vocab,output=output,compiler='hb_compile')
 assert receipt['status']==candidate.STATUS and receipt['formal_compile'] is False and receipt['board_acceptance'] is False


def test_real_candidate_cli_stays_blocked_without_canonical_pilot_closure(tmp_path,monkeypatch):
 paths=[tmp_path/name for name in ('pilot.json','identity.json','config.yaml','model.onnx','vocab.json')]
 for path in paths: path.write_text('{}')
 monkeypatch.setattr(candidate,'_pilot_binding',lambda *_:{'sha256':'a'*64})
 receipt=candidate.execute(pilot_manifest=paths[0],pilot_record_index=0,compiler_identity=paths[1],compile_config=paths[2],model=paths[3],vocabulary=paths[4],output=tmp_path/'out')
 assert receipt['status']=='BLOCKED' and any('candidate_calibration_pilot_closure_invalid' in item for item in receipt['blockers'])


def _valid_candidate_receipt(tmp_path,monkeypatch):
 import yaml
 output=tmp_path/'candidate-output'; output.mkdir(); model=tmp_path/'model.onnx'; vocabulary=tmp_path/'vocab.json'; compiler=tmp_path/'hb_compile'; identity=tmp_path/'identity.json'; pilot=tmp_path/'pilot_manifest.json'; raw=tmp_path/'raw.rgb'; logs=[output/'hb_compile.stdout.txt',output/'hb_compile.stderr.txt']
 for path,data in ((model,b'model'),(vocabulary,b'vocab'),(compiler,b'compiler'),(raw,b'r'*1221120)): path.write_bytes(data)
 pilot.write_text(json.dumps({'record_sha256':'a'*64})); (pilot.parent/'samples').mkdir(exist_ok=True)
 recipe={'march':'nash-m','input_name':'images','input_type_train':'rgb','input_layout_train':'NCHW','input_shape':'1x3x640x640','input_batch':1,'norm_type':'data_scale','scale_value':0.003921568627451,'input_layout_rt':'NHWC','input_type_rt':'nv12','cal_data_type':'float32','preprocess_on':False,'calibration_type':'max','max_percentile':0.99995,'optimization':'set_all_nodes_int16;','compile_mode':'latency','optimize_level':'O2','jobs_default':1,'output_model_file_prefix':'dosod_mlp3x_s_tzcup_rep-int16'}
 compile_contract=tmp_path/'compile-contract.json'; oracle_contract=tmp_path/'oracle-contract.json'; compile_contract.write_text(json.dumps({'model':{'sha256':_sha(model)},'vocabulary':{'sha256':_sha(vocabulary)},'toolchain':{'required_versions':{'x':'1'}},'compile_recipe':recipe})); oracle_contract.write_text('{}')
 identity.write_text(json.dumps({'identity_verified':True,'hb_compile_probe_returncode':0,'required_versions':{'x':'1'},'hb_compile_executable':str(compiler.resolve()),'hb_compile_executable_sha256':_sha(compiler)}))
 config=tmp_path/'config.yaml'; config.write_text(yaml.safe_dump(candidate._expected_compile_config(model=model,work=output/'candidate_work',calibration=pilot.parent/'samples',recipe=recipe)))
 for path,data in zip(logs,('stdout','stderr')): path.write_text(data)
 hbm=output/'candidate_work'/'dosod_mlp3x_s_tzcup_rep-int16.hbm'; hbm.parent.mkdir(); hbm.write_bytes(b'hbm')
 producer=tmp_path/'public_gazebo_dosod_calibration.py'; producer.write_text('producer')
 raw_binding={'path':str(raw.resolve()),'sha256':_sha(raw),'byte_size':raw.stat().st_size,'width':848,'height':480,'step':2544,'encoding':'rgb8','frame_id':'camera','stamp_ns':1,'pilot_manifest_path':str(pilot.resolve()),'pilot_manifest_sha256':_sha(pilot),'pilot_record_sha256':'b'*64,'pilot_record_index':0}
 monkeypatch.setattr(candidate,'CANONICAL_COMPILE_CONTRACT',compile_contract); monkeypatch.setattr(candidate,'CANONICAL_ORACLE_CONTRACT',oracle_contract); monkeypatch.setattr(candidate,'PILOT_PRODUCER',producer); monkeypatch.setattr(candidate,'_pilot_binding',lambda *_:raw_binding); monkeypatch.setattr(candidate,'_candidate_calibration',lambda *_:{'candidate_route':'BOOTSTRAP_SYMMETRIC_BLACK_V1','records_sha256':'a'*64})
 receipt=output/'dosod_nonformal_oracle_candidate_compile_receipt.json'
 value={'schema_version':1,'receipt_id':candidate.RECEIPT_ID,'status':candidate.STATUS,'formal_compile':False,'board_acceptance':False,'receipt_path':str(receipt.resolve()),'producer_script_path':str(Path(candidate.__file__).resolve()),'producer_script_sha256':_sha(Path(candidate.__file__)),'blockers':[],'returncode':0,'canonical_compile_contract_sha256':_sha(compile_contract),'canonical_oracle_contract_sha256':_sha(oracle_contract),'pilot_producer_script_path':str(producer.resolve()),'pilot_producer_script_sha256':_sha(producer),'pilot_raw':raw_binding,'pilot_manifest_sha256':_sha(pilot),'pilot_record_count':25,'pilot_records_sha256':'a'*64,'candidate_calibration_records_sha256':'a'*64,'candidate_route':'BOOTSTRAP_SYMMETRIC_BLACK_V1','model_path':str(model.resolve()),'model_sha256':_sha(model),'vocabulary_path':str(vocabulary.resolve()),'vocabulary_sha256':_sha(vocabulary),'compiler_identity_path':str(identity.resolve()),'compiler_identity_sha256':_sha(identity),'compile_config_path':str(config.resolve()),'compile_config_sha256':_sha(config),'expected_hbm_path':str(hbm.resolve()),'command':[str(compiler.resolve()),'-c',str(config.resolve())],'execution':{'deadline_seconds':3600,'term_grace_seconds':10,'timed_out':False,'zero_survivor':True},'raw_stdout_path':logs[0].name,'raw_stdout_sha256':_sha(logs[0]),'raw_stderr_path':logs[1].name,'raw_stderr_sha256':_sha(logs[1]),'candidate_hbm':{'path':str(hbm.resolve()),'sha256':_sha(hbm),'byte_size':hbm.stat().st_size}}
 receipt.write_text(json.dumps(value)); return receipt,value,{'pilot':pilot,'config':config,'identity':identity,'stdout':logs[0],'hbm':hbm}


def test_candidate_validator_accepts_complete_reauditable_baseline(tmp_path,monkeypatch):
 receipt,_,_=_valid_candidate_receipt(tmp_path,monkeypatch)
 assert candidate.validate_candidate_receipt(receipt)['status']==candidate.STATUS


@pytest.mark.parametrize('mutation', ['producer','pilot','route','config','identity','compiler_path','logs','execution','hbm'])
def test_candidate_validator_rejects_each_mutated_binding(tmp_path,monkeypatch,mutation):
 receipt,value,paths=_valid_candidate_receipt(tmp_path,monkeypatch)
 if mutation=='producer': value['producer_script_sha256']='0'*64
 elif mutation=='pilot': value['pilot_records_sha256']='0'*64
 elif mutation=='route': value['candidate_route']='not-the-bootstrap-route'
 elif mutation=='config':
  paths['config'].write_text('model_parameters: {}'); value['compile_config_sha256']=_sha(paths['config'])
 elif mutation=='identity':
  paths['identity'].write_text(json.dumps({'identity_verified':False})); value['compiler_identity_sha256']=_sha(paths['identity'])
 elif mutation=='compiler_path':
  current=json.loads(paths['identity'].read_text()); current['hb_compile_executable']=str(tmp_path/'same-bytes-copy'); paths['identity'].write_text(json.dumps(current)); value['compiler_identity_sha256']=_sha(paths['identity'])
 elif mutation=='logs': paths['stdout'].write_text('tampered')
 elif mutation=='execution': value['execution']['zero_survivor']=False
 elif mutation=='hbm': paths['hbm'].write_bytes(b'tampered')
 receipt.write_text(json.dumps(value))
 with pytest.raises(ValueError): candidate.validate_candidate_receipt(receipt)


@pytest.mark.parametrize('consumer', [parity._validate_compile_receipt, lambda path,hbm: metric._validate_compile_receipt(path,hbm=hbm,calibration_manifest=hbm)])
def test_parity_and_metric_reject_candidate_receipt(tmp_path,consumer):
 hbm=tmp_path/'candidate.hbm'; hbm.write_bytes(b'hbm'); receipt=tmp_path/'candidate.json'; receipt.write_text(json.dumps({'receipt_id':candidate.RECEIPT_ID,'status':candidate.STATUS}))
 with pytest.raises(ValueError): consumer(receipt,hbm)


def test_s100_admission_rejects_candidate_receipt():
 blockers=[]
 assert not predeploy._validate_offline_compile_receipt({'receipt_id':candidate.RECEIPT_ID,'status':candidate.STATUS},hbm_contract_path=HERE.parent/'config'/'dosod_s100p_hbm_compile_contract.json',blockers=blockers)
 assert 'dosod_hbm_compile_receipt_not_canonical_offline_evidence' in blockers
