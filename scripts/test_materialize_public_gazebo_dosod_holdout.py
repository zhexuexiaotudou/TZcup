from __future__ import annotations
import hashlib, importlib.util, json, sys
from pathlib import Path
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
spec=importlib.util.spec_from_file_location("subject", HERE / "materialize_public_gazebo_dosod_holdout.py"); subject=importlib.util.module_from_spec(spec); assert spec.loader; spec.loader.exec_module(subject)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def test_materializer_requires_exact_100_and_binds_adapter_and_sidecar(tmp_path):
    root=tmp_path/'capture'; root.mkdir(); rows=[]
    for i in range(100):
        npy=root/f'{i}.npy'; np.save(npy,np.zeros((1,3,640,640),np.float32),allow_pickle=False)
        side=root/f'{i}.json'; side.write_text('{}')
        raw=root/f'{i}.bin'; raw.write_bytes(bytes.fromhex(f'{i:064x}'))
        rows.append({'source_role':'evaluation_holdout_only','source_sha256':sha(raw),'relative_path':npy.name,'byte_size':npy.stat().st_size,'sha256':sha(npy),'raw_sensor':{'relative_path':raw.name,'sha256':sha(raw),'byte_size':raw.stat().st_size,'width':1,'height':1,'step':32,'encoding':'rgb8','frame_id':'camera','stamp_ns':i+1},'gt_sidecar':{'relative_path':side.name,'byte_size':side.stat().st_size,'sha256':sha(side)},'generation_nonce':'1'*32,'episode_manifest_sha256':'a'*64})
    manifest=root/'capture.json'; manifest.write_text(json.dumps({'status':'FROZEN','holdout_records':rows}))
    def adapter(src,y,uv): y.write_bytes(b'\0'*409600); uv.write_bytes(b'\0'*204800)
    identity={'status':'VERIFIED','path':'/official/adapter','sha256':'b'*64,'version':'v1','command':['official','--input']}
    result=subject.materialize(capture_manifest=manifest,capture_root=root,output=tmp_path/'out',adapter=adapter,adapter_identity=identity)
    value=json.loads(result.read_text()); assert len(value['records'])==100 and value['hbm_input_adapter']==identity
    assert [(item['role'],item['byte_size']) for item in value['records'][0]['hbm_input_files']]==[('images_y',409600),('images_uv',204800)]
    parity_spec=importlib.util.spec_from_file_location("parity_consumer",HERE/'run_dosod_hbm_x86_parity.py'); parity=importlib.util.module_from_spec(parity_spec); assert parity_spec.loader; parity_spec.loader.exec_module(parity)
    metric_spec=importlib.util.spec_from_file_location("metric_consumer",HERE/'validate_dosod_quantized_metric_regression.py'); metric=importlib.util.module_from_spec(metric_spec); assert metric_spec.loader; metric_spec.loader.exec_module(metric)
    consumed, records=parity._holdout_records(result,set()); sources, adapter=metric._holdout_sources_and_adapter(result)
    assert consumed['hbm_input_adapter']==identity and len(records)==100 and sources=={row['source_sha256'] for row in value['records']} and adapter==identity
    rows[0]['raw_sensor']['sha256']='0'*64; manifest.write_text(json.dumps({'status':'FROZEN','holdout_records':rows}))
    with pytest.raises(subject.HoldoutBlocked,match='raw_sensor_drift'): subject.materialize(capture_manifest=manifest,capture_root=root,output=tmp_path/'raw-drift',adapter=adapter,adapter_identity=identity)
    with pytest.raises(subject.HoldoutBlocked): subject.materialize(capture_manifest=manifest,capture_root=root,output=tmp_path/'again',adapter=adapter,adapter_identity={})
