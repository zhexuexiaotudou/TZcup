#!/usr/bin/env python3
"""Seal externally produced official Y/UV preprocessing evidence; never produce pixels."""
from __future__ import annotations
import argparse, json, time, platform
from pathlib import Path
from hbm_evidence_common import atomic_json, fresh_directory, normal_file, sha256_file
from execute_dosod_nonformal_oracle_candidate_compile import _pilot_binding

ROOT=Path(__file__).resolve().parents[1]; RECEIPT_ID="tzcup_dosod_official_preprocess_capture_receipt_v1"
def b(path:Path): normal_file(path,"official_capture_input"); return {"path":str(path.resolve()),"sha256":sha256_file(path),"byte_size":path.stat().st_size}
def _official_identity(binary:Path,source:Path,dpkg:Path,dpkg_returncode:int)->dict:
 contract=json.loads((ROOT/'config/dosod_single_frame_preprocessing_oracle_contract.json').read_text(encoding='utf-8')); expected=contract.get('official_preprocess_identity')
 if not isinstance(expected,dict) or expected.get('status')!='VERIFIED': raise ValueError('official_preprocess_identity_unavailable')
 if dpkg_returncode!=0: raise ValueError('official_dpkg_query_failed')
 lines=dpkg.read_text(encoding='utf-8').splitlines()
 if len(lines)!=1 or len(lines[0].split('\t'))!=3: raise ValueError('official_dpkg_output_invalid')
 package,version,path_role=lines[0].split('\t')
 if (package,version,path_role)!=(expected.get('dpkg_package'),expected.get('dpkg_version'),expected.get('dpkg_path_role')): raise ValueError('official_dpkg_identity_mismatch')
 if str(binary.resolve())!=expected.get('binary_path') or sha256_file(binary)!=expected.get('binary_sha256') or str(source.resolve())!=expected.get('source_path') or sha256_file(source)!=expected.get('source_sha256') or not isinstance(expected.get('source_revision'),str) or not expected['source_revision']:
  raise ValueError('official_binary_or_source_identity_mismatch')
 return {'package':package,'version':version,'path_role':path_role,'source_revision':expected['source_revision'],'dpkg_returncode':dpkg_returncode}
def capture(*,pilot_manifest:Path,pilot_record_index:int,images_y:Path,images_uv:Path,adapter_binary:Path,adapter_source:Path,dpkg_output:Path,stdout:Path,stderr:Path,command:list[str],returncode:int,output:Path,test_fixture:bool=False,dpkg_returncode:int=0)->dict:
 fresh_directory(output,'official_capture_output')
 r={"receipt_id":RECEIPT_ID,"status":"BLOCKED","test_fixture":test_fixture,"raw_sensor":None,"official_preprocessor":None,"inputs":[],"producer_script_path":str(Path(__file__).resolve()),"producer_script_sha256":sha256_file(Path(__file__).resolve()),"started_epoch_ns":time.time_ns(),"ended_epoch_ns":None,"blockers":[]}
 try:
  if test_fixture: raise ValueError('test_fixture_forbidden')
  raw=_pilot_binding(pilot_manifest,pilot_record_index); y,uv=b(images_y),b(images_uv)
  if returncode!=0 or not command or command[0]!=str(adapter_binary.resolve()): raise ValueError('official_adapter_command_or_returncode_invalid')
  if y['byte_size']!=409600 or uv['byte_size']!=204800: raise ValueError('official_planes_size_invalid')
  identity=_official_identity(adapter_binary,adapter_source,dpkg_output,dpkg_returncode); r["raw_sensor"]={k:raw[k] for k in ('path','sha256','byte_size','width','height','step','encoding','frame_id','stamp_ns')}; r["official_preprocessor"]={"binary":b(adapter_binary),"source":b(adapter_source),"dpkg":b(dpkg_output),"stdout":b(stdout),"stderr":b(stderr),"command":command,"returncode":returncode,"identity":identity,"execution_host":{"system":platform.system(),"machine":platform.machine()},"zero_survivor":None}; r["inputs"]=[{"role":"images_y",**y},{"role":"images_uv",**uv}]; r["status"]="OFFICIAL_PREPROCESS_CAPTURED"
 except Exception as e: r['blockers'].append(f'capture_failed:{type(e).__name__}:{e}'); r['status']='TEST_FIXTURE_BLOCKED' if test_fixture else 'BLOCKED'
 r['ended_epoch_ns']=time.time_ns(); atomic_json(output/'dosod_official_preprocess_capture_receipt.json',r); return r
def main()->int:
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--pilot-manifest',required=True,type=Path); p.add_argument('--pilot-record-index',required=True,type=int); p.add_argument('--images-y',required=True,type=Path); p.add_argument('--images-uv',required=True,type=Path); p.add_argument('--adapter-binary',required=True,type=Path); p.add_argument('--adapter-source',required=True,type=Path); p.add_argument('--dpkg-output',required=True,type=Path); p.add_argument('--dpkg-returncode',required=True,type=int); p.add_argument('--stdout',required=True,type=Path); p.add_argument('--stderr',required=True,type=Path); p.add_argument('--command',required=True,nargs='+'); p.add_argument('--returncode',required=True,type=int); p.add_argument('--output',required=True,type=Path); p.add_argument('--test-fixture',action='store_true'); a=p.parse_args()
 r=capture(pilot_manifest=a.pilot_manifest,pilot_record_index=a.pilot_record_index,images_y=a.images_y,images_uv=a.images_uv,adapter_binary=a.adapter_binary,adapter_source=a.adapter_source,dpkg_output=a.dpkg_output,stdout=a.stdout,stderr=a.stderr,command=a.command,returncode=a.returncode,output=a.output,test_fixture=a.test_fixture,dpkg_returncode=a.dpkg_returncode); print(json.dumps(r,indent=2)); return 0 if r['status']=='OFFICIAL_PREPROCESS_CAPTURED' else 2
if __name__=='__main__': raise SystemExit(main())
