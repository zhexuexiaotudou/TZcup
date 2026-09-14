#!/usr/bin/env python3
"""Recompute acceptance strictly from recorded cell sets and six-axis GT speed."""
import argparse,json,math
from pathlib import Path

def evaluate(rows,mode):
 dirt=[r for r in rows if r['kind']=='dirt'];gt=[r for r in rows if r['kind']=='gt'];events={r['kind']:r for r in rows if r['kind'] in ['motion_start','estop_trigger']}
 result={'scope':'COMPETITION_HARD_MINIMUM Gazebo simulation','mode':mode,'width_status':'FAIL','efficiency_status':'FAIL','estop_status':'FAIL','linear_threshold_mps':.01,'angular_threshold_radps':.01,'hold_sim_s':1.0}
 if 'motion_start' not in events or 'estop_trigger' not in events:return result
 t0=events['motion_start']['sim_s'];trigger=events['estop_trigger'];te=trigger['sim_s'];before=trigger['data']
 result['estop_trigger_sim_s']=te;result['pre_estop_velocity']=before;result['estop_clock_note']='trigger uses latest physical dirt sim stamp; stop uses GT source stamp; trigger timestamp lag makes delay conservative'
 result['contact_positive_samples']={k:sum(r['kind']=='contact_'+k and r['data']['count']>0 for r in rows) for k in ['left_side_brush','right_side_brush','central_roller']}
 after=[dict(r, sim_s=r['data'].get('stamp',r['sim_s'])) for r in gt if r['data'].get('stamp',r['sim_s']) is not None and r['data'].get('stamp',r['sim_s'])>=te];stable=None
 for i,r in enumerate(after):
  if r['data']['linear']>.01 or r['data']['angular']>.01:continue
  tail=[x for x in after[i:] if x['sim_s']<=r['sim_s']+1.0]
  if tail and tail[-1]['sim_s']-r['sim_s']>=.98 and all(x['data']['linear']<=.01 and x['data']['angular']<=.01 for x in after[i:]):stable=r;break
 if stable:
  delay=stable['sim_s']-te;result.update(estop_delay_sim_s=delay,estop_delay_wall_s=stable['wall_s']-trigger['wall_s'],stopped_sim_s=stable['sim_s'],estop_hold_observed_sim_s=after[-1]['sim_s']-stable['sim_s'])
  if before and (before['linear']>.01 or before['angular']>.01) and delay<=1:result['estop_status']='PASS'
 def cells(r):return {(round(x,6),round(y,6)) for x,y in r['data']['cleaned_cells_xy']}
 if mode=='steady':
  begins=[r for r in dirt if r['sim_s']>=t0+5];ends=[r for r in dirt if r['sim_s']>=t0+15]
  if not begins or not ends:return result
  a,b=begins[0],ends[0]
 else:a,b=dirt[0],dirt[-1]
 new=cells(b)-cells(a);dt=b['sim_s']-a['sim_s'];area=len(new)*.01
 # Cells are nonoverlapping axis-aligned .1 m squares. Width is the largest
 # contiguous cleared row within a longitudinal column, not the span over gaps.
 columns={}
 for x,y in new:columns.setdefault(x,[]).append(y)
 widths=[]
 for x,ys in columns.items():
  seq=0;prev=None
  for y in sorted(ys):
   seq=seq+1 if prev is not None and abs(y-prev-.1)<1e-5 else 1;prev=y;widths.append(seq*.1)
 width=max(widths,default=0);eff=area/dt*3600 if dt>0 else 0
 window_gt=[r['data'] for r in gt if r['sim_s'] is not None and a['sim_s']<=r['sim_s']<=b['sim_s']]
 result.update(window_start_sim_s=a['sim_s'],window_end_sim_s=b['sim_s'],window_duration_sim_s=dt,new_cleaned_cell_count=len(new),new_cleaned_union_area_m2=area,effective_contiguous_width_m=width,transverse_span_m=max((y for x,y in new),default=0)-min((y for x,y in new),default=0)+(.1 if new else 0),efficiency_m2_h=eff,cleaned_cells_xy=sorted(new),all_three_ready_samples=sum(all(r['data'][k] for k in ['left_ready','right_ready','roller_ready']) for r in dirt),steady_linear_range_mps=[min((g['linear'] for g in window_gt),default=0),max((g['linear'] for g in window_gt),default=0)])
 result['width_status']='PASS' if width>=.6-1e-9 else 'FAIL'
 if mode=='steady' and eff>=3500:result['efficiency_status']='PASS'
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();protocol=json.loads((a.run/'protocol.json').read_text(encoding='utf-8-sig'));r=evaluate([json.loads(l) for l in (a.run/'timeline.jsonl').read_text().splitlines()],protocol['mode']);(a.run/'metrics.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='cleaned_cells_xy'},indent=2))
