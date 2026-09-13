import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from analyze_competition_width_efficiency_estop import evaluate

def test_sparse_side_strips_do_not_inflate_contiguous_width_or_union_area():
 def d(t,cells):return {'kind':'dirt','sim_s':t,'wall_s':t*2,'data':{'cleaned_cells_xy':cells,'left_ready':True,'right_ready':True,'roller_ready':True}}
 rows=[{'kind':'motion_start','sim_s':0,'wall_s':0,'data':None},d(5,[[0,0]])]
 cells=[[0,i*.1] for i in range(6)]+[[0,1.0],[0,1.1]]
 rows += [d(15,cells),{'kind':'estop_trigger','sim_s':15,'wall_s':30,'data':{'linear':1,'angular':0}}]
 for t in [15.1,15.3,15.6,16.,16.3,16.5]:rows.append({'kind':'gt','sim_s':t,'wall_s':t*2,'data':{'linear':0,'angular':.02 if t==15.1 else 0}})
 r=evaluate(rows,'steady')
 assert r['new_cleaned_cell_count']==7
 assert abs(r['new_cleaned_union_area_m2']-.07)<1e-9
 assert abs(r['effective_contiguous_width_m']-.5)<1e-9
 assert r['width_status']=='FAIL'
 assert r['efficiency_status']=='FAIL'
 assert r['estop_status']=='PASS'
 assert abs(r['estop_delay_sim_s']-.3)<1e-9

def test_missing_physical_motion_is_fail_closed():
 assert evaluate([],'steady')['estop_status']=='FAIL'
