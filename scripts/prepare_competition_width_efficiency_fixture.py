import pathlib,json,xml.etree.ElementTree as E,shutil,argparse
p=argparse.ArgumentParser();p.add_argument('--base-episode',type=pathlib.Path,required=True);p.add_argument('--output-dir',type=pathlib.Path,required=True);a=p.parse_args()
run=a.output_dir;run.mkdir(exist_ok=False);out=run/'episode';shutil.copytree(a.base_episode,out)
w=E.parse(out/'public/world.sdf');world=w.find('world')
if not any(x.get('filename')=='gz-sim-contact-system' for x in world.findall('plugin')):
 E.SubElement(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
# Dedicated empty straight-lane fixture: retain ground/environment, remove only
# discrete fixtures and old dirt models; never move a running vehicle.
removed=[]
for m in list(world.findall('model')):
 name=m.get('name','')
 if name.startswith(('surface_','object_','pedestrian_','material_cube')):
  removed.append(name);world.remove(m)
m=E.SubElement(world,'model',name='surface_width_efficiency_strip');E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text='-98 0 0.002 0 0 0';link=E.SubElement(m,'link',name='strip')
cells=[]
for i in range(220):
 for j in range(20):
  x=.05+i*.1;y=-.95+j*.1;cells.append([round(-98+x,5),round(y,5)])
  v=E.SubElement(link,'visual',name=f'leaf_{i}_{j}');E.SubElement(v,'pose').text=f'{x} {y} 0 0 0 0';box=E.SubElement(E.SubElement(v,'geometry'),'box');E.SubElement(box,'size').text='.1 .1 .002';mat=E.SubElement(v,'material');E.SubElement(mat,'diffuse').text='.3 .15 .03 1'
w.write(out/'public/world.sdf',encoding='utf-8',xml_declaration=True)
(out/'environment/pedestrian_schedule.json').write_text('{"pedestrians":[]}')
(run/'fixture.json').write_text(json.dumps({'purpose':'straight physical raster diagnostic','grid_m':.1,'cell_area_m2':.01,'strip_m':[22,2],'cells_xy':cells,'removed_fixture_models':removed,'fixed_window':'5 to 15 simulation seconds after motion start; no retrospective selection','scope':'COMPETITION_HARD_MINIMUM'},indent=2))
print(run)
