#!/usr/bin/env python3
"""Actual suffix planner replay; compare saved feasible route vs spare-time optimization.
Static map and mission depth rules only. No dynamic laser replay or closed-loop motion.
"""
import json,math,pathlib,sys,time
from PIL import Image
import yaml
root=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/smartcar_navigation/scripts'))
import ackermann_core as c
from nav_config import configure,from_rear
from inspection_depth import rules_for_goal
out=root/'data/logs/recovery_validation'
meta=yaml.safe_load((root/'data/maps/sim_field_v1/map.yaml').read_text())
im=Image.open(root/'data/maps/sim_field_v1/map.pgm').convert('L');w,h=im.size
pixels=list(im.getdata());data=[100 if (255-v)/255.>meta['occupied_thresh'] else 0 if (255-v)/255.<meta['free_thresh'] else -1 for y in range(h-1,-1,-1) for v in pixels[y*w:(y+1)*w]]
lines=list(json.loads((out/'lines_snapshot.json').read_text())['maps'].values())[0]
results=[]
def metrics(path,g):
 return dict(gear_changes=c.gear_route_effort(path)[0],segments=len(c.segments(path)),length_m=sum(math.hypot(a[0]-b[0],a[1]-b[1]) for a,b in zip(path,path[1:])),estimated_seconds=c.route_seconds(path),collision_free=all(g.free(p[:3]) for p in path))
for name in ('smartcar-biglines-v1-1','smartcar-biglines-v1-2'):
 tasks=list(map(json.loads,(out/name/'mission.jsonl').read_text().splitlines()))
 tasks=[t for t in tasks if 'target_id' in t]
 report=json.loads((out/name/'mission-results.json').read_text())
 samples=list(map(json.loads,(out/name/'mission-trace.jsonl').read_text().splitlines()))
 regions=[dict(id=t['target_id'],type=t['inspection_type'],x=t['x'],y=t['y']) for t in tasks if t.get('inspect')]
 # Inspection tasks occur once each in these fixture missions.
 known={};cases=[]
 for t in tasks:
  label=t['target_id'];rows=[s for s in samples if s.get('target')==label]
  if not rows:continue
  goal=(t['x'],t['y'],t['yaw_rad']);target_id=label if label.startswith('inspect_') else None
  rules=rules_for_goal(regions,from_rear(tuple(rows[0]['pose']),'goal'),goal,target_id,known)
  suffix=next((s for s in rows if s.get('stage')=='LINE_SUFFIX'),None)
  if suffix and len(cases)<3:
   events=next(r['events'] for r in report['results'] if r['target']==label)
   mode=next(e.split(': ',1)[1].split(';')[0] for e in events if e.startswith('GOAL_MODE:'))
   cases.append((label,goal,rules,suffix,mode))
 for label,goal,rules,sample,mode in cases:
  configure();g=c.Grid(w,h,meta['resolution'],tuple(meta['origin']),data,.04)
  g.zero_cost_line=lines;g.depth_rules=rules
  start=tuple(sample['pose']);heading=0. if math.cos(start[2])>0 else math.pi
  line=lines[c.nearest_line_number(lines,goal)];candidates=[];attempts=[];saved=[]
  original_score=c.suffix_route_effort;original_improve=c.improve_gear_route;original_plan=c.plan
  def score(a,b):
   candidates.append((a+b[1:],b));return original_score(a,b)
  def plan(*a,**kw):
   if 'max_direction_changes' not in kw:return original_plan(*a,**kw)
   record=dict(limit=kw['max_direction_changes']);attempts.append(record)
   try:
    p=original_plan(*a,**kw);record['success']=True;return p
   except (RuntimeError,ValueError) as exc:
    record.update(success=False,error=str(exc));raise
  def improve(*a,**kw):
   incumbent=a[3]
   match=next(route for route,third in candidates if third is incumbent)
   saved.append(match)
   return original_improve(*a,**kw)
  c.suffix_route_effort=score;c.improve_gear_route=improve;c.plan=plan
  began=time.time();row=dict(run=name,target=label,mode=mode,budget_seconds=15,rules=rules)
  try:
   path,retreat=c.plan_line_suffix(g,start,c.rear_target(goal),line,heading,1.3,15.,lambda:False,.025,math.radians(10),mode,goal)
   row.update(success=True,before=metrics(saved[0],g),after=metrics(path,g),retreat=retreat,attempts=attempts)
  except (RuntimeError,ValueError) as exc:row.update(success=False,error=str(exc),attempts=attempts)
  finally:c.suffix_route_effort=original_score;c.improve_gear_route=original_improve;c.plan=original_plan
  row['search_seconds']=time.time()-began;results.append(row)
  (out/'anytime_planner_screen.json').write_text(json.dumps(results,indent=2))
  print(json.dumps(row),flush=True)
print('DONE',len(results),flush=True)
