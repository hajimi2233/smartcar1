import sys,json,math,time,pathlib
root=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/smartcar_navigation/scripts'))
import ackermann_core as c
from nav_config import configure,from_rear
out=root/'data/logs/recovery_validation'
from PIL import Image
import yaml
meta=yaml.safe_load((root/'data/maps/sim_field_v1/map.yaml').read_text());im=Image.open(root/'data/maps/sim_field_v1/map.pgm').convert('L');w,h=im.size
pix=list(im.getdata());occ=[100 if (255-v)/255.>meta['occupied_thresh'] else 0 if (255-v)/255.<meta['free_thresh'] else -1 for y in range(h-1,-1,-1) for v in pix[y*w:(y+1)*w]]
d=dict(width=w,height=h,resolution=meta['resolution'],origin=meta['origin'],data=occ);lines=list(json.loads((out/'lines_snapshot.json').read_text())['maps'].values())[0]
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--steer',type=float,default=.02);parser.add_argument('--switch',type=float,default=.45);parser.add_argument('--tag',default='baseline');parser.add_argument('--max-changes',type=int,default=None);parser.add_argument('--budget-scale',type=float,default=1.);parser.add_argument('--case-limit',type=int,default=3);args=parser.parse_args()
results=[]
for name in ['smartcar-biglines-v1-1','smartcar-biglines-v1-2']:
 goals={r['target_id']:r for r in map(json.loads,(out/name/'mission.jsonl').read_text().splitlines()) if 'target_id' in r}
 samples=list(map(json.loads,(out/name/'mission-trace.jsonl').read_text().splitlines()))
 seen=set();cases=[]
 for s in samples:
  if s.get('stage')=='LINE_SUFFIX' and s['target'] not in seen:
   seen.add(s['target']);cases.append(s)
 for s in cases[:args.case_limit]:
  configure({'planner_steer_cost':args.steer,'planner_switch_cost':args.switch});g=c.Grid(d['width'],d['height'],d['resolution'],tuple(d['origin']),d['data'],.04);g.zero_cost_line=lines
  t=goals[s['target']];goal=(t['x'],t['y'],t['yaw_rad']);start=tuple(s['pose']);heading=0. if math.cos(start[2])>0 else math.pi
  g.depth_rules=[dict(id=s['target'],type=t.get('inspection_type','A'),x=goal[0],y=goal[1],direction=1 if math.sin(goal[2])>0 else -1)]
  line=lines[c.nearest_line_number(lines,goal)]
  # Same pool of real planner results for both selectors; isolates selection
  # from CPU timing and random variation. Budget15s total, 5s/candidate.
  pool=[]
  for shift in (.3,.6,.9):
   retreat=c.line_retreat_target(line,goal,heading,1.5+shift)
   try:
    second=c.plan(g,start,retreat,1.3,2.*args.budget_scale,lambda:False,.06,math.radians(5),max_direction_changes=args.max_changes)
    third=c.plan(g,second[-1][:3],c.rear_target(goal),1.3,3.*args.budget_scale,lambda:False,.025,math.radians(10),'LATERAL',goal,max_direction_changes=args.max_changes)
    route=second+third[1:];eff=c.suffix_route_effort(second,third)
    k=[math.atan(.62*p[4]) for p in route[1:]]
    pool.append(dict(offset=shift,extra_switches=eff[0],effort=eff[1],seconds=c.route_seconds(route),segments=len(c.segments(route)),length=sum(math.hypot(a[0]-b[0],a[1]-b[1]) for a,b in zip(route,route[1:])),steering_variation=sum(abs(a-b) for a,b in zip([0.]+k,k+[0.])),collision_free=all(g.free(p[:3]) for p in route)))
   except (ValueError,RuntimeError) as exc:pass
  zero=next((p for p in pool if p['extra_switches']==0),None)
  old=zero or (min(pool,key=lambda p:p['seconds']) if pool else None)
  new=zero or (min(pool,key=lambda p:(p['extra_switches'],p['effort'])) if pool else None)
  row=dict(run=name,target=s['target'],feasible_candidates=len(pool),old=old,new=new,candidates=pool)
  results.append(row);print(json.dumps(row),flush=True)
  (out/('steering_search_'+args.tag+'.json')).write_text(json.dumps(results,indent=2))
print('DONE',len(results),flush=True)
