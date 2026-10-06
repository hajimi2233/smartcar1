#!/usr/bin/env python3
"""Start isolated real-motion missions with explicit fixtures and run manifests."""
import argparse,concurrent.futures,json,pathlib,subprocess,time
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'data/logs/recovery_validation'
parser=argparse.ArgumentParser()
parser.add_argument('--effort',action='store_true',help='run paired effort-optimization trials')
parser.add_argument('--big-lines',action='store_true',help='four latest big-map missions with efficiency monitoring')
parser.add_argument('--tag',default='v1',help='unique big-lines run label')
parser.add_argument('--cases',default='1,2,3,4',help='big-lines layout indices, e.g. 1,2')
parser.add_argument('--append',action='store_true',help='retain existing big-lines runs in active manifest')
parser.add_argument('--image',default='smartcar:layered-v1',help='pinned image for paired baseline/candidate')
args=parser.parse_args()
try:cases=[int(v) for v in args.cases.split(',')]
except ValueError:parser.error('cases must be comma-separated indices 1..4')
if not cases or len(set(cases))!=len(cases) or any(i not in range(1,5) for i in cases):
 parser.error('cases must be unique indices 1..4')
if args.append and not args.big_lines:parser.error('append requires big-lines')
if not args.tag.replace('-','').isalnum():parser.error('tag must be alphanumeric or hyphen')
suite=([('smartcar-effort-big','sim_field_v1','B1B2A3B4A5A6B7A8B9A10'),
        ('smartcar-effort-narrow','sim_field_narrow','B1B2A3B4A5A6B7A8B9A10')]
       if args.effort else
       [('smartcar-batch-big1','sim_field_v1','B1B2A3B4A5A6B7A8B9A10'),
        ('smartcar-batch-big2','sim_field_v1','A1A2B3B4A5B6A7A8A9A10'),
        ('smartcar-batch-narrow1','sim_field_narrow','A1B2B3A4B5A6A7B8A9B10')])

if args.big_lines:
 suite=[('smartcar-biglines-'+args.tag+'-'+str(i+1),'sim_field_v1',layout)
        for i,layout in enumerate(('B1B2A3B4A5A6B7A8B9A10',
                                  'A1A2B3B4A5B6A7A8A9A10',
                                  'A1B2A3B4A5B6A7B8A9B10',
                                  'B1A2B3A4B5A6B7A8B9A10')) if i+1 in cases]

def call(args,**kw):return subprocess.run(args,check=True,text=True,capture_output=True,**kw)
def launch(item):
 name,mapname,layout=item
 command=['docker','run','-d','--name',name,'--cpus=3','--memory=2g',
          '-e','ROS_MASTER_URI=http://localhost:11311','-e','GAZEBO_MASTER_URI=http://localhost:11345',
          args.image,'drivers-sim','gui:=false','rviz:=false']
 if mapname=='sim_field_narrow':
  command+=['data_dir:=/home/hajimi/smartcar/data/maps/'+mapname,
            'world_file:=/home/hajimi/smartcar/data/maps/'+mapname+'/inspection.world']
 call(command)
 setup='source /home/hajimi/smartcar_ws/devel/setup.bash; '
 deadline=time.time()+60
 while time.time()<deadline:
  try:
   nodes=call(['docker','exec',name,'bash','-c',setup+'rosnode list'],timeout=4).stdout
   if '/sim_reference_map' in nodes and '/gazebo' in nodes:break
  except (subprocess.CalledProcessError,subprocess.TimeoutExpired):pass
  time.sleep(1)
 else:raise RuntimeError(name+' driver not ready')
 for src,dst in [('tests/ros/mission_endurance.py','/tmp/mission_endurance.py'),
                 ('tests/ros/bind_test_lines.py','/tmp/bind_test_lines.py'),
                 ('data/logs/recovery_validation/points_snapshot.csv','/tmp/points.csv'),
                 ('data/logs/recovery_validation/lines_snapshot.json','/tmp/lines-source.json')]:
  call(['docker','cp',str(root/src),name+':'+dst])
 loc=(setup+'roslaunch smartcar_localization localization.launch map_file:=/home/hajimi/smartcar/data/maps/'+mapname+
      '/map.yaml params:=/home/hajimi/smartcar/config/localization.yaml nav_config:=/home/hajimi/smartcar/config/navigation.yaml > /tmp/localization.log 2>&1')
 call(['docker','exec','-d',name,'bash','-c',loc])
 bound=call(['docker','exec',name,'bash','-c',setup+'python /tmp/bind_test_lines.py'],timeout=50).stdout
 run=setup+'python -u /tmp/mission_endurance.py --lines --layout '+layout+' --case-timeout 900 --output /tmp/mission-results.json'+(' --max-recoveries 12' if args.big_lines else '')+' > /tmp/mission-suite.log 2>&1'
 call(['docker','exec','-d',name,'bash','-c',run])
 return dict(name=name,map=mapname,layout=layout,lines=True,image=call(['docker','inspect','-f','{{.Image}}',name]).stdout.strip(),fixture='unchanged user line coordinate copy',binding=bound.strip(),started=time.time())
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=4 if args.big_lines else 3) as pool:
 futures={pool.submit(launch,item):item for item in suite}
 for future in concurrent.futures.as_completed(futures):
  try:row=future.result()
  except Exception as exc:row=dict(name=futures[future][0],error=str(exc))
  results.append(row)
  print(json.dumps(row),flush=True)
manifest='big_lines_manifest.json' if args.big_lines else 'effort_manifest.json' if args.effort else 'batch_manifest.json'
active=results
if args.append and (out/manifest).exists():
 old=json.loads((out/manifest).read_text())
 existing={r['name'] for r in old}
 active=old+[r for r in results if r['name'] not in existing]
(out/manifest).write_text(json.dumps(active,indent=2))
if args.big_lines:(out/('big_lines_manifest_'+args.tag+'.json')).write_text(json.dumps(results,indent=2))
