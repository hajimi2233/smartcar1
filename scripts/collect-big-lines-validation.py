#!/usr/bin/env python3
"""Collect isolated big-map missions, including in-progress efficiency warnings."""
import json,pathlib,subprocess,time
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'data/logs/recovery_validation'
manifest=json.loads((out/'big_lines_manifest.json').read_text())
rows=[]
for item in manifest:
 name=item['name'];folder=out/name;folder.mkdir(exist_ok=True)
 row=dict(item)
 state=subprocess.run(['docker','inspect','-f','{{.State.Status}}',name],capture_output=True,text=True)
 row['container_state']=state.stdout.strip()
 process=subprocess.run(['docker','top',name,'-eo','pid,args'],capture_output=True,text=True)
 row['mission_process_running']=('python -u /tmp/mission_endurance.py' in process.stdout) if process.returncode==0 else None
 if process.returncode:row['process_check_error']=process.stderr.strip()
 for source,key in [('mission-results.json','report'),('mission-progress.json','progress')]:
  result=subprocess.run(['docker','cp',name+':/tmp/'+source,str(folder/source)],capture_output=True)
  if result.returncode==0:
   try:row[key]=json.loads((folder/source).read_text())
   except ValueError:row[key]={'error':'invalid JSON'}
 for source in ['mission-suite.log','mission-trace.jsonl','mission.jsonl']:
  subprocess.run(['docker','cp',name+':/tmp/'+source,str(folder/source)],capture_output=True)
 trace_path=folder/'mission-trace.jsonl'
 if trace_path.exists():
  samples=[]
  for line in trace_path.read_text().splitlines()[-31:]:
   try:
    sample=json.loads(line)
    stamp=sample.get('localization',{}).get('stamp')
    if stamp is not None:samples.append((sample['time'],stamp))
   except (ValueError,KeyError):pass
  if len(samples)>1 and samples[-1][0]-samples[0][0]>0:
   row['recent_sim_time_ratio']=(samples[-1][1]-samples[0][1])/(samples[-1][0]-samples[0][0])
   row['resource_warning']=row['recent_sim_time_ratio']<.85
 report=row.get('report',{});completed=report.get('results',[])
 row['summary']=dict(full_mission_passed=bool(completed) and len(completed)==report.get('total_targets') and all(r['success'] for r in completed),
                     completed=len(completed),passed=sum(bool(r['success']) for r in completed),
                     total_targets=report.get('total_targets'),
                     elapsed=sum(r['elapsed'] for r in completed),
                     recoveries=sum(r.get('recoveries',0) for r in completed),
                     efficiency_warnings=[r['target'] for r in completed if r.get('efficiency_warning')],
                     failures=[dict(target=r['target'],outcome=r['outcome'],last_events=r['events'][-6:]) for r in completed if not r['success']])
 rows.append(row)
 print(json.dumps(dict(name=name,summary=row['summary'],progress=row.get('progress')),ensure_ascii=False))
(out/'big_lines_summary.json').write_text(json.dumps(dict(collected=time.time(),runs=rows),indent=2))
