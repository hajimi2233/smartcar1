#!/usr/bin/env python3
"""Copy results only from dedicated recovery-test containers; never touch live nav."""
import json
import pathlib
import subprocess

root = pathlib.Path(__file__).resolve().parents[1]
out = root / 'data/logs/recovery_validation'
out.mkdir(parents=True, exist_ok=True)
summary = {}
for name, report, log in (
    ('smartcar-recovery-a','recovery-results.json','recovery-suite.log'),
    ('smartcar-recovery-a','mission-results.json','mission-suite.log'),
    ('smartcar-recovery-b','recovery-results.json','recovery-suite.log'),
    ('smartcar-recovery-b','region-results.json','region-suite.log'),
    ('smartcar-recovery-b','mission-results.json','mission-suite.log'),
    ('smartcar-recovery-mission','mission-results.json','mission-suite.log'),
    ('smartcar-effort-big','mission-results.json','mission-suite.log'),
    ('smartcar-effort-narrow','mission-results.json','mission-suite.log'),
    ('smartcar-batch-big1','mission-results.json','mission-suite.log'),
    ('smartcar-batch-big2','mission-results.json','mission-suite.log'),
    ('smartcar-batch-narrow1','mission-results.json','mission-suite.log'),
):
    label = name + ('-region' if report.startswith('region-') else '-mission' if report.startswith('mission-') and not name.endswith('mission') else '')
    subprocess.run(['docker','cp',name+':/tmp/'+log,str(out/(label+'.log'))],capture_output=True)
    result = subprocess.run(['docker','exec',name,'cat','/tmp/'+report], capture_output=True,text=True)
    if result.returncode:
        summary[label] = {'state':'no completed case yet'}
        continue
    data=json.loads(result.stdout)
    (out/(label+'.json')).write_text(json.dumps(data,indent=2))
    subprocess.run(['docker','cp',name+':/tmp/'+log,str(out/(label+'.log'))],check=True)
    rows=data['results']
    summary[label]=dict(completed=len(rows),passed=sum(bool(r['success']) for r in rows),
                       direction_changes=sum(r.get('direction_changes',0) for r in rows) if all('direction_changes' in r for r in rows) else None,
                       recoveries=sum(r.get('recoveries',0) for r in rows) if all('recoveries' in r for r in rows) else None,
                       max_true_path_error=max([r.get('max_true_path_error',0) for r in rows]) if rows and all('max_true_path_error' in r for r in rows) else None,
                       elapsed_cases=sum(r['elapsed'] for r in rows),
                       max_localization_error=max([r['max_localization_error'] for r in rows] or [0]),
                       failures=[{'case':r.get('case',r.get('target')),'round':r.get('round'),
                                  'last_events':r['events'][-3:]} for r in rows if not r['success']])
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
