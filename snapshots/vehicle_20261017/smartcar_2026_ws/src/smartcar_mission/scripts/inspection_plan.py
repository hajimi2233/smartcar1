#!/usr/bin/env python
"""Generate a C route, preview its goals, then load the existing multi-goal queue."""
from __future__ import print_function
import os
import subprocess
import sys
import tempfile
from plan_io import load_plan, load_regions, b_lateral_goals
from inspection_reporting import parse_description, annotate_plan


def main():
    legacy_mode = '--legacy-layout' in sys.argv
    if legacy_mode: sys.argv.remove('--legacy-layout')
    if len(sys.argv) not in (3, 4):
        sys.exit('Usage: inspection_plan.py LAYOUT /absolute/points.csv [--preview|--first8|--preview-first8]')
    if len(sys.argv) == 4 and sys.argv[3] not in ('--preview','--first8','--preview-first8'):
        sys.exit('Unknown plan mode')
    source = sys.argv[2]
    if not os.path.isabs(source) or not os.path.isfile(source):
        sys.exit('Expected an existing absolute points.csv path')
    b_lateral=legacy_mode and sys.argv[1].upper()=='B1B3B5B7B9'
    layout='B1A2B3A4B5A6B7A8B9A10' if b_lateral else sys.argv[1]
    reports = {}
    if not legacy_mode: layout, reports = parse_description(layout)
    raw = subprocess.check_output(['rosrun', 'smartcar_mission', 'inspection_plan14', layout, source])
    if reports: raw = annotate_plan(raw, reports)
    fd, path = tempfile.mkstemp(prefix='inspection-plan-', suffix='.jsonl')
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
        points, labels = load_plan(path)
        if b_lateral:points,labels=b_lateral_goals(points,labels,load_regions(path))
        for label in sorted(reports, key=lambda x: reports[x]['channel_number']):
            print((('Channel %d: ' % reports[label]['channel_number'])+reports[label]['speech_text']).encode('utf-8') if sys.version_info[0]<3 else ('Channel %d: ' % reports[label]['channel_number'])+reports[label]['speech_text'])
        first8=len(sys.argv)==4 and sys.argv[3] in ('--first8','--preview-first8')
        if b_lateral and first8:raise ValueError('Do not combine B lateral test and first8')
        if first8:
            count=0
            for stop,label in enumerate(labels):
                if label.startswith('inspect_'):count+=1
                if count==8:break
            points,labels=points[:stop+1],labels[:stop+1]
        for label, (x, y, yaw) in zip(labels, points):
            print('%-12s x=% .4f y=% .4f yaw=% .2f deg' % (label, x, y, yaw*180./3.141592653589793))
        print('Generated %d goals. Start is a calibration reference, not a drive goal.' % len(points))
        if len(sys.argv)==4 and sys.argv[3] in ('--preview','--preview-first8'):
            return
        import rosnode
        nodes = rosnode.get_node_names()
        if '/single_goal_nav' not in nodes:
            sys.exit('Start navigation with sim.sh nav first.')
        if '/multi_goal_nav' in nodes:
            sys.exit('Stop the existing multi-goal terminal before loading another plan.')
        print('Loading queue only. To drive, run sim.sh multi-execute in another terminal.')
        sys.stdout.flush()
        subprocess.check_call(['roslaunch', 'smartcar_mission', 'multi_goal_nav.launch', 'plan_file:='+path,'inspection_limit:='+('8' if first8 else '0'),'b_lateral_test:='+('true' if b_lateral else 'false')])
    finally:
        os.unlink(path)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
