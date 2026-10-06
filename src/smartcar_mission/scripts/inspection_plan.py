#!/usr/bin/env python
"""Generate a C route, preview its goals, then load the existing multi-goal queue."""
from __future__ import print_function
import os
import subprocess
import sys
import tempfile
from plan_io import load_plan


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit('Usage: inspection_plan.py LAYOUT /absolute/points.csv [--preview]')
    if len(sys.argv) == 4 and sys.argv[3] != '--preview':
        sys.exit('Only --preview is supported')
    source = sys.argv[2]
    if not os.path.isabs(source) or not os.path.isfile(source):
        sys.exit('Expected an existing absolute points.csv path')
    raw = subprocess.check_output(['rosrun', 'smartcar_mission', 'inspection_plan14', sys.argv[1], source])
    fd, path = tempfile.mkstemp(prefix='inspection-plan-', suffix='.jsonl')
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
        points, labels = load_plan(path)
        for label, (x, y, yaw) in zip(labels, points):
            print('%-12s x=% .4f y=% .4f yaw=% .2f deg' % (label, x, y, yaw*180./3.141592653589793))
        print('Generated %d goals. Start is a calibration reference, not a drive goal.' % len(points))
        if len(sys.argv) == 4:
            return
        import rosnode
        nodes = rosnode.get_node_names()
        if '/single_goal_nav' not in nodes:
            sys.exit('Start navigation with sim.sh nav first.')
        if '/multi_goal_nav' in nodes:
            sys.exit('Stop the existing multi-goal terminal before loading another plan.')
        print('Loading queue only. To drive, run sim.sh multi-execute in another terminal.')
        sys.stdout.flush()
        subprocess.check_call(['roslaunch', 'smartcar_mission', 'multi_goal_nav.launch', 'plan_file:='+path])
    finally:
        os.unlink(path)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
