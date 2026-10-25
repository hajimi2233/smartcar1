#!/usr/bin/env python
"""Map-bound scene entry. Loading never executes; only user-run service does."""
from __future__ import print_function
import csv,json,math,os,subprocess,sys
HERE=os.path.dirname(os.path.abspath(__file__))
WS='/home/smartcar/smartcar_2026_ws'


def check_map():
    import rospy
    from nav_msgs.msg import OccupancyGrid
    sys.path.insert(0,WS+'/src/smartcar_navigation/scripts')
    from low_cost_lines import map_key
    rospy.init_node('patrol_map15_check',anonymous=True,disable_signals=True)
    expected=json.load(open(HERE+'/deployment_manifest.json'))['map_key']
    current=rospy.wait_for_message('/map',OccupancyGrid,timeout=5.)
    if map_key(current)!=expected:raise ValueError('Current map differs from map15; no queue or goal sent')


def main():
    mode=sys.argv[1] if len(sys.argv)>1 else 'help'
    layout=sys.argv[2] if len(sys.argv)>2 else 'A1'*10
    if mode in ('run','cancel'):
        if mode=='run':check_map()
        return subprocess.call(['rosservice','call','/multi_goal_nav/'+('execute' if mode=='run' else 'cancel')])
    if mode=='marks':
        check_map()
        return subprocess.call(['rosrun','smartcar_mission','mission_points_visual.py',HERE+'/points_front.csv'])
    if mode in ('preview','load','preview-first8','first8'):
        if mode in ('load','first8'):check_map()
        args=['rosrun','smartcar_mission','inspection_plan.py',layout,HERE+'/points_front.csv']
        if mode=='preview':args.append('--preview')
        if mode=='first8':args.append('--first8')
        if mode=='preview-first8':args.append('--preview-first8')
        return subprocess.call(args)
    if mode in ('bslide','preview-bslide'):
        plan=HERE+'/bslide_full.plan.jsonl'
        if mode=='preview-bslide':
            with open(plan) as f:
                for i,line in enumerate(f):
                    if i>=5:break
                    q=json.loads(line);print('%s x=%.4f y=%.4f yaw=%.2fdeg'%(q['target_id'],q['x'],q['y'],math.degrees(q['yaw_rad'])))
            print('Preview only: B1 -> B3 -> B5 -> B7 -> B9');return 0
        check_map()
        import rosnode
        if '/multi_goal_nav' in rosnode.get_node_names():raise ValueError('Exit the old queue before loading a new queue')
        return subprocess.call(['roslaunch','smartcar_mission','multi_goal_nav.launch',
                'plan_file:='+plan,'inspection_limit:=5','b_lateral_test:=false'])
    print('~/patrol-map15 marks | preview [description] | load [description] | preview-first8 | first8 | preview-bslide | bslide | run | cancel')
    print('Load only prepares the queue. Run is a separate user action. Default description: ten A1 channels.')
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:sys.exit(str(exc))
