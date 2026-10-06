#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Isolated Gazebo endurance: real nav/control, truth used only for scoring.
Run ONLY in a dedicated simulator/master. Injected failures are labelled.
"""
from __future__ import print_function
import argparse, json, math, os, signal, subprocess, sys, threading, time
import rospy, tf, yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
sys.path.insert(0, '/home/hajimi/smartcar_ws/src/smartcar_navigation/scripts')
import single_goal_nav as navigation
from nav_config import from_rear

parser=argparse.ArgumentParser()
parser.add_argument('--rounds',type=int,default=3)
parser.add_argument('--output',default='/tmp/recovery-results.json')
parser.add_argument('--case-timeout',type=float,default=900.)
parser.add_argument('--layout',default='B1B2A3B4A5A6B7A8B9A10')
parser.add_argument('--lines',action='store_true')
args=parser.parse_args()
rospy.init_node('single_goal_nav')
with open('/home/hajimi/smartcar/config/navigation.yaml') as f:
    for key,value in yaml.safe_load(f).items(): rospy.set_param('~'+key,value)
truth=[None]; statuses=[]; results=[]; fault={'reject_region':False}
rospy.Subscriber('/sim/ground_truth/odom',Odometry,lambda m:truth.__setitem__(0,m))
rospy.Subscriber('/single_nav/status',String,lambda m:statuses.append((time.time(),m.data)))
initial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
listener=tf.TransformListener()

def truth_pose():
    m=truth[0]; p=m.pose.pose; q=p.orientation
    yaw=tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]
    return (p.position.x,p.position.y,yaw)

def wait(check,seconds,label):
    deadline=time.time()+seconds
    while time.time()<deadline and not rospy.is_shutdown():
        if check():return
        time.sleep(.1)
    raise RuntimeError('timeout '+label)

def tf_ready():
    try: listener.lookupTransform('map','base_footprint',rospy.Time(0)); return True
    except tf.Exception:return False

wait(lambda:truth[0] is not None,30,'truth')
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
rospy.wait_for_service('/gazebo/set_model_state')
state=ModelState();state.model_name='inspection_car';state.reference_frame='world'
state.pose.position.x=.0248465083;state.pose.position.y=1.291814777;state.pose.position.z=.015
angle=-1.581938036
state.pose.orientation.z=math.sin(angle/2);state.pose.orientation.w=math.cos(angle/2)
rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state)
time.sleep(3)
p=PoseWithCovarianceStamped(); p.header.frame_id='map'; x,y,yaw=truth_pose()
p.pose.pose.position.x=x;p.pose.pose.position.y=y
p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2)
for _ in range(3):initial.publish(p);time.sleep(.5)
wait(tf_ready,30,'localization')
if args.lines:
    rospy.set_param('~load_saved_low_cost_lines',True)
    rospy.set_param('~low_cost_lines_file','/tmp/lines.json')
node=navigation.Navigator()
wait(lambda:node.grid is not None and node.scan is not None,20,'nav sensors')
time.sleep(2)

sys.path.insert(0,'/home/hajimi/smartcar_ws/src/smartcar_mission/scripts')
from plan_io import load_plan,load_regions
raw=subprocess.check_output(['rosrun','smartcar_mission','inspection_plan14',args.layout,'/tmp/points.csv'])
with open('/tmp/mission.jsonl','wb') as f:f.write(raw)
points,labels=load_plan('/tmp/mission.jsonl');regions=load_regions('/tmp/mission.jsonl')

def save():
    with open(args.output,'w') as f:json.dump(dict(layout=args.layout,lines=args.lines,results=results),f,indent=2)
try:
    for point,label in [(p,l) for p,l in zip(points,labels) if l=='inspect_5']:
        begin=len(statuses);started=time.time();done=False;max_error=0.;distance=0.;last=truth_pose()
        message=String(data=json.dumps(dict(frame_id='map',x=point[0],y=point[1],yaw=point[2],
                       target_id=label if label.startswith('inspect_') else None,regions=regions)))
        node.on_inspection_goal(message)
        with node.lock:
            node.line_stage='FINAL'
            node.relaxed_goal=True
            node.stop_state='VERIFY_STOP'
            node.stop_started=time.time()-100
            node.wait_retry('TEST_REPLAY: recorded inspect_5 stop with expired prior timer')
        next_sample=0.
        while time.time()-started<args.case_timeout and not rospy.is_shutdown():
            actual=truth_pose();distance+=math.hypot(actual[0]-last[0],actual[1]-last[1]);last=actual
            try:
                xyz,_=listener.lookupTransform('map','base_footprint',rospy.Time(0))
                max_error=max(max_error,math.hypot(xyz[0]-actual[0],xyz[1]-actual[1]))
            except tf.Exception:pass
            if time.time()>=next_sample:
                next_sample=time.time()+1.
                try:
                    pose=node.pose()
                    row=dict(time=time.time(),state=node.state,pose=pose,truth=actual,
                             blocked=node.grid.blocked_detail(pose),stage=node.line_stage)
                    with open('/tmp/mission-trace.jsonl','a') as trace:trace.write(json.dumps(row)+'\n')
                except Exception:pass
            log=[s for _,s in statuses[begin:]]
            if any(s.startswith('SUCCEEDED:') for s in log):done=True;break
            if node.goal is None and time.time()-started>2:break
            time.sleep(.1)
        result=dict(target=label,point=point,success=done,elapsed=time.time()-started,
                    distance=distance,max_localization_error=max_error,
                    outcome='arrived' if done else ('harness_timeout' if node.goal is not None else 'terminal_failure'),
                    last_state=node.state,events=[s for _,s in statuses[begin:]])
        results.append(result);save()
        print(json.dumps({k:v for k,v in result.items() if k!='events'}));sys.stdout.flush()
        if not done:break
        time.sleep(1.)
finally:
    with node.lock:node.halt('CANCELLED: mission test finished')
    node.shutdown();save();rospy.signal_shutdown('suite complete')
