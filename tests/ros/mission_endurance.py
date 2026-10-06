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
from nav_config import from_rear, to_rear
from path_tracking import project

parser=argparse.ArgumentParser()
parser.add_argument('--rounds',type=int,default=3)
parser.add_argument('--output',default='/tmp/recovery-results.json')
parser.add_argument('--case-timeout',type=float,default=900.)
parser.add_argument('--layout',default='B1B2A3B4A5A6B7A8B9A10')
parser.add_argument('--lines',action='store_true')
parser.add_argument('--max-recoveries',type=int,default=0,help='test-only inefficiency cutoff; zero disables')
args=parser.parse_args()
rospy.init_node('single_goal_nav')
with open('/home/hajimi/smartcar/config/navigation.yaml') as f:
    for key,value in yaml.safe_load(f).items(): rospy.set_param('~'+key,value)
localization_detail=[{}]
rospy.Subscriber('/wall_localization/status',String,lambda m:localization_detail.__setitem__(0,json.loads(m.data)))
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

def atomic_json(path, data):
    with open(path+'.tmp','w') as f:json.dump(data,f,indent=2)
    os.rename(path+'.tmp',path)

def planning_seconds(events,now):
    total=0.;begin=None
    for stamp,event in events:
        if event=='PLANNING':
            if begin is not None:total+=stamp-begin
            begin=stamp
        elif begin is not None and event.startswith(('DRIVING:','RECOVERY_WAIT: goal retained;', 'PLAN_FAILED:', 'CANCELLED:')):
            total+=stamp-begin;begin=None
    return total+(now-begin if begin is not None else 0.)

def save():
    atomic_json(args.output,dict(layout=args.layout,lines=args.lines,total_targets=len(points),results=results))
try:
    for point,label in zip(points,labels):
        begin=len(statuses);started=time.time();done=False;max_error=0.;distance=0.;last=truth_pose();peak_tracking=0.;peak_actual_tracking=0.;switches=0;last_sign=0
        message=String(data=json.dumps(dict(frame_id='map',x=point[0],y=point[1],yaw=point[2],
                       target_id=label if label.startswith('inspect_') else None,regions=regions)))
        node.on_inspection_goal(message)
        next_sample=0.;inefficiency=None
        while time.time()-started<args.case_timeout and not rospy.is_shutdown():
            if node.state=='DRIVING' and node.parts:
                try:
                    with node.lock:
                        part=node.parts[node.part];ref=project(node.pose(),part,node.index)
                        peak_tracking=max(peak_tracking,ref['distance'])
                        real_ref=project(to_rear(truth_pose(),'base'),part,node.index)
                        peak_actual_tracking=max(peak_actual_tracking,real_ref['distance'])
                        sign=part[-1][3]
                        if last_sign and last_sign!=sign:switches+=1
                        last_sign=sign
                except (IndexError,ValueError,RuntimeError):pass
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
                             blocked=node.grid.blocked_detail(pose),stage=node.line_stage,
                             localization=localization_detail[0])
                    log=[s for _,s in statuses[begin:]]
                    recoveries=sum(s.startswith('RECOVERY_WAIT: goal retained;') for s in log)
                    row.update(target=label,elapsed=time.time()-started,recoveries=recoveries,
                               planning_attempts=sum(s=='PLANNING' for s in log),
                               planning_seconds=planning_seconds(statuses[begin:],time.time()),
                               direction_changes=switches,distance=distance,
                               efficiency_warning=recoveries>=3)
                    atomic_json('/tmp/mission-progress.json',row)
                    with open('/tmp/mission-trace.jsonl','a') as trace:trace.write(json.dumps(row)+'\n')
                except Exception:pass
            log=[s for _,s in statuses[begin:]]
            if any(s.startswith('SUCCEEDED:') for s in log):done=True;break
            if args.max_recoveries and sum(s.startswith('RECOVERY_WAIT: goal retained;') for s in log)>=args.max_recoveries:
                inefficiency='excessive_recovery';break
            if node.goal is None and time.time()-started>2:break
            time.sleep(.1)
        result=dict(target=label,point=point,success=done,elapsed=time.time()-started,
                    distance=distance,max_localization_error=max_error,
                    outcome='arrived' if done else inefficiency or ('harness_timeout' if node.goal is not None else 'terminal_failure'),
                    last_state=node.state,planning_seconds=planning_seconds(statuses[begin:],time.time()),
                    efficiency_warning=sum(s.startswith('RECOVERY_WAIT: goal retained;') for _,s in statuses[begin:])>=3,
                    max_tracking_error=peak_tracking,
                    max_true_path_error=peak_actual_tracking,direction_changes=switches,
                    recoveries=sum(s.startswith('RECOVERY_WAIT: goal retained;') for _,s in statuses[begin:]),
                    planning_attempts=sum(s=='PLANNING' for _,s in statuses[begin:]),events=[s for _,s in statuses[begin:]])
        results.append(result);save()
        print(json.dumps({k:v for k,v in result.items() if k!='events'}));sys.stdout.flush()
        if not done:break
        time.sleep(1.)
finally:
    with node.lock:node.halt('CANCELLED: mission test finished')
    node.shutdown();save();rospy.signal_shutdown('suite complete')
