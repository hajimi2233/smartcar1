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
parser.add_argument('--case-timeout',type=float,default=160.)
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
state.pose.position.x=3.86;state.pose.position.y=2.8;state.pose.position.z=.015
state.pose.orientation.z=math.sin(math.pi/4);state.pose.orientation.w=math.cos(math.pi/4)
rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state)
time.sleep(3)
p=PoseWithCovarianceStamped(); p.header.frame_id='map'; x,y,yaw=truth_pose()
p.pose.pose.position.x=x;p.pose.pose.position.y=y
p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2)
for _ in range(3):initial.publish(p);time.sleep(.5)
wait(tf_ready,30,'localization')
node=navigation.Navigator()
wait(lambda:node.grid is not None and node.scan is not None,20,'nav sensors')
time.sleep(2)

try:
    for i in range(6):
        target=(3.86,3.8 if i%2==0 else 2.9,math.pi/2)
        begin=len(statuses);start=time.time();completed=False;max_error=0.
        msg=PoseStamped();msg.header.frame_id='map'
        msg.pose.position.x,msg.pose.position.y=target[:2]
        msg.pose.orientation.z=math.sin(math.pi/4);msg.pose.orientation.w=math.cos(math.pi/4)
        node.on_goal(msg)
        while time.time()-start<args.case_timeout:
            actual=truth_pose()
            try:
                xyz,_=listener.lookupTransform('map','base_footprint',rospy.Time(0))
                max_error=max(max_error,math.hypot(xyz[0]-actual[0],xyz[1]-actual[1]))
            except tf.Exception:pass
            log=[s for _,s in statuses[begin:]]
            if any(s.startswith('SUCCEEDED:') for s in log):completed=True;break
            if node.goal is None and time.time()-start>2:break
            time.sleep(.1)
        actual=truth_pose();front=(actual[0]+.31*math.cos(actual[2]),actual[1]+.31*math.sin(actual[2]))
        record=dict(case='blocked_centre_region' if i%2==0 else 'reverse_departure',round=i//2+1,
                    success=completed,target=target,elapsed=time.time()-start,max_localization_error=max_error,
                    final_front_error=math.hypot(front[0]-target[0],front[1]-target[1]),
                    events=[s for _,s in statuses[begin:]])
        results.append(record)
        with open(args.output,'w') as f:json.dump(dict(results=results),f,indent=2)
        print(json.dumps({k:v for k,v in record.items() if k!='events'}));sys.stdout.flush()
        with node.lock:node.halt('CANCELLED: region case complete')
        time.sleep(2)
finally:
    node.shutdown();rospy.signal_shutdown('region suite complete')
