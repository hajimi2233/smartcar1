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
time.sleep(3)
p=PoseWithCovarianceStamped(); p.header.frame_id='map'; x,y,yaw=truth_pose()
p.pose.pose.position.x=x;p.pose.pose.position.y=y
p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2)
for _ in range(3):initial.publish(p);time.sleep(.5)
wait(tf_ready,30,'localization')
node=navigation.Navigator()
wait(lambda:node.grid is not None and node.scan is not None,20,'nav sensors')
time.sleep(2)
original_plan=navigation.plan

def injected_plan(*a,**kw):
    if fault['reject_region'] and kw.get('goal_region',False):
        fault['reject_region']=False
        raise RuntimeError('TEST_INJECTED: regional search failure after physical progress')
    return original_plan(*a,**kw)
navigation.plan=injected_plan

def save():
    with open(args.output,'w') as f:json.dump(dict(results=results,completed=len(results)),f,indent=2)

def laser_pid():
    output=subprocess.check_output(['pgrep','-f','smartcar_sim/scripts/laser_noise.py']).decode().split()
    if len(output)!=1:raise RuntimeError('ambiguous laser pid '+str(output))
    return int(output[0])

try:
    for cycle in range(args.rounds):
        for case in ('normal','scan_outage','return_stop','normal_after_recovery'):
            # Same heading, alternating forward/reverse on a free upper lane.
            # Fixed map coordinates prevent accumulating target drift.
            current=truth_pose()
            target_x=2.45 if current[0]>3.0 else 3.55
            target=(target_x,2.8,math.pi)
            msg=PoseStamped();msg.header.frame_id='map'
            msg.pose.position.x,msg.pose.position.y=target[:2]
            msg.pose.orientation.z=1.;msg.pose.orientation.w=0.
            begin=len(statuses); start=time.time(); start_pose=current
            peak_cross=0.; peak_loc=0.; distance=0.; last=current; injected=False
            paused=None; resume_at=None; completed=False; return_seen=False
            node.on_goal(msg)
            while time.time()-start<args.case_timeout and not rospy.is_shutdown():
                now=time.time(); actual=truth_pose()
                distance+=math.hypot(actual[0]-last[0],actual[1]-last[1]);last=actual
                peak_cross=max(peak_cross,abs(actual[1]-target[1]))
                try:
                    xyz,_=listener.lookupTransform('map','base_footprint',rospy.Time(0))
                    peak_loc=max(peak_loc,math.hypot(xyz[0]-actual[0],xyz[1]-actual[1]))
                except tf.Exception:pass
                moved=math.hypot(actual[0]-start_pose[0],actual[1]-start_pose[1])
                if not injected and moved>.30 and node.state=='DRIVING':
                    if case=='scan_outage':
                        paused=laser_pid();os.kill(paused,signal.SIGSTOP);resume_at=now+3.
                        injected=True
                    elif case=='return_stop':
                        fault['reject_region']=True
                        with node.lock:node.wait_retry('TEST_INJECTED: recovery requested after physical progress')
                        injected=True
                if paused and now>=resume_at:
                    os.kill(paused,signal.SIGCONT);paused=None
                log=[s for _,s in statuses[begin:]]
                return_seen=return_seen or any(s.startswith('RECOVERY_RETURNED:') for s in log)
                if any(s.startswith('SUCCEEDED:') for s in log):completed=True;break
                if node.goal is None and now-start>2:break
                time.sleep(.1)
            if paused:os.kill(paused,signal.SIGCONT)
            actual=truth_pose();front=(actual[0]+.31*math.cos(actual[2]),actual[1]+.31*math.sin(actual[2]))
            log=[s for _,s in statuses[begin:]]
            success=completed and (case not in ('scan_outage','return_stop') or injected)
            if case=='return_stop':success=success and return_seen
            result=dict(round=cycle+1,case=case,success=success,arrived=completed,injected=injected,
                        returned=return_seen,elapsed=time.time()-start,distance=distance,
                        max_cross_track=peak_cross,max_localization_error=peak_loc,
                        final_front_error=math.hypot(front[0]-target[0],front[1]-target[1]),
                        events=log,target=target)
            results.append(result);save()
            print(json.dumps({k:v for k,v in result.items() if k!='events'}));sys.stdout.flush()
            with node.lock:node.halt('CANCELLED: test case complete')
            fault['reject_region']=False
            time.sleep(2)
finally:
    node.shutdown();save();rospy.signal_shutdown('suite complete')
