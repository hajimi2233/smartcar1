#!/usr/bin/env python
"""Real ROS transport check: all navigation topics remapped away from the car."""
from __future__ import print_function
import csv
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    binary, script_dir = sys.argv[1:]
    directory = tempfile.mkdtemp(prefix='plan14-ros-test-')
    csv_path = os.path.join(directory, 'points.csv')
    names = ['inspect_%d' % i for i in range(1,11)] + ['outer_left_top','outer_left_bottom','outer_right_top','outer_right_bottom','start','end']
    xy = [(i//2, 1. if i%2 == 0 else -1.) for i in range(10)] + [(-1,2),(-1,-2),(5,2),(5,-2),(-1,2),(-1,-2)]
    with open(csv_path,'w') as stream:
        writer = csv.writer(stream); writer.writerow(['point_id','x','y'])
        for name, pair in zip(names,xy): writer.writerow([name]+list(pair))
    plan_path = os.path.join(directory,'plan.jsonl')
    with open(plan_path,'wb') as stream:
        subprocess.check_call([binary, 'B1B2A3B4A5A6B7A8B9B10', csv_path],stdout=stream)
    with open(plan_path) as stream: expected = [json.loads(line) for line in stream]
    rospy.init_node('inspection_transport_test', anonymous=True)
    prefix = '/inspection_transport_%d' % os.getpid()
    node = prefix+'/queue'
    received, statuses = [], []
    lock = threading.Lock()
    def capture(msg):
        with lock: received.append(msg)
    rospy.Subscriber(prefix+'/inspection_goal',String,capture,queue_size=100)
    rospy.Subscriber(prefix+'/queue_status',String,lambda m: statuses.append(m.data),queue_size=100)
    feedback = rospy.Publisher(prefix+'/feedback',String,queue_size=10)
    args = [sys.executable,os.path.join(script_dir,'multi_goal_nav.py'), '__name:='+node,
            '_plan_file:='+plan_path,
            '/move_base_simple/goal:='+prefix+'/goal', '/single_nav/status:='+prefix+'/feedback',
            '/single_nav/inspection_goal:='+prefix+'/inspection_goal',
            '/single_nav/cancel:='+prefix+'/cancel', '/multi_nav/goal:='+prefix+'/selection',
            '/multi_nav/points:='+prefix+'/marks', '/multi_nav/status:='+prefix+'/queue_status']
    log = open(os.path.join(directory,'queue.log'),'w')
    process = subprocess.Popen(args,stdout=log,stderr=log,preexec_fn=os.setsid)
    def wait_for(condition):
        deadline=time.time()+10
        while not condition():
            if process.poll() is not None or time.time()>deadline:
                raise RuntimeError('Queue transport test timed out; logs: '+directory)
            time.sleep(.02)
    try:
        rospy.wait_for_service(node+'/execute',timeout=10)
        wait_for(lambda: feedback.get_num_connections()>0 and statuses)
        assert not received, 'Loading the queue must not move the car'
        assert rospy.ServiceProxy(node+'/execute',Trigger)().success
        for i,row in enumerate(expected):
            wait_for(lambda: len(received)>i)
            data=json.loads(received[i].data)
            assert data['frame_id']=='map'
            assert abs(data['x']-row['x'])<1e-9
            assert abs(data['y']-row['y'])<1e-9
            assert abs(data['yaw']-row['yaw_rad'])<1e-9
            assert len(data['regions'])==10
            assert data['regions'][0]['type']=='B'
            assert data['target_id']==(row['target_id'] if row['target_id'].startswith('inspect_') else None)
            feedback.publish(String(data='PLANNING'))
            feedback.publish(String(data='SUCCEEDED: isolated test feedback'))
        wait_for(lambda: any(s.startswith('SUCCEEDED: all') for s in statuses))
        assert len(received)==len(expected)
        print('PASS: %d goals received in order; x/y/yaw match C output; no goals sent before execute.' % len(expected))
        print('Only isolated topics under '+prefix+' were used. No driving test performed.')
    finally:
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        process.wait(); log.close()


if __name__=='__main__': main()
