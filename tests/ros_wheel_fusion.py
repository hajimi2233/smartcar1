#!/usr/bin/env python
"""Isolated ROS wire test with synthetic ray scans; never drives a live robot."""
from __future__ import print_function
import json
import math
import os
import sys
import time
import tempfile
import shutil
import subprocess
import threading
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,ROOT+'/src/smartcar_localization/scripts')
from map_wall_extraction import extract_groups
from wall_features import fingerprint


def run():
    rospy.init_node('wheel_integration');folder=tempfile.mkdtemp();children=[]
    state={};summary={};edges={};distance=[0.];feed=[True];running=[True];mutex=threading.Lock()
    def status(kind,msg): state[kind]=json.loads(msg.data)
    from tf2_msgs.msg import TFMessage
    def transforms(msg):
        for t in msg.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom': edges[msg._connection_header['callerid']]=True
    rospy.Subscriber('/tf',TFMessage,transforms)
    rospy.Subscriber('/wall_localization/status',String,lambda m:status('fused',m))
    rospy.Subscriber('/lidar_baseline/status',String,lambda m:status('lidar',m))
    rospy.Subscriber('/localization_eval/summary',String,lambda m:summary.update(json.loads(m.data)))
    pubmap=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
    pubdef=rospy.Publisher('/wall_features/definition',String,queue_size=1,latch=True)
    pubregion=rospy.Publisher('/external_region/definition',String,queue_size=1,latch=True)
    pubtruth=rospy.Publisher('/sim/ground_truth/odom',Odometry,queue_size=100)
    pubscan=rospy.Publisher('/scan',LaserScan,queue_size=1)
    pubinitial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
    broadcaster=tf.TransformBroadcaster()
    def launch(script,args):
        log=open(folder+'/'+script+'.log','w')
        proc=subprocess.Popen([sys.executable,ROOT+'/src/smartcar_localization/scripts/'+script+'.py']+args,stdout=log,stderr=subprocess.STDOUT)
        children.append((proc,log))
    def wait(predicate,label,seconds=8):
        end=time.time()+seconds
        while time.time()<end:
            if predicate(): print('PASS '+label);return
            time.sleep(.05)
        raise AssertionError(label+': '+str(state)+' '+str(summary))
    def feed_sensors():
        counter=0
        while running[0]:
            stamp=rospy.Time.now();x=distance[0]
            # Simulate 5% scan-odometry longitudinal error. No true yaw is sent to estimator.
            broadcaster.sendTransform((x*1.05,0,0),(0,0,0,1),stamp,'base_footprint','odom')
            if feed[0]:
                truth=Odometry();truth.header.frame_id='sim_world';truth.header.stamp=stamp
                truth.pose.pose.position.x=x;truth.pose.pose.orientation.w=1.;pubtruth.publish(truth)
            if counter%5==0:
                scan=LaserScan();scan.header.frame_id='base_footprint';scan.header.stamp=stamp
                scan.range_min=.05;scan.range_max=12.;scan.angle_min=-math.pi;scan.angle_increment=math.pi/180.
                ranges=[]
                for i in range(360):
                    c,s=math.cos(scan.angle_min+i*scan.angle_increment),math.sin(scan.angle_min+i*scan.angle_increment)
                    values=[]
                    if s>1e-5: values.append(2.025/s)
                    if c>1e-5: values.append((4.025-x)/c)
                    ranges.append(min(values) if values else float('inf'))
                scan.ranges=ranges;pubscan.publish(scan)
            counter+=1;time.sleep(.02)
    thread=threading.Thread(target=feed_sensors);thread.daemon=True
    try:
        rospy.set_param('/wall_localizer/wheel_fusion',True)
        launch('wall_localizer',[])
        launch('wall_localizer',['__name:=lidar_baseline','_wheel_fusion:=false','_broadcast_tf:=false','_parameter_source:=/wall_localizer',
            '/wall_localization/status:=/lidar_baseline/status','/wall_localization/pose:=/lidar_baseline/pose'])
        launch('sim_wheel_pulses',[])
        launch('localization_evaluator',['_output:='+folder+'/evaluation.jsonl'])
        time.sleep(.7)
        grid=np.zeros((240,240),dtype=int);grid[160,:]=100;grid[:,200]=100
        m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=240;m.info.resolution=float(np.float32(.05))
        m.info.origin.position.x=m.info.origin.position.y=-6.;m.info.origin.orientation.w=1.;m.data=grid.ravel().tolist()
        key=fingerprint(m);pubmap.publish(m)
        selectors=dict(inside=[],outside=[dict(id=1,start=[-3,2.],end=[3.5,2.]),dict(id=2,start=[4,-3],end=[4,3])])
        groups=extract_groups(m,selectors)
        pubdef.publish(json.dumps(dict(schema_version=3,source='map_mask',frame_id='map',map_sha256=key,groups=groups)))
        pubregion.publish(json.dumps(dict(map_sha256=key,frame_id='map',corners=[[-3,-3],[-2,-3],[-2,-2],[-3,-2]])))
        thread.start();time.sleep(.7)
        initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.pose.pose.orientation.w=1.;pubinitial.publish(initial)
        wait(lambda:state.get('fused',{}).get('wheel_state')=='FUSED','wheel pulses enter estimator')
        wait(lambda:'lidar' in summary and summary['lidar']['samples']>4,'paired timestamp evaluation')
        assert list(edges)==['/wall_localizer'],edges
        print('PASS sole map TF publisher; baseline is shadow only')
        for i in range(25): distance[0]+=.002;time.sleep(.05)
        for i in range(25): distance[0]-=.002;time.sleep(.05)
        wait(lambda:summary.get('fused',{}).get('samples',0)>20,'forward and reverse pulse integration')
        feed[0]=False
        wait(lambda:state.get('fused',{}).get('wheel_state','').startswith('FALLBACK'),'pulse loss falls back explicitly')
        feed[0]=True
        wait(lambda:state.get('fused',{}).get('wheel_state')=='FUSED','fresh pulses restore fusion')
        assert os.path.getsize(folder+'/evaluation.jsonl')>0
        print('ALL ROS WHEEL TESTS PASSED')
    finally:
        running[0]=False
        for proc,log in children:
            proc.terminate();proc.wait();log.close()
        if sys.exc_info()[0]:
            for name in os.listdir(folder):
                if name.endswith('.log'):
                    with open(folder+'/'+name) as stream: print(name,stream.read()[-2000:])
        rospy.signal_shutdown('finished');shutil.rmtree(folder)

if __name__=='__main__': run()
