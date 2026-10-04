#!/usr/bin/env python
"""Automated low-speed Gazebo experiment. Run ONLY in an isolated ROS container.

The explicit --isolated-sim flag is required because this test sends cmd_vel.
Initial pose is a fixed known reset pose, not a live ground-truth feedback input.
"""
from __future__ import print_function
import json
import math
import os
import signal
import subprocess
import sys
import time
import rospy
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,ROOT+'/src/smartcar_localization/scripts')
from map_wall_extraction import extract_groups
from wall_features import fingerprint,save_file


def run():
    if '--isolated-sim' not in sys.argv: raise RuntimeError('Requires --isolated-sim and an isolated ROS master')
    rospy.init_node('gazebo_wheel_experiment')
    output=os.environ.get('WHEEL_TEST_OUTPUT','/tmp/wheel_experiment')
    if not os.path.isdir(output): os.makedirs(output)
    control=rospy.Publisher('/sim/cmd_vel',Twist,queue_size=1)
    initial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
    latest={};summary={}
    rospy.Subscriber('/wall_localization/status',String,lambda m:latest.update(json.loads(m.data)))
    rospy.Subscriber('/localization_eval/summary',String,lambda m:summary.update(json.loads(m.data)))
    rospy.wait_for_service('/gazebo/set_model_state',timeout=40)
    m=rospy.wait_for_message('/geometry_map',OccupancyGrid,timeout=30);m.header.frame_id='map'
    key=fingerprint(m)
    selectors=dict(inside=[],outside=[dict(id=1,start=[-5.8,4],end=[5.8,4]),
        dict(id=2,start=[6,-3.8],end=[6,3.8]),dict(id=3,start=[-5.8,-4],end=[5.8,-4]),
        dict(id=4,start=[-6,-3.8],end=[-6,3.8])])
    parallel_only=os.environ.get('WHEEL_PARALLEL_ONLY')=='1'
    if parallel_only: selectors['outside']=selectors['outside'][:1]
    save_file(output+'/walls.json',extract_groups(m,selectors),key,'map',selectors)
    with open(output+'/region.json','w') as stream:
        json.dump(dict(schema_version=1,map_sha256=key,frame_id='map',corners=[[-2.5,-1.8],[2.5,-1.8],[2.5,1.8],[-2.5,1.8]]),stream)
    trials=[]
    def stage(seconds,speed=0.,steering=0.):
        start=rospy.Time.now().to_sec();wall_end=time.time()+seconds*4+10
        while rospy.Time.now().to_sec()-start<seconds and time.time()<wall_end and not rospy.is_shutdown():
            msg=Twist();msg.linear.x=speed;msg.angular.z=speed*math.tan(steering)/.62
            control.publish(msg);time.sleep(.04)
        control.publish(Twist())
    for label,bias in (('ideal',0.),('plus5',.05),('minus5',-.05)):
        nodes=[];files=[];latest.clear();summary.clear()
        try:
            stage(.5)
            reset=ModelState();reset.model_name='inspection_car';reset.reference_frame='world'
            reset.pose.position.x=3.86;reset.pose.position.y=2.8;reset.pose.position.z=.015
            reset.pose.orientation.z=1.;reset.pose.orientation.w=0.
            result=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(reset)
            if not result.success: raise RuntimeError(result.status_message)
            stage(1.)
            commands=[['roslaunch','smartcar_localization','localization.launch',
                'map_file:=/project/data/maps/sim_field_v1/map.yaml','params:=/project/config/localization.yaml',
                'nav_config:=/project/config/navigation.yaml','wall_features:=true','wheel_fusion:=true',
                'walls_file:='+output+'/walls.json','region_file:='+output+'/region.json'],
                ['roslaunch','smartcar_localization','wheel_experiment.launch','scale_error:='+str(bias),'output:='+output+'/'+label+'.jsonl']]
            for index,command in enumerate(commands):
                log=open(output+'/'+label+'_'+str(index)+'.log','w');files.append(log)
                nodes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
            stage(3.)
            p=PoseWithCovarianceStamped();p.header.frame_id='map'
            p.pose.pose.position.x=3.88;p.pose.pose.position.y=2.79;p.pose.pose.orientation.z=math.sin((math.pi+.005)/2)
            p.pose.pose.orientation.w=math.cos((math.pi+.005)/2);initial.publish(p)
            deadline=time.time()+10
            while time.time()<deadline and latest.get('wheel_state')!='FUSED': stage(.2)
            if latest.get('wheel_state')!='FUSED': raise RuntimeError('Fusion not ready: '+str(latest))
            schedule=[('stationary',3.,0.,0.),('forward',12. if parallel_only else 5.,.12,0.),('reverse',12. if parallel_only else 5.,-.12,0.),
                      ('left_arc',5.,.12,.4),('reverse_arc',5.,-.12,.4),('stop',3.,0.,0.)]
            phases=[]
            for name,seconds,speed,steering in schedule:
                print(label,name);sys.stdout.flush()
                start=rospy.Time.now().to_sec();stage(seconds,speed,steering)
                phases.append(dict(name=name,start=start,end=rospy.Time.now().to_sec()))
            if not summary: raise RuntimeError('No paired estimates')
            trials.append(dict(label=label,parallel_only=parallel_only,scale_error=bias,summary=dict(summary),phases=phases,last_status=dict(latest)))
            with open(output+'/results.json','w') as stream: json.dump(trials,stream,indent=2)
        finally:
            control.publish(Twist())
            for node in reversed(nodes): node.send_signal(signal.SIGINT)
            for node in nodes: node.wait()
            for log in files: log.close()
    print('GAZEBO WHEEL EXPERIMENT COMPLETE')

if __name__=='__main__': run()
