#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only stage snapshot; checks connectivity/freshness, not driving accuracy."""
from __future__ import print_function
import time
import sys
import math
import yaml
import rospy
import rosgraph
import tf
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node('smartcar_stage_check', anonymous=True)
    stage = rospy.get_param('~stage', 'drivers')
    stages = ('drivers', 'mapping', 'localization', 'single', 'multi')
    if stage not in stages:
        raise ValueError('unknown stage: '+stage)
    profile = rospy.get_param('~profile', 'real')
    with open(rospy.get_param('~navigation_config')) as stream:
        config = yaml.safe_load(stream)
    tuning = config['tuning']
    scan_topic = '/scan'
    samples = {}
    def record(name):
        def callback(msg): samples[name] = (time.time(), msg)
        return callback
    subscriptions = [rospy.Subscriber(scan_topic,LaserScan,record('scan'),queue_size=1),
                     rospy.Subscriber(config['joint_topic'],JointState,record('joints'),queue_size=1)]
    if stage != 'drivers':
        subscriptions.append(rospy.Subscriber('/map',OccupancyGrid,record('map'),queue_size=1))
    listener = tf.TransformListener()
    deadline = time.time()+5.
    while time.time()<deadline and not rospy.is_shutdown(): time.sleep(.05)
    failures = []
    def check(ok, label):
        print(('OK   ' if ok else 'FAIL ')+label)
        if not ok: failures.append(label)
    for name in ('scan','joints'):
        sample = samples.get(name)
        age = (rospy.Time.now()-sample[1].header.stamp).to_sec() if sample else float('inf')
        check(sample is not None and time.time()-sample[0]<1. and -.05<=age<1., name+' fresh messages and timestamps')
    if 'scan' in samples:
        scan = samples['scan'][1]
        check(any(not math.isnan(v) and not math.isinf(v) and scan.range_min<=v<=scan.range_max for v in scan.ranges),'scan has valid ranges')
        check(listener.canTransform(tuning['base_frame'],scan.header.frame_id,rospy.Time(0)), 'base -> laser TF')
    if 'joints' in samples:
        joint=samples['joints'][1]
        required=('front_left_steer_joint','front_right_steer_joint','rear_left_wheel_joint','rear_right_wheel_joint')
        check(all(n in joint.name for n in required),'navigation joint feedback names')
    publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
    check(any(t==config['cmd_topic'] and nodes for t,nodes in subscribers),'chassis subscribes to '+config['cmd_topic'])
    model=rospy.get_param(config['actuator_model_param'],{})
    check(model.get('version')==1 and all(k in model for k in ('wheelbase','front_track','wheel_radius','acceleration','braking','steer_rate')), 'actuator model contract')
    check(all(abs(model.get(k,float('inf'))-tuning[k])<1e-6 for k in ('wheelbase','front_track','wheel_radius')), 'actuator geometry matches navigation config')
    check(bool(rospy.get_param('/use_sim_time',False))==(profile=='sim'),'clock mode matches '+profile)
    if stage!='drivers':
        check('map' in samples and samples['map'][1].info.width>0,'map available')
        check(listener.canTransform(tuning['odom_frame'],tuning['base_frame'],rospy.Time(0)),'odom -> base TF')
        check(listener.canTransform(tuning['map_frame'],tuning['base_frame'],rospy.Time(0)),'map -> base TF')
    nodes=set(n for _,ns in publishers+subscribers for n in ns)
    if stage in ('localization','single','multi'): check('/amcl' in nodes,'AMCL running')
    if stage=='mapping': check('/slam_gmapping' in nodes,'gmapping running')
    if stage in ('single','multi'): check('/single_goal_nav' in nodes,'single-goal executor running')
    if stage=='multi': check('/multi_goal_nav' in nodes,'mission queue running')
    print('Snapshot only: manually validate dimensions, scan alignment, motion and final pose.')
    return 1 if failures else 0


if __name__ == '__main__':
    try: sys.exit(main())
    except Exception as exc: sys.exit('Stage check failed: '+str(exc))
