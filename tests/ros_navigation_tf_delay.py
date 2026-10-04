#!/usr/bin/env python
"""Real TF regression, run only in an isolated ROS master. Sends no motion."""
from __future__ import print_function
import sys,time,math
sys.path.insert(0,'/home/hajimi/smartcar_ws/src/smartcar_navigation/scripts')
import rospy,tf
from single_goal_nav import Navigator
from nav_config import to_rear
import numpy as np
rospy.init_node('navigation_tf_delay_test')
n=Navigator.__new__(Navigator);n.ground_truth_test=False;n.tf=tf.TransformListener()
b=tf.TransformBroadcaster();publish_map=[True]
def broadcast(event):
    now=rospy.Time.now()
    b.sendTransform((3,4,0),tf.transformations.quaternion_from_euler(0,0,.2),now-rospy.Duration(.034),'base_footprint','odom')
    if publish_map[0]:
        b.sendTransform((1,2,0),tf.transformations.quaternion_from_euler(0,0,math.pi/2),now-rospy.Duration(.134),'odom','map')
timer=rospy.Timer(rospy.Duration(.02),broadcast)
try:
    time.sleep(1.)
    for i in range(20):
        np.testing.assert_allclose(n.pose(),to_rear((-3,5,math.pi/2+.2),'base'),atol=1e-6)
        time.sleep(.025)
    print('PASS current odometry composed with 100 ms delayed map correction')
    publish_map[0]=False;time.sleep(.5)
    try:n.pose();raise AssertionError('Stale map accepted')
    except RuntimeError as e:assert 'map localization TF stale' in str(e),str(e)
    print('PASS map timeout despite continuing fresh odometry')
finally:
    timer.shutdown();rospy.signal_shutdown('done')
