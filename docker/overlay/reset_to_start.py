#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Put the simulated car back at field start and publish an AMCL hint."""
from __future__ import print_function
import math
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


SPAWN_X = 3.86
SPAWN_Y = 2.80
SPAWN_Z = 0.08
SPAWN_YAW = math.pi


def main():
    rospy.init_node('reset_to_start')
    cmd = rospy.Publisher('/sim/cmd_vel', Twist, queue_size=1)
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.3)
    for _ in range(5):
        cmd.publish(Twist())
        rospy.sleep(0.05)

    rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    st = ModelState()
    st.model_name = 'inspection_car'
    st.reference_frame = 'world'
    st.pose.position.x = SPAWN_X
    st.pose.position.y = SPAWN_Y
    st.pose.position.z = SPAWN_Z
    st.pose.orientation.z = math.sin(SPAWN_YAW / 2.0)
    st.pose.orientation.w = math.cos(SPAWN_YAW / 2.0)
    st.twist = Twist()
    res = set_state(st)
    if not res.success:
        raise rospy.ROSException(res.status_message)

    rospy.sleep(0.2)
    ip = PoseWithCovarianceStamped()
    ip.header.stamp = rospy.Time.now()
    ip.header.frame_id = 'map'
    ip.pose.pose.position.x = SPAWN_X
    ip.pose.pose.position.y = SPAWN_Y
    ip.pose.pose.orientation.z = math.sin(SPAWN_YAW / 2.0)
    ip.pose.pose.orientation.w = math.cos(SPAWN_YAW / 2.0)
    ip.pose.covariance[0] = 0.02
    ip.pose.covariance[7] = 0.02
    ip.pose.covariance[35] = 0.01
    for _ in range(10):
        init_pub.publish(ip)
        rospy.sleep(0.08)
    print('Car reset to start (%.2f, %.2f), yaw 180 deg. Check RViz overlay.' % (SPAWN_X, SPAWN_Y))


if __name__ == '__main__':
    main()
