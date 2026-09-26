#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Normalize the simulation IMU topic for the radar+IMU frontend.

There is no wheel odometry in this project.  This bridge never reads
/sim/ground_truth/odom and never invents an absolute orientation.  The real
vehicle can bypass this node and publish the same output contract directly.
"""
from __future__ import print_function

import rospy
from sensor_msgs.msg import Imu


class ImuBridge(object):
    def __init__(self):
        input_topic = rospy.get_param('~input_topic', '/sim/imu')
        output_topic = rospy.get_param('~output_topic', '/imu/data')
        self.pub = rospy.Publisher(output_topic, Imu, queue_size=50)
        rospy.Subscriber(input_topic, Imu, self.on_imu, queue_size=50)
        rospy.loginfo('imu_bridge: %s -> %s (no ground-truth and no wheel odometry)',
                      input_topic, output_topic)

    def on_imu(self, msg):
        out = msg
        if not out.header.frame_id:
            out.header.frame_id = 'imu_link'
        self.pub.publish(out)


def main():
    rospy.init_node('imu_bridge')
    ImuBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
