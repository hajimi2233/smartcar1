#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Small Gaussian range noise on the lidar, like a real 2D scanner."""
from __future__ import print_function
import math
import random
import rospy
from sensor_msgs.msg import LaserScan


class LaserNoise(object):
    def __init__(self):
        self.std = rospy.get_param('~stddev', 0.02)
        if rospy.get_param('/sensors/ideal', False):
            self.std = 0.0
            rospy.loginfo('laser_noise IDEAL: /scan passthrough of /sim/scan')
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        rospy.Subscriber('/sim/scan', LaserScan, self.on_scan, queue_size=1)

    def on_scan(self, msg):
        if self.std <= 0.0:
            msg.header.frame_id = msg.header.frame_id or 'laser'
            self.pub.publish(msg)
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        noisy = []
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                noisy.append(r)
                continue
            v = r + random.gauss(0.0, self.std)
            if v < msg.range_min:
                v = msg.range_min
            if v > msg.range_max:
                v = msg.range_max
            noisy.append(v)
        out.ranges = noisy
        if msg.intensities:
            out.intensities = msg.intensities
        self.pub.publish(out)


def main():
    rospy.init_node('laser_noise')
    LaserNoise()
    rospy.spin()


if __name__ == '__main__':
    main()
