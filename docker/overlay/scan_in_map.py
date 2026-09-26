#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Project the current lidar scan into the prior map frame using AMCL pose.
This is the live cloud that should overlay the pre-built occupancy map."""
from __future__ import print_function
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import struct


class ScanInMap(object):
    def __init__(self):
        self.pose = None
        self.pub = rospy.Publisher('/scan_in_map', PointCloud2, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.on_pose, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)

    def on_pose(self, msg):
        p = msg.pose.pose
        yaw = tf.transformations.euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
        self.pose = (p.position.x, p.position.y, yaw)

    def on_scan(self, msg):
        if self.pose is None:
            return
        x0, y0, yaw = self.pose
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r) and msg.range_min < r < msg.range_max:
                ang = yaw + a
                pts.append((x0 + r * math.cos(ang), y0 + r * math.sin(ang), 0.05))
            a += msg.angle_increment
        self.pub.publish(self._cloud(msg.header.stamp, pts))

    @staticmethod
    def _cloud(stamp, pts):
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id='map')
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        buf = b''.join(struct.pack('<fff', p[0], p[1], p[2]) for p in pts)
        msg.data = buf
        msg.is_dense = True
        return msg


def main():
    rospy.init_node('scan_in_map')
    ScanInMap()
    rospy.spin()


if __name__ == '__main__':
    main()
