#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import math
import time
import rospy
import tf
from geometry_msgs.msg import PoseStamped


def main():
    parser=argparse.ArgumentParser(description='Map-frame front axle target: X Y YAW_DEGREES')
    parser.add_argument('x',type=float)
    parser.add_argument('y',type=float)
    parser.add_argument('yaw_degrees',type=float)
    args=parser.parse_args(rospy.myargv()[1:])
    if any(math.isnan(v) or math.isinf(v) for v in (args.x,args.y,args.yaw_degrees)):
        parser.error('coordinates and heading must be finite')
    rospy.init_node('send_single_goal',anonymous=True)
    pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
    deadline=time.time()+5
    while not rospy.is_shutdown() and not pub.get_num_connections():
        if time.time()>deadline:raise RuntimeError('no goal subscriber; start single-goal navigation first')
        time.sleep(.05)
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=rospy.Time(0)
    msg.pose.position.x=args.x;msg.pose.position.y=args.y
    q=tf.transformations.quaternion_from_euler(0,0,math.radians(args.yaw_degrees))
    msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=q
    pub.publish(msg);time.sleep(.5)
    print('Goal sent; watch /single_nav/status for acceptance and outcome.')

if __name__ == '__main__':main()
