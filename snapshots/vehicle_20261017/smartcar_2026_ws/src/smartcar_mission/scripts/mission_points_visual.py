#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Show all sixteen configured front-axle points; publish no navigation goal."""
import csv,sys
import rospy
from visualization_msgs.msg import Marker,MarkerArray
from plan_io import POINT_NAMES

def main():
    args=rospy.myargv()
    if len(args)!=2:raise ValueError('Usage: mission_points_visual.py /absolute/points.csv')
    with open(args[1]) as f:points={row['point_id']:(float(row['x']),float(row['y'])) for row in csv.DictReader(f)}
    if set(points)!=set(POINT_NAMES):raise ValueError('Expected exactly sixteen known point names')
    rospy.init_node('mission_points_visual')
    pub=rospy.Publisher('/mission/points16',MarkerArray,queue_size=1,latch=True)
    arr=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
    for number,name in enumerate(POINT_NAMES):
        x,y=points[name];dot=Marker();dot.header.frame_id='map';dot.ns='points16';dot.id=number;dot.type=Marker.SPHERE;dot.action=Marker.ADD
        dot.pose.position.x=x;dot.pose.position.y=y;dot.pose.position.z=.12;dot.pose.orientation.w=1.
        dot.scale.x=dot.scale.y=dot.scale.z=.12;dot.color.r=1.;dot.color.g=.25;dot.color.a=1.
        arr.markers.append(dot)
        label=Marker();label.header.frame_id='map';label.ns='points16_names';label.id=number;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
        label.pose.position.x=x+.12;label.pose.position.y=y;label.pose.position.z=.22;label.pose.orientation.w=1.
        label.scale.z=.14;label.color.r=label.color.g=label.color.b=label.color.a=1.;label.text='%d %s'%(number+1,name)
        arr.markers.append(label)
    rate=rospy.Rate(1.)
    while not rospy.is_shutdown():pub.publish(arr);rate.sleep()

if __name__=='__main__':main()
