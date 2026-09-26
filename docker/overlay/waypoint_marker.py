#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import sys
import rospy
import tf
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse

SUGGESTED = [
    'start',
    'col1_top_entry', 'col2_top_entry', 'col3_top_entry', 'col4_top_entry', 'col5_top_entry',
    'col1_bottom_entry', 'col2_bottom_entry', 'col3_bottom_entry', 'col4_bottom_entry', 'col5_bottom_entry',
    'inspect_1', 'inspect_2', 'inspect_3', 'inspect_4', 'inspect_5',
    'inspect_6', 'inspect_7', 'inspect_8', 'inspect_9', 'inspect_10',
    'map_left_top', 'map_left_bottom', 'map_right_top', 'map_right_bottom',
    'end',
]


class WaypointMarker(object):
    def __init__(self):
        default_dir = os.path.expanduser('~/smartcar_2026_ws/data/maps/slam_current')
        self.out_dir = rospy.get_param('~output_dir', default_dir)
        self.frame = rospy.get_param('~frame_id', 'map')
        self.points = []
        self.skipped = set()
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/waypoint_marks', MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber('/clicked_point', PointStamped, self.on_click, queue_size=1)
        rospy.Service('~undo', Trigger, self.on_undo)
        rospy.Service('~skip', Trigger, self.on_skip)
        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)
        self.load_existing()
        self.publish_marks()
        self.print_status()

    def used_names(self):
        return set(n for n, _, _ in self.points) | self.skipped

    def next_name(self):
        used = self.used_names()
        for name in SUGGESTED:
            if name not in used:
                return name
        return 'point_%d' % (len(self.points) + 1)

    def load_existing(self):
        path = os.path.join(self.out_dir, 'points.csv')
        if not os.path.isfile(path):
            return
        with open(path) as f:
            lines = f.read().splitlines()
        for line in lines[1:]:
            if not line.strip():
                continue
            parts = line.split(',')
            if len(parts) < 3:
                continue
            self.points.append((parts[0], float(parts[1]), float(parts[2])))
        if self.points:
            print('Loaded %d existing points from %s' % (len(self.points), path))

    def save(self):
        csv_path = os.path.join(self.out_dir, 'points.csv')
        with open(csv_path, 'w') as f:
            f.write('point_id,x,y\n')
            for name, x, y in self.points:
                f.write('%s,%.6f,%.6f\n' % (name, x, y))
        info_path = os.path.join(self.out_dir, 'map_info.txt')
        with open(info_path, 'w') as f:
            f.write('map_id=slam_current\n')
            f.write('map_version=slam_v1\n')
            f.write('frame_id=%s\n' % self.frame)
            f.write('units=m\n')
            f.write('target_reference=front_axle_midpoint\n')
        print('Saved %d points -> %s' % (len(self.points), csv_path))
        sys.stdout.flush()

    def publish_marks(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        now = rospy.Time.now()
        for i, (name, x, y) in enumerate(self.points):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.08
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r = 1.0
            m.color.g = 0.85
            m.color.b = 0.1
            m.color.a = 1.0
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns = 'waypoint_names'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = 0.28
            t.pose.orientation.w = 1.0
            t.scale.z = 0.16
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = name
            arr.markers.append(t)
        self.pub.publish(arr)

    def print_status(self):
        nxt = self.next_name()
        print('')
        print('Marked %d / %d' % (len(self.points), len(SUGGESTED)))
        if nxt:
            print('Next click name: %s' % nxt)
        print('RViz toolbar: Publish Point, then click the map (front-axle target).')
        print('Undo: rosservice call /waypoint_marker/undo')
        print('Skip: rosservice call /waypoint_marker/skip')
        if self.points:
            last = self.points[-1]
            print('Last: %s  x=%.3f y=%.3f' % last)
        sys.stdout.flush()

    def on_click(self, msg):
        try:
            stamp = msg.header.stamp if msg.header.stamp else rospy.Time(0)
            self.listener.waitForTransform(self.frame, msg.header.frame_id, stamp, rospy.Duration(0.8))
            pt = self.listener.transformPoint(self.frame, msg)
            x, y = pt.point.x, pt.point.y
        except Exception as exc:
            rospy.logwarn('TF to %s failed (%s), using click frame %s', self.frame, exc, msg.header.frame_id)
            x, y = msg.point.x, msg.point.y
        name = self.next_name()
        self.points = [(n, px, py) for n, px, py in self.points if n != name]
        self.points.append((name, x, y))
        order = {n: i for i, n in enumerate(SUGGESTED)}
        self.points.sort(key=lambda item: (order.get(item[0], 1000), item[0]))
        self.save()
        self.publish_marks()
        self.print_status()

    def on_undo(self, _req):
        if not self.points:
            return TriggerResponse(success=False, message='no points')
        removed = self.points.pop()
        self.save()
        self.publish_marks()
        self.print_status()
        return TriggerResponse(success=True, message='removed %s' % removed[0])

    def on_skip(self, _req):
        name = self.next_name()
        if name.startswith('point_'):
            return TriggerResponse(success=False, message='nothing to skip')
        self.skipped.add(name)
        self.print_status()
        return TriggerResponse(success=True, message='skipped %s' % name)


def main():
    rospy.init_node('waypoint_marker')
    WaypointMarker()
    rospy.spin()


if __name__ == '__main__':
    main()
