#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Select map points in click order and execute them through single_goal_nav."""
from __future__ import print_function
import math
import threading
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse


class MultiGoalNav(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame_id', 'map')
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.points = []  # (x, y, yaw), in click order
        self.active = False
        self.index = -1
        self.seen_active_status = False
        self.lock = threading.RLock()
        self.listener = tf.TransformListener()
        self.mark_pub = rospy.Publisher('/multi_nav/points', MarkerArray, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/single_nav/cancel', String, queue_size=1)
        self.status_pub = rospy.Publisher('/multi_nav/status', String, queue_size=1, latch=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal_click, queue_size=20)
        rospy.Subscriber('/single_nav/status', String, self.on_nav_status, queue_size=10)
        rospy.Service('~clear', Trigger, self.clear)
        rospy.Service('~undo', Trigger, self.undo)
        rospy.Service('~execute', Trigger, self.execute)
        rospy.Service('~cancel', Trigger, self.cancel)
        self.publish_marks()
        self.pending_goal_keys = set()
        self.status('READY: use RViz 2D Nav Goal in execution order; call /multi_goal_nav/execute')

    def status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo(text)

    def on_goal_click(self, msg):
        try:
            stamp = msg.header.stamp if msg.header.stamp else rospy.Time(0)
            if msg.header.frame_id.lstrip('/') == self.frame:
                transformed = msg
            else:
                self.listener.waitForTransform(self.frame, msg.header.frame_id, stamp, rospy.Duration(.8))
                transformed = self.listener.transformPose(self.frame, msg)
            q = transformed.pose.orientation
            yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            x, y = transformed.pose.position.x, transformed.pose.position.y
        except Exception as exc:
            self.status('REJECTED: click transform failed: %s' % exc)
            return
        with self.lock:
            key = (round(x, 5), round(y, 5), round(yaw, 5))
            if key in self.pending_goal_keys:
                self.pending_goal_keys.remove(key)
                return
            if self.active:
                self.status('REJECTED: queue is executing; cancel before editing')
                return
            # A 2D Nav Goal is also consumed by single_goal_nav. Stop it
            # immediately so selecting a point never starts driving.
            self.cancel_pub.publish(String(data='cancel'))
            self.points.append((x, y, yaw))
            number = len(self.points)
            self.publish_marks()
            self.status('POINT %d: (%.3f, %.3f, %.1fdeg)' %
                        (number, x, y, math.degrees(yaw)))

    def publish_marks(self):
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
        now = rospy.Time.now()
        for i, (x, y, yaw) in enumerate(self.points):
            m = Marker(); m.header.frame_id = self.frame; m.header.stamp = now
            m.ns = 'multi_nav_points'; m.id = i; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = .08
            m.pose.orientation.w = 1.; m.scale.x = m.scale.y = m.scale.z = .16
            m.color.r = 1.; m.color.g = .55; m.color.b = .05; m.color.a = 1.; arr.markers.append(m)
            t = Marker(); t.header = m.header; t.ns = 'multi_nav_numbers'; t.id = i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = .28
            t.pose.orientation.w = 1.; t.scale.z = .16
            t.color.r = t.color.g = t.color.b = 1.; t.color.a = 1.; t.text = str(i + 1)
            arr.markers.append(t)
        self.mark_pub.publish(arr)

    def clear(self, _req):
        with self.lock:
            if self.active:
                return TriggerResponse(False, 'cancel queue first')
            self.points = []; self.publish_marks()
        self.status('CLEARED')
        return TriggerResponse(True, 'points cleared')

    def undo(self, _req):
        with self.lock:
            if self.active: return TriggerResponse(False, 'cancel queue first')
            if not self.points: return TriggerResponse(False, 'no points')
            self.points.pop(); self.publish_marks()
        self.status('UNDO: %d points remain' % len(self.points))
        return TriggerResponse(True, 'last point removed')

    def execute(self, _req):
        with self.lock:
            if self.active: return TriggerResponse(False, 'already executing')
            if not self.points: return TriggerResponse(False, 'no points selected')
            self.active = True; self.index = 0; self.seen_active_status = False
            self.status('STARTING: %d points' % len(self.points))
            self.publish_current()
        return TriggerResponse(True, 'queue started')

    def publish_current(self):
        i = self.index; x, y, yaw = self.points[i]
        msg = PoseStamped(); msg.header.frame_id = self.frame; msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x; msg.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0., 0., yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.pending_goal_keys.add((round(x, 5), round(y, 5), round(yaw, 5)))
        self.goal_pub.publish(msg)
        self.status('GOAL %d/%d: (%.3f, %.3f, %.1fdeg)' %
                    (i + 1, len(self.points), x, y, math.degrees(yaw)))

    def on_nav_status(self, msg):
        text = msg.data
        with self.lock:
            if not self.active: return
            if text.startswith(('GOAL_MODE:', 'WAIT_STOP:', 'PLANNING', 'DRIVING:')):
                self.seen_active_status = True
            if text.startswith(('PLAN_FAILED:', 'REJECTED:', 'STOPPED:', 'CANCELLED',
                                'FINAL_TOLERANCE_FAILED:', 'REPLAN_LIMIT:')):
                self.active = False
                self.status('FAILED at goal %d/%d: %s' % (self.index + 1, len(self.points), text))
                self.clear_queue()
                return
            if not text.startswith('SUCCEEDED:') or not self.seen_active_status:
                return
            if self.index + 1 >= len(self.points):
                self.active = False
                self.status('SUCCEEDED: all %d points' % len(self.points))
                self.clear_queue()
                return
            self.index += 1; self.seen_active_status = False
            self.publish_current()

    def cancel(self, _req):
        with self.lock:
            if not self.active: return TriggerResponse(False, 'queue is idle')
            self.active = False
            self.cancel_pub.publish(String(data='cancel'))
            self.clear_queue()
        self.status('CANCELLED: queue stopped at point %d' % (self.index + 1))
        return TriggerResponse(True, 'queue cancelled')

    def clear_queue(self):
        """Forget the completed or aborted run before the next execution."""
        self.points = []
        self.index = -1
        self.pending_goal_keys = set()
        self.publish_marks()


if __name__ == '__main__':
    rospy.init_node('multi_goal_nav')
    MultiGoalNav()
    rospy.spin()
