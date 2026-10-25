#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Select map points in click order and execute them through single_goal_nav."""
from __future__ import print_function
import math
import json
import threading
import time
from patrol_preview import PatrolPreview
from inspection_reporting import load_reports, cached_audio, ArrivalSpeaker
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse
from plan_io import load_plan, load_regions, b_lateral_goals
from mission_goal_policy import is_end_goal


class MultiGoalNav(object):
    def __init__(self):
        self.frame = rospy.get_param('/single_goal_nav/tuning/map_frame', rospy.get_param('~frame_id', 'map'))
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.points = []  # (x, y, yaw), in click order or C plan order
        self.labels = []
        self.inspection_regions = []
        self.failed = False
        plan_file = rospy.get_param('~plan_file', '')
        if plan_file:
            reference = rospy.get_param('/single_goal_nav/tuning/goal_reference', 'front_axle')
            if reference != 'front_axle':
                raise ValueError('Inspection plans require goal_reference=front_axle')
            self.points, labels = load_plan(plan_file, self.frame)
            self.labels = labels
            self.inspection_regions = load_regions(plan_file)
            b_lateral=bool(rospy.get_param('~b_lateral_test',False))
            if b_lateral:
                self.points,self.labels=b_lateral_goals(self.points,self.labels,self.inspection_regions)
                rospy.loginfo('B lateral test: B1 -> B3 -> B5 -> B7 -> B9; upper entrance, no exit goal')
            limit=int(rospy.get_param('~inspection_limit',0))
            if not 0<=limit<=10:raise ValueError('inspection_limit must be 0..10')
            if b_lateral and limit:raise ValueError('Do not combine B lateral test and inspection_limit')
            if limit:
                count=0
                for stop,label in enumerate(self.labels):
                    if label.startswith('inspect_'):count+=1
                    if count==limit:break
                else:raise ValueError('plan has fewer inspection targets than requested')
                self.points=self.points[:stop+1];self.labels=self.labels[:stop+1]
                rospy.loginfo('Partial test: stop after %d inspection points; no exit goal',limit)
            rospy.loginfo('Execution plan loaded: %s', ' -> '.join(self.labels))
        self.reports = load_reports(plan_file) if plan_file else {}
        self.speech_pub = rospy.Publisher('/inspection/speech_event', String, queue_size=32)
        workspace = rospy.get_param('~speech_workspace', '/home/smartcar/smartcar_2026_ws')
        self.speaker = ArrivalSpeaker(cached_audio(workspace) if self.reports else {}, self.speech_event,
                                     bool(rospy.get_param('~speech_enabled', True)))
        rospy.on_shutdown(self.speaker.close)
        self.active = False
        self.index = -1
        self.seen_active_status = False
        self.lock = threading.RLock()
        self.patrol_preview=PatrolPreview(self.reports)
        self.patrol_context_pub=rospy.Publisher('/inspection/patrol_context',String,queue_size=10,latch=True)
        self.patrol_stage_sub=rospy.Subscriber('/single_nav/patrol_stage',String,self.on_patrol_stage,queue_size=32)
        self.patrol_context_timer=rospy.Timer(rospy.Duration(.5),self.publish_patrol_context)
        rospy.on_shutdown(self.close_patrol_preview)
        self.listener = tf.TransformListener()
        self.mark_pub = rospy.Publisher('/multi_nav/points', MarkerArray, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.inspection_goal_pub = rospy.Publisher('/single_nav/inspection_goal', String, queue_size=1)
        self.cancel_pub = rospy.Publisher('/single_nav/cancel', String, queue_size=1)
        self.status_pub = rospy.Publisher('/multi_nav/status', String, queue_size=1, latch=True)
        rospy.Subscriber('/multi_nav/goal', PoseStamped, self.on_goal_click, queue_size=20)
        rospy.Subscriber('/single_nav/status', String, self.on_nav_status, queue_size=10)
        rospy.Service('~clear', Trigger, self.clear)
        rospy.Service('~undo', Trigger, self.undo)
        rospy.Service('~execute', Trigger, self.execute)
        rospy.Service('~cancel', Trigger, self.cancel)
        self.publish_marks()
        self.status('READY: %d goals; call /multi_goal_nav/execute to start' % len(self.points))

    def publish_patrol_context(self, _event=None):
        try:
            with self.lock:
                self.patrol_context_pub.publish(String(data=json.dumps(self.patrol_preview.snapshot(time.time()))))
        except Exception as exc:rospy.logwarn_throttle(5.,'Patrol preview metadata unavailable: '+str(exc))

    def on_patrol_stage(self, msg):
        try:event=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:
            if self.patrol_preview.stage(event):self.publish_patrol_context()

    def close_patrol_preview(self):
        try:
            with self.lock:
                self.patrol_preview.clear('SHUTDOWN');self.publish_patrol_context()
        except Exception:pass

    def speech_event(self, event):
        try:
            self.speech_pub.publish(String(data=json.dumps(event, ensure_ascii=True)))
            rospy.loginfo('Inspection audio: %s', json.dumps(event, ensure_ascii=True))
        except Exception as exc:
            rospy.logwarn('Inspection audio event failed: %s', exc)

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
            if self.active:
                self.status('REJECTED: queue is executing; cancel before editing')
                return
            # Dedicated selection topic; execution starts only via execute service.
            self.points.append((x, y, yaw))
            self.labels.append('point_%d' % len(self.points))
            self.failed = False
            number = len(self.points)
            self.publish_marks()
            self.status('POINT %d: (%.3f, %.3f, %.1fdeg)' %
                        (number, x, y, math.degrees(yaw)))

    def publish_marks(self):
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
        now = rospy.Time.now()
        for i, (x, y, yaw) in enumerate(self.points):
            current = i == (self.index if self.index >= 0 else 0)
            following = i == (self.index+1 if self.index >= 0 else 1)
            color = ((1., .15, .15) if self.failed and current else
                     (1., .85, .05) if current else
                     (.1, .9, 1.) if following else (.6, .6, .6))
            m = Marker(); m.header.frame_id = self.frame; m.header.stamp = now
            m.ns = 'multi_nav_points'; m.id = i; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = .08
            m.pose.orientation.w = 1.; m.scale.x = m.scale.y = m.scale.z = .16
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.; arr.markers.append(m)
            arrow = Marker(); arrow.header = m.header
            arrow.ns = 'multi_nav_headings'; arrow.id = i
            arrow.type = Marker.ARROW; arrow.action = Marker.ADD
            arrow.pose.position.x = x; arrow.pose.position.y = y; arrow.pose.position.z = .12
            q = tf.transformations.quaternion_from_euler(0., 0., yaw)
            arrow.pose.orientation.x, arrow.pose.orientation.y = q[0], q[1]
            arrow.pose.orientation.z, arrow.pose.orientation.w = q[2], q[3]
            arrow.scale.x = .65 if current else .4
            arrow.scale.y = .07; arrow.scale.z = .10
            arrow.color.r, arrow.color.g, arrow.color.b = color
            arrow.color.a = 1.
            arr.markers.append(arrow)
            t = Marker(); t.header = m.header; t.ns = 'multi_nav_numbers'; t.id = i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = .28
            t.pose.orientation.w = 1.; t.scale.z = .16
            t.color.r = t.color.g = t.color.b = 1.; t.color.a = 1.
            label = self.labels[i] if i < len(self.labels) else 'point_%d' % (i+1)
            state = 'FAILED' if self.failed and current else 'TARGET' if current else 'NEXT' if following else ''
            t.text = '%d %s %s\n(%.2f, %.2f) %.1f deg' % (i+1, label, state, x, y, math.degrees(yaw))
            arr.markers.append(t)
        self.mark_pub.publish(arr)

    def clear(self, _req):
        with self.lock:
            if self.active:
                return TriggerResponse(False, 'cancel queue first')
            self.points = []; self.labels = []; self.failed = False; self.index = -1; self.publish_marks()
        self.status('CLEARED')
        return TriggerResponse(True, 'points cleared')

    def undo(self, _req):
        with self.lock:
            if self.active: return TriggerResponse(False, 'cancel queue first')
            if not self.points: return TriggerResponse(False, 'no points')
            self.points.pop()
            if self.labels: self.labels.pop()
            self.failed = False; self.index = -1; self.publish_marks()
        self.status('UNDO: %d points remain' % len(self.points))
        return TriggerResponse(True, 'last point removed')

    def execute(self, _req):
        with self.lock:
            if self.active: return TriggerResponse(False, 'already executing')
            if not self.points: return TriggerResponse(False, 'no points selected')
            self.speaker.begin_run()
            self.patrol_preview.begin()
            self.active = True; self.index = 0; self.seen_active_status = False
            self.failed = False
            self.status('STARTING: %d points' % len(self.points))
            self.publish_current()
        return TriggerResponse(True, 'queue started')

    def publish_current(self):
        self.publish_marks()
        i = self.index; x, y, yaw = self.points[i]
        # Fixed final pose: parallel to low-cost lines, toward arena right.
        if is_end_goal(self.labels,i):yaw=math.radians(-163.5)
        label=self.labels[i] if i<len(self.labels) else ''
        preview_token=self.patrol_preview.goal(label)
        self.publish_patrol_context()
        msg = PoseStamped(); msg.header.frame_id = self.frame; msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x; msg.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0., 0., yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        end_goal=is_end_goal(self.labels,i)
        position_only=False
        if self.inspection_regions or end_goal:
            label = self.labels[i] if i < len(self.labels) else ''
            self.inspection_goal_pub.publish(String(data=json.dumps(dict(
                frame_id=self.frame, x=x, y=y, yaw=yaw,
                target_id=label if label.startswith('inspect_') else None,
                position_only=position_only,preview_token=preview_token,
                regions=self.inspection_regions))))
        else:
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
                self.failed = True
                self.patrol_preview.clear('FAILED');self.publish_patrol_context()
                self.publish_marks()
                self.status('FAILED at goal %d/%d: %s' % (self.index + 1, len(self.points), text))
                return
            if not text.startswith('SUCCEEDED:') or not self.seen_active_status:
                return
            label = self.labels[self.index] if self.index < len(self.labels) else ''
            self.patrol_preview.arrived(label);self.publish_patrol_context()
            if label in self.reports:
                self.speaker.enqueue(label, self.reports[label])
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
            self.speaker.begin_run()
            self.clear_queue()
        self.status('CANCELLED: queue stopped at point %d' % (self.index + 1))
        return TriggerResponse(True, 'queue cancelled')

    def clear_queue(self):
        """Forget the completed or aborted run before the next execution."""
        self.patrol_preview.clear();self.publish_patrol_context()
        self.points = []
        self.labels = []
        self.failed = False
        self.index = -1
        self.publish_marks()


if __name__ == '__main__':
    rospy.init_node('multi_goal_nav')
    MultiGoalNav()
    rospy.spin()
