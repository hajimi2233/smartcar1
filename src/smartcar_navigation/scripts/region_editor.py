#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Four-click preview, explicit commit, atomic map-bound persistence."""
import os
import json
import threading
import rospy
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from region_geometry import validate, fingerprint


class Editor(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.path = rospy.get_param('~file')
        self.map_id = None
        self.active, self.draft = [], []
        self.editing = False
        self.poly = rospy.Publisher('/external_region/polygon', PolygonStamped, queue_size=1, latch=True)
        self.marks = rospy.Publisher('/external_region/markers', MarkerArray, queue_size=1, latch=True)
        self.status = rospy.Publisher('/external_region/status', String, queue_size=1, latch=True)
        for action in ('begin', 'undo', 'save', 'cancel', 'clear'):
            rospy.Service('~'+action, Trigger, lambda req, a=action: self.action(a))
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/external_region/corner', PointStamped, self.click, queue_size=10)
        self.publish()

    def report(self, text):
        self.status.publish(text)
        rospy.loginfo(text)

    def on_map(self, msg):
        key = fingerprint(msg)
        with self.lock:
            if key == self.map_id:
                return
            self.map_id = key
            self.active, self.draft, self.editing = [], [], False
            try:
                with open(self.path) as f:
                    data = json.load(f)
                if data['map_sha256'] != key or data['frame_id'] != 'map':
                    raise ValueError('Saved region belongs to a different map; redraw it')
                self.active = validate(data['corners'])
                self.report('Saved external region loaded')
            except Exception as exc:
                self.report('No valid region: %s' % exc)
            self.publish()

    def action(self, action):
        with self.lock:
            try:
                if action == 'begin':
                    self.draft, self.editing = [], True
                elif action == 'undo':
                    if self.draft: self.draft.pop()
                elif action == 'cancel':
                    self.draft, self.editing = [], False
                elif action == 'clear':
                    if os.path.exists(self.path): os.unlink(self.path)
                    self.active, self.draft, self.editing = [], [], False
                elif action == 'save':
                    if not self.editing or not self.map_id:
                        raise ValueError('Begin editing and wait for map first')
                    validate(self.draft)
                    data = dict(schema_version=1, frame_id='map', map_sha256=self.map_id, corners=self.draft)
                    directory = os.path.dirname(self.path)
                    if not os.path.isdir(directory): os.makedirs(directory)
                    with open(self.path+'.tmp', 'w') as f:
                        json.dump(data, f, indent=2); f.flush(); os.fsync(f.fileno())
                    os.rename(self.path+'.tmp', self.path)
                    self.active, self.draft, self.editing = list(self.draft), [], False
                self.publish()
                message = '%s: %d draft corners; %d saved corners' % (action, len(self.draft), len(self.active))
                self.report(message)
                return TriggerResponse(True, message)
            except Exception as exc:
                self.report(str(exc))
                return TriggerResponse(False, str(exc))

    def click(self, msg):
        with self.lock:
            if not self.editing:
                self.report('Run: bash scripts/smartcar.sh region begin'); return
            if msg.header.frame_id.lstrip('/') != 'map':
                self.report('Set RViz Fixed Frame to map'); return
            if len(self.draft) >= 4:
                self.report('Four corners selected; save, undo or begin again'); return
            self.draft.append([msg.point.x, msg.point.y])
            self.publish()
            self.report('Draft corner %d/4; save explicitly to apply' % len(self.draft))

    def publish(self):
        poly = PolygonStamped(); poly.header.frame_id = 'map'
        poly.polygon.points = [Point32(x,y,0) for x,y in self.active]
        self.poly.publish(poly)
        markers = []
        for ident, points, draft in ((0,self.active,False),(1,self.draft,True)):
            m = Marker(); m.header.frame_id='map'; m.ns='external_region'; m.id=ident
            m.pose.orientation.w=1; m.type=Marker.LINE_STRIP; m.scale.x=.025
            m.color.r=1; m.color.g=1 if draft else .25; m.color.a=1
            m.action=Marker.ADD if points else Marker.DELETE
            m.points=[Point(x,y,.035) for x,y in points+(points[:1] if len(points)==4 else [])]
            markers.append(m)
            fill = Marker(); fill.header.frame_id='map'; fill.ns=m.ns; fill.id=ident+2
            fill.pose.orientation.w=1; fill.type=Marker.TRIANGLE_LIST
            fill.scale.x=fill.scale.y=fill.scale.z=1
            fill.color=m.color; fill.color.a=.16
            fill.action=Marker.DELETE
            if len(points)==4:
                try:
                    validate(points)
                    fill.action=Marker.ADD
                    fill.points=[Point(points[i][0],points[i][1],.02) for i in (0,1,2,0,2,3)]
                except ValueError: pass
            markers.append(fill)
        self.marks.publish(MarkerArray(markers))


if __name__ == '__main__':
    rospy.init_node('region_editor')
    Editor()
    rospy.spin()
