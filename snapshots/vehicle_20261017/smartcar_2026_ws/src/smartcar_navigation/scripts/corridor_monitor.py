#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only corridor state from the committed editor polygon and fresh TF."""
import threading
import rospy
import tf
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_config import configure, P
from region_geometry import validate
from corridor_geometry import classify


class CorridorMonitor(object):
    def __init__(self):
        configure(rospy.get_param('~tuning', {}))
        if P['map_frame'] != 'map':
            raise ValueError('Region editor requires map_frame=map')
        self.lock = threading.RLock()
        self.polygon = []
        self.listener = tf.TransformListener()
        self.state = rospy.Publisher('/corridor/state', String, queue_size=1, latch=True)
        self.reason = rospy.Publisher('/corridor/reason', String, queue_size=1, latch=True)
        self.marker = rospy.Publisher('/corridor/marker', Marker, queue_size=1, latch=True)
        self.last = None
        rospy.Subscriber('/external_region/polygon', PolygonStamped, self.on_polygon, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick)

    def on_polygon(self, msg):
        with self.lock:
            self.polygon = []
            if msg.header.frame_id.lstrip('/') != P['map_frame']: return
            try:
                self.polygon = validate([(p.x,p.y) for p in msg.polygon.points])
            except ValueError:
                pass

    def tick(self, event):
        with self.lock:
            state, reason, xyz = 'UNKNOWN', 'No committed region for current map', None
            try:
                if self.polygon:
                    stamp = self.listener.getLatestCommonTime(P['map_frame'], P['base_frame'])
                    age = (rospy.Time.now()-stamp).to_sec()
                    if stamp.to_sec() == 0 or age < 0 or age > P['pose_timeout']:
                        raise ValueError('Missing or stale localization TF')
                    xyz, quat = self.listener.lookupTransform(P['map_frame'], P['base_frame'], stamp)
                    pose = (xyz[0], xyz[1], tf.transformations.euler_from_quaternion(quat)[2])
                    state = classify(self.polygon, pose)
                    reason = 'Reference point: '+P['base_frame']
            except (tf.Exception, ValueError) as exc:
                state, reason = 'UNKNOWN', str(exc)
            self.state.publish(state)
            self.reason.publish(reason)
            if (state, reason) != self.last:
                rospy.loginfo('Corridor: %s (%s)', state, reason)
                self.last = (state, reason)
            m = Marker(); m.header.frame_id = P['map_frame']; m.header.stamp = rospy.Time.now()
            m.ns = 'corridor_state'; m.id = 0; m.type = Marker.TEXT_VIEW_FACING
            m.pose.orientation.w = 1.; m.pose.position.z = 1.
            if xyz is not None: m.pose.position.x, m.pose.position.y = xyz[:2]
            m.scale.z = .22; m.color.a = 1.; m.text = 'Corridor: '+state
            m.color.r, m.color.g, m.color.b = {
                'INSIDE': (0.,1.,0.), 'OUTSIDE': (0.,.7,1.),
                'UNKNOWN': (1.,.2,.2)}[state]
            self.marker.publish(m)


if __name__ == '__main__':
    rospy.init_node('corridor_monitor')
    CorridorMonitor()
    rospy.spin()
