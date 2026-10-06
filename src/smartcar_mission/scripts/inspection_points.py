#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Capture sixteen map positions using RViz Publish Point; never publish goals."""
from __future__ import print_function
import csv
import os
import sys
import tempfile
import math
import threading
import rospy
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from plan_io import POINT_NAMES


class Collector(object):
    def __init__(self, path, outer_only=False):
        self.path, self.points = path, []
        self.names = POINT_NAMES[10:14] if outer_only else POINT_NAMES
        self.preserved = {}
        if outer_only:
            with open(path) as stream:
                for row in csv.DictReader(stream):
                    name = row['point_id']
                    if name in POINT_NAMES[:10]+['start','end']:
                        if name in self.preserved: raise ValueError('Duplicate point: '+name)
                        xy = (float(row['x']),float(row['y']))
                        if any(math.isnan(v) or math.isinf(v) for v in xy): raise ValueError('Invalid point: '+name)
                        self.preserved[name] = xy
            if len(self.preserved) != 12: raise ValueError('Need 10 saved inspection points plus start/end')
        self.lock = threading.RLock()
        rospy.Service('~undo', Trigger, self.undo)
        rospy.Service('~save', Trigger, self.save)
        rospy.Subscriber('/clicked_point', PointStamped, self.click, queue_size=1)
        self.prompt()

    def prompt(self):
        if len(self.points) < len(self.names):
            rospy.loginfo('Click %d/%d: %s (map frame, front axle midpoint)', len(self.points)+1, len(self.names), self.names[len(self.points)])
        else:
            rospy.loginfo('Points ready. Run sim.sh points-save, or points-undo to correct the last point.')

    def click(self, msg):
        with self.lock:
            if msg.header.frame_id.lstrip('/') != 'map':
                rospy.logwarn('Set RViz Fixed Frame to map before marking points.'); return
            x, y = msg.point.x, msg.point.y
            if len(self.points) >= len(self.names) or any(math.isnan(v) or math.isinf(v) for v in (x, y)):
                return
            self.points.append((x, y))
            rospy.loginfo('%s = (%.4f, %.4f)', self.names[len(self.points)-1], x, y)
            self.prompt()

    def undo(self, _req):
        with self.lock:
            if self.points: self.points.pop()
            self.prompt()
            return TriggerResponse(True, 'last point removed')

    def save(self, _req):
        with self.lock:
            if len(self.points) != len(self.names):
                return TriggerResponse(False, 'Mark all requested points first')
            merged = dict(self.preserved)
            merged.update(zip(self.names, self.points))
            ordered = [merged[name] for name in POINT_NAMES]
            for c in range(5):
                a, b = ordered[2*c:2*c+2]
                if math.hypot(b[0]-a[0], b[1]-a[1]) < .01:
                    return TriggerResponse(False, 'Paired inspection points must be at least 1 cm apart')
            directory = os.path.dirname(self.path)
            try:
                if not os.path.isdir(directory): os.makedirs(directory)
                fd, tmp = tempfile.mkstemp(prefix='.points-', dir=directory)
                try:
                    with os.fdopen(fd, 'w') as stream:
                        writer = csv.writer(stream)
                        writer.writerow(['point_id', 'x', 'y'])
                        for name, p in zip(POINT_NAMES, ordered): writer.writerow([name]+list(p))
                        stream.flush(); os.fsync(stream.fileno())
                    if self.preserved:
                        import shutil
                        import time
                        shutil.copy2(self.path, self.path+'.backup-'+str(int(time.time()*1000)))
                    os.rename(tmp, self.path)
                finally:
                    if os.path.exists(tmp): os.unlink(tmp)
            except (IOError, OSError) as exc:
                return TriggerResponse(False, str(exc))
            return TriggerResponse(True, 'Saved '+self.path)


if __name__ == '__main__':
    args = rospy.myargv()
    if len(args) not in (2,3) or not os.path.isabs(args[1]) or (len(args)==3 and args[2]!='--outer-only'):
        sys.exit('Usage: inspection_points.py /absolute/points.csv')
    rospy.init_node('inspection_points')
    Collector(args[1], len(args)==3)
    rospy.spin()
