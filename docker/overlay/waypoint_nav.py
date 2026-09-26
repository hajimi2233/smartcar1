#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import csv
import heapq
import math
import os
import sys
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Occupancy(object):
    def __init__(self, msg, inflate_m):
        self.w = msg.info.width
        self.h = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        raw = msg.data
        occ = [False] * (self.w * self.h)
        r = max(1, int(math.ceil(inflate_m / self.res)))
        blocked = []
        for j in range(self.h):
            row = j * self.w
            for i in range(self.w):
                v = raw[row + i]
                if v >= 50:
                    occ[row + i] = True
                    blocked.append((i, j))
        extra = [False] * (self.w * self.h)
        for i, j in blocked:
            for dj in range(-r, r + 1):
                jj = j + dj
                if jj < 0 or jj >= self.h:
                    continue
                for di in range(-r, r + 1):
                    if di * di + dj * dj > r * r:
                        continue
                    ii = i + di
                    if 0 <= ii < self.w:
                        extra[jj * self.w + ii] = True
        self.occ = extra if extra else occ

    def world_to_cell(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def cell_to_world(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def in_bounds(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def blocked(self, i, j):
        return (not self.in_bounds(i, j)) or self.occ[j * self.w + i]

    def astar(self, start_xy, goal_xy):
        si, sj = self.world_to_cell(start_xy[0], start_xy[1])
        gi, gj = self.world_to_cell(goal_xy[0], goal_xy[1])
        if self.blocked(si, sj):
            for d in range(1, 6):
                found = False
                for di in range(-d, d + 1):
                    for dj in range(-d, d + 1):
                        if not self.blocked(si + di, sj + dj):
                            si, sj = si + di, sj + dj
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
        if self.blocked(gi, gj):
            for d in range(1, 8):
                found = False
                for di in range(-d, d + 1):
                    for dj in range(-d, d + 1):
                        if not self.blocked(gi + di, gj + dj):
                            gi, gj = gi + di, gj + dj
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
        if self.blocked(si, sj) or self.blocked(gi, gj):
            return None
        nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        h = lambda i, j: math.hypot(i - gi, j - gj)
        openh = [(h(si, sj), 0.0, si, sj)]
        came = {}
        gscore = {(si, sj): 0.0}
        seen = set()
        while openh:
            _, g, i, j = heapq.heappop(openh)
            if (i, j) in seen:
                continue
            seen.add((i, j))
            if i == gi and j == gj:
                cells = [(i, j)]
                while (i, j) in came:
                    i, j = came[(i, j)]
                    cells.append((i, j))
                cells.reverse()
                return [self.cell_to_world(a, b) for a, b in cells]
            for di, dj, c in nbrs:
                ni, nj = i + di, j + dj
                if self.blocked(ni, nj):
                    continue
                if di != 0 and dj != 0 and (self.blocked(i + di, j) or self.blocked(i, j + dj)):
                    continue
                ng = g + c
                if ng < gscore.get((ni, nj), 1e9):
                    gscore[(ni, nj)] = ng
                    came[(ni, nj)] = (i, j)
                    heapq.heappush(openh, (ng + h(ni, nj), ng, ni, nj))
        return None


class WaypointNav(object):
    def __init__(self):
        default_csv = os.path.expanduser('~/smartcar_2026_ws/data/maps/slam_current/points.csv')
        self.csv_path = rospy.get_param('~points', default_csv)
        self.frame = rospy.get_param('~frame_id', 'map')
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.wheelbase = rospy.get_param('~wheelbase', 0.62)
        self.length = rospy.get_param('~length', 0.80)
        self.width = rospy.get_param('~width', 0.55)
        self.max_delta = rospy.get_param('~max_steer', 0.55)
        self.v_max = rospy.get_param('~v_max', 0.16)
        self.goal_tol = rospy.get_param('~goal_tol', 0.12)
        self.inflate = rospy.get_param('~inflate', 0.30)
        self.points = self.load_points()
        self.grid = None
        self.goal = None
        self.goal_name = ''
        self.path_xy = []
        self.path_i = 0
        self.delta_f = 0.0
        self.goal_yaw = None
        self.amcl_pose = None
        self.listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher('/sim/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/nav/path', Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/nav/status', String, queue_size=1, latch=True)
        self.mark_pub = rospy.Publisher('/nav/goal_mark', Marker, queue_size=1, latch=True)
        self.body_pub = rospy.Publisher('/nav/robot_body', MarkerArray, queue_size=1)
        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.on_amcl, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_pose_goal)
        rospy.Subscriber('/navigate_to', String, self.on_name_goal)
        rospy.Subscriber('/nav/cancel', String, self.on_cancel)
        rospy.Timer(rospy.Duration(0.05), self.on_timer)
        self.status('idle: set 2D Pose Estimate first, then 2D Nav Goal or /navigate_to')
        print('Loaded %d waypoints from %s' % (len(self.points), self.csv_path))
        sys.stdout.flush()

    def load_points(self):
        out = {}
        if not os.path.isfile(self.csv_path):
            return out
        with open(self.csv_path) as f:
            for row in csv.DictReader(f):
                out[row['point_id']] = (float(row['x']), float(row['y']))
        return out

    def on_map(self, msg):
        self._last_map = msg
        try:
            self.grid = Occupancy(msg, self.inflate)
        except Exception as exc:
            rospy.logwarn('map inflate failed: %s', exc)

    def status(self, text):
        self.status_pub.publish(String(data=text))
        print(text)
        sys.stdout.flush()

    def on_amcl(self, msg):
        p = msg.pose.pose
        yaw = tf.transformations.euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
        self.amcl_pose = (p.position.x, p.position.y, yaw)

    def pose(self):
        if self.amcl_pose is not None:
            return self.amcl_pose
        self.listener.waitForTransform(self.frame, self.base, rospy.Time(0), rospy.Duration(0.08))
        trans, rot = self.listener.lookupTransform(self.frame, self.base, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return trans[0], trans[1], yaw

    def on_cancel(self, _msg):
        self.goal = None
        self.path_xy = []
        self.cmd_pub.publish(Twist())
        self.status('cancelled')

    def on_name_goal(self, msg):
        name = msg.data.strip()
        if name not in self.points:
            self.status('unknown point: %s' % name)
            return
        x, y = self.points[name]
        self.set_goal(x, y, name)

    def on_pose_goal(self, msg):
        try:
            pose = self.listener.transformPose(self.frame, msg)
            x, y = pose.pose.position.x, pose.pose.position.y
        except Exception:
            x, y = msg.pose.position.x, msg.pose.position.y
        self.set_goal(x, y, 'clicked')

    def set_goal(self, x, y, name):
        # Named points are front-axle targets; control point is base_footprint (0.31 m behind).
        try:
            cx, cy, yaw = self.pose()
        except Exception:
            self.status('no TF, cannot plan')
            return
        approach = math.atan2(y - cy, x - cx)
        fa = self.wheelbase / 2.0
        bx = x - fa * math.cos(approach)
        by = y - fa * math.sin(approach)
        self.goal = (bx, by)
        self.goal_name = name
        self.delta_f = 0.0
        path = None
        last = getattr(self, '_last_map', None)
        if self.grid is not None:
            path = self.grid.astar((cx, cy), (bx, by))
        if path is None and last is not None:
            path = Occupancy(last, 0.22).astar((cx, cy), (bx, by))
        if path is None:
            self.status('no collision-free path to %s' % name)
            self.goal = None
            return
        self.path_xy = self.simplify(path)
        self.path_i = 0
        if len(self.path_xy) >= 2:
            a, b = self.path_xy[-2], self.path_xy[-1]
            self.goal_yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        else:
            self.goal_yaw = math.atan2(by - cy, bx - cx)
        self.publish_path()
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = 'nav_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.12
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.22
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 1.0
        self.mark_pub.publish(m)
        self.status('going to %s  (%d path pts)' % (name, len(self.path_xy)))

    def simplify(self, pts):
        if len(pts) <= 2:
            return pts
        keep = [pts[0]]
        acc = 0.0
        for a, b in zip(pts, pts[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= 0.12:
                keep.append(b)
                acc = 0.0
        if keep[-1] != pts[-1]:
            keep.append(pts[-1])
        return keep

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = rospy.Time.now()
        for px, py in self.path_xy:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def lookahead(self, x, y):
        if not self.path_xy:
            return self.goal
        i = self.path_i
        while i < len(self.path_xy) - 1:
            px, py = self.path_xy[i]
            if math.hypot(px - x, py - y) > 0.20:
                break
            i += 1
        self.path_i = i
        target = self.path_xy[min(i + 2, len(self.path_xy) - 1)]
        return target

    def publish_body(self, x, y, yaw):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = 'robot_body'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.24
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x = self.length
        m.scale.y = self.width
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.45, 0.95, 0.55
        arr.markers.append(m)
        n = Marker()
        n.header = m.header
        n.ns = 'robot_body'
        n.id = 1
        n.type = Marker.ARROW
        n.action = Marker.ADD
        n.pose = m.pose
        n.pose.position.z = 0.32
        n.scale.x = 0.45
        n.scale.y = 0.06
        n.scale.z = 0.06
        n.color.r, n.color.g, n.color.b, n.color.a = 1.0, 0.8, 0.1, 1.0
        arr.markers.append(n)
        self.body_pub.publish(arr)

    def on_timer(self, _evt):
        try:
            x, y, yaw = self.pose()
            self.publish_body(x, y, yaw)
        except Exception:
            if self.goal is not None:
                self.cmd_pub.publish(Twist())
            return
        if self.goal is None:
            return
        gx, gy = self.goal
        dist = math.hypot(gx - x, gy - y)
        yaw_err = 0.0 if self.goal_yaw is None else wrap(self.goal_yaw - yaw)
        if dist < self.goal_tol and abs(yaw_err) < 0.18:
            self.goal = None
            self.path_xy = []
            self.cmd_pub.publish(Twist())
            self.status('arrived %s' % self.goal_name)
            return
        if dist < self.goal_tol and abs(yaw_err) >= 0.18:
            # Ackermann cannot spin in place: creep forward while steering to heading.
            delta = clamp(yaw_err, -self.max_delta, self.max_delta)
            self.delta_f = 0.65 * self.delta_f + 0.35 * delta
            v = 0.05 if abs(yaw_err) < 1.2 else 0.08
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = v * math.tan(self.delta_f) / self.wheelbase
            self.cmd_pub.publish(cmd)
            return
        lx, ly = self.lookahead(x, y)
        dx = lx - x
        dy = ly - y
        heading = math.atan2(dy, dx)
        err = wrap(heading - yaw)
        reverse = False
        if dist > 0.55 and abs(err) > 2.2:
            reverse = True
            err = wrap(err - math.pi if err > 0 else err + math.pi)
        if dist < 0.45:
            err = clamp(err, -0.35, 0.35)
        delta = clamp(err, -self.max_delta, self.max_delta)
        self.delta_f = 0.65 * self.delta_f + 0.35 * delta
        speed = min(self.v_max, 0.05 + 0.22 * dist)
        speed *= max(0.25, 1.0 - abs(self.delta_f) / self.max_delta)
        if dist < 0.40:
            speed = min(speed, 0.07)
        v = (-speed) if reverse else speed
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * math.tan(self.delta_f) / self.wheelbase
        self.cmd_pub.publish(cmd)


def main():
    rospy.init_node('waypoint_nav')
    WaypointNav()
    rospy.spin()


if __name__ == '__main__':
    main()
