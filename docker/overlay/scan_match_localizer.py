#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Lidar-only pose on the prior occupancy map.

No /odom topic, no IMU. 2D Pose Estimate seeds the first pose.
Search uses a distance field (fast); the winner is accepted only if
ray-range score is not worse than the previous pose (avoids wall-snap).
TF is map→odom (identity) + odom→base_footprint, published on a
side thread with current sim time so RViz RobotModel keeps a live tree.
"""
from __future__ import print_function
import math
import threading
from collections import deque
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_of(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def quat_yaw(yaw):
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


class DistField(object):
    def __init__(self, grid):
        self.w = grid.info.width
        self.h = grid.info.height
        self.res = grid.info.resolution
        self.ox = grid.info.origin.position.x
        self.oy = grid.info.origin.position.y
        inf = 1e6
        dist = [inf] * (self.w * self.h)
        occ = bytearray(self.w * self.h)
        q = deque()
        for j in range(self.h):
            row = j * self.w
            for i in range(self.w):
                v = grid.data[row + i]
                if v >= 50:
                    dist[row + i] = 0.0
                    occ[row + i] = 1
                    q.append((i, j))
                elif v < 0:
                    occ[row + i] = 2
        while q:
            i, j = q.popleft()
            d0 = dist[j * self.w + i]
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if ni < 0 or nj < 0 or ni >= self.w or nj >= self.h:
                    continue
                nd = d0 + self.res
                idx = nj * self.w + ni
                if nd < dist[idx]:
                    dist[idx] = nd
                    q.append((ni, nj))
        self.dist = dist
        self.occ = occ

    def lookup(self, x, y):
        i = int(math.floor((x - self.ox) / self.res))
        j = int(math.floor((y - self.oy) / self.res))
        if i < 0 or j < 0 or i >= self.w or j >= self.h:
            return 2.0
        return self.dist[j * self.w + i]

    def ray(self, x, y, ang, zmax):
        res = self.res
        w, h = self.w, self.h
        ox, oy = self.ox, self.oy
        occ = self.occ
        dx = math.cos(ang)
        dy = math.sin(ang)
        step = res * 2.0
        n = int(zmax / step)
        if n < 1:
            n = 1
        for k in range(1, n + 1):
            r = k * step
            i = int(math.floor((x + dx * r - ox) / res))
            j = int(math.floor((y + dy * r - oy) / res))
            if i < 0 or j < 0 or i >= w or j >= h:
                return r
            if occ[j * w + i]:
                return r
        return zmax


class ScanMatchLocalizer(object):
    def __init__(self):
        self.sigma = rospy.get_param('~sigma', 0.08)
        self.min_score = rospy.get_param('~min_score', 0.30)
        self.xy_win = rospy.get_param('~xy_win', 0.18)
        self.xy_step = rospy.get_param('~xy_step', 0.06)
        self.yaw_win = rospy.get_param('~yaw_win', 0.12)
        self.yaw_step = rospy.get_param('~yaw_step', 0.04)
        self.stride = int(rospy.get_param('~stride', 8))
        self.max_xy = rospy.get_param('~max_xy', 0.15)
        self.max_yaw = rospy.get_param('~max_yaw', 0.14)
        self.freeze_xy = rospy.get_param('~freeze_xy', 0.02)
        self.freeze_yaw = rospy.get_param('~freeze_yaw', 0.015)
        self.v_max = rospy.get_param('~v_max', 0.40)
        self.w_max = rospy.get_param('~w_max', 0.60)
        self.field = None
        self.pose = None
        self.stamp = None
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.lock = threading.Lock()
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('/lidar_pose', PoseWithCovarianceStamped, queue_size=1)
        self.pub_rviz = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.on_human, queue_size=1)
        self._tf_thread = threading.Thread(target=self._tf_loop)
        self._tf_thread.daemon = True
        self._tf_thread.start()
        rospy.loginfo('scan_match LIDAR-ONLY: fast dist-field search, ray accept, TF thread')

    def _tf_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._broadcast_tf()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _broadcast_tf(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        now = rospy.Time.now()
        if now.to_sec() == 0:
            return
        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        self.br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), now, 'odom', 'map')
        self.br.sendTransform((pose[0], pose[1], 0.0), q, now, 'base_footprint', 'odom')

    def on_map(self, msg):
        self.field = DistField(msg)
        rospy.loginfo('scan_match map %dx%d res=%.3f', self.field.w, self.field.h, self.field.res)

    def on_human(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        with self.lock:
            self.pose = pose
            self.stamp = msg.header.stamp
            self.vx = self.vy = self.wz = 0.0
        self._publish_pose(pose, msg.header.stamp)
        self._broadcast_tf()
        rospy.loginfo('scan_match locked to 2D Pose Estimate')

    def on_scan(self, scan):
        if self.field is None:
            return
        with self.lock:
            pose0 = self.pose
            stamp0 = self.stamp
            vx, vy, wz = self.vx, self.vy, self.wz
        if pose0 is None:
            return
        pts, rays = self._beams(scan)
        if len(pts) < 16:
            return
        zmax = min(scan.range_max, 12.0)
        dt = 0.1
        if stamp0 is not None:
            dt = (scan.header.stamp - stamp0).to_sec()
            if dt <= 0.0 or dt > 0.5:
                dt = 0.1
        pred = (pose0[0] + vx * dt, pose0[1] + vy * dt, wrap(pose0[2] + wz * dt))
        cand, _dscore = self._search_field(pred, pts)
        dx = cand[0] - pred[0]
        dy = cand[1] - pred[1]
        dyaw = wrap(cand[2] - pred[2])
        if math.hypot(dx, dy) > self.max_xy or abs(dyaw) > self.max_yaw:
            cand = pred
        ray_pred = self._ray_score(pred, rays, zmax)
        ray_cand = self._ray_score(cand, rays, zmax)
        if ray_cand + 0.02 >= ray_pred and ray_cand >= self.min_score:
            pose = cand
        elif ray_pred >= self.min_score:
            pose = pred
        else:
            pose = pose0
        if (math.hypot(pose[0] - pose0[0], pose[1] - pose0[1]) < self.freeze_xy
                and abs(wrap(pose[2] - pose0[2])) < self.freeze_yaw):
            pose = pose0
            vx = vy = wz = 0.0
        elif dt > 1e-3:
            vx = max(-self.v_max, min(self.v_max, (pose[0] - pose0[0]) / dt))
            vy = max(-self.v_max, min(self.v_max, (pose[1] - pose0[1]) / dt))
            wz = max(-self.w_max, min(self.w_max, wrap(pose[2] - pose0[2]) / dt))
        with self.lock:
            self.pose = pose
            self.stamp = scan.header.stamp
            self.vx, self.vy, self.wz = vx, vy, wz
        self._publish_pose(pose, scan.header.stamp)

    def _beams(self, scan):
        pts = []
        rays = []
        a = scan.angle_min
        n = 0
        zlim = min(scan.range_max, 12.0)
        for r in scan.ranges:
            if n % self.stride == 0:
                if not math.isnan(r) and not math.isinf(r) and scan.range_min < r < zlim:
                    pts.append((r * math.cos(a), r * math.sin(a)))
                    rays.append((a, r))
            a += scan.angle_increment
            n += 1
        return pts, rays

    def _field_score(self, pose, pts):
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        acc = 0.0
        two_s = 2.0 * self.sigma * self.sigma
        field = self.field
        for lx, ly in pts:
            d = field.lookup(x0 + c * lx - s * ly, y0 + s * lx + c * ly)
            acc += math.exp(-(d * d) / two_s)
        return acc / float(len(pts))

    def _ray_score(self, pose, rays, zmax):
        x0, y0, yaw = pose
        acc = 0.0
        inv = 1.0 / (self.sigma * self.sigma)
        field = self.field
        for a, z in rays:
            zstar = field.ray(x0, y0, yaw + a, zmax)
            dz = z - zstar
            acc += math.exp(-0.5 * dz * dz * inv)
        return acc / float(len(rays))

    def _search_field(self, pred, pts):
        best_pose = pred
        best = self._field_score(pred, pts)
        yaw = pred[2] - self.yaw_win
        while yaw <= pred[2] + self.yaw_win + 1e-9:
            x = pred[0] - self.xy_win
            while x <= pred[0] + self.xy_win + 1e-9:
                y = pred[1] - self.xy_win
                while y <= pred[1] + self.xy_win + 1e-9:
                    pose = (x, y, wrap(yaw))
                    sc = self._field_score(pose, pts)
                    if sc > best:
                        best = sc
                        best_pose = pose
                    y += self.xy_step
                x += self.xy_step
            yaw += self.yaw_step
        return best_pose, best

    def _publish_pose(self, pose, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation = quat_yaw(pose[2])
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.01
        self.pub.publish(msg)
        self.pub_rviz.publish(msg)


def main():
    rospy.init_node('scan_match_localizer')
    ScanMatchLocalizer()
    rospy.spin()


if __name__ == '__main__':
    main()
