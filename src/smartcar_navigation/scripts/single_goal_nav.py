#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simulation-only single front-axle pose navigation. Python 2 / ROS Kinetic."""
from __future__ import print_function, division
import math
import os
import threading
import time
from collections import deque
import rospy
import rosgraph
from nav_config import P, configure, from_rear, to_rear
import tf
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from local_planner import choose, explain, guarded_track
from actuator_model import decode_joints
from path_tracking import project, motion_observed, replay_command
from ground_truth_pose import alignment, rear_pose
from low_cost_lines import LineStore, map_key
from nav_obstacles import smooth_ranges, ConfirmedHits, Recovery, ConsecutiveFailures, obstacle_key
from ackermann_core import Grid, plan, plan_line_approach, line_retreat_target, segments, speed_profile, rear_target, front_position, wrap, finite, StopWindow


class Navigator(object):
    def __init__(self):
        configure(rospy.get_param('~tuning', {}))
        rospy.set_param('~effective_tuning', dict(P))
        self.lock = threading.RLock()
        self.tf = tf.TransformListener()
        self.ground_truth_test = bool(rospy.get_param('~ground_truth_test', False))
        self.test_replay_speed_scale = float(rospy.get_param('~test_replay_speed_scale', 2.0))
        self.test_auto_arrive = bool(rospy.get_param('~test_auto_arrive', False))
        self.test_auto_arrive_delay = float(rospy.get_param('~test_auto_arrive_delay', 1.0))
        if self.test_auto_arrive_delay < 0 or not finite([self.test_auto_arrive_delay]):
            raise ValueError('test_auto_arrive_delay must be finite and nonnegative')
        if not finite([self.test_replay_speed_scale]) or not 0 < self.test_replay_speed_scale <= 4.:
            raise ValueError('test_replay_speed_scale must be within (0, 4]')
        self.truth_sample = None
        self.truth_samples = deque(maxlen=30)
        self.truth_alignment = None
        self.path_errors = []
        self.test_arrival_deadline = None
        self.test_pose_override = None
        self.master = rosgraph.Master(rospy.get_name())
        self.radius = float(rospy.get_param('~turn_radius', 1.3))
        if not finite([self.radius]) or self.radius < P['global_min_radius']:
            raise ValueError('turn_radius must be finite and >= tuning/global_min_radius')
        self.local_radius = float(rospy.get_param('~local_turn_radius', 1.1))
        self.steering_rate = float(rospy.get_param('~steering_rate', 1.0))
        self.tracking_options = dict(lateral_gain=float(rospy.get_param('~tracking_lateral_gain',6.0)),
                                     heading_gain=float(rospy.get_param('~tracking_heading_gain',4.0)),
                                     preview_distance=float(rospy.get_param('~tracking_preview_distance',.35)))
        if not finite(list(self.tracking_options.values())) or min(self.tracking_options.values()) <= 0:
            raise ValueError('tracking gains and preview distance must be finite and positive')
        if (not finite([self.local_radius, self.steering_rate])
                or not P['physical_min_radius'] <= self.local_radius <= self.radius or self.steering_rate <= 0):
            raise ValueError('physical_min_radius <= local_turn_radius <= turn_radius; steering_rate > 0')
        self.actuator = rospy.get_param(rospy.get_param('~actuator_model_param', '/sim/actuator_model'), {})
        if self.actuator.get('version') != 1:
            raise ValueError('updated simulation plugin required: rebuild and restart simulation')
        required = ('wheelbase', 'front_track', 'wheel_radius', 'acceleration', 'braking', 'steer_rate')
        if any(key not in self.actuator for key in required):
            raise ValueError('incomplete simulation actuator model')
        if not finite([self.actuator[key] for key in required]) or any(self.actuator[key] <= 0 for key in required):
            raise ValueError('invalid simulation actuator model')
        for key, setting in (('wheelbase', 'wheelbase'), ('front_track', 'front_track'), ('wheel_radius', 'wheel_radius')):
            if abs(self.actuator[key] - P[setting]) > 1e-6:
                raise ValueError('actuator model and tuning disagree: ' + key)
        self.joint_feedback = None
        self.last_control_stamp = None
        self.margin = float(rospy.get_param('~collision_margin', .04))
        self.planning_margin = float(rospy.get_param('~planning_collision_margin', .04))
        if not finite([self.planning_margin]) or self.planning_margin < self.margin:
            raise ValueError('planning_collision_margin must be >= collision_margin')
        self.local_guard_distance = float(rospy.get_param('~local_guard_distance', .10))
        if not finite([self.local_guard_distance]) or self.local_guard_distance < 0:
            raise ValueError('local_guard_distance must be finite and nonnegative')
        self.zero_cost_width = float(rospy.get_param('~zero_cost_line_width', .01))
        self.line_store=LineStore(rospy.get_param('~low_cost_lines_file',os.path.join(
            os.environ.get('SMARTCAR_ROOT',os.path.expanduser('~/.ros/smartcar')),'data','navigation','low_cost_lines.json')))
        self.line_map_key=None
        line = [float(rospy.get_param('~zero_cost_line_x1', float('nan'))), float(rospy.get_param('~zero_cost_line_y1', float('nan'))), float(rospy.get_param('~zero_cost_line_x2', float('nan'))), float(rospy.get_param('~zero_cost_line_y2', float('nan')))]
        self.zero_cost_line = [((line[0],line[1]),(line[2],line[3]))] if finite(line) else []
        # Disk records are map-bound. Do not restore unbound ROS parameters
        # left by another map/session; the first map callback loads the record.
        self.timeout = float(rospy.get_param('~planning_timeout', 5.))
        self.scan_timeout = float(rospy.get_param('~scan_timeout', 1.0))
        self.goal_position_tolerance = float(rospy.get_param('~goal_position_tolerance', .08))
        self.goal_heading_tolerance = math.radians(float(rospy.get_param('~goal_heading_tolerance_deg', 3.0)))
        if (not finite([self.margin,self.timeout,self.scan_timeout,self.goal_position_tolerance,self.goal_heading_tolerance])
                or self.margin < .02 or not 1 <= self.timeout <= 60
                or self.scan_timeout < .4 or self.goal_position_tolerance <= 0 or self.goal_heading_tolerance <= 0):
            raise ValueError('collision_margin >= .02; planning_timeout within 1..60 seconds')
        self.grid, self.scan, self.goal = None, None, None
        self._pending_line_point = None
        self.execution_heading = None
        self.line_stage = None
        self.line_route = None
        self.stage_target = None
        self.failures = ConsecutiveFailures()
        self.recovery = None
        self.wait_reason = ''
        self.replan_count = 0
        self.obstacle_stamp = None
        self.scan_received = 0.
        self.scans = deque(maxlen=8)
        self.parts, self.part, self.index = [], 0, 0
        self.speed_caps = []
        self.generation = 0
        self.state = 'IDLE'
        self.stop_window = StopWindow()
        self.stop_state = None
        self.stop_started = 0.
        self.last_pose, self.last_pose_wall = None, 0.
        self.last_movement = time.time()
        self.still_since, self.pause_until = None, 0.
        self.progress_time = time.time()
        self.goal_started = time.time()
        self.last_authority_check, self.authority_ok = 0., False
        self.stop_event = threading.Event()
        self.cmd_topic = rospy.resolve_name(rospy.get_param('~cmd_topic', '/sim/cmd_vel'))
        self.cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.local_pub = rospy.Publisher('/single_nav/local_path', Path, queue_size=1)
        self.line_marker_pub = rospy.Publisher('/single_nav/zero_cost_line', Marker, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/single_nav/path', Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/single_nav/status', String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher('/single_nav/result', String, queue_size=1, latch=True)
        if self.ground_truth_test:
            self.error_pub = rospy.Publisher('/single_nav/path_error', String, queue_size=1)
            self.virtual_pose_pub = rospy.Publisher('/single_nav/virtual_pose', PoseStamped, queue_size=1, latch=True)
            self.virtual_marker_pub = rospy.Publisher('/single_nav/virtual_marker', Marker, queue_size=1, latch=True)
            rospy.Subscriber('/sim/ground_truth/odom', Odometry, self.on_truth, queue_size=1)
        rospy.Subscriber('/single_nav/line_point', PointStamped, self.on_line_point, queue_size=20)
        rospy.Service('/single_nav/clear_zero_cost_line', Trigger, self.clear_zero_cost_line)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber(rospy.get_param('~joint_topic', '/sim/joint_states'), JointState, self.on_joints, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber('/single_nav/cancel', String, self.on_cancel, queue_size=1)
        self.publish_line_markers()
        rospy.on_shutdown(self.shutdown)
        self.status('IDLE: %s; RViz 2D Nav Goal (configured goal reference)' %
                    ('GAZEBO GROUND TRUTH TEST, initial map alignment required' if self.ground_truth_test
                     else 'set initial pose'))
        # Wall-time loop: ROS /clock pausing must not suspend the stop watchdog.
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    def status(self, value):
        msg = String(data=value)
        self.status_pub.publish(msg)
        # Keep terminal outcomes available to late subscribers and RViz panels.
        if value.startswith(('SUCCEEDED:', 'STOPPED:', 'CANCELLED',
                             'FINAL_TOLERANCE_FAILED:', 'REPLAN_LIMIT:',
                             'REPLAN_FAILED:', 'TEST_FAILED:')):
            self.result_pub.publish(msg)
        rospy.loginfo(value)

    def on_joints(self, msg):
        # Atomic snapshot; callbacks must not wait for a long planning tick.
        try:
            steer, speed = decode_joints(msg.name, msg.position, msg.velocity,
                                        self.actuator['wheelbase'], self.actuator['front_track'],
                                        self.actuator['wheel_radius'])
            self.joint_feedback = (msg.header.stamp.to_sec(), time.time(), steer, speed)
        except (ValueError, KeyError, IndexError, TypeError):
            self.joint_feedback = None

    def on_truth(self, msg):
        if msg.header.frame_id.lstrip('/') != P['truth_frame'] or msg.child_frame_id.lstrip('/') != P['base_frame']:
            return
        q = msg.pose.pose.orientation
        try:
            roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
            p = msg.pose.pose.position
            if not finite((p.x,p.y,p.z,q.x,q.y,q.z,q.w,yaw)) or abs(roll)+abs(pitch) > .1:
                return
            self.truth_sample = (msg.header.stamp, time.time(), (p.x,p.y,yaw))
            self.truth_samples.append(self.truth_sample)
        except (ValueError, TypeError):
            return

    def current_truth(self):
        sample = self.truth_sample
        if sample is None or time.time()-sample[1] > P['truth_timeout']:
            raise RuntimeError('Gazebo ground truth missing/stale')
        age = (rospy.Time.now()-sample[0]).to_sec()
        if not -.02 <= age <= P['truth_timeout']:
            raise RuntimeError('Gazebo ground truth simulation timestamp stale')
        return sample

    def calibrate_truth(self):
        self.current_truth()
        stamp = self.tf.getLatestCommonTime(P['map_frame'],P['base_frame'])
        if not 0 <= (rospy.Time.now()-stamp).to_sec() <= .35:
            raise RuntimeError('map localization stale during ground truth alignment')
        sample = min(self.truth_samples, key=lambda item: abs((item[0]-stamp).to_sec()))
        if abs((sample[0]-stamp).to_sec()) > .10:
            raise RuntimeError('map localization and Gazebo truth are not time-aligned')
        world_base = sample[2]
        xyz, q = self.tf.lookupTransform(P['map_frame'],P['base_frame'],stamp)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        self.truth_alignment = alignment((xyz[0],xyz[1],yaw), world_base)
        self.path_errors = []
        self.status('GROUND_TRUTH_TEST: frozen map alignment x=%.3f y=%.3f yaw=%.2fdeg; initial localization error remains' %
                    (self.truth_alignment[0],self.truth_alignment[1],math.degrees(self.truth_alignment[2])))

    def actuator_state(self):
        sample = self.joint_feedback
        now = rospy.Time.now().to_sec()
        # JointState and /clock travel independently; allow one clock delivery skew.
        if (sample is None or not -.02 <= now-sample[0] <= P['feedback_timeout']
                or not 0 <= time.time()-sample[1] <= P['feedback_timeout']):
            raise RuntimeError('joint feedback missing/stale; navigation stopped')
        return sample[2], sample[3]

    def local_candidate(self, p, part, diagnostics=None, cap=None):
        steer, speed = self.actuator_state()
        mode=getattr(self,'maneuver_mode','NORMAL')
        wall_steer=None
        if mode=='STRAIGHT' and self.scan is not None:
            def side_clearance(lo,hi):
                vals=[v for i,v in enumerate(self.scan.ranges)
                      if lo<=self.scan.angle_min+i*self.scan.angle_increment<=hi
                      and finite([v]) and self.scan.range_min<=v<min(self.scan.range_max,2.)]
                return sorted(vals)[len(vals)//2] if vals else None
            left,right=side_clearance(.55,1.57),side_clearance(-1.57,-.55)
            if left is not None and right is not None:
                imbalance=max(-.35,min(.35,left-right))
                limit=math.radians(P['wall_max_angle_deg'])
                # Reverse motion needs the opposite steering sign to move away
                # from the same side wall.
                direction=part[-1][3] if part else 1
                wall_steer=direction*max(-limit,min(limit,imbalance*P['wall_gain']))
        controller = guarded_track if getattr(self, 'local_guard_distance', 0.) > 0 else choose
        options = dict(self.tracking_options)
        if controller is guarded_track:
            options['guard_distance'] = self.local_guard_distance
        candidate = controller(self.grid,p,part,self.index,self.local_radius,steer,diagnostics,cap,
                           control_dt=self.control_dt, current_speed=speed,
                           steering_rate=self.steering_rate, actuator_steer_rate=self.actuator['steer_rate'],
                           acceleration=self.actuator['acceleration'], braking=self.actuator['braking'],
                           maneuver_mode=mode,wall_steer=wall_steer,wall_weight=P['wall_straight_weight'] if wall_steer is not None else 0.,
                           **options)
        # Execute exactly the candidate checked by the actuator rollout and
        # collision tests. Post-hoc wall blending changes curvature without
        # validating its swept path and can prevent steering from ramping up.
        return candidate

    def active_speed_cap(self):
        if not self.speed_caps:
            return None
        profile = self.speed_caps[self.part]
        gi = min(self.index,len(profile)-1)
        cap = profile[gi]
        if cap <= 1e-4:
            # A zero-speed cusp anchor has already passed stop verification.
            for future in profile[gi:min(len(profile),gi+8)]:
                if future > 1e-4:
                    return future
        return cap

    def halt(self, reason):
        if self.ground_truth_test and self.path_errors:
            rms = math.sqrt(sum(e*e for e in self.path_errors)/len(self.path_errors))
            self.status('PATH_ERROR: samples=%d rms=%.3fm max=%.3fm' %
                        (len(self.path_errors), rms, max(self.path_errors)))
            self.path_errors = []
        self.generation += 1
        self.parts, self.goal = [], None
        self.execution_heading = None
        self.line_stage = None
        self.line_route = None
        self.stage_target = None
        self.truth_alignment = None
        self.recovery = None
        self.replan_count = 0
        self.failures = ConsecutiveFailures()
        self.state = 'IDLE'
        self.stop_state = None
        self.cmd.publish(Twist())
        if self.ground_truth_test:
            self.publish_virtual_pose(None)
        empty = Path(); empty.header.frame_id = P['map_frame']; empty.header.stamp = rospy.Time.now()
        self.path_pub.publish(empty)
        self.status(reason)

    def publish_virtual_pose(self, pose):
        """Publish the planning-only pose used by nav-test, for RViz inspection."""
        if not self.ground_truth_test:
            return
        marker = Marker(); marker.header.frame_id = P['map_frame']; marker.header.stamp = rospy.Time.now()
        marker.ns = 'single_nav_virtual_pose'; marker.id = 0
        if pose is None:
            marker.action = Marker.DELETE
            self.virtual_marker_pub.publish(marker)
            return
        msg = PoseStamped(); msg.header = marker.header
        msg.pose.position.x, msg.pose.position.y = from_rear(pose, 'path')[:2]
        q = tf.transformations.quaternion_from_euler(0., 0., pose[2])
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.virtual_pose_pub.publish(msg)
        marker.type = Marker.ARROW; marker.action = Marker.ADD
        marker.pose = msg.pose; marker.scale.x = .65; marker.scale.y = .12; marker.scale.z = .12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, 1., .9, .95
        self.virtual_marker_pub.publish(marker)

    def on_cancel(self, _msg):
        with self.lock:
            self.halt('CANCELLED')

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            self.generation += 1
            self.cmd.publish(Twist())

    def clear_zero_cost_line(self, _req):
        with self.lock:
            try:self.line_store.save(self.line_map_key,[])
            except (IOError,OSError,ValueError) as exc:return TriggerResponse(False,'Lines not cleared: '+str(exc))
            self.zero_cost_line = []
            rospy.set_param('/single_nav/zero_cost_lines', [])
            self._pending_line_point = None
            self.publish_line_markers()
            self.status('ZERO_COST_LINE: all lines and pending start cleared; applies to next plan')
        return TriggerResponse(True, 'zero-cost line cleared')

    def on_line_point(self, msg):
        if msg.header.frame_id.lstrip('/') != P['map_frame']: return
        point=(msg.point.x,msg.point.y)
        if not finite(point):
            self.status('ZERO_COST_LINE: rejected nonfinite point'); return
        with self.lock:
            if self.line_map_key is None:
                self.status('ZERO_COST_LINE: wait for map before drawing');return
            pending = self._pending_line_point
            if pending is None:
                self._pending_line_point=point
                self.publish_line_markers()
                self.status('ZERO_COST_LINE: start selected; click endpoint for line %d' % (len(self.zero_cost_line)+1))
                return
            if math.hypot(point[0]-pending[0],point[1]-pending[1]) < .001:
                self.status('ZERO_COST_LINE: endpoint too close; choose another endpoint'); return
            lines=self.zero_cost_line+[(pending,point)]
            try:self.line_store.save(self.line_map_key,lines)
            except (IOError,OSError,ValueError) as exc:
                self.status('ZERO_COST_LINE: save failed; endpoint not committed: '+str(exc));return
            self.zero_cost_line=lines; self._pending_line_point=None
            rospy.set_param('/single_nav/zero_cost_lines', self.zero_cost_line)
            self.publish_line_markers()
            self.status('ZERO_COST_LINE: %d lines saved; click next start/end pair; applies to next plan' % len(self.zero_cost_line))

    def publish_line_markers(self):
        # One latched message contains every line and the pending-start cross.
        # A newly opened RViz receives the complete drawing, not only the last line.
        marker=Marker(); marker.header.frame_id=P['map_frame']; marker.ns='zero_cost_line'; marker.id=0
        marker.type=Marker.LINE_LIST; marker.action=Marker.ADD; marker.scale.x=.01
        marker.color.r=.1; marker.color.g=.9; marker.color.b=1.; marker.color.a=1.
        marker.points=[]
        for line in self.zero_cost_line:
            for x,y in line: marker.points.append(Point(x,y,.04))
        if self._pending_line_point is not None:
            x,y=self._pending_line_point
            marker.points.extend([Point(x-.04,y,.04),Point(x+.04,y,.04),
                                  Point(x,y-.04,.04),Point(x,y+.04,.04)])
        if not marker.points: marker.action=Marker.DELETE
        self.line_marker_pub.publish(marker)

    def on_map(self, msg):
        if msg.header.frame_id.lstrip('/') != P['map_frame']:
            rospy.logerr('Map must use frame map'); return
        q = msg.info.origin.orientation
        yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]
        if not finite([msg.info.resolution,yaw]) or msg.info.resolution <= 0 or len(msg.data) != msg.info.width*msg.info.height:
            rospy.logerr('Invalid occupancy grid'); return
        grid = Grid(msg.info.width,msg.info.height,msg.info.resolution,
                    (msg.info.origin.position.x,msg.info.origin.position.y,yaw),msg.data,self.margin)
        with self.lock:
            if self.goal is not None:
                self.halt('MAP_CHANGED: resend goal after localization stabilizes')
            self.grid = grid
            self.hit_confirmation = ConfirmedHits()
            key=map_key(msg)
            if key!=self.line_map_key:
                first=self.line_map_key is None
                self.line_map_key=key;self._pending_line_point=None
                try:
                    saved=self.line_store.load(key)
                    self.zero_cost_line=saved if saved is not None else (self.zero_cost_line if first else [])
                    if saved is None and self.zero_cost_line:self.line_store.save(key,self.zero_cost_line)
                    self.status('ZERO_COST_LINE: loaded %d lines for this map; draw with RViz L tool' % len(self.zero_cost_line))
                except (IOError,OSError,ValueError) as exc:
                    self.zero_cost_line=[];self.status('ZERO_COST_LINE: load failed: '+str(exc))
                rospy.set_param('/single_nav/zero_cost_lines',self.zero_cost_line)
                self.publish_line_markers()

    def on_scan(self, msg):
        # Sensor delivery must not queue behind planning/control work.
        self.scan, self.scan_received = msg, time.time()
        self.scans.append(msg)

    def pose(self):
        if self.ground_truth_test:
            if self.test_pose_override is not None:
                return self.test_pose_override
            if self.truth_alignment is None:
                raise RuntimeError('ground truth map alignment unavailable')
            return rear_pose(self.current_truth()[2], self.truth_alignment)
        now = rospy.Time.now()
        stamp = self.tf.getLatestCommonTime(P['odom_frame'],P['base_frame'])
        age = (now-stamp).to_sec()
        if stamp.to_sec()==0 or not 0 <= age <= P['pose_timeout']:
            raise RuntimeError('local odom TF stale or time reset')
        # Map matching finishes after scan odometry. Hold the latest fresh map
        # correction and advance it with odometry; never query a map TF in its
        # future or silently freeze the vehicle at the previous scan position.
        map_stamp = self.tf.getLatestCommonTime(P['map_frame'],P['base_frame'])
        map_age = (now-map_stamp).to_sec()
        if map_stamp.to_sec()==0 or not 0 <= map_age <= P['pose_timeout']:
            raise RuntimeError('map localization TF stale or time reset')
        correction, cq = self.tf.lookupTransform(P['map_frame'],P['odom_frame'],map_stamp)
        xyz, q = self.tf.lookupTransform(P['odom_frame'],P['base_frame'],stamp)
        angle = tf.transformations.euler_from_quaternion(cq)[2]
        c,s = math.cos(angle),math.sin(angle)
        yaw = wrap(angle+tf.transformations.euler_from_quaternion(q)[2])
        p = to_rear((correction[0]+c*xyz[0]-s*xyz[1],
                     correction[1]+s*xyz[0]+c*xyz[1],yaw), 'base')
        if not finite(p): raise RuntimeError('nonfinite pose')
        return p

    def sensors(self):
        self.actuator_state()
        if self.grid is None or self.scan is None:
            raise RuntimeError('map or scan unavailable')
        if time.time()-self.scan_received > self.scan_timeout or not 0 <= (rospy.Time.now()-self.scan.header.stamp).to_sec() <= self.scan_timeout:
            raise RuntimeError('scan stale (age limit %.2fs); navigation stopped' % self.scan_timeout)
        if sum(1 for v in self.scan.ranges if finite([v]) and self.scan.range_min <= v <= self.scan.range_max) < 5:
            raise RuntimeError('insufficient valid laser returns')

    def obstacles(self):
        """Transform fresh laser hits at scan timestamp, including laser extrinsic."""
        scan = next((s for s in reversed(tuple(self.scans))
                     if 0 <= (rospy.Time.now()-s.header.stamp).to_sec() <= self.scan_timeout
                     and self.tf.canTransform(P['map_frame'],s.header.frame_id,s.header.stamp)),None)
        if scan is None:
            raise RuntimeError('no fresh scan with time-aligned laser TF')
        xyz, q = self.tf.lookupTransform(P['map_frame'],scan.header.frame_id,scan.header.stamp)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        cells = set()
        self.obstacle_stamp = scan.header.stamp.to_nsec()
        ranges = smooth_ranges(scan.ranges, scan.range_min, scan.range_max)
        for i, distance in enumerate(ranges[::P['scan_stride']]):
            if finite([distance]) and scan.range_min <= distance < min(scan.range_max, P['scan_obstacle_range']):
                a = yaw + scan.angle_min + (P['scan_stride']*i)*scan.angle_increment
                cell = self.grid.cell(xyz[0]+distance*math.cos(a),xyz[1]+distance*math.sin(a))
                # A mapped wall is already represented by the static grid.  Do
                # not feed its normal laser return back as LIVE_SCAN, otherwise
                # the safety supervisor sees the same wall twice and may stop.
                if not self.grid.known_static_near(cell[0], cell[1], 1):
                    cells.add(cell)
        if not hasattr(self, 'hit_confirmation'):
            self.hit_confirmation = ConfirmedHits()
        self.grid.set_dynamic(self.hit_confirmation.update(cells, scan.header.stamp.to_sec()))

    def wait_obstacle(self, reason):
        self.cmd.publish(Twist())
        key = obstacle_key(reason)
        if not self.failures.record(key):
            self.halt('BLOCKED: same cause twice consecutively [%s]; %s' % (key, reason))
            return
        self.state = 'WAIT_OBSTACLE'
        self.wait_reason = reason
        self.recovery = Recovery(time.time())
        self.stop_window = StopWindow()
        self.status('WAIT_OBSTACLE: goal retained; consecutive cause 1/2 [%s]; %s' % (key, reason))

    def wait_tracking(self, error):
        self.cmd.publish(Twist())
        if not self.failures.record('TRACKING_DEVIATION'):
            self.halt('TRACKING_FAILED: same cause twice consecutively [TRACKING_DEVIATION]')
            return
        self.state = 'WAIT_TRACKING'
        self.recovery = Recovery(time.time())
        self.stop_window = StopWindow()
        self.status('WAIT_TRACKING: deviation %.3fm; goal retained; stopping before replan; consecutive cause 1/2'
                    % error)

    def recover_tracking(self, p, now):
        self.cmd.publish(Twist())
        self.obstacles()
        stopped = self.stop_window.update(now, p) and abs(self.actuator_state()[1]) <= .005
        clear = self.grid.free(p)
        decision = self.recovery.update(now, self.obstacle_stamp, clear, stopped)
        if decision == 'resume':
            # Never resume the deviated path: plan anew from the settled pose.
            self.status('REPLANNING: tracking deviation; stable pose and fresh scans verified')
            self.start_plan(p)
        elif decision in ('replan', 'fail'):
            reason = ('current footprint blocked; '+self.grid.blocked_detail(p)
                      if not clear else 'unable to verify stable stop and fresh clear scans')
            self.halt('TRACKING_FAILED: '+reason)

    def recover_obstacle(self, p, now):
        self.cmd.publish(Twist())
        self.obstacles()
        stopped = self.stop_window.update(now, p) and abs(self.actuator_state()[1]) <= .005
        part = self.parts[self.part]
        clear = self.grid.free(p) and self.local_candidate(p, part) is not None
        decision = self.recovery.update(now, self.obstacle_stamp, clear, stopped)
        if decision == 'resume':
            if 'TRACKING_ENVELOPE' in self.wait_reason or 'LOCAL_PLAN_BLOCKED' in self.wait_reason:
                self.status('REPLANNING: local tracking failure; stable pose and fresh scans verified')
                self.replan_from_current(p, 'local tracking envelope')
            else:
                self.state = 'DRIVING'
                self.last_movement = self.progress_time = now
                self.status('RESUMING: stopped and clear for 5 fresh scans over 0.5 seconds')
        elif decision == 'replan':
            if not self.grid.free(p):
                self.halt('BLOCKED: current footprint still blocked; '+self.grid.blocked_detail(p))
            else:
                self.status('REPLANNING: obstruction persisted; original goal retained')
                self.start_plan(p)
        elif decision == 'fail':
            self.halt('BLOCKED: unable to verify stop within 10 seconds')

    def authority(self):
        if time.time()-self.last_authority_check > .4:
            publishers, _, _ = self.master.getSystemState()
            other = [node for topic,nodes in publishers if topic == self.cmd_topic
                     for node in nodes if node != rospy.get_name()]
            self.authority_ok = not other
            self.last_authority_check = time.time()
        if not self.authority_ok:
            raise RuntimeError('another /sim/cmd_vel publisher exists; close keyboard/old navigation')

    def on_goal(self, msg):
        with self.lock:
            self.halt('NEW_GOAL: stopping before planning')
            try:
                if msg.header.frame_id.lstrip('/') != P['map_frame']:
                    raise ValueError('goal must be in map frame; no coordinate fallback')
                q = msg.pose.orientation
                values = [msg.pose.position.x,msg.pose.position.y,q.x,q.y,q.z,q.w]
                if not finite(values) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
                    raise ValueError('invalid goal quaternion/position')
                roll,pitch,yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
                if abs(roll)+abs(pitch) > .02: raise ValueError('goal must be planar')
                if self.ground_truth_test:
                    if self.grid is None:
                        raise RuntimeError('map unavailable')
                    self.authority()
                    self.calibrate_truth()
                else:
                    self.sensors(); self.authority()
                current = self.pose()  # Reject missing/stale localization immediately.
                delta = wrap(yaw-current[2])
                dx, dy = msg.pose.position.x-current[0], msg.pose.position.y-current[1]
                along = dx*math.cos(current[2]) + dy*math.sin(current[2])
                lateral = -dx*math.sin(current[2]) + dy*math.cos(current[2])
                ad, al = abs(delta), abs(lateral)
                if math.radians(60) <= ad <= math.radians(120):
                    self.maneuver_mode = 'TURN_90_LEFT' if delta > 0 else 'TURN_90_RIGHT'
                    self.maneuver_heading_tolerance = math.radians(15.0)
                elif ad >= math.radians(150) and al >= .20:
                    self.maneuver_mode = 'LATERAL_TURN_180'
                    self.maneuver_heading_tolerance = math.radians(20.0)
                elif al >= .20 and abs(along) <= max(.60, al):
                    self.maneuver_mode = 'LATERAL'
                    self.maneuver_heading_tolerance = math.radians(12.0)
                elif ad <= math.radians(15) and abs(lateral) <= .20:
                    self.maneuver_mode = 'STRAIGHT'
                    self.maneuver_heading_tolerance = math.radians(8.0)
                else:
                    self.maneuver_mode = 'NORMAL'
                    self.maneuver_heading_tolerance = math.radians(3.0)
                self.goal = (msg.pose.position.x,msg.pose.position.y,yaw)
                self.line_stage = None
                self.line_route = None
                self.stage_target = None
                self.status('GOAL_MODE: %s; heading_delta=%.1fdeg' % (self.maneuver_mode, math.degrees(delta)))
                if not self.grid.free(to_rear(self.goal, 'goal')):
                    raise ValueError('goal blocked; '+self.grid.blocked_detail(to_rear(self.goal, 'goal')))
                self.state = 'WAIT_STOP'
                self.status('WAIT_STOP: checking pose stability for 1 second (timeout 10 seconds)')
                self.still_since = None
                self.last_movement = time.time()
                self.last_pose = None
                self.goal_started = time.time()
            except Exception as exc:
                self.halt('REJECTED: '+str(exc))

    def replan_from_current(self, pose, reason):
        """Bounded automatic replan for a stalled but valid vehicle pose."""
        self.cmd.publish(Twist())
        if self.goal is None:
            self.halt('REPLAN_FAILED: no active goal; '+reason)
            return
        if self.replan_count >= P['max_replans']:
            self.halt('REPLAN_LIMIT: configured automatic replan limit exhausted; '+reason)
            return
        if not self.grid.free(pose):
            self.halt('REPLAN_FAILED: current footprint blocked; '+self.grid.blocked_detail(pose))
            return
        self.obstacles()
        self.replan_count += 1
        self.progress_time = time.time()
        self.last_movement = time.time()
        self.status('REPLANNING: %d/%d; %s' % (self.replan_count, P['max_replans'], reason))
        self.start_plan(pose)

    def start_plan(self, pose):
        generation = self.generation
        requested_goal = self.goal
        line_target = self.stage_target
        mode = self.maneuver_mode
        sector = (min(self.maneuver_heading_tolerance, math.radians(P['maneuver_plan_heading_deg']))
                  if mode != 'NORMAL' else self.goal_heading_tolerance)
        goal = to_rear(requested_goal, 'goal')
        # A snapshot prevents /scan and map updates mutating a planner's grid.
        grid = Grid(self.grid.w,self.grid.h,self.grid.res,self.grid.origin,self.grid.data,self.planning_margin)
        grid.dynamic = set(self.grid.dynamic)
        grid.zero_cost_width = self.zero_cost_width
        grid.zero_cost_line = list(self.zero_cost_line)
        self.state = 'PLANNING'; self.status('PLANNING')
        def work():
            try:
                cancel = lambda: self.generation != generation or self.stop_event.is_set()
                stage = self.line_stage
                selected = None
                if stage == 'FINAL':
                    # The final approach may finish within the configured
                    # left/right heading window.  The maneuver heading cap
                    # belongs to the line entry/retreat stages and must not
                    # silently reduce the final-goal tolerance.
                    final_sector = self.goal_heading_tolerance
                    path = plan(grid,pose,goal,self.radius,self.timeout,cancel,
                                min(self.goal_position_tolerance, .025),final_sector,mode,requested_goal)
                    target = requested_goal
                elif stage == 'RETREAT_LINE':
                    target = line_target
                    path = plan(grid,pose,target,self.radius,self.timeout,cancel,
                                P['stage_plan_position'],math.radians(P['stage_plan_heading_deg']),mode)
                else:
                    special = mode in ('LATERAL','LATERAL_TURN_180')
                    path, selected = (plan_line_approach(
                        grid,pose,goal,self.radius,self.timeout,cancel,
                        min(self.goal_position_tolerance, .025),min(sector, self.goal_heading_tolerance),mode,requested_goal)
                        if special and grid.zero_cost_line else (None,None))
                    if selected is None:
                        if special and grid.zero_cost_line:
                            raise RuntimeError('no reachable entry on a drawn line; special move will not bypass line stages')
                        path = plan(grid,pose,goal,self.radius,self.timeout,cancel,
                                    self.goal_position_tolerance,sector,mode,requested_goal,
                                    start_reverse_only=mode == 'LATERAL')
                        stage, target = None, requested_goal
                    else:
                        stage, target = 'APPROACH_LINE', selected[1]
                with self.lock:
                    if generation != self.generation: return
                    if selected is not None:
                        self.line_route = selected
                        self.status('VIA_LINE: line %d; entry=(%.2f,%.2f)' %
                                    (selected[0]+1,selected[1][0],selected[1][1]))
                    self.line_stage = stage
                    self.stage_target = target
                    self.execution_heading = path[-1][2] if stage or mode != 'NORMAL' else requested_goal[2]
                    self.status('PLAN_STAGE: %s; target=(%.2f,%.2f,%.1fdeg)' %
                                (stage or 'DIRECT',target[0],target[1],math.degrees(self.execution_heading)))
                    self.parts, self.part, self.index = segments(path), 0, 0
                    # Each direction segment has its own anchor and local index.
                    scale = self.test_replay_speed_scale if self.ground_truth_test else 1.
                    self.speed_caps = [speed_profile(part, forward=P['forward_speed']*scale, reverse=P['reverse_speed']*scale)
                                       for part in self.parts]
                    self.last_movement = time.time()
                    self.progress_time = time.time()
                    self.state = 'DRIVING' if self.parts else 'VERIFY_STOP'
                    self.test_arrival_deadline = (time.time() + self.test_auto_arrive_delay
                                                  if self.ground_truth_test and self.test_auto_arrive
                                                  else None)
                    self.still_since = None
                    self.publish_path(path)
                    self.status('DRIVING: %d direction segments' % len(self.parts))
            except Exception as exc:
                with self.lock:
                    if generation == self.generation: self.halt('PLAN_FAILED: '+str(exc))
        worker = threading.Thread(target=work); worker.daemon=True; worker.start()

    def finish_stage(self, pose):
        if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE'):
            target = self.stage_target
            error = math.hypot(pose[0]-target[0],pose[1]-target[1])
            heading = abs(wrap(pose[2]-target[2]))
            if error > P['stage_finish_position'] or heading > math.radians(P['stage_finish_heading_deg']):
                self.replan_from_current(pose, 'line stage pose not verified (%.3fm, %.1fdeg)' %
                                         (error, math.degrees(heading)))
                return
            if (self.line_stage == 'APPROACH_LINE'
                    and self.maneuver_mode in ('LATERAL', 'LATERAL_TURN_180')):
                try:
                    line = self.zero_cost_line[self.line_route[0]]
                    next_target = line_retreat_target(line, self.goal, target[2])
                    if not self.grid.free(next_target):
                        raise ValueError('second line-stage target blocked; '+self.grid.blocked_detail(next_target))
                except (IndexError, TypeError, ValueError) as exc:
                    self.halt('PLAN_FAILED: '+str(exc))
                    return
                self.line_stage = 'RETREAT_LINE'
                self.stage_target = next_target
                self.status('LINE_CONFIRMED: error %.3fm, %.1fdeg; planning second line point' %
                            (error,math.degrees(heading)))
                self.start_plan(pose)
                return
            self.line_stage = 'FINAL'
            self.status('LINE_CONFIRMED: error %.3fm, %.1fdeg; replanning to goal' %
                        (error,math.degrees(heading)))
            self.start_plan(pose)
            return
        fx, fy = from_rear(pose, 'goal')[:2]
        position_error = math.hypot(fx-self.goal[0],fy-self.goal[1])
        heading_error = abs(wrap(pose[2]-self.goal[2]))
        detail = 'goal-reference error %.3fm, heading error %.2fdeg' % (position_error,math.degrees(heading_error))
        if position_error <= P['final_finish_position'] and heading_error <= math.radians(P['final_finish_heading_deg'] if self.maneuver_mode != 'NORMAL' else P['normal_finish_heading_deg']):
            self.halt('SUCCEEDED: '+detail)
        else:
            self.halt('FINAL_TOLERANCE_FAILED: '+detail)

    def publish_path(self, path):
        msg = Path(); msg.header.frame_id=P['map_frame']; msg.header.stamp=rospy.Time.now()
        for p in path:
            ps=PoseStamped(); ps.header=msg.header
            ps.pose.position.x,ps.pose.position.y=from_rear(p, 'path')[:2]
            q=tf.transformations.quaternion_from_euler(0,0,p[2])
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def wait_control(self, elapsed):
        # Invalidate any asynchronous planner before retaining the goal to retry.
        self.generation += 1
        self.cmd.publish(Twist())
        self.parts = []
        self.state = 'WAIT_CONTROL'
        self.control_wait_started = time.time()
        self.control_stable_since = None
        self.control_stop_window = StopWindow()
        self.control_recovery = Recovery(time.time())
        self.status('WAIT_CONTROL: interval %.3fs; stopped, goal retained for replan' % elapsed)

    def recover_control(self, elapsed):
        self.cmd.publish(Twist())
        now = time.time()
        if elapsed < 0:
            self.halt('STOPPED: simulation time moved backwards; reset localization and resend goal')
            return
        if now-self.control_wait_started >= 10.:
            self.halt('STOPPED: control timing/feedback did not recover within 10 seconds')
            return
        if not 0 < elapsed <= P['max_control_dt']:
            self.control_stable_since = None
            self.control_stop_window = StopWindow()
            self.control_recovery = Recovery(now)
            return
        try:
            self.sensors(); self.authority()
            p = self.pose()
            self.obstacles()
        except Exception:
            self.control_stable_since = None
            self.control_stop_window = StopWindow()
            self.control_recovery = Recovery(now)
            return
        if self.control_stable_since is None:
            self.control_stable_since = now
        stopped = self.control_stop_window.update(now, p)
        stopped = stopped and abs(self.actuator_state()[1]) <= .005
        decision = self.control_recovery.update(now, self.scan.header.stamp.to_nsec(),
                                               self.grid.free(p), stopped)
        if now-self.control_stable_since >= 1. and decision == 'resume':
            self.last_pose = p
            self.last_pose_wall = now
            self.replan_from_current(p, 'control timing recovered; stopped and fresh scans verified')

    def tick_test(self):
        self.authority()
        p = self.pose()
        now = time.time()
        # Auto-arrival mode is a planning-only test: never drive Gazebo.
        if getattr(self, 'test_auto_arrive', False):
            self.cmd.publish(Twist())
        if (getattr(self, 'test_auto_arrive', False) and self.state == 'DRIVING'
                and self.test_arrival_deadline is not None
                and now >= self.test_arrival_deadline):
            # Test-only virtual arrival: keep the published path visible, then
            # feed the exact stage target through the normal confirmation logic.
            self.cmd.publish(Twist())
            if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE'):
                virtual_pose = self.stage_target
            else:
                virtual_pose = to_rear(self.goal, 'goal')
            self.test_pose_override = virtual_pose
            self.publish_virtual_pose(virtual_pose)
            self.test_arrival_deadline = None
            self.state = 'VERIFY_STOP'
            self.finish_stage(virtual_pose)
            return
        if getattr(self, 'test_auto_arrive', False) and self.state == 'DRIVING':
            return
        if self.state in ('WAIT_STOP', 'PLANNING', 'CUSP', 'VERIFY_STOP'):
            if self.stop_state != self.state:
                self.stop_state = self.state
                self.stop_window = StopWindow()
            self.cmd.publish(Twist())
            stopped = self.stop_window.update(now, p)
            if self.state == 'WAIT_STOP' and stopped:
                self.start_plan(p)
            elif self.state == 'CUSP' and stopped:
                self.part += 1
                self.index = 0
                self.state = 'DRIVING'
            elif self.state == 'VERIFY_STOP' and stopped:
                self.finish_stage(p)
            return
        self.stop_state = None
        part = self.parts[self.part]
        reference = project(p, part, self.index)
        self.index = max(self.index, reference['index'])
        self.path_errors.append(reference['distance'])
        self.error_pub.publish(String(data='lateral=%.3fm heading=%.2fdeg part=%d index=%d' %
                                      (reference['lateral'], math.degrees(reference['heading']),
                                       self.part+1, self.index)))
        if reference['remaining'] <= .012 and self.index >= len(part)-2:
            self.state = 'VERIFY_STOP' if self.part == len(self.parts)-1 else 'CUSP'
            self.cmd.publish(Twist())
            return
        v, k = replay_command(reference, part[-1][3], self.active_speed_cap(),
                              self.test_replay_speed_scale)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v*k
        self.cmd.publish(cmd)

    def early_arrival_tolerances(self):
        # FINAL is also a nonempty line-stage name, but it must use the
        # final-goal thresholds rather than the looser intermediate stop.
        transition = self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE')
        return (P['stage_early_position'] if transition else P['final_early_position'],
                math.radians(P['stage_early_heading_deg'] if transition else P['final_early_heading_deg']))

    def tick(self):
        stamp = rospy.Time.now().to_sec()
        elapsed = P['control_period'] if self.last_control_stamp is None else stamp-self.last_control_stamp
        self.last_control_stamp = stamp
        if self.goal is None:
            # Idle node never competes with a manual source; entering IDLE sent zero.
            return
        if self.ground_truth_test:
            self.tick_test()
            return
        if self.state == 'WAIT_CONTROL':
            self.recover_control(elapsed)
            return
        if elapsed < 0:
            raise RuntimeError('simulation time moved backwards; reset localization and resend goal')
        if not 0 < elapsed <= P['max_control_dt']:
            self.wait_control(elapsed)
            return
        self.control_dt = elapsed
        self.sensors(); self.authority()
        p = self.pose()
        now = time.time()
        moved = False
        if self.last_pose is not None:
            dt = max(.001,now-self.last_pose_wall)
            d = math.hypot(p[0]-self.last_pose[0],p[1]-self.last_pose[1])
            dyaw = abs(wrap(p[2]-self.last_pose[2]))
            if d > .15 or dyaw > .20:
                raise RuntimeError('localization jump; reset pose and resend goal')
            moved = motion_observed(p,self.last_pose,dt,self.actuator_state()[1])
        self.last_pose,self.last_pose_wall=p,now
        if moved:
            self.still_since=None; self.last_movement=now
        elif self.still_since is None: self.still_since=now
        if self.state == 'WAIT_TRACKING':
            self.recover_tracking(p, now)
            return
        if self.state == 'WAIT_OBSTACLE':
            self.recover_obstacle(p, now)
            return
        if self.state in ('WAIT_STOP','PLANNING','VERIFY_STOP','CUSP'):
            if self.stop_state != self.state:
                self.stop_state = self.state
                self.stop_started = now
                self.stop_window = StopWindow()
            stopped = self.stop_window.update(now, p)
            if self.state != 'PLANNING' and now-self.stop_started > P['stop_timeout']:
                raise RuntimeError('stop not verified within configured timeout: vehicle motion or localization drift; check pose and resend goal')
            self.cmd.publish(Twist())
            # Pose stability alone cannot prove that the wheels have stopped.
            stopped = stopped and abs(self.actuator_state()[1]) <= .005
            if self.state == 'WAIT_STOP' and stopped:
                self.obstacles(); self.start_plan(p)
            elif self.state == 'VERIFY_STOP' and stopped:
                self.finish_stage(p)
            elif self.state == 'CUSP' and stopped:
                self.part+=1; self.index=0; self.state='DRIVING'; self.last_movement=now
                self.progress_time=now
            return
        self.stop_state = None
        self.obstacles()
        if not self.grid.free(p):
            self.wait_obstacle('current footprint blocked; '+self.grid.blocked_detail(p))
            return
        part=self.parts[self.part]
        reference=project(p,part,self.index)
        index,remaining=reference['index'],reference['remaining']
        if index > self.index: self.progress_time=now
        self.index=index
        lateral=reference['distance']
        if lateral > P['tracking_max_error']:
            self.wait_tracking(lateral)
            return
        final=self.part == len(self.parts)-1
        fx,fy=from_rear(p, 'goal')[:2]
        target_xy = (self.stage_target[:2] if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE')
                     else self.goal[:2])
        actual_xy = p[:2] if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE') else (fx,fy)
        early_heading = (self.execution_heading if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE')
                         else self.goal[2])
        early_position, early_angle = self.early_arrival_tolerances()
        if final and math.hypot(actual_xy[0]-target_xy[0],actual_xy[1]-target_xy[1]) <= early_position and abs(wrap(p[2]-early_heading)) <= early_angle:
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        if not final and remaining < .035 and index >= len(part)-8:
            self.state='CUSP';self.still_since=None;self.cmd.publish(Twist());return
        if final and index >= len(part)-2 and remaining < .005:
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        if now-self.last_movement > P['progress_timeout']:
            raise RuntimeError('no measurable motion within configured timeout')
        if now-self.progress_time > P['progress_timeout']:
            self.replan_from_current(p, 'path progress stalled beyond configured timeout')
            return
        diagnostics = {}
        cap = self.active_speed_cap()
        candidate = self.local_candidate(p,part,diagnostics,cap)
        if candidate is None:
            self.local_pub.publish(Path())
            self.wait_obstacle('LOCAL_PLAN_BLOCKED: '+explain(diagnostics))
            return
        _,v,k,steer,local_path = candidate
        msg = Path(); msg.header.frame_id=P['map_frame']; msg.header.stamp=rospy.Time.now()
        for point in local_path:
            ps=PoseStamped();ps.header=msg.header
            ps.pose.position.x,ps.pose.position.y=from_rear(point, 'path')[:2]
            q=tf.transformations.quaternion_from_euler(0,0,point[2])
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
            msg.poses.append(ps)
        self.local_pub.publish(msg)
        self.sensors()  # Recheck freshness after trajectory computation.
        cmd=Twist();cmd.linear.x=v;cmd.angular.z=v*k
        self.cmd.publish(cmd)

    def run(self):
        while not self.stop_event.wait(P['control_period']) and not rospy.is_shutdown():
            with self.lock:
                try: self.tick()
                except Exception as exc:
                    self.halt('STOPPED: '+str(exc))


if __name__ == '__main__':
    rospy.init_node('single_goal_nav')
    Navigator()
    rospy.spin()
