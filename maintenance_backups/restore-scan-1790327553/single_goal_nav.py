#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simulation-only single front-axle pose navigation. Python 2 / ROS Kinetic."""
from __future__ import print_function, division
import math
import threading
import time
from collections import deque
import rospy
import rosgraph
import tf
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from local_planner import choose, explain
from actuator_model import decode_joints
from path_tracking import project, motion_observed
from nav_obstacles import smooth_ranges, Recovery, ConsecutiveFailures, obstacle_key
from ackermann_core import Grid, plan, segments, speed_profile, rear_target, front_position, wrap, finite, StopWindow


class Navigator(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.tf = tf.TransformListener()
        self.master = rosgraph.Master(rospy.get_name())
        self.radius = float(rospy.get_param('~turn_radius', 1.3))
        if not finite([self.radius]) or self.radius < 1.3:
            raise ValueError('turn_radius must be finite and >= 1.3 m')
        self.local_radius = float(rospy.get_param('~local_turn_radius', 1.2))
        self.steering_rate = float(rospy.get_param('~steering_rate', 1.0))
        self.tracking_options = dict(lateral_gain=float(rospy.get_param('~tracking_lateral_gain',6.0)),
                                     heading_gain=float(rospy.get_param('~tracking_heading_gain',4.0)),
                                     preview_distance=float(rospy.get_param('~tracking_preview_distance',.35)))
        if not finite(list(self.tracking_options.values())) or min(self.tracking_options.values()) <= 0:
            raise ValueError('tracking gains and preview distance must be finite and positive')
        if (not finite([self.local_radius, self.steering_rate])
                or not 1.2 <= self.local_radius <= self.radius or self.steering_rate <= 0):
            raise ValueError('1.2 <= local_turn_radius <= turn_radius; steering_rate > 0')
        self.actuator = rospy.get_param('/sim/actuator_model', {})
        if self.actuator.get('version') != 1:
            raise ValueError('updated simulation plugin required: rebuild and restart simulation')
        required = ('wheelbase', 'front_track', 'wheel_radius', 'acceleration', 'braking', 'steer_rate')
        if any(key not in self.actuator for key in required):
            raise ValueError('incomplete simulation actuator model')
        if not finite([self.actuator[key] for key in required]) or any(self.actuator[key] <= 0 for key in required):
            raise ValueError('invalid simulation actuator model')
        if abs(self.actuator['wheelbase'] - .62) > 1e-6:
            raise ValueError('actuator wheelbase must match navigation geometry (0.62 m)')
        self.joint_feedback = None
        self.last_control_stamp = None
        self.margin = float(rospy.get_param('~collision_margin', .04))
        self.zero_cost_width = float(rospy.get_param('~zero_cost_line_width', .01))
        line = [float(rospy.get_param('~zero_cost_line_x1', float('nan'))), float(rospy.get_param('~zero_cost_line_y1', float('nan'))), float(rospy.get_param('~zero_cost_line_x2', float('nan'))), float(rospy.get_param('~zero_cost_line_y2', float('nan')))]
        self.zero_cost_line = [((line[0],line[1]),(line[2],line[3]))] if finite(line) else []
        self.timeout = float(rospy.get_param('~planning_timeout', 12.))
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
        self.cmd = rospy.Publisher('/sim/cmd_vel', Twist, queue_size=1)
        self.local_pub = rospy.Publisher('/single_nav/local_path', Path, queue_size=1)
        self.line_marker_pub = rospy.Publisher('/single_nav/zero_cost_line', Marker, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/single_nav/path', Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/single_nav/status', String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher('/single_nav/result', String, queue_size=1, latch=True)
        rospy.Subscriber('/single_nav/line_point', PointStamped, self.on_line_point, queue_size=20)
        rospy.Service('/single_nav/clear_zero_cost_line', Trigger, self.clear_zero_cost_line)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/sim/joint_states', JointState, self.on_joints, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber('/single_nav/cancel', String, self.on_cancel, queue_size=1)
        self.publish_line_markers()
        rospy.on_shutdown(self.shutdown)
        self.status('IDLE: set initial pose, then RViz 2D Nav Goal (front axle target)')
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
                             'REPLAN_FAILED:')):
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

    def actuator_state(self):
        sample = self.joint_feedback
        now = rospy.Time.now().to_sec()
        # JointState and /clock travel independently; allow one clock delivery skew.
        if (sample is None or not -.02 <= now-sample[0] <= .30
                or not 0 <= time.time()-sample[1] <= .30):
            raise RuntimeError('joint feedback missing/stale; navigation stopped')
        return sample[2], sample[3]

    def local_candidate(self, p, part, diagnostics=None, cap=None):
        steer, speed = self.actuator_state()
        candidate = choose(self.grid,p,part,self.index,self.local_radius,steer,diagnostics,cap,
                           control_dt=self.control_dt, current_speed=speed,
                           steering_rate=self.steering_rate, actuator_steer_rate=self.actuator['steer_rate'],
                           acceleration=self.actuator['acceleration'], braking=self.actuator['braking'],
                           **self.tracking_options)
        if candidate is None or self.scan is None:
            return candidate
        # Mode-specific blend: actions 1/2/3 favor path control (80/20);
        # straight motion favors wall-centering (30/70); NORMAL stays 50/50.
        ranges = self.scan.ranges; amin = self.scan.angle_min; inc = self.scan.angle_increment
        def side_clearance(lo, hi):
            vals=[]
            for i, value in enumerate(ranges):
                a=amin+i*inc
                if lo <= a <= hi and finite([value]) and self.scan.range_min <= value < min(self.scan.range_max, 2.0):
                    vals.append(value)
            if not vals: return None
            vals.sort(); return vals[len(vals)//2]
        left, right = side_clearance(0.55, 1.57), side_clearance(-1.57, -0.55)
        if left is None or right is None: return candidate
        imbalance=max(-0.35,min(0.35,left-right))
        planned_angle=math.atan(.62*candidate[2])
        wall_angle=max(-math.radians(15.0),min(math.radians(15.0), imbalance*.45))
        path_weight, scan_weight = ((.3,.7) if self.maneuver_mode == 'STRAIGHT' else ((.8,.2) if self.maneuver_mode in ('TURN_90_LEFT','TURN_90_RIGHT','LATERAL','LATERAL_TURN_180') else (.5,.5)))
        blended=path_weight*planned_angle+scan_weight*wall_angle
        return (candidate[0],candidate[1],math.tan(blended)/.62,blended,candidate[4])

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
        self.generation += 1
        self.parts, self.goal = [], None
        self.execution_heading = None
        self.recovery = None
        self.replan_count = 0
        self.failures = ConsecutiveFailures()
        self.state = 'IDLE'
        self.stop_state = None
        self.cmd.publish(Twist())
        empty = Path(); empty.header.frame_id = 'map'; empty.header.stamp = rospy.Time.now()
        self.path_pub.publish(empty)
        self.status(reason)

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
            self.zero_cost_line = []
            self._pending_line_point = None
            self.publish_line_markers()
            self.status('ZERO_COST_LINE: all lines and pending start cleared; applies to next plan')
        return TriggerResponse(True, 'zero-cost line cleared')

    def on_line_point(self, msg):
        if msg.header.frame_id.lstrip('/') != 'map': return
        point=(msg.point.x,msg.point.y)
        if not finite(point):
            self.status('ZERO_COST_LINE: rejected nonfinite point'); return
        with self.lock:
            pending = self._pending_line_point
            if pending is None:
                self._pending_line_point=point
                self.publish_line_markers()
                self.status('ZERO_COST_LINE: start selected; click endpoint for line %d' % (len(self.zero_cost_line)+1))
                return
            if math.hypot(point[0]-pending[0],point[1]-pending[1]) < .001:
                self.status('ZERO_COST_LINE: endpoint too close; choose another endpoint'); return
            self.zero_cost_line.append((pending,point)); self._pending_line_point=None
            self.publish_line_markers()
            self.status('ZERO_COST_LINE: %d lines; click next start/end pair; applies to next plan' % len(self.zero_cost_line))

    def publish_line_markers(self):
        # One latched message contains every line and the pending-start cross.
        # A newly opened RViz receives the complete drawing, not only the last line.
        marker=Marker(); marker.header.frame_id='map'; marker.ns='zero_cost_line'; marker.id=0
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
        if msg.header.frame_id.lstrip('/') != 'map':
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

    def on_scan(self, msg):
        with self.lock:
            self.scan, self.scan_received = msg, time.time()
            self.scans.append(msg)

    def pose(self):
        now = rospy.Time.now()
        stamp = self.tf.getLatestCommonTime('odom','base_footprint')
        age = (now-stamp).to_sec()
        if not 0 <= age <= .40:
            raise RuntimeError('local odom TF stale or time reset')
        # Query map pose at the continuous odom sample time, not stale AMCL pose.
        xyz, q = self.tf.lookupTransform('map','base_footprint',stamp)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        p = (xyz[0]-.31*math.cos(yaw),xyz[1]-.31*math.sin(yaw),yaw)
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
        scan = next((s for s in reversed(self.scans)
                     if 0 <= (rospy.Time.now()-s.header.stamp).to_sec() <= self.scan_timeout
                     and self.tf.canTransform('map',s.header.frame_id,s.header.stamp)),None)
        if scan is None:
            raise RuntimeError('no fresh scan with time-aligned laser TF')
        xyz, q = self.tf.lookupTransform('map',scan.header.frame_id,scan.header.stamp)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        cells = set()
        self.obstacle_stamp = scan.header.stamp.to_nsec()
        ranges = smooth_ranges(scan.ranges, scan.range_min, scan.range_max)
        for i, distance in enumerate(ranges):
            if finite([distance]) and scan.range_min <= distance <= min(scan.range_max, 0.25):
                a = yaw + scan.angle_min + i*scan.angle_increment
                cells.add(self.grid.cell(xyz[0]+distance*math.cos(a),xyz[1]+distance*math.sin(a)))
        self.grid.dynamic = cells  # live scan avoidance only within 25 cm

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
            other = [node for topic,nodes in publishers if topic == '/sim/cmd_vel'
                     for node in nodes if node != rospy.get_name()]
            self.authority_ok = not other
            self.last_authority_check = time.time()
        if not self.authority_ok:
            raise RuntimeError('another /sim/cmd_vel publisher exists; close keyboard/old navigation')

    def on_goal(self, msg):
        with self.lock:
            self.halt('NEW_GOAL: stopping before planning')
            try:
                if msg.header.frame_id.lstrip('/') != 'map':
                    raise ValueError('goal must be in map frame; no coordinate fallback')
                q = msg.pose.orientation
                values = [msg.pose.position.x,msg.pose.position.y,q.x,q.y,q.z,q.w]
                if not finite(values) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
                    raise ValueError('invalid goal quaternion/position')
                roll,pitch,yaw = tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))
                if abs(roll)+abs(pitch) > .02: raise ValueError('goal must be planar')
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
                elif ad >= math.radians(150) and al >= .20 and abs(along) <= .60:
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
                self.status('GOAL_MODE: %s; heading_delta=%.1fdeg' % (self.maneuver_mode, math.degrees(delta)))
                if not self.grid.free(rear_target(self.goal)):
                    raise ValueError('goal blocked; '+self.grid.blocked_detail(rear_target(self.goal)))
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
        if self.replan_count >= 2:
            self.halt('REPLAN_LIMIT: 2 automatic replans exhausted; '+reason)
            return
        if not self.grid.free(pose):
            self.halt('REPLAN_FAILED: current footprint blocked; '+self.grid.blocked_detail(pose))
            return
        self.obstacles()
        self.replan_count += 1
        self.progress_time = time.time()
        self.last_movement = time.time()
        self.status('REPLANNING: %d/2; %s' % (self.replan_count, reason))
        self.start_plan(pose)

    def start_plan(self, pose):
        generation = self.generation
        requested_goal = self.goal
        mode = self.maneuver_mode
        sector = self.maneuver_heading_tolerance if mode != 'NORMAL' else self.goal_heading_tolerance
        goal = rear_target(requested_goal)
        # A snapshot prevents /scan and map updates mutating a planner's grid.
        grid = Grid(self.grid.w,self.grid.h,self.grid.res,self.grid.origin,self.grid.data,self.margin)
        grid.dynamic = set(self.grid.dynamic)
        grid.zero_cost_width = self.zero_cost_width
        grid.zero_cost_line = list(self.zero_cost_line)
        self.state = 'PLANNING'; self.status('PLANNING')
        def work():
            try:
                path = plan(grid,pose,goal,self.radius,self.timeout,
                            goal_position_tolerance=self.goal_position_tolerance,
                            goal_heading_tolerance=sector,
                            maneuver_mode=mode, front_goal=requested_goal,
                            cancel=lambda: self.generation != generation or self.stop_event.is_set())
                with self.lock:
                    if generation != self.generation: return
                    self.execution_heading = path[-1][2] if mode != 'NORMAL' else requested_goal[2]
                    self.status('SELECTED_HEADING: requested=%.1fdeg selected=%.1fdeg sector=+/-%.1fdeg' %
                                (math.degrees(requested_goal[2]), math.degrees(self.execution_heading), math.degrees(sector)))
                    self.parts, self.part, self.index = segments(path), 0, 0
                    # Each direction segment has its own anchor and local index.
                    self.speed_caps = [speed_profile(part) for part in self.parts]
                    self.last_movement = time.time()
                    self.progress_time = time.time()
                    self.state = 'DRIVING' if self.parts else 'VERIFY_STOP'
                    self.still_since = None
                    self.publish_path(path)
                    self.status('DRIVING: %d direction segments' % len(self.parts))
            except Exception as exc:
                with self.lock:
                    if generation == self.generation: self.halt('PLAN_FAILED: '+str(exc))
        worker = threading.Thread(target=work); worker.daemon=True; worker.start()

    def publish_path(self, path):
        msg = Path(); msg.header.frame_id='map'; msg.header.stamp=rospy.Time.now()
        for p in path:
            ps=PoseStamped(); ps.header=msg.header
            ps.pose.position.x,ps.pose.position.y=p[0],p[1]
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
        if not 0 < elapsed <= .35:
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

    def tick(self):
        stamp = rospy.Time.now().to_sec()
        elapsed = .05 if self.last_control_stamp is None else stamp-self.last_control_stamp
        self.last_control_stamp = stamp
        if self.goal is None:
            # Idle node never competes with a manual source; entering IDLE sent zero.
            return
        if self.state == 'WAIT_CONTROL':
            self.recover_control(elapsed)
            return
        if elapsed < 0:
            raise RuntimeError('simulation time moved backwards; reset localization and resend goal')
        if not 0 < elapsed <= .35:
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
            if self.state != 'PLANNING' and now-self.stop_started > 10.:
                raise RuntimeError('stop not verified within 10 seconds: vehicle motion or localization drift; check pose and resend goal')
            self.cmd.publish(Twist())
            # Pose stability alone cannot prove that the wheels have stopped.
            stopped = stopped and abs(self.actuator_state()[1]) <= .005
            if self.state == 'WAIT_STOP' and stopped:
                self.obstacles(); self.start_plan(p)
            elif self.state == 'VERIFY_STOP' and stopped:
                fx,fy=front_position(p)
                if math.hypot(fx-self.goal[0],fy-self.goal[1]) <= .15 and abs(wrap(p[2]-self.execution_heading)) <= math.radians(10 if self.maneuver_mode != 'NORMAL' else 8):
                    self.halt('SUCCEEDED: front axle position and heading verified at rest')
                else: self.halt('FINAL_TOLERANCE_FAILED: stopped; resend goal to replan')
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
        if lateral > .20:
            self.wait_tracking(lateral)
            return
        final=self.part == len(self.parts)-1
        fx,fy=front_position(p)
        if final and math.hypot(fx-self.goal[0],fy-self.goal[1]) <= .12 and abs(wrap(p[2]-self.execution_heading)) <= math.radians(10 if self.maneuver_mode != 'NORMAL' else 8):
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        if not final and remaining < .035 and index >= len(part)-8:
            self.state='CUSP';self.still_since=None;self.cmd.publish(Twist());return
        if final and index >= len(part)-2 and remaining < .025:
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        if now-self.last_movement > 8:
            raise RuntimeError('no measurable motion for 8 seconds')
        if now-self.progress_time > 8:
            self.replan_from_current(p, 'path progress stalled for 8 seconds')
            return
        diagnostics = {}
        cap = self.active_speed_cap()
        candidate = self.local_candidate(p,part,diagnostics,cap)
        if candidate is None:
            self.local_pub.publish(Path())
            self.wait_obstacle('LOCAL_PLAN_BLOCKED: '+explain(diagnostics))
            return
        _,v,k,steer,local_path = candidate
        msg = Path(); msg.header.frame_id='map'; msg.header.stamp=rospy.Time.now()
        for point in local_path:
            ps=PoseStamped();ps.header=msg.header
            ps.pose.position.x,ps.pose.position.y=point[:2]
            q=tf.transformations.quaternion_from_euler(0,0,point[2])
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
            msg.poses.append(ps)
        self.local_pub.publish(msg)
        self.sensors()  # Recheck freshness after trajectory computation.
        cmd=Twist();cmd.linear.x=v;cmd.angular.z=v*k
        self.cmd.publish(cmd)

    def run(self):
        while not self.stop_event.wait(.05) and not rospy.is_shutdown():
            with self.lock:
                try: self.tick()
                except Exception as exc:
                    self.halt('STOPPED: '+str(exc))


if __name__ == '__main__':
    rospy.init_node('single_goal_nav')
    Navigator()
    rospy.spin()
