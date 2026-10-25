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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from local_planner import choose, explain, guarded_track
from actuator_model import decode_joints
from path_tracking import project, motion_observed, replay_command, steering, PreparedPath, tracking_steering
from ground_truth_pose import alignment, rear_pose
from low_cost_lines import LineStore, map_key
from nav_obstacles import smooth_ranges, ConfirmedHits, Recovery, ConsecutiveFailures, obstacle_key
from ackermann_core import plan_fewer_changes as plan
from ackermann_core import (plan_reliability_fallback, Grid, plan_line_approach, plan_line_retreat, line_retreat_target,
                             segments, speed_profile, rear_target, front_position, wrap, finite,
                             StopWindow, load_turn90_trace, plan_turn90_trace)
from inspection_depth import validate_regions, rules_for_goal
from execution_patch import (configure as execution_configure,settings as execution_settings,
                             ExecutionGrid,RecoveryGeometryError,geometry_wait,cusp_speed_limit,align_regions,
                             hold_recovery,recheck_recovery_hold)
import json


class Navigator(object):
    def __init__(self):
        configure(rospy.get_param('~tuning', {}))
        self.execution_options=execution_configure(rospy.get_param('~execution_patch',{}))
        self.execution_options['depth_margin']=max(self.execution_options['depth_margin'],P['safety_stop_extra'])
        rospy.set_param('~effective_execution_patch',dict(self.execution_options))
        rospy.set_param('~effective_tuning', dict(P))
        self.lock = threading.RLock()
        self.tf = tf.TransformListener()
        self.ground_truth_test = bool(rospy.get_param('~ground_truth_test', False))
        # Planning-only tests must still terminate and report infeasible tasks.
        self.persistent_recovery = not self.ground_truth_test
        self.stop_history = deque(maxlen=100)
        self.relaxed_goal = False
        self.goal_position_only = False
        self.return_context = None
        self.return_pending = False
        self.recovery_checkpoint = None
        self.planning_budget_failures = 0
        self.failed_tracking = []
        self.execution_target = None
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
        self.inspection_regions = []
        self.inspection_sides = {}
        self.master = rosgraph.Master(rospy.get_name())
        self.radius = float(rospy.get_param('~turn_radius', 1.25))
        if not finite([self.radius]) or self.radius < P['global_min_radius']:
            raise ValueError('turn_radius must be finite and >= tuning/global_min_radius')
        self.local_radius = float(rospy.get_param('~local_turn_radius', 1.2))
        self.steering_rate = float(rospy.get_param('~steering_rate', 1.0))
        self.tracking_options = dict(lateral_gain=float(rospy.get_param('~tracking_lateral_gain',6.0)),
                                     heading_gain=float(rospy.get_param('~tracking_heading_gain',4.0)),
                                     preview_distance=float(rospy.get_param('~tracking_preview_distance',.35)))
        if not finite(list(self.tracking_options.values())) or min(self.tracking_options.values()) <= 0:
            raise ValueError('tracking gains and preview distance must be finite and positive')
        self.reverse_tracking_options = dict(
            lateral_gain=float(rospy.get_param('~tracking_reverse_lateral_gain',self.tracking_options['lateral_gain'])),
            heading_gain=float(rospy.get_param('~tracking_reverse_heading_gain',self.tracking_options['heading_gain'])),
            preview_distance=float(rospy.get_param('~tracking_reverse_preview_distance',self.tracking_options['preview_distance'])))
        if not finite(list(self.reverse_tracking_options.values())) or min(self.reverse_tracking_options.values()) <= 0:
            raise ValueError('reverse tracking gains and preview distance must be finite and positive')
        rospy.set_param('~effective_tracking',dict(forward=self.tracking_options,reverse=self.reverse_tracking_options))
        if (not finite([self.local_radius, self.steering_rate])
                or not P['physical_min_radius'] <= self.local_radius <= self.radius or self.steering_rate <= 0):
            raise ValueError('physical_min_radius <= local_turn_radius <= turn_radius; steering_rate > 0')
        model_param=rospy.get_param('~actuator_model_param', '/sim/actuator_model')
        self.actuator=rospy.get_param(model_param,{})
        # roslaunch starts the bridge and this node concurrently. Wait for its
        # actual model; never invent dynamics or start driving without a model.
        model_deadline=time.time()+10.
        while not self.actuator and not rospy.is_shutdown() and time.time()<model_deadline:
            time.sleep(.05)
            self.actuator=rospy.get_param(model_param,{})
        if self.actuator.get('version') != 1:
            raise ValueError('actuator model unavailable or invalid at '+model_param+'; check bridge startup')
        required = ('wheelbase', 'front_track', 'wheel_radius', 'acceleration', 'braking', 'steer_rate')
        if any(key not in self.actuator for key in required):
            raise ValueError('incomplete simulation actuator model')
        if not finite([self.actuator[key] for key in required]) or any(self.actuator[key] <= 0 for key in required):
            raise ValueError('invalid simulation actuator model')
        for key, setting in (('wheelbase', 'wheelbase'), ('front_track', 'front_track'), ('wheel_radius', 'wheel_radius')):
            if abs(self.actuator[key] - P[setting]) > 1e-6:
                raise ValueError('actuator model and tuning disagree: ' + key)
        from steering_dynamics import SteeringHistory
        measured_response=self.actuator.get('steering_response')
        self.steering_history=SteeringHistory(measured_response) if measured_response is not None else None
        self.steering_history_direction=1
        self.joint_feedback = None
        self.motion_estimate=None
        self.localization_samples=deque(maxlen=30)
        self.localization_quality=None
        self.localization_pending_pose=None
        self.use_speed_prediction=bool(rospy.get_param('~use_speed_prediction',False))
        self.steering_request_pub=rospy.Publisher('/vehicle/steering_request',String,queue_size=1)
        self.reverse_steering_start=float(rospy.get_param('~reverse_steering_wait_start',.10))
        self.reverse_steering_ready=float(rospy.get_param('~reverse_steering_wait_ready',.035))
        self.reverse_steering_timeout=float(rospy.get_param('~reverse_steering_wait_timeout',4.))
        self.reverse_steering_wait=None
        self.forward_steering_start=float(rospy.get_param('~forward_steering_wait_start',.10))
        self.forward_steering_ready=float(rospy.get_param('~forward_steering_wait_ready',.035))
        self.steering_ready_hold=float(rospy.get_param('~steering_ready_hold',.20))
        self.steering_preload={1:math.radians(float(rospy.get_param('~forward_steering_preload_deg',2.))),-1:math.radians(float(rospy.get_param('~reverse_steering_preload_deg',2.)))}
        if not finite([self.steering_ready_hold]+list(self.steering_preload.values())) or self.steering_ready_hold<=0 or any(v<0 or v>math.radians(5.) for v in self.steering_preload.values()):raise ValueError('invalid steering preload')
        self.steering_limits={}
        for direction,key in [(1,'steering_table'),(-1,'reverse_steering_table')]:
            table=rospy.get_param('/vehicle_bridge/'+key,[])
            limit=math.atan(P['wheelbase']/self.local_radius)
            self.steering_limits[direction]=(math.atan(P['wheelbase']*table[0][1]),math.atan(P['wheelbase']*table[-1][1])) if len(table)>=2 else (-limit,limit)
        if not 0<self.forward_steering_ready<self.forward_steering_start:raise ValueError('invalid forward steering thresholds')
        self.initial_goal_relax_after=float(rospy.get_param('~initial_goal_relax_after',5.))
        self.initial_goal_relax_distance=float(rospy.get_param('~initial_goal_relax_distance',.03))
        if not finite([self.initial_goal_relax_after,self.initial_goal_relax_distance]) or self.initial_goal_relax_after<=0 or self.initial_goal_relax_distance<=0:raise ValueError('invalid initial goal relaxation')
        self.initial_planning_timeout=float(rospy.get_param('~initial_planning_timeout',20.))
        if not finite([self.initial_planning_timeout]) or not 0<self.initial_planning_timeout<=20.:raise ValueError('invalid initial planning timeout')
        self.stage_planning_attempts={}
        self.planning_window_probe=float(rospy.get_param('~planning_window_probe',2.))
        if not finite([self.planning_window_probe]) or self.planning_window_probe<=0:raise ValueError('invalid planning_window_probe')
        if not 0<self.reverse_steering_ready<self.reverse_steering_start or self.reverse_steering_timeout<=0:raise ValueError('invalid reverse steering thresholds')
        self.arrival_locked=False
        self.arrival_tolerance_since=None
        self.arrival_confirm_duration=float(rospy.get_param('~arrival_confirm_duration',.5))
        if not finite([self.arrival_confirm_duration]) or self.arrival_confirm_duration<=0:raise ValueError('invalid arrival confirmation duration')
        self.arrival_hold_angle=0.;self.arrival_hold_direction=1
        self.final_stop_reserve=float(rospy.get_param('~final_stop_reserve',.01))
        self.final_stop_heading_reserve=math.radians(float(rospy.get_param('~final_stop_heading_reserve_deg',.5)))
        if not finite([self.final_stop_reserve,self.final_stop_heading_reserve]) or not 0<=self.final_stop_reserve<self.space_stop_tolerance('final')['position'] or not 0<=self.final_stop_heading_reserve<math.radians(self.space_stop_tolerance('final')['heading_deg']):raise ValueError('invalid final stop reserve')
        self.path_replan_distance=float(rospy.get_param('~path_replan_distance',.05))
        if not finite([self.path_replan_distance]) or self.path_replan_distance<=0:raise ValueError('invalid path_replan_distance')
        self.last_control_stamp = None
        self.margin = float(rospy.get_param('~collision_margin', .04))
        self.wheel_collision_enabled=bool(rospy.get_param('~wheel_collision_enabled',True))
        self.planning_margin = float(rospy.get_param('~planning_collision_margin', .04))
        if not finite([self.planning_margin]) or self.planning_margin < self.margin:
            raise ValueError('planning_collision_margin must be >= collision_margin')
        if self.planning_margin+1e-9<self.margin+P['safety_stop_extra']:
            raise ValueError('planning margin must cover the configured normal safety stop margin')
        self.local_guard_distance = float(rospy.get_param('~local_guard_distance', .10))
        if not finite([self.local_guard_distance]) or self.local_guard_distance < 0:
            raise ValueError('local_guard_distance must be finite and nonnegative')
        self.zero_cost_width = float(rospy.get_param('~zero_cost_line_width', .01))
        self.line_store=LineStore(rospy.get_param('~low_cost_lines_file',os.path.join(
            os.environ.get('SMARTCAR_ROOT',os.path.expanduser('~/.ros/smartcar')),'data','navigation','low_cost_lines.json')))
        self.turn90_trace = None
        turn90_file = rospy.get_param('~turn90_primitive_file', '')
        self.turn90_pre_extension = float(rospy.get_param('~turn90_pre_extension', .30))
        self.turn90_post_extension = float(rospy.get_param('~turn90_post_extension', .30))
        if (not finite([self.turn90_pre_extension, self.turn90_post_extension]) or
                self.turn90_pre_extension < 0. or self.turn90_post_extension < 0.):
            raise ValueError('turn90 straight extensions must be finite and nonnegative')
        if turn90_file:
            try:
                self.turn90_trace = load_turn90_trace(turn90_file)
                rospy.loginfo('TURN90_PRIMITIVE: loaded %d moving samples from %s',
                              len(self.turn90_trace['samples']), turn90_file)
            except Exception as exc:
                raise ValueError('invalid turn90_primitive_file: %s' % exc)
        self.line_map_key=None
        self.load_saved_lines = rospy.get_param('~load_saved_low_cost_lines', False)
        line = [float(rospy.get_param('~zero_cost_line_x1', float('nan'))), float(rospy.get_param('~zero_cost_line_y1', float('nan'))), float(rospy.get_param('~zero_cost_line_x2', float('nan'))), float(rospy.get_param('~zero_cost_line_y2', float('nan')))]
        self.zero_cost_line = [((line[0],line[1]),(line[2],line[3]))] if finite(line) else []
        # Disk records are map-bound. Do not restore unbound ROS parameters
        # left by another map/session. Disk restore is explicitly opt-in.
        self.timeout = float(rospy.get_param('~planning_timeout', 20.))
        self.feasibility_fallback_enabled=bool(rospy.get_param('~feasibility_fallback_enabled',True))
        self.feasibility_fallback_timeout=float(rospy.get_param('~feasibility_fallback_timeout',120.))
        self.feasibility_fallback_max_records=int(rospy.get_param('~feasibility_fallback_max_records',300000))
        if (not finite([self.feasibility_fallback_timeout]) or not 20.<=self.feasibility_fallback_timeout<=180.
                or self.feasibility_fallback_max_records<P['planner_max_records']):
            raise ValueError('fallback must retain at least baseline search time and record allowance')
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
        self.locked_line = None
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
        self.patrol_stage_pub=rospy.Publisher('/single_nav/patrol_stage',String,queue_size=32,latch=True)
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
        rospy.Subscriber('/vehicle/motion_estimate',String,self.on_motion_estimate,queue_size=1)
        rospy.Subscriber('/wall_localization/pose',PoseWithCovarianceStamped,self.on_matched_pose,queue_size=3)
        rospy.Subscriber('/wall_localization/status',String,self.on_localization_status,queue_size=3)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber('/single_nav/inspection_goal', String, self.on_inspection_goal, queue_size=1)
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
        # Observational metadata only. No change to route, control or acceptance.
        token=getattr(self,'patrol_preview_token',None)
        if token is not None and getattr(self,'patrol_stage_pub',None) is not None:
            try:
                self.patrol_stage_pub.publish(String(data=json.dumps(dict(schema_version=1,
                    goal_token=token,status=value,maneuver_mode=getattr(self,'maneuver_mode',None),
                    line_stage=getattr(self,'line_stage',None)))))
            except Exception as exc:rospy.logwarn_throttle(5.,'Patrol metadata unavailable: '+str(exc))
        # Keep terminal outcomes available to late subscribers and RViz panels.
        if value.startswith(('SUCCEEDED:', 'STOPPED:', 'CANCELLED',
                             'FINAL_TOLERANCE_FAILED:', 'REPLAN_LIMIT:',
                             'REPLAN_FAILED:', 'TEST_FAILED:')):
            self.result_pub.publish(msg)
        rospy.loginfo(value)

    def on_matched_pose(self,msg):
        if msg.header.frame_id.lstrip('/')!=P['map_frame']:return
        try:
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            roll,pitch,yaw=tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            stamp=msg.header.stamp.to_sec()
            if (not finite([stamp,p.x,p.y,q.x,q.y,q.z,q.w,roll,pitch,yaw])
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.02
                    or abs(roll)+abs(pitch)>.10):return
            self.localization_pending_pose=(stamp,time.time(),(p.x,p.y,yaw))
            self.accept_matched_pose()
        except (ValueError,TypeError):return

    def on_localization_status(self,msg):
        try:
            d=json.loads(msg.data);stamp=float(d['stamp']);rms=float(d.get('rms',float('inf')))
            good=(d.get('state')=='MATCHED' and d.get('rank')==3
                  and finite([stamp,rms]) and 0<=rms<=.05)
            self.localization_quality=(stamp,time.time(),good)
            self.accept_matched_pose()
        except (ValueError,TypeError,KeyError):self.localization_quality=None

    def accept_matched_pose(self):
        pose=self.localization_pending_pose;quality=self.localization_quality
        if (pose is None or quality is None or not quality[2]
                or not 0<=quality[0]-pose[0]<=P['pose_timeout']
                or abs(quality[1]-pose[1])>.10):return
        if not self.localization_samples or pose[0]>self.localization_samples[-1][0]:
            self.localization_samples.append(pose)

    def on_motion_estimate(self,msg):
        with self.lock:
            try:
                data=json.loads(msg.data)
                if data.get('valid') is not True:self.motion_estimate=None;return
                stamp=float(data['stamp']);velocity=float(data['predicted_velocity'])
                if not finite([stamp,velocity]) or abs(velocity)>.5:raise ValueError('invalid prediction')
                self.motion_estimate=(stamp,time.time(),velocity)
            except (ValueError,TypeError,KeyError):self.motion_estimate=None

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

    def tracking_steering_target(self, reference, part):
        direction=part[-1][3]
        if direction!=getattr(self,'steering_history_direction',direction):
            from steering_dynamics import SteeringHistory
            model=self.actuator.get('steering_response_reverse' if direction<0 else 'steering_response',
                                    self.actuator.get('steering_response'))
            self.steering_history=SteeringHistory(model) if model is not None else None
            self.steering_history_direction=direction
        actual,speed=self.actuator_state()
        sample=getattr(self,'motion_estimate',None)
        if (getattr(self,'use_speed_prediction',False) and sample is not None and
                0<=rospy.Time.now().to_sec()-sample[0]<=P['feedback_timeout'] and
                0<=time.time()-sample[1]<=P['feedback_timeout']):
            speed=sample[2]
        response=None
        if self.steering_history is not None:
            from steering_dynamics import response_clock
            response=self.steering_history.snapshot(response_clock(),actual)
        options=self.reverse_tracking_options if direction<0 else self.tracking_options
        target,unused=tracking_steering(reference,part,self.local_radius,options['lateral_gain'],
                                       options['heading_gain'],response,speed)
        if response is not None:self.steering_history.preview=response.preview
        return target

    def recovery_departure_cap(self, pose, cap):
        origin=getattr(self,'recovery_resume_origin',None)
        if origin is None:return cap
        distance=math.hypot(pose[0]-origin[0],pose[1]-origin[1])
        if distance>=P['recovery_resume_distance']:
            self.recovery_resume_origin=None
            return cap
        base=cap if cap is not None else max(P['tracking_forward_speed'],P['tracking_reverse_speed'])
        initial=min(base,P['recovery_resume_speed'])
        return initial+(base-initial)*distance/P['recovery_resume_distance']

    def steering_pause(self, reference, part, now):
        direction=part[-1][3]
        label='reverse' if direction<0 else 'forward'
        options=self.reverse_tracking_options if direction<0 else self.tracking_options
        start_limit=self.reverse_steering_start if direction<0 else self.forward_steering_start
        ready_limit=self.reverse_steering_ready if direction<0 else self.forward_steering_ready
        actual,speed=self.actuator_state()
        wait=self.reverse_steering_wait
        if wait is not None and (wait['generation']!=self.generation or wait['part']!=self.part):
            self.reverse_steering_wait=None;wait=None
        if wait is None:
            desired=self.tracking_steering_target(reference,part)
            prepared=getattr(self,'steering_ready_key',None)==(self.generation,self.part)
            if prepared and abs(desired-actual)<=start_limit:
                self.steering_ready_key=(self.generation,self.part)
                return False
            target=desired
            if abs(desired)>.05:target+=math.copysign(self.steering_preload[direction],desired)
            lower,upper=self.steering_limits[direction]
            target=max(lower,min(upper,target))
            wait=dict(start=now,ready=None,generation=self.generation,part=self.part,target=target)
            self.reverse_steering_wait=wait
            self.status('STEERING_STOP: %s; lag %.1fdeg; segment_prepared=%s' %
                        (label,math.degrees(abs(desired-actual)),
                         getattr(self,'steering_ready_key',None)==(self.generation,self.part)))
        target=wait['target']
        lag=abs(target-actual)
        # Publish the request even during braking; bridge holds actual angle until stopped.
        self.steering_request_pub.publish(String(data=json.dumps(dict(angle=target,direction=direction))))
        self.cmd.publish(Twist())
        if abs(speed)<=.005 and self.steering_history is not None:
            from steering_dynamics import response_clock
            self.steering_history.commit(response_clock(),target,actual)
        from actuator_model import steering_ready
        if steering_ready(wait,lag,speed,ready_limit,self.steering_ready_hold,now,
                          min(max(0.,start_limit-ready_limit),math.radians(P['steering_ready_hysteresis_deg']))):
            self.reverse_steering_wait=None
            self.steering_ready_key=(self.generation,self.part)
            self.last_movement=self.progress_time=now
            self.status('DRIVING: %s locked steering ready' % label)
            # Keep this release cycle at zero; movement starts on the next tick.
            return True
        self.last_movement=self.progress_time=now
        if now-wait['start']>self.reverse_steering_timeout:
            self.reverse_steering_wait=None
            self.wait_retry('%s locked steering response timeout' % label)
        else:self.status('WAIT_STEERING: %s locked target %.1fdeg; lag %.1fdeg' % (label,math.degrees(target),math.degrees(lag)))
        return True

    def moving_steering_candidate(self, p, part, reference, cap):
        """Same-direction correction only, after this segment's initial preload."""
        if (self.steering_history is None or self.local_guard_distance<=0
                or self.reverse_steering_wait is not None
                or getattr(self,'steering_ready_key',None)!=(self.generation,self.part)):
            return None
        direction=part[-1][3]
        actual,speed=self.actuator_state()
        # A brief noisy odometry sign must not disable the guarded moving path.
        # This is a command reference only; local_candidate retains observed
        # velocity for braking clearance and persistent-opposite-motion checks.
        sample=getattr(self,'motion_estimate',None)
        moving_floor=P['direction_speed_deadband']
        if (getattr(self,'use_speed_prediction',False) and sample is not None and
                0<=rospy.Time.now().to_sec()-sample[0]<=P['feedback_timeout'] and
                0<=time.time()-sample[1]<=P['feedback_timeout']):
            speed=sample[2]
            moving_floor=.001  # Include the first 0.01 m/s departure command.
        options=self.reverse_tracking_options if direction<0 else self.tracking_options
        start=self.reverse_steering_start if direction<0 else self.forward_steering_start
        desired=self.tracking_steering_target(reference,part)
        lag=abs(desired-actual)
        if not (speed*direction>moving_floor and start<lag<=math.radians(P['moving_steer_lag_deg'])
                and reference['distance']<=min(P['moving_steer_error'],self.path_replan_distance*.7)
                and abs(reference['lateral'])<=min(P['moving_steer_error'],self.path_replan_distance*.7)
                and abs(reference['heading'])<=math.radians(P['moving_steer_heading_deg'])):
            return None
        diagnostics={}
        result=self.local_candidate(p,part,diagnostics,cap,path_error_limit=self.path_replan_distance)
        if result is not None and result[1]*direction>moving_floor:
            if not getattr(self,'moving_steering_active',False):
                self.status('DRIVING: moving steering correction %.1fdeg; response and braking within path limit' % math.degrees(lag))
            self.moving_steering_active=True
            return result
        return None

    def confirmed_opposite_motion(self, direction, speed):
        from actuator_model import DirectionPersistence,matched_motion_progress
        guard=getattr(self,'direction_guard',None)
        if guard is None:
            guard=DirectionPersistence(P['direction_speed_deadband'],P['direction_confirm_time'],P['feedback_timeout'])
            self.direction_guard=guard
        now=rospy.Time.now().to_sec()
        confirmed=guard.update(direction,speed,self.joint_feedback[0],now,(self.generation,self.part))
        if not confirmed:return False
        samples=tuple(getattr(self,'localization_samples',()))
        quality=getattr(self,'localization_quality',None)
        fresh=(samples and quality is not None and quality[2]
               and 0<=time.time()-quality[1]<=P['direction_map_max_age']
               and 0<=time.time()-samples[-1][1]<=P['direction_map_max_age'])
        progress=(matched_motion_progress(samples,now,direction,P['direction_map_window'],
                  P['direction_map_min_progress'],P['direction_map_max_age']) if fresh else None)
        if progress is None:return True
        if now-getattr(self,'direction_evidence_report',0.)>=1.:
            self.direction_evidence_report=now
            if hasattr(self,'status'):
                self.status('DIRECTION_EVIDENCE: map matches confirm planned direction %.3fm; odom velocity %.3fm/s inconsistent' % (progress,speed))
        return False

    def local_candidate(self, p, part, diagnostics=None, cap=None,path_error_limit=None,grid_override=None):
        steer, speed = self.actuator_state()
        direction=part[-1][3] if part else 1
        if direction!=self.steering_history_direction:
            from steering_dynamics import SteeringHistory
            model=self.actuator.get('steering_response_reverse' if direction<0 else 'steering_response',
                                    self.actuator.get('steering_response'))
            self.steering_history=SteeringHistory(model) if model is not None else None
            self.steering_history_direction=direction
        if self.steering_history is not None:
            from steering_dynamics import response_clock
            steering_response=self.steering_history.snapshot(response_clock(),steer)
        else:steering_response=None
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
        # Reverse heading damping can be tuned without overdriving forward steering.
        direction = part[-1][3] if part else 1
        options = dict(self.reverse_tracking_options if direction < 0 else self.tracking_options)
        options['direction_confirmed']=self.confirmed_opposite_motion(direction,speed)
        if controller is guarded_track:
            options['guard_distance'] = self.local_guard_distance
            options['observed_speed']=speed
            if path_error_limit is not None:options['path_error_limit']=path_error_limit
        elif path_error_limit is not None:
            # Continuous correction requires the complete braking guard.
            return None
        sample=self.motion_estimate
        if (self.use_speed_prediction and sample is not None and
                0<=rospy.Time.now().to_sec()-sample[0]<=P['feedback_timeout'] and
                0<=time.time()-sample[1]<=P['feedback_timeout']):
            speed=sample[2]
        control_grid=grid_override
        if control_grid is None:
            control_grid=self.grid.snapshot()
            control_grid.margin+=P['safety_stop_extra']
            control_grid.cache.clear()
            if execution_settings(self)['enabled']:
                control_grid=ExecutionGrid(control_grid,execution_settings(self)['depth_margin'])
        candidate = controller(control_grid,p,part,self.index,self.local_radius,steer,diagnostics,cap,
                           control_dt=self.control_dt, current_speed=speed,
                           steering_rate=self.steering_rate, actuator_steer_rate=self.actuator['steer_rate'],
                           acceleration=self.actuator['acceleration'], braking=self.actuator['braking'],
                           steering_response=steering_response,
                           maneuver_mode=mode,wall_steer=wall_steer,wall_weight=P['wall_straight_weight'] if wall_steer is not None else 0.,
                           **options)
        # Candidate rollouts use cloned response states. Only the initial
        # selected command's geometry is retained when that command is sent.
        self.candidate_preview=steering_response.preview if candidate is not None and steering_response is not None else None
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
        success=reason.startswith('SUCCEEDED:')
        if success and not self.ground_truth_test:
            self.arrival_hold_angle=self.actuator_state()[0]
            self.arrival_hold_direction=self.parts[self.part][-1][3] if self.parts else 1
        self.arrival_locked=success
        self.arrival_tolerance_since=None
        rospy.set_param('~arrival_locked',success)
        if (getattr(self, 'persistent_recovery', False) and self.goal is not None
                and reason.startswith(('STOPPED:', 'BLOCKED:', 'TRACKING_FAILED:',
                                       'PLAN_FAILED:', 'FINAL_TOLERANCE_FAILED:',
                                       'REPLAN_FAILED:', 'REPLAN_LIMIT:'))):
            self.wait_retry(reason)
            return
        if self.ground_truth_test and self.path_errors:
            rms = math.sqrt(sum(e*e for e in self.path_errors)/len(self.path_errors))
            self.status('PATH_ERROR: samples=%d rms=%.3fm max=%.3fm' %
                        (len(self.path_errors), rms, max(self.path_errors)))
            self.path_errors = []
        self.generation += 1
        self.parts, self.goal = [], None
        self.goal_position_only = False
        self.stage_planning_attempts={}
        self.execution_heading = None
        self.line_stage = None
        self.line_route = None
        self.stage_target = None
        self.truth_alignment = None
        self.recovery = None
        self.replan_count = 0
        self.relaxed_goal = False
        self.return_context = None
        self.return_pending = False
        self.recovery_checkpoint = None
        self.planning_budget_failures = 0
        self.failed_tracking = []
        self.execution_target = None
        self.failures = ConsecutiveFailures()
        self.state = 'ARRIVED_LOCKED' if success else 'IDLE'
        self.stop_state = None
        self.cmd.publish(Twist())
        if self.ground_truth_test:
            self.publish_virtual_pose(None)
        empty = Path(); empty.header.frame_id = P['map_frame']; empty.header.stamp = rospy.Time.now()
        self.path_pub.publish(empty)
        self.status(reason)

    def remember_stop(self, pose):
        """Only called after pose AND wheel-speed stop confirmation."""
        if not getattr(self, 'persistent_recovery', False) or self.line_stage == 'RETURN_STOP':
            return
        point = tuple(pose[:3])
        if not self.stop_history or math.hypot(point[0]-self.stop_history[-1][0],
                                              point[1]-self.stop_history[-1][1]) > .05:
            self.stop_history.append(point)
            self.recovery_checkpoint = None
            self.planning_budget_failures = 0

    def wait_retry(self, reason):
        if getattr(self,'arrival_locked',False):
            self.cmd.publish(Twist());return
        self.generation += 1
        self.cmd.publish(Twist())
        self.parts = []
        self.recovery_resume_pending=True
        self.steering_ready_key=None
        self.reverse_steering_wait=None
        if self.steering_history is not None:self.steering_history.reset()
        if any(tag in reason for tag in ('LOCAL_PLAN_BLOCKED','tracking deviation','path progress stalled')):
            try:
                p=self.pose()
                history=getattr(self,'failed_tracking',[])
                if not any(math.hypot(p[0]-q[0],p[1]-q[1])<.15 for q in history):
                    self.failed_tracking=(history+[p[:2]])[-8:]
            except Exception:pass
        if reason.startswith('PLAN_FAILED:') and any(tag in reason for tag in (
                'planning budget exceeded', 'line suffix search budget exhausted')):
            self.planning_budget_failures = min(2, getattr(self, 'planning_budget_failures', 0)+1)
        # Only a failed regional search requests a return. Sensor outages do not.
        if reason.startswith('PLAN_FAILED:') and self.relaxed_goal and self.line_stage != 'RETURN_STOP':
            self.return_pending = True
        self.relaxed_goal = True
        self.state = 'WAIT_RETRY'
        self.stop_state = None
        self.retry_after = time.time() + 2.
        self.retry_window = StopWindow()
        self.retry_last_pose = None
        self.retry_scans = set()
        self.retry_geometry_failures=0
        self.laser_depth_escape_floor=None
        self.laser_hold_check=None
        self.status('RECOVERY_WAIT: goal retained; '+reason)

    def recover_retry(self, elapsed):
        self.cmd.publish(Twist())
        now = time.time()
        input_stage='control timing'
        try:
            if not 0 < elapsed <= P['max_control_dt']:
                raise RuntimeError('waiting for control timing')
            input_stage='scan and actuator feedback'
            self.sensors()
            input_stage='command authority'
            self.authority()
            input_stage='map and odom TF'
            pose = self.pose()
            if self.retry_last_pose is not None and (
                    math.hypot(pose[0]-self.retry_last_pose[0], pose[1]-self.retry_last_pose[1]) > .15
                    or abs(wrap(pose[2]-self.retry_last_pose[2])) > .20):
                self.retry_last_pose = pose
                raise RuntimeError('waiting for stable localization')
            self.retry_last_pose = pose
            input_stage='scan-to-map TF'
            self.obstacles()
            input_stage='stop odometry and wheel feedback'
            pose_stopped = self.retry_window.update(now, self.stop_pose(pose))
            observed_speed = self.actuator_state()[1]
            stopped = pose_stopped and abs(observed_speed) <= .005
            self.retry_input_since=None
            if len(self.retry_scans) < 5:
                self.retry_scans.add(self.scan.header.stamp.to_nsec())
            if self.grid.collision(pose,extra=P['safety_stop_extra']):
                if not P['laser_recovery_enabled']:
                    raise RuntimeError('current safety footprint blocked; '+self.grid.blocked_detail(pose))
                if not stopped or len(self.retry_scans)<3 or now<self.retry_after:
                    # Normal accumulation is not a sensor failure. Keep the
                    # window and distinct scan stamps across control cycles.
                    if now-getattr(self,'retry_last_report',0.)>=5.:
                        self.retry_last_report=now
                        self.status('RECOVERY_WAIT: pose_stopped=%s speed=%.4fm/s scans=%d retry_delay=%.2fs; waiting for verified stop and fresh scans before laser recovery' %
                                    (pose_stopped,observed_speed,len(self.retry_scans),max(0.,self.retry_after-now)))
                    return
                if self.begin_laser_recovery(pose,now):return
                geometry_wait(self,'no observed short safe escape',now)
                return
        except RecoveryGeometryError as exc:
            # Valid sensor/stop evidence survives a geometric rejection.
            geometry_wait(self,str(exc),now)
            return
        except Exception as exc:
            self.retry_window = StopWindow()
            self.retry_scans = set()
            if getattr(self,'retry_input_since',None) is None:self.retry_input_since=now
            self.retry_last_input_error=str(exc)
            if now-getattr(self, 'retry_last_report', 0.) >= 5.:
                self.retry_last_report = now
                self.status('RECOVERY_WAIT: check=%s failing_for=%.1fs; %s' %
                            (input_stage,max(0.,now-self.retry_input_since),exc))
            return
        if now < self.retry_after or not stopped or len(self.retry_scans) < 5:
            if now-getattr(self,'retry_last_report',0.)>=5.:
                self.retry_last_report=now
                self.status('RECOVERY_WAIT: normal clearance clear; pose_stopped=%s speed=%.4fm/s scans=%d/5 retry_delay=%.2fs' %
                            (pose_stopped,observed_speed,len(self.retry_scans),max(0.,self.retry_after-now)))
            return
        if self.return_pending:
            # Do not choose the failure stop itself, or silently skip an unsafe
            # latest checkpoint in favour of an older one.
            previous = getattr(self, 'recovery_checkpoint', None)
            if previous is None:
                previous = next((p for p in reversed(self.stop_history)
                                 if math.hypot(p[0]-pose[0], p[1]-pose[1]) > .15), None)
                self.recovery_checkpoint = previous
            self.return_pending = False
            if previous is not None and math.hypot(previous[0]-pose[0], previous[1]-pose[1]) > .15:
                self.return_context = (self.line_stage, self.stage_target)
                self.line_stage, self.stage_target = 'RETURN_STOP', previous
                self.status('RECOVERY_RETURN: previous confirmed stop (%.3f, %.3f)' % previous[:2])
            else:
                self.status('RECOVERY_RETRY: already at recovery stop or no checkpoint; retrying original goal region')
        self.last_pose, self.last_pose_wall = pose, now
        self.last_movement = self.progress_time = now
        self.replan_count += 1
        self.status('RECOVERY_RETRY: attempt %d; no retry limit' % self.replan_count)
        self.start_plan(pose)

    def raw_recovery_grid(self,pose):
        from laser_recovery import LiveScanGrid
        if time.time()-self.scan_received>P['laser_recovery_scan_age']:
            raise RuntimeError('raw recovery scan reception stale')
        now=rospy.Time.now()
        scan=next((s for s in reversed(tuple(self.scans))
                   if 0<=(now-s.header.stamp).to_sec()<=P['laser_recovery_scan_age']
                   and self.tf.canTransform(P['map_frame'],s.header.frame_id,s.header.stamp)
                   and self.tf.canTransform(P['map_frame'],P['base_frame'],s.header.stamp)),None)
        if scan is None:raise RuntimeError('no fresh raw recovery scan with synchronized TF')
        xyz,q=self.tf.lookupTransform(P['map_frame'],scan.header.frame_id,scan.header.stamp)
        roll,pitch,yaw=tf.transformations.euler_from_quaternion(q)
        if abs(roll)+abs(pitch)>.10:raise RuntimeError('laser scan plane tilted during recovery')
        bxyz,bq=self.tf.lookupTransform(P['map_frame'],P['base_frame'],scan.header.stamp)
        base=to_rear((bxyz[0],bxyz[1],tf.transformations.euler_from_quaternion(bq)[2]),'base')
        if math.hypot(base[0]-pose[0],base[1]-pose[1])>.03 or abs(wrap(base[2]-pose[2]))>math.radians(3.):
            raise RuntimeError('recovery scan/pose displacement exceeds synchronization bound')
        raw=LiveScanGrid(self.grid,scan,(xyz[0],xyz[1],yaw),base,
                         getattr(self,'laser_recovery_origin',pose),P['laser_recovery_distance'])
        raw.capture_age=(rospy.Time.now()-scan.header.stamp).to_sec();raw.created=time.time()
        raw.scan_stamp=scan.header.stamp.to_nsec()
        options=execution_settings(self)
        if options['enabled']:
            raw.depth_margin=options['depth_margin']
            escape=raw.enable_depth_escape(options['depth_escape_limit'],options['depth_margin'],
                                           getattr(self,'laser_depth_escape_floor',None))
            if escape.budgets:self.laser_depth_escape_floor=dict(escape.floor)
        return raw

    def begin_laser_recovery(self,pose,now):
        from laser_recovery import recovery_path
        from path_tracking import PreparedPath
        attempts=getattr(self,'laser_recovery_count',0)
        if attempts>=P['laser_recovery_attempts']:
            hold_recovery(self,'attempt limit; original goal retained',now,pose)
            return True
        self.laser_recovery_origin=tuple(pose)
        raw=self.raw_recovery_grid(pose)
        if not raw.free(pose):
            raise RecoveryGeometryError('raw scan does not certify current footprint: '+raw.blocked_detail(pose))
        path=recovery_path(raw,self.grid,pose,self.actuator_state()[0],self.local_radius,now+.20)
        if path is None:return False
        self.generation+=1;self.laser_recovery_count=attempts+1
        self.laser_recovery_part=PreparedPath(path);self.laser_recovery_index=0
        self.laser_recovery_started=time.time();self.reverse_steering_wait=None
        self.steering_ready_key=None
        if self.steering_history is not None:self.steering_history.reset()
        self.state='LASER_RECOVERY'
        self.publish_path(path)
        self.status('LASER_RECOVERY: raw scan only; attempt %d; %.2fm bound, %.2fm/s cap; original goal retained' %
                    (self.laser_recovery_count,P['laser_recovery_distance'],P['laser_recovery_speed']))
        self.status('LASER_RECOVERY_PATH: %s %.3fm; measured steering prepared before motion' %
                    ('reverse' if path[-1][3]<0 else 'forward',
                     sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))))
        if getattr(raw,'depth_escape',None) is not None and raw.depth_escape.budgets:
            self.status('DEPTH_EXIT: bounded reverse-only exit; physical scan/region checks remain active')
        return True

    def tick_laser_recovery(self,pose,now):
        self.obstacles()
        if self.state=='LASER_RECOVERY_HOLD':
            self.cmd.publish(Twist());recheck_recovery_hold(self,pose,now);return
        try:raw=self.raw_recovery_grid(pose)
        except Exception as exc:
            self.cmd.publish(Twist());hold_recovery(self,str(exc),now,pose,recheck=True);return
        if now-self.laser_recovery_started>P['laser_recovery_timeout']:
            self.cmd.publish(Twist());hold_recovery(self,'recovery time limit',now,pose);return
        if not raw.free(pose):
            self.cmd.publish(Twist());hold_recovery(self,'coverage/real obstacle limit',now,pose,recheck=True);return
        rejoined=not self.grid.collision(pose,extra=P['safety_stop_extra']+.01)
        if self.state=='LASER_RECOVERY_STOP' or rejoined:
            self.cmd.publish(Twist())
            if self.state!='LASER_RECOVERY_STOP':
                self.state='LASER_RECOVERY_STOP';self.laser_stop_window=StopWindow()
                self.laser_rejoin_scans=set()
            stopped=self.laser_stop_window.update(now,self.stop_pose(pose)) and abs(self.actuator_state()[1])<=.005
            if rejoined:self.laser_rejoin_scans.add(self.obstacle_stamp)
            else:self.laser_rejoin_scans.clear()
            if stopped and len(self.laser_rejoin_scans)>=3:
                self.status('LASER_RECOVERY_REJOINED: stopped with normal map clearance; replanning original goal')
                self.wait_retry('laser recovery rejoined normal clearance')
            elif stopped and not rejoined:
                self.wait_retry('laser recovery endpoint still inside normal safety clearance')
            return
        part=self.laser_recovery_part
        ref=project(pose,part,self.laser_recovery_index)
        self.laser_recovery_index=ref['index']
        if ref['distance']>self.path_replan_distance:
            self.cmd.publish(Twist());hold_recovery(self,'recovery tracking deviation',now,pose);return
        if ref['remaining']<.015:
            self.state='LASER_RECOVERY_STOP';self.laser_stop_window=StopWindow();self.laser_rejoin_scans=set()
            self.cmd.publish(Twist());return
        if self.steering_pause(ref,part,now):
            return
        old_index=self.index;self.index=self.laser_recovery_index
        try:
            exit_cap=(min(P['laser_recovery_speed'],execution_settings(self)['depth_escape_speed'])
                      if getattr(raw,'depth_escape',None) is not None and raw.depth_escape.budgets else P['laser_recovery_speed'])
            candidate=self.local_candidate(pose,part,cap=exit_cap,
                                           path_error_limit=self.path_replan_distance,grid_override=raw)
        finally:self.index=old_index
        if candidate is None:
            self.cmd.publish(Twist());hold_recovery(self,'raw swept response/braking blocked',now,pose,recheck=True);return
        self.sensors()
        elapsed_model=time.time()-raw.created
        if (time.time()-self.scan_received>P['laser_recovery_scan_age'] or elapsed_model<0
                or raw.capture_age+elapsed_model>P['laser_recovery_scan_age']):
            self.cmd.publish(Twist());hold_recovery(self,'scan aged during trajectory evaluation',now,pose,recheck=True);return
        unused,v,k,steer,path=candidate
        if v*part[-1][3]<=0:raise RuntimeError('recovery candidate direction disagrees with selected path')
        cmd=Twist();cmd.linear.x=v;cmd.angular.z=v*k;self.cmd.publish(cmd)
        if self.steering_history is not None:
            from steering_dynamics import response_clock
            self.steering_history.preview=getattr(self,'candidate_preview',None)
            self.steering_history.commit(response_clock(),steer,self.actuator_state()[0])

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
        grid.wheel_collision_enabled=self.wheel_collision_enabled
        grid.fast_turn90_enabled=bool(rospy.get_param('~fast_turn90_enabled',True))
        grid.fast_turn90_budget=float(rospy.get_param('~fast_turn90_budget',.35))
        grid.fast_turn90_max_detour_ratio=float(rospy.get_param('~fast_turn90_max_detour_ratio',3.))
        grid.fast_turn90_small_distance=float(rospy.get_param('~fast_turn90_small_distance',1.50))
        grid.fast_turn90_large_distance=float(rospy.get_param('~fast_turn90_large_distance',2.12))
        grid.fast_turn90_small_max_changes=int(rospy.get_param('~fast_turn90_small_max_changes',32))
        grid.fast_turn90_cache_enabled=bool(rospy.get_param('~fast_turn90_cache_enabled',True))
        grid.fast_turn90_final_position_offsets_enabled=bool(rospy.get_param('~fast_turn90_final_position_offsets_enabled',False))
        grid.fast_turn90_precise_static_collision=bool(rospy.get_param('~fast_turn90_precise_static_collision',True))
        grid.fast_turn90_separate_sweep=bool(rospy.get_param('~fast_turn90_separate_sweep',True))
        grid.line_heading_from_low_cost_line=bool(rospy.get_param('~line_heading_from_low_cost_line',True))
        grid.fast_turn90_entry_diagnostics_enabled=bool(rospy.get_param('~fast_turn90_entry_diagnostics_enabled',False))
        grid.fast_turn90_channel_width=float(rospy.get_param('~fast_turn90_channel_width',.90))
        grid.fast_turn90_channel_entries=rospy.get_param('~fast_turn90_channel_entries',[])
        grid.fast_turn90_entry_distance=rospy.get_param('~fast_turn90_entry_distance',None)
        grid.fast_turn90_max_changes=int(rospy.get_param('~fast_turn90_max_changes',3))
        grid.fast_turn90_many_cusps_enabled=bool(rospy.get_param('~fast_turn90_many_cusps_enabled',False))
        grid.fast_turn90_tolerances_enabled=bool(rospy.get_param('~fast_turn90_tolerances_enabled',True))
        grid.fast_turn90_large_align_cross_track=float(rospy.get_param('~fast_turn90_large_align_cross_track',0.003))
        grid.fast_turn90_large_align_heading_deg=float(rospy.get_param('~fast_turn90_large_align_heading_deg',0.3))
        grid.fast_turn90_large_final_heading_deg=float(rospy.get_param('~fast_turn90_large_final_heading_deg',5.0))
        grid.fast_turn90_large_final_plan_heading_deg=float(rospy.get_param('~fast_turn90_large_final_plan_heading_deg',1.5))
        grid.fast_turn90_large_final_plan_position=float(rospy.get_param('~fast_turn90_large_final_plan_position',0.03))
        grid.fast_turn90_large_final_position=float(rospy.get_param('~fast_turn90_large_final_position',0.05))
        grid.fast_turn90_large_stage_heading_deg=float(rospy.get_param('~fast_turn90_large_stage_heading_deg',10.0))
        grid.fast_turn90_large_stage_plan_heading_deg=float(rospy.get_param('~fast_turn90_large_stage_plan_heading_deg',1.5))
        grid.fast_turn90_large_stage_plan_position=float(rospy.get_param('~fast_turn90_large_stage_plan_position',0.03))
        grid.fast_turn90_large_stage_position=float(rospy.get_param('~fast_turn90_large_stage_position',0.12))
        grid.fast_turn90_medium_align_cross_track=float(rospy.get_param('~fast_turn90_medium_align_cross_track',0.003))
        grid.fast_turn90_medium_align_heading_deg=float(rospy.get_param('~fast_turn90_medium_align_heading_deg',0.3))
        grid.fast_turn90_medium_final_heading_deg=float(rospy.get_param('~fast_turn90_medium_final_heading_deg',5.0))
        grid.fast_turn90_medium_final_plan_heading_deg=float(rospy.get_param('~fast_turn90_medium_final_plan_heading_deg',1.5))
        grid.fast_turn90_medium_final_plan_position=float(rospy.get_param('~fast_turn90_medium_final_plan_position',0.03))
        grid.fast_turn90_medium_final_position=float(rospy.get_param('~fast_turn90_medium_final_position',0.05))
        grid.fast_turn90_medium_stage_heading_deg=float(rospy.get_param('~fast_turn90_medium_stage_heading_deg',10.0))
        grid.fast_turn90_medium_stage_plan_heading_deg=float(rospy.get_param('~fast_turn90_medium_stage_plan_heading_deg',1.5))
        grid.fast_turn90_medium_stage_plan_position=float(rospy.get_param('~fast_turn90_medium_stage_plan_position',0.03))
        grid.fast_turn90_medium_stage_position=float(rospy.get_param('~fast_turn90_medium_stage_position',0.12))
        grid.fast_turn90_small_align_cross_track=float(rospy.get_param('~fast_turn90_small_align_cross_track',0.02))
        grid.fast_turn90_small_align_heading_deg=float(rospy.get_param('~fast_turn90_small_align_heading_deg',3.0))
        grid.fast_turn90_small_final_heading_deg=float(rospy.get_param('~fast_turn90_small_final_heading_deg',5.0))
        grid.fast_turn90_small_final_plan_heading_deg=float(rospy.get_param('~fast_turn90_small_final_plan_heading_deg',2.0))
        grid.fast_turn90_small_final_plan_position=float(rospy.get_param('~fast_turn90_small_final_plan_position',0.025))
        grid.fast_turn90_small_final_position=float(rospy.get_param('~fast_turn90_small_final_position',0.05))
        grid.fast_turn90_small_stage_heading_deg=float(rospy.get_param('~fast_turn90_small_stage_heading_deg',3.0))
        grid.fast_turn90_small_stage_plan_heading_deg=float(rospy.get_param('~fast_turn90_small_stage_plan_heading_deg',1.0))
        grid.fast_turn90_small_stage_plan_position=float(rospy.get_param('~fast_turn90_small_stage_plan_position',0.03))
        grid.fast_turn90_small_stage_position=float(rospy.get_param('~fast_turn90_small_stage_position',0.05))
        if grid.fast_turn90_tolerances_enabled:
            from fast_turn90 import stop_policy
            for validated_space_tolerances in ('large','medium','small'):
                validation_grid=grid.snapshot()
                validation_grid.fast_turn90_profile_override=validated_space_tolerances
                stop_policy(validation_grid,(0.,0.,0.),self.radius)
        grid.fast_turn90_large_entry_distance=float(rospy.get_param('~fast_turn90_large_entry_distance',1.8))
        grid.fast_turn90_large_retreat_distance=float(rospy.get_param('~fast_turn90_large_retreat_distance',1.8))
        grid.fast_turn90_large_candidate_step=float(rospy.get_param('~fast_turn90_large_candidate_step',0.45))
        grid.fast_turn90_medium_entry_distance=float(rospy.get_param('~fast_turn90_medium_entry_distance',1.5))
        grid.fast_turn90_medium_retreat_distance=float(rospy.get_param('~fast_turn90_medium_retreat_distance',1.5))
        grid.fast_turn90_medium_candidate_step=float(rospy.get_param('~fast_turn90_medium_candidate_step',0.375))
        grid.fast_turn90_small_entry_distance=float(rospy.get_param('~fast_turn90_small_entry_distance',0.8))
        grid.fast_turn90_small_retreat_distance=float(rospy.get_param('~fast_turn90_small_retreat_distance',0.8))
        grid.fast_turn90_small_candidate_step=float(rospy.get_param('~fast_turn90_small_candidate_step',0.2))
        grid.fast_turn90_stage_candidate_count=int(rospy.get_param('~fast_turn90_stage_candidate_count',1))
        if (not finite([grid.fast_turn90_small_distance,grid.fast_turn90_large_distance])
                or not 0<grid.fast_turn90_small_distance<grid.fast_turn90_large_distance):
            raise ValueError('invalid fast space distances')
        if not 2<=grid.fast_turn90_small_max_changes<=64:
            raise ValueError('invalid fast small gear limit')
        if not finite([grid.fast_turn90_budget]) or not 0<grid.fast_turn90_budget<=10.:
            raise ValueError('invalid fast_turn90_budget')
        if not finite([grid.fast_turn90_max_detour_ratio]) or grid.fast_turn90_max_detour_ratio<=0:
            raise ValueError('invalid fast_turn90_max_detour_ratio')
        with self.lock:
            if self.goal is not None:
                self.halt('MAP_CHANGED: resend goal after localization stabilizes')
            self.grid = grid
            self.hit_confirmation = ConfirmedHits()
            key=map_key(msg)
            if key!=self.line_map_key:
                if hasattr(self, 'stop_history'):
                    self.stop_history.clear()
                first=self.line_map_key is None
                self.line_map_key=key;self._pending_line_point=None
                try:
                    saved=self.line_store.load(key) if self.load_saved_lines else None
                    self.zero_cost_line=saved if saved is not None else (self.zero_cost_line if first else [])
                    if saved is None and self.zero_cost_line:self.line_store.save(key,self.zero_cost_line)
                    self.status('ZERO_COST_LINE: %d active lines; saved-line restore %s; draw with RViz L tool' %
                                (len(self.zero_cost_line), 'enabled' if self.load_saved_lines else 'disabled'))
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
        stamp = self.tf.getLatestCommonTime(P['odom_frame'],P['base_frame'])
        # TF may arrive while querying it. Read the comparison clock afterwards.
        age = (rospy.Time.now()-stamp).to_sec()
        if stamp.to_sec()==0 or not 0 <= age <= P['pose_timeout']:
            raise RuntimeError('local odom TF invalid age=%.3fs limit=%.3fs (stale or time reset)' % (age,P['pose_timeout']))
        # Map matching finishes after scan odometry. Hold the latest fresh map
        # correction and advance it with odometry; never query a map TF in its
        # future or silently freeze the vehicle at the previous scan position.
        map_stamp = self.tf.getLatestCommonTime(P['map_frame'],P['base_frame'])
        map_age = (rospy.Time.now()-map_stamp).to_sec()
        if map_stamp.to_sec()==0 or not 0 <= map_age <= P['pose_timeout']:
            raise RuntimeError('map localization TF invalid age=%.3fs limit=%.3fs (stale or time reset)' % (map_age,P['pose_timeout']))
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

    def stop_pose(self, map_pose):
        """Measure standstill in continuous odom, not jittering map correction.

        Map pose remains mandatory for planning, collision and goal acceptance.
        """
        if not getattr(self, 'persistent_recovery', False):
            return map_pose
        stamp = self.tf.getLatestCommonTime(P['odom_frame'], P['base_frame'])
        age = (rospy.Time.now()-stamp).to_sec()
        if stamp.to_sec() == 0 or not 0 <= age <= P['pose_timeout']:
            raise RuntimeError('stop odometry stale')
        xyz, q = self.tf.lookupTransform(P['odom_frame'], P['base_frame'], stamp)
        pose = (xyz[0], xyz[1], tf.transformations.euler_from_quaternion(q)[2])
        if not finite(pose):
            raise RuntimeError('nonfinite stop odometry')
        return pose

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
        points=[]
        self.obstacle_stamp = scan.header.stamp.to_nsec()
        ranges = smooth_ranges(scan.ranges, scan.range_min, scan.range_max)
        for i, distance in enumerate(ranges[::P['scan_stride']]):
            if finite([distance]) and scan.range_min <= distance < min(scan.range_max, P['scan_obstacle_range']):
                a = yaw + scan.angle_min + (P['scan_stride']*i)*scan.angle_increment
                point=(xyz[0]+distance*math.cos(a),xyz[1]+distance*math.sin(a))
                cell = self.grid.cell(*point)
                # A mapped wall is already represented by the static grid.  Do
                # not feed its normal laser return back as LIVE_SCAN, otherwise
                # the safety supervisor sees the same wall twice and may stop.
                if not self.grid.known_static_near(cell[0], cell[1], 1):
                    cells.add(cell)
                    points.append((cell,point))
        if not hasattr(self, 'hit_confirmation'):
            self.hit_confirmation = ConfirmedHits()
        confirmed=self.hit_confirmation.update(cells, scan.header.stamp.to_sec())
        self.grid.set_dynamic(confirmed,[p for cell,p in points if cell in confirmed])

    def live_block_detail(self,pose):
        hit=self.grid.live_collision(pose,extra=P['safety_stop_extra']).get('LIVE_SCAN')
        if hit is None:return ''
        dx,dy=hit[0]-pose[0],hit[1]-pose[1];c,s=math.cos(pose[2]),math.sin(pose[2])
        x,y=c*dx+s*dy,-s*dx+c*dy
        gap=max(abs(x-P['collision_center_x'])-P['collision_half_length'],abs(y)-self.grid.half_width)
        return '; live_point=(%.3f,%.3f) body_point=(%.3f,%.3f) body_axis_gap=%.3fm uncertainty=%.3fm' % (hit[0],hit[1],x,y,gap,P['live_scan_uncertainty'])

    def wait_obstacle(self, reason):
        if getattr(self, 'persistent_recovery', False):
            self.wait_retry(reason)
            return
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

    def wait_path_deviation(self, error, reason="path deviation"):
        # Stop immediately at the existing limit. Only a transient path-distance
        # incident may later resume this exact path; terminal failures cannot.
        self.path_resume_context = ((self.parts, self.part) if reason == 'path deviation' else None)
        self.path_resume_recovery = Recovery(time.time())
        self.generation += 1
        self.cmd.publish(Twist())
        self.state = 'WAIT_TRACKING'
        self.recovery = Recovery(time.time())
        self.stop_window = StopWindow()
        self.status('WAIT_TRACKING: %s %.3fm; path limit %.3fm; stop and recheck retained path' % (reason,error,self.path_replan_distance))

    def wait_tracking(self, error):
        self.path_resume_context = None
        if getattr(self, 'persistent_recovery', False):
            self.wait_retry('tracking deviation %.3fm' % error)
            return
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
        stopped = self.stop_window.update(now, self.stop_pose(p)) and abs(self.actuator_state()[1]) <= .005
        clear = self.grid.free(p)
        decision = self.recovery.update(now, self.obstacle_stamp, clear, stopped)
        resume = False
        context = getattr(self, 'path_resume_context', None)
        if context is not None and context[0] is self.parts and context[1] == self.part:
            part = self.parts[self.part]
            reference = project(p, part, self.index)
            within = (reference['distance'] <= self.path_replan_distance*P['deviation_resume_ratio']
                      and abs(reference['heading']) <= math.radians(P['deviation_resume_heading_deg']))
            # Five fresh clear scans over 0.5 s, a measured stop, a smaller
            # recovery threshold and a checked control candidate are all needed.
            eligible = within and clear
            verdict = self.path_resume_recovery.update(now, self.obstacle_stamp, eligible, stopped)
            if verdict == 'resume':
                if self.steering_history is not None:self.steering_history.reset()
                resume = self.local_candidate(p, part, cap=self.active_speed_cap()) is not None
        if decision == 'resume':
            self.path_resume_context = None
            if resume:
                self.sensors()
                self.index = reference['index']
                self.state = 'DRIVING'
                self.last_movement = self.progress_time = now
                self.reverse_steering_wait = None
                self.steering_ready_key=None
                self.recovery_resume_origin=tuple(p[:2])
                self.status('RESUMING: transient path deviation cleared; stopped, fresh scans and original path verified')
            else:
                self.status('REPLANNING: persistent tracking deviation; stable pose and fresh scans verified')
                self.start_plan(p)
        elif decision in ('replan', 'fail'):
            reason = ('current footprint blocked; '+self.grid.blocked_detail(p)
                      if not clear else 'unable to verify stable stop and fresh clear scans')
            self.halt('TRACKING_FAILED: '+reason)

    def recover_obstacle(self, p, now):
        self.cmd.publish(Twist())
        self.obstacles()
        stopped = self.stop_window.update(now, self.stop_pose(p)) and abs(self.actuator_state()[1]) <= .005
        part = self.parts[self.part]
        clear = self.grid.free(p) and self.local_candidate(p, part) is not None
        decision = self.recovery.update(now, self.obstacle_stamp, clear, stopped)
        if decision == 'resume':
            if 'TRACKING_ENVELOPE' in self.wait_reason or 'LOCAL_PLAN_BLOCKED' in self.wait_reason:
                self.status('REPLANNING: local tracking failure; stable pose and fresh scans verified')
                self.replan_from_current(p, 'local tracking envelope')
            else:
                self.state = 'DRIVING'
                self.steering_ready_key=None
                self.reverse_steering_wait=None
                self.recovery_resume_origin=tuple(p[:2])
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

    def on_inspection_goal(self, msg):
        # Coordinates and constraints travel in ONE message; no parameter/topic race.
        try:
            data = json.loads(msg.data)
            from position_goal import position_only_payload
            position_only=position_only_payload(data)
            regions = (validate_regions(data['regions']) if data['regions']
                       else list(getattr(self,'inspection_regions',[])) if position_only
                       else validate_regions(data['regions']))
            values = [data[k] for k in ('x', 'y', 'yaw')]
            if not finite(values): raise ValueError('Nonfinite inspection goal')
            if P['goal_reference'] != 'front_axle': raise ValueError('Inspection goals require front axle reference')
            target = PoseStamped(); target.header.frame_id = data['frame_id']
            target.pose.position.x, target.pose.position.y = values[:2]
            q = tf.transformations.quaternion_from_euler(0., 0., values[2])
            target.pose.orientation.x, target.pose.orientation.y = q[0], q[1]
            target.pose.orientation.z, target.pose.orientation.w = q[2], q[3]
            token=data.get('preview_token')
            if token is not None and (not isinstance(token,(str,type(u''))) or not 1<=len(token)<=96):
                raise ValueError('Invalid optional patrol preview token')
            self.on_goal(target, (regions, None if position_only else data.get('target_id'),position_only,token))
        except (ValueError, TypeError, KeyError) as exc:
            with self.lock: self.halt('REJECTED: inspection goal: '+str(exc))

    def on_goal(self, msg, inspection_context=None):
        with self.lock:
            self.patrol_preview_token=(inspection_context[3] if inspection_context is not None and len(inspection_context)>3 else None)
            self.laser_recovery_count=0
            self.recovery_resume_pending=False
            self.recovery_resume_origin=None
            if self.goal is None or getattr(self,'arrival_locked',False):
                self.last_control_stamp=None
            self.halt('NEW_GOAL: stopping before planning')
            self.goal_position_only=bool(inspection_context is not None and len(inspection_context)>2 and inspection_context[2])
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
                if self.goal_position_only:yaw=current[2]
                regions = (inspection_context[0] if inspection_context is not None
                           else getattr(self, 'inspection_regions', []))
                target_id = inspection_context[1] if inspection_context is not None else None
                if execution_settings(self)['enabled'] and execution_settings(self)['align_depth_with_channels']:
                    regions=align_regions(regions,execution_settings(self)['depth_margin'])
                sides = dict(getattr(self, 'inspection_sides', {}))
                old = {r['id']: r for r in getattr(self, 'inspection_regions', [])}
                sides = {key: value for key, value in sides.items()
                         if any(r['id'] == key and r == old.get(key) for r in regions)}
                front = front_position(current)
                rules = rules_for_goal(regions, (front[0], front[1], current[2]),
                                       (msg.pose.position.x,msg.pose.position.y,yaw), target_id, sides)
                self.inspection_regions, self.inspection_sides = regions, sides
                self.grid.depth_rules = rules
                if rules:
                    self.status('DEPTH_LIMIT: '+', '.join('%s direction=%+d axis=%.1fdeg max_depth=0.40m width=0.80m' %
                        (r['id'],r['direction'],math.degrees(r.get('axis_yaw',math.pi/2.))) for r in rules))
                delta = wrap(yaw-current[2])
                dx, dy = msg.pose.position.x-current[0], msg.pose.position.y-current[1]
                lateral = -dx*math.sin(current[2]) + dy*math.cos(current[2])
                if getattr(self.grid,'fast_turn90_enabled',False):
                    from fast_turn90 import maneuver_lateral
                    lateral=maneuver_lateral(current,(msg.pose.position.x,msg.pose.position.y,yaw))
                ad, al = abs(delta), abs(lateral)
                if self.goal_position_only:
                    self.maneuver_mode = 'POSITION_ONLY'
                    self.maneuver_heading_tolerance = math.pi
                elif math.radians(60) <= ad <= math.radians(120):
                    self.maneuver_mode = 'TURN_90_LEFT' if delta > 0 else 'TURN_90_RIGHT'
                    self.maneuver_heading_tolerance = math.radians(15.0)
                elif ad >= math.radians(150) and al >= .20:
                    self.maneuver_mode = 'LATERAL_TURN_180'
                    self.maneuver_heading_tolerance = math.radians(20.0)
                elif al >= .20:
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
                if not self.goal_position_only and not self.grid.free(to_rear(self.goal, 'goal')):
                    if getattr(self, 'persistent_recovery', False):
                        self.wait_retry('goal centre blocked; searching original 10cm region')
                        return
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
        """Stop and retry a retained goal after feedback recovers."""
        if getattr(self, 'persistent_recovery', False):
            self.wait_retry(reason)
            return
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
        if (self.maneuver_mode in ('LATERAL', 'LATERAL_TURN_180') and not self.zero_cost_line
                and self.line_stage != 'RETURN_STOP'):
            self.maneuver_mode = 'NORMAL'
            self.line_stage = None
            self.line_route = None
            self.stage_target = None
            self.locked_line = None
            self.status('GOAL_MODE: NORMAL; no low-cost lines, using ordinary navigation')
        generation = self.generation
        requested_goal = self.goal
        position_only=getattr(self,'goal_position_only',False)
        line_target = self.stage_target
        mode = self.maneuver_mode
        relaxed = getattr(self, 'relaxed_goal', False)
        budget = self.timeout
        if mode in ('NORMAL','POSITION_ONLY'):
            budget = max(budget, P['normal_planning_timeout'])
        if (mode in ('LATERAL', 'LATERAL_TURN_180') and self.zero_cost_line
                and self.line_stage in (None, 'APPROACH_LINE')):
            budget = max(budget, P['line_entry_timeout'])
        if self.line_stage == 'RETREAT_LINE':
            budget = max(budget, P['line_suffix_timeout'])
        if relaxed:
            # Grow from this stage's budget, not the unrelated 5s base budget.
            budget = max(budget, min(20., budget * 2**getattr(self, 'planning_budget_failures', 0)))
        budget = min(20., budget)
        attempt_key=(self.line_stage or 'DIRECT',tuple(requested_goal),tuple(line_target) if line_target is not None else None)
        attempts=self.stage_planning_attempts.get(attempt_key,0)
        self.stage_planning_attempts[attempt_key]=attempts+1
        if attempts==0:budget=min(budget,self.initial_planning_timeout)
        self.status('PLANNING_ATTEMPT: stage %s attempt %d budget %.1fs' % (self.line_stage or 'DIRECT',attempts+1,budget))
        sector = (min(self.maneuver_heading_tolerance, math.radians(P['maneuver_plan_heading_deg']))
                  if mode != 'NORMAL' else self.goal_heading_tolerance)
        goal = to_rear(requested_goal, 'goal')
        # A snapshot prevents /scan and map updates mutating a planner's grid.
        grid = self.grid.snapshot()
        grid.live_points = None  # Preserve the deployed global grid collision rules.
        grid.margin = self.planning_margin
        grid.depth_rules = list(self.grid.depth_rules)
        grid.failed_tracking = list(getattr(self, 'failed_tracking', []))
        grid.zero_cost_width = self.zero_cost_width
        grid.zero_cost_line = list(self.zero_cost_line)
        grid.fast_turn90_reference_goal=tuple(requested_goal)
        self._space_stop_start_pose=tuple(pose)
        grid.fast_turn90_start_reference=tuple(pose)
        grid.fast_turn90_profile_override=self.space_stop_tolerance('final')['profile']
        grid.fast_turn90_request_role=('stage' if self.line_stage in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') else 'final')
        # Initial special maneuvers use a bounded window. Any recovery/replan
        # deliberately switches back to full-map search.
        grid.search_mode = (mode if self.replan_count == 0 and not position_only else 'NORMAL')
        if grid.search_mode == 'NORMAL':
            self.status('PLANNING_WINDOW: full map')
        else:
            self.status('PLANNING_WINDOW: special start-goal box + %.2fm margin' %
                        P.get('planner_special_search_margin', 0.))
        # Tracking may stop inside the additional planning clearance while
        # remaining outside the controller's mandatory safety envelope. Use
        # that same safety envelope to escape; never waive actual collisions.
        if relaxed and (not grid.free(pose) or (self.line_stage == 'RETURN_STOP'
                                                and not grid.free(line_target))):
            grid.margin = self.margin
            grid.cache.clear()
            self.status('RECOVERY_CLEARANCE: using controller safety margin %.3fm' % self.margin)
        self.state = 'PLANNING'; self.status('PLANNING')
        if budget > self.timeout:
            self.status('PLANNING_BUDGET: %.1fs' % budget)
        def work():
            try:
                cancel = lambda: self.generation != generation or self.stop_event.is_set()
                planning_started=time.time()
                planning_deadline=planning_started+budget
                def search_plan(search_grid,start,target,radius,seconds,search_cancel,position_tolerance,heading_tolerance,*args,**kwargs):
                    local_deadline=min(planning_deadline,time.time()+seconds)
                    relax_first=attempts==0 and self.line_stage in (None,'FINAL')
                    relax_at=planning_started+self.initial_goal_relax_after
                    window_end=planning_started+self.planning_window_probe
                    expanded=search_grid.search_mode in (None,'NORMAL')
                    relaxed_reported=False
                    while time.time()<local_deadline:
                        if cancel():raise RuntimeError('cancelled')
                        now=time.time()
                        if not expanded and now>=window_end:
                            expanded=True;search_grid.search_mode='NORMAL'
                            with self.lock:
                                if self.generation!=generation:raise RuntimeError('cancelled')
                                self.status('PLANNING_WINDOW: probe exhausted; full map with shared budget')
                        position=position_tolerance
                        phase_end=local_deadline
                        if not expanded:phase_end=min(phase_end,window_end)
                        if relax_first:
                            if now<relax_at:
                                position=min(position_tolerance,.025)
                                phase_end=min(phase_end,relax_at)
                            else:
                                position=self.initial_goal_relax_distance
                                if not relaxed_reported:
                                    with self.lock:
                                        if self.generation!=generation:raise RuntimeError('cancelled')
                                        self.status('PLANNING_GOAL_RELAX: after %.1fs allow %.3fm original-goal offset; shared budget' % (self.initial_goal_relax_after,position))
                                    relaxed_reported=True
                        duration=phase_end-time.time()
                        if duration<=0:continue
                        try:
                            return plan(search_grid,start,target,radius,duration,search_cancel,position,heading_tolerance,*args,
                                        acceptance_offset=max(0.,time.time()-planning_started),**kwargs)
                        except RuntimeError as exc:
                            if cancel():raise RuntimeError('cancelled')
                            if getattr(exc,'search_exhausted',False):
                                if expanded:raise
                                expanded=True;search_grid.search_mode='NORMAL'
                                with self.lock:
                                    if self.generation!=generation:raise RuntimeError('cancelled')
                                    self.status('PLANNING_WINDOW: frontier exhausted; full map immediately')
                    raise RuntimeError('planning budget exceeded; no feasible route')
                try:
                    stage = self.line_stage
                    selected = None
                    if position_only and stage != 'RETURN_STOP':
                        from position_goal import plan_to_position
                        path=plan_to_position(grid,pose,requested_goal,self.radius,budget,cancel,
                                              min(self.goal_position_tolerance,self.initial_goal_relax_distance,.025))
                        stage,target=None,requested_goal
                    elif stage == 'RETURN_STOP':
                        target = line_target
                        path = search_plan(grid,pose,target,self.radius,budget,cancel,
                                    .025,math.radians(P['stage_plan_heading_deg']),'NORMAL')
                    elif stage == 'RETREAT_LINE':
                        path, retreat = plan_line_retreat(
                            grid, pose, self.locked_line, requested_goal, line_target[2],
                            self.radius, budget, cancel)
                        stage, target = 'RETREAT_LINE', retreat
                    elif stage == 'FINAL':
                        # The final approach may finish within the configured
                        # left/right heading window.  The maneuver heading cap
                        # belongs to the line entry/retreat stages and must not
                        # silently reduce the final-goal tolerance.
                        final_sector = self.goal_heading_tolerance
                        path = search_plan(grid,pose,goal,self.radius,budget,cancel,
                                    .08 if relaxed else min(self.goal_position_tolerance, .025),
                                    final_sector,mode,requested_goal,goal_region=relaxed)
                        target = requested_goal
                    elif stage == 'APPROACH_LINE':
                        target = line_target
                        path = search_plan(grid,pose,target,self.radius,budget,cancel,
                                    P['stage_plan_position'],math.radians(P['stage_plan_heading_deg']),mode,
                                    start_reverse_only=stage == 'APPROACH_LINE' and mode == 'LATERAL')
                    else:
                        special = mode in ('LATERAL','LATERAL_TURN_180')
                        primitive = None
                        if mode in ('TURN_90_LEFT', 'TURN_90_RIGHT') and self.turn90_trace:
                            desired = 1 if mode.endswith('LEFT') else -1
                            primitive = plan_turn90_trace(grid, pose, goal, self.turn90_trace,
                                                          desired,
                                                          pre_extension=self.turn90_pre_extension,
                                                          post_extension=self.turn90_post_extension,
                                                          cancel=cancel)
                        if primitive is not None:
                            path, selected = primitive, None
                            stage, target = None, requested_goal
                            self.status('TURN90_PRIMITIVE: verified measured turn + normal suffix')
                        elif mode in ('TURN_90_LEFT', 'TURN_90_RIGHT') and self.turn90_trace:
                            self.status('TURN90_PRIMITIVE: rejected by endpoint/clearance; fallback Hybrid A*')
                        if primitive is None:
                            path, selected = (plan_line_approach(
                            grid,pose,goal,self.radius,budget,cancel,
                            min(self.goal_position_tolerance, .025),min(sector, self.goal_heading_tolerance),mode,requested_goal)
                            if special and grid.zero_cost_line else (None,None))
                            if selected is None:
                                if special and grid.zero_cost_line:
                                    raise RuntimeError('nearest low-cost line planning failed; switching lines is forbidden')
                                path = search_plan(grid,pose,goal,self.radius,budget,cancel,
                                            .08 if relaxed else self.goal_position_tolerance,sector,mode,requested_goal,
                                            start_reverse_only=mode == 'LATERAL',goal_region=relaxed)
                                stage, target = None, requested_goal
                            else:
                                stage, target = 'APPROACH_LINE', selected[1]
                except (RuntimeError,ValueError) as primary_error:
                    if cancel() or not self.feasibility_fallback_enabled:raise
                    with self.lock:
                        if generation!=self.generation:raise RuntimeError('cancelled')
                        self.status('PLANNING_FALLBACK: stay stopped; full map, unrestricted gears, %.1fs; primary=%s' % (self.feasibility_fallback_timeout,primary_error))
                    if position_only and self.line_stage != 'RETURN_STOP':
                        from position_goal import plan_to_position
                        path=plan_to_position(grid,pose,requested_goal,self.radius,self.feasibility_fallback_timeout,cancel,
                                              min(self.goal_position_tolerance,self.initial_goal_relax_distance,.025),
                                              self.feasibility_fallback_max_records)
                        selected,stage,target=None,None,requested_goal
                    else:
                        path,selected,stage,target=plan_reliability_fallback(
                            grid,pose,goal,requested_goal,self.radius,self.feasibility_fallback_timeout,cancel,
                            mode,self.line_stage,line_target,getattr(self,'locked_line',None),
                            min(self.goal_position_tolerance,self.initial_goal_relax_distance,.03),
                            min(self.goal_heading_tolerance,math.radians(4.)),
                            self.feasibility_fallback_max_records)
                with self.lock:
                    if generation != self.generation: return
                    if selected is not None:
                        self.line_route = selected
                        self.locked_line = tuple(tuple(p) for p in grid.zero_cost_line[selected[0]])
                        self.status('VIA_LINE: line %d; entry=(%.2f,%.2f)' %
                                    (selected[0]+1,selected[1][0],selected[1][1]))
                    if stage == 'RETREAT_LINE':
                        self.status('LINE_RETREAT: stop=(%.2f,%.2f); final turn deferred until confirmed stop' % target[:2])
                    self.line_stage = stage
                    self.stage_target = target
                    self.execution_target = (from_rear(path[-1][:3], 'goal')
                                             if relaxed and stage in (None, 'FINAL') else None)
                    self.execution_heading = path[-1][2] if stage or mode != 'NORMAL' else requested_goal[2]
                    self.status('PLAN_STAGE: %s; target=(%.2f,%.2f,%.1fdeg)' %
                                (stage or 'DIRECT',target[0],target[1],math.degrees(self.execution_heading)))
                    self.parts, self.part, self.index = [PreparedPath(part) for part in segments(path)], 0, 0
                    self.steering_ready_key=None
                    self.reverse_steering_wait=None
                    if self.steering_history is not None:self.steering_history.reset()
                    if getattr(self,'recovery_resume_pending',False):
                        self.recovery_resume_origin=tuple(pose[:2])
                        self.recovery_resume_pending=False
                    # Each direction segment has its own anchor and local index.
                    scale = self.test_replay_speed_scale if self.ground_truth_test else 1.
                    self.speed_caps = [speed_profile(part, forward=P['forward_speed']*scale, reverse=P['reverse_speed']*scale)
                                       for part in self.parts]
                    self.last_movement = time.time()
                    self.progress_time = time.time()
                    self.state = 'DRIVING' if self.parts else 'VERIFY_STOP'
                    # A planner can finish between two control ticks, including
                    # a zero-length route already inside the goal region. Never
                    # reuse VERIFY_STOP's timestamp from the previous attempt.
                    self.stop_state = None
                    self.stop_started = time.time()
                    self.stop_window = StopWindow()
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

    def final_target_ready(self, pose, reserve=True):
        fx,fy=from_rear(pose,'goal')[:2]
        distance=math.hypot(fx-self.goal[0],fy-self.goal[1])
        heading=abs(wrap(pose[2]-self.goal[2]))
        tolerance=self.space_stop_tolerance('final')['position']
        angle=math.radians(self.space_stop_tolerance('final')['heading_deg'] if self.maneuver_mode=='NORMAL' else self.space_stop_tolerance('final')['heading_deg'])
        return (distance<=tolerance-(self.final_stop_reserve if reserve else 0.)
                and (getattr(self,'goal_position_only',False)
                     or heading<=angle-(self.final_stop_heading_reserve if reserve else 0.)))

    def confirm_arrival(self, pose, stopped, now):
        if not stopped or not self.final_target_ready(pose,reserve=False):
            self.arrival_tolerance_since=None
            return False
        if self.arrival_tolerance_since is None:self.arrival_tolerance_since=now
        return now-self.arrival_tolerance_since>=self.arrival_confirm_duration

    def finish_stage(self, pose):
        if self.line_stage == 'RETURN_STOP':
            target = self.stage_target
            if (math.hypot(pose[0]-target[0], pose[1]-target[1]) > self.space_stop_tolerance('stage')['position']
                    or abs(wrap(pose[2]-target[2])) > math.radians(self.space_stop_tolerance('stage')['heading_deg'])):
                self.wait_retry('return stop not yet reached')
                return
            self.line_stage, self.stage_target = self.return_context
            self.return_context = None
            self.status('RECOVERY_RETURNED: previous stop confirmed; retrying original goal region')
            self.wait_retry('returned to previous stop')
            return
        self.remember_stop(pose)
        if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE'):
            target = self.stage_target
            error = math.hypot(pose[0]-target[0],pose[1]-target[1])
            heading = abs(wrap(pose[2]-target[2]))
            heading_limit=math.radians(self.space_stop_tolerance('stage')['heading_deg'])
            if (self.line_stage=='RETREAT_LINE' and error<=self.space_stop_tolerance('stage')['position']
                    and heading_limit<heading<=heading_limit+math.radians(P['line_exit_heading_grace_deg'])
                    and self.grid.free(pose)):
                heading_limit+=math.radians(P['line_exit_heading_grace_deg'])
            if error > self.space_stop_tolerance('stage')['position'] or heading > heading_limit:
                self.replan_from_current(pose, 'line stage pose not verified (%.3fm, %.1fdeg)' %
                                         (error, math.degrees(heading)))
                return
            if (self.line_stage == 'APPROACH_LINE'
                    and self.maneuver_mode in ('LATERAL', 'LATERAL_TURN_180')):
                self.line_stage = 'RETREAT_LINE'
                # Keep the confirmed entry heading and locked line for retries.
                self.stage_target = target
                self.status('LINE_CONFIRMED: error %.3fm, %.1fdeg; planning line travel only' %
                            (error,math.degrees(heading)))
                self.start_plan(pose)
                return
            self.line_stage = 'FINAL'
            self.status('LINE_CONFIRMED: error %.3fm, %.1fdeg; line travel stopped, planning final turn from actual pose' %
                        (error,math.degrees(heading)))
            self.start_plan(pose)
            return
        fx, fy = from_rear(pose, 'goal')[:2]
        position_error = math.hypot(fx-self.goal[0],fy-self.goal[1])
        heading_error = abs(wrap(pose[2]-self.goal[2]))
        detail = 'goal-reference error %.3fm, heading error %.2fdeg' % (position_error,math.degrees(heading_error))
        if getattr(self,'goal_position_only',False):detail='goal-reference error %.3fm; terminal heading unrestricted' % position_error
        if position_error <= self.space_stop_tolerance('final')['position'] and (getattr(self,'goal_position_only',False) or heading_error <= math.radians(self.space_stop_tolerance('final')['heading_deg'] if self.maneuver_mode != 'NORMAL' else self.space_stop_tolerance('final')['heading_deg'])):
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
        if getattr(self, 'persistent_recovery', False):
            self.wait_retry('control interval %.3fs' % elapsed)
            return
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
            stopped = self.stop_window.update(now, self.stop_pose(p))
            if self.state == 'WAIT_STOP' and stopped:
                self.start_plan(p)
            elif self.state == 'CUSP' and stopped:
                self.part += 1
                self.index = 0
                self.state = 'DRIVING'
            elif self.state == 'VERIFY_STOP' and stopped:
                if self.line_stage in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') or self.confirm_arrival(p,stopped,now):
                    self.finish_stage(p)
                elif not self.final_target_ready(p,reserve=False):
                    self.arrival_tolerance_since=None
                    self.wait_path_deviation(math.hypot(from_rear(p,'goal')[0]-self.goal[0],from_rear(p,'goal')[1]-self.goal[1]),'arrival outside tolerance after stop')
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

    def space_stop_tolerance(self, kind='final'):
        if kind=='final' and getattr(self,'goal_position_only',False):
            return dict(profile=None,position=P['final_finish_position'],heading_deg=180.,
                        early_position=P['final_early_position'],early_heading_deg=180.)
        # Only the first lateral entry is handed off with wider stop limits.
        # The line-travel planner still starts at the measured stopped pose.
        if (kind=='stage' and getattr(self,'line_stage',None)=='APPROACH_LINE'
                and getattr(self,'maneuver_mode',None) in ('LATERAL','LATERAL_TURN_180')):
            return dict(profile='lateral_entry',position=P['lateral_entry_finish_position'],
                        heading_deg=P['lateral_entry_finish_heading_deg'],
                        early_position=P['lateral_entry_early_position'],
                        early_heading_deg=P['lateral_entry_early_heading_deg'])
        from fast_turn90 import stop_policy
        policy=None
        if getattr(self,'goal',None) is not None and getattr(self,'grid',None) is not None:
            key=id(self.goal)
            if getattr(self,'_space_stop_goal_ref',None) is not self.goal:
                snapshot=self.grid.snapshot()
                snapshot.zero_cost_line=list(self.zero_cost_line)
                snapshot.fast_turn90_reference_goal=tuple(self.goal)
                snapshot.fast_turn90_start_reference=getattr(self,'_space_stop_start_pose',None)
                self._space_stop_policy=stop_policy(snapshot,self.goal,self.radius)
                self._space_stop_goal_key=key
                self._space_stop_goal_ref=self.goal
            policy=self._space_stop_policy
        if policy is not None:
            return dict(profile=policy['profile'],position=policy[kind+'_position'],
                        heading_deg=policy[kind+'_heading_deg'],
                        early_position=policy[kind+'_position']*.75,
                        early_heading_deg=policy[kind+'_heading_deg']*.75)
        prefix='stage' if kind=='stage' else 'final'
        position=P.get(prefix+'_finish_position',.10)
        heading=P.get(prefix+'_finish_heading_deg',20.)
        if kind=='final':
            if getattr(self,'maneuver_mode','NORMAL')=='NORMAL':heading=P.get('normal_finish_heading_deg',heading)
        return dict(profile=None,position=position,
                    heading_deg=heading,
                    early_position=P.get(prefix+'_early_position',.06),
                    early_heading_deg=P.get(prefix+'_early_heading_deg',4.))

    def early_arrival_tolerances(self):
        # FINAL is also a nonempty line-stage name, but it must use the
        # final-goal thresholds rather than the looser intermediate stop.
        transition = self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE', 'RETURN_STOP')
        return (self.space_stop_tolerance('stage')['early_position'] if transition else self.space_stop_tolerance('final')['early_position'],
                math.radians(self.space_stop_tolerance('stage')['early_heading_deg'] if transition else self.space_stop_tolerance('final')['early_heading_deg']))

    def tick(self):
        stamp = rospy.Time.now().to_sec()
        elapsed = P['control_period'] if self.last_control_stamp is None else stamp-self.last_control_stamp
        self.last_control_stamp = stamp
        if getattr(self,'arrival_locked',False):
            self.steering_request_pub.publish(String(data=json.dumps(dict(angle=self.arrival_hold_angle,direction=self.arrival_hold_direction))))
            self.cmd.publish(Twist())
            return
        if self.goal is None:
            # Idle node never competes with a manual source; entering IDLE sent zero.
            return
        if self.state == 'WAIT_RETRY':
            self.recover_retry(elapsed)
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
        if self.state in ('LASER_RECOVERY','LASER_RECOVERY_STOP','LASER_RECOVERY_HOLD'):
            self.tick_laser_recovery(p,now)
            return
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
            stopped = self.stop_window.update(now, self.stop_pose(p))
            if self.state != 'PLANNING' and now-self.stop_started > P['stop_timeout']:
                raise RuntimeError('stop not verified within configured timeout: vehicle motion or localization drift; check pose and resend goal')
            self.cmd.publish(Twist())
            # Pose stability alone cannot prove that the wheels have stopped.
            stopped = stopped and abs(self.actuator_state()[1]) <= .005
            if not stopped:self.arrival_tolerance_since=None
            if self.state == 'WAIT_STOP' and stopped:
                self.remember_stop(p)
                self.obstacles(); self.start_plan(p)
            elif self.state == 'VERIFY_STOP' and stopped:
                if self.line_stage in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') or self.confirm_arrival(p,stopped,now):
                    self.finish_stage(p)
                elif not self.final_target_ready(p,reserve=False):
                    self.arrival_tolerance_since=None
                    self.wait_path_deviation(math.hypot(from_rear(p,'goal')[0]-self.goal[0],from_rear(p,'goal')[1]-self.goal[1]),'arrival outside tolerance after stop')
            elif self.state == 'CUSP' and stopped:
                self.remember_stop(p)
                self.obstacles()
                next_part=self.parts[self.part+1]
                # Reconnect only after a real stop, using the measured pose.
                # A stale or geometrically unreachable next segment is replanned.
                next_ref=project(p,next_part,0)
                endpoint=self.parts[self.part][-1]
                if (self.grid.collision(p,extra=P['safety_stop_extra'])
                        or math.hypot(p[0]-endpoint[0],p[1]-endpoint[1])>execution_settings(self)['cusp_stop_distance']+.02
                        or next_ref['distance']>self.path_replan_distance
                        or abs(next_ref['heading'])>math.radians(10.)):
                    self.wait_retry('cusp stopped pose cannot safely reconnect next segment')
                    return
                self.part+=1; self.index=0; self.state='DRIVING'; self.last_movement=now
                self.progress_time=now
            return
        self.stop_state = None
        self.obstacles()
        if self.grid.collision(p,extra=P['safety_stop_extra']):
            sources=','.join(sorted(self.grid.collision(p,detailed=True,extra=P['safety_stop_extra'])))
            self.wait_obstacle('current safety footprint blocked; sources=[%s]; margin=%.3fm' %
                               (sources,self.margin+P['safety_stop_extra'])+self.live_block_detail(p))
            return
        if (getattr(self,'goal_position_only',False) and self.line_stage != 'RETURN_STOP'
                and self.final_target_ready(p)):
            self.state='VERIFY_STOP';self.arrival_tolerance_since=None;self.still_since=None;self.cmd.publish(Twist());return
        part=self.parts[self.part]
        reference=project(p,part,self.index)
        index,remaining=reference['index'],reference['remaining']
        if index > self.index: self.progress_time=now
        self.index=index
        lateral=reference['distance']
        if lateral > self.path_replan_distance:
            self.wait_path_deviation(lateral)
            return
        if lateral > P['tracking_max_error']:
            self.wait_tracking(lateral)
            return
        final=self.part == len(self.parts)-1
        fx,fy=from_rear(p, 'goal')[:2]
        target_xy = (self.stage_target[:2] if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE', 'RETURN_STOP')
                     else (getattr(self, 'execution_target', None) or self.goal)[:2])
        actual_xy = p[:2] if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE', 'RETURN_STOP') else (fx,fy)
        early_heading = (self.stage_target[2] if self.line_stage in ('APPROACH_LINE', 'RETREAT_LINE', 'RETURN_STOP')
                         else self.goal[2])
        early_position, early_angle = self.early_arrival_tolerances()
        inside_region = (not getattr(self, 'relaxed_goal', False)
                         or self.line_stage not in (None, 'FINAL')
                         or math.hypot(fx-self.goal[0], fy-self.goal[1]) <= .10)
        terminal_ready = self.line_stage in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') or self.final_target_ready(p)
        if final and self.line_stage not in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') and self.final_target_ready(p):
            self.state='VERIFY_STOP';self.arrival_tolerance_since=None;self.still_since=None;self.cmd.publish(Twist());return
        if final and inside_region and terminal_ready and math.hypot(actual_xy[0]-target_xy[0],actual_xy[1]-target_xy[1]) <= early_position and abs(wrap(p[2]-early_heading)) <= early_angle:
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        cusp_stop=(execution_settings(self)['cusp_stop_distance'] if execution_settings(self)['enabled'] else .035)
        if not final and remaining <= cusp_stop+1e-9:
            self.state='CUSP';self.still_since=None;self.cmd.publish(Twist());return
        if final and index >= len(part)-2 and remaining < .005:
            if self.line_stage not in ('APPROACH_LINE','RETREAT_LINE','RETURN_STOP') and not self.final_target_ready(p,reserve=False):
                self.wait_path_deviation(math.hypot(fx-self.goal[0],fy-self.goal[1]),'terminal target outside tolerance')
                return
            self.state='VERIFY_STOP';self.still_since=None;self.cmd.publish(Twist());return
        if now-self.last_movement > P['progress_timeout']:
            raise RuntimeError('no measurable motion within configured timeout')
        if now-self.progress_time > P['progress_timeout']:
            self.replan_from_current(p, 'path progress stalled beyond configured timeout')
            return
        diagnostics = {}
        cap = self.active_speed_cap()
        if not final and execution_settings(self)['enabled']:
            limit=cusp_speed_limit(remaining,self.actuator['braking'],execution_settings(self))
            cap=limit if cap is None else min(abs(cap),limit)
        cap = self.recovery_departure_cap(p,cap)
        candidate = self.moving_steering_candidate(p,part,reference,cap)
        if candidate is None:
            self.moving_steering_active=False
            if self.steering_pause(reference,part,now):
                return
            candidate = self.local_candidate(p,part,diagnostics,cap)
        if candidate is None:
            self.local_pub.publish(Path())
            self.wait_obstacle('LOCAL_PLAN_BLOCKED: '+explain(diagnostics))
            return
        _,v,k,steer,local_path = candidate
        if v*part[-1][3]<=0:raise RuntimeError('tracking candidate direction disagrees with planned segment')
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
        if self.steering_history is not None:
            from steering_dynamics import response_clock
            self.steering_history.preview=getattr(self,'candidate_preview',None)
            self.steering_history.commit(response_clock(),steer,self.actuator_state()[0])

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
