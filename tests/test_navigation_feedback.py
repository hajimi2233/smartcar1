"""Exercise navigation integration without ROS using lightweight message stubs."""
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


def load_navigation():
    stubs = {}
    for package, classes in {
        'geometry_msgs.msg': ('PoseStamped', 'Twist', 'PointStamped', 'Point'),
        'nav_msgs.msg': ('OccupancyGrid', 'Path', 'Odometry'),
        'sensor_msgs.msg': ('LaserScan', 'JointState'),
        'std_msgs.msg': ('String',),
        'std_srvs.srv': ('Trigger', 'TriggerResponse'),
        'visualization_msgs.msg': ('Marker',),
    }.items():
        module = types.ModuleType(package)
        for name in classes:
            if name == 'String':
                setattr(module, name, type(name, (), {
                    '__init__': lambda self, **kwargs: self.__dict__.update(kwargs)}))
            elif name == 'Twist':
                setattr(module, name, type(name, (), {
                    '__init__': lambda self: self.__dict__.update(
                        linear=types.SimpleNamespace(x=0.),
                        angular=types.SimpleNamespace(z=0.))}))
            else:
                setattr(module, name, type(name, (), {}))
        stubs[package] = module
    for name in ('rospy', 'rosgraph', 'tf'):
        stubs[name] = types.ModuleType(name)
    stubs['rospy'].Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 10.))
    path = Path(__file__).resolve().parents[1] / 'src/smartcar_navigation/scripts/single_goal_nav.py'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('navigation_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class FeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_navigation()

    def node(self):
        n = self.module.Navigator.__new__(self.module.Navigator)
        n.joint_feedback = None
        n.ground_truth_test = False
        n.scan = None
        n.tracking_options = dict(lateral_gain=6.0,heading_gain=4.,preview_distance=.35)
        n.actuator = dict(wheelbase=.62, front_track=.45, wheel_radius=.09,
                          acceleration=.3, braking=.5, steer_rate=2.5)
        return n

    def test_guard_controller_receives_measured_actuator_state(self):
        n=self.node()
        n.local_guard_distance=.1
        n.grid=object();n.index=2;n.local_radius=1.1
        n.steering_rate=1.;n.control_dt=.05
        n.actuator_state=lambda:(.2,.04)
        with patch.object(self.module,'guarded_track',return_value='guarded') as guard:
            self.assertEqual(n.local_candidate((0,0,0),[]),'guarded')
            self.assertEqual(guard.call_args[1]['guard_distance'],.1)
            self.assertEqual(guard.call_args[1]['current_speed'],.04)

    def test_feedback_freshness_both_clocks(self):
        n = self.node()
        with patch.object(self.module.time, 'time', return_value=20.):
            for sample in (None, (9.,20.,.2,.03), (10.,19.,.2,.03), (11.,20.,.2,.03)):
                n.joint_feedback = sample
                with self.assertRaises(RuntimeError):
                    n.actuator_state()
            n.joint_feedback = (9.9,19.9,.2,.03)
            self.assertEqual(n.actuator_state(), (.2,.03))

    def test_small_clock_delivery_skew_is_allowed(self):
        n = self.node()
        with patch.object(self.module.time, 'time', return_value=20.):
            n.joint_feedback = (10.002,19.99,.2,.03)
            self.assertEqual(n.actuator_state(), (.2,.03))
            for sample in ((10.05,20.,.2,.03), (9.69,20.,.2,.03),
                           (10.002,19.69,.2,.03)):
                n.joint_feedback = sample
                with self.assertRaises(RuntimeError):
                    n.actuator_state()

    def test_local_uses_feedback_and_separate_radius(self):
        n = self.node()
        n.grid = object(); n.index = 2; n.radius = 1.3; n.local_radius = 1.2
        n.steering_rate = 1.; n.control_dt = .12
        n.actuator_state = lambda: (.24,.04)
        with patch.object(self.module,'choose',return_value='candidate') as choose:
            self.assertEqual(n.local_candidate((0,0,0),[]), 'candidate')
            args, kwargs = choose.call_args
            self.assertEqual(args[4:6], (1.2,.24))
            self.assertEqual(kwargs['current_speed'], .04)
            self.assertEqual(kwargs['control_dt'], .12)
            self.assertEqual(kwargs['actuator_steer_rate'], 2.5)
        self.assertEqual(n.radius, 1.3)

    def test_replay_drives_without_collision_or_sensor_checks(self):
        from unittest.mock import Mock
        n = self.node()
        n.ground_truth_test = True
        n.test_replay_speed_scale = 2.
        n.state = 'DRIVING'; n.stop_state = None
        n.parts = [[(0.,0.,0.,1,.5), (1.,0.,0.,1,.5)]]
        n.part = 0; n.index = 0; n.speed_caps = [[.08,.08]]
        n.path_errors = []; n.error_pub = Mock(); n.cmd = Mock()
        n.grid = Mock(); n.sensors = Mock(side_effect=AssertionError('sensors used'))
        n.authority = Mock(); n.pose = Mock(return_value=(0.,.08,.15))
        n.tick_test()
        self.assertEqual(len(n.path_errors), 1)
        self.assertAlmostEqual(n.cmd.publish.call_args[0][0].linear.x, .08)
        self.assertAlmostEqual(n.cmd.publish.call_args[0][0].angular.z, .04)
        n.grid.free.assert_not_called()
        n.sensors.assert_not_called()

    def test_paused_or_long_interval_retains_goal(self):
        from unittest.mock import Mock
        for previous in (10., 9.):
            n = self.node(); n.goal = (1,0,0); n.state = 'DRIVING'
            n.last_control_stamp = previous; n.wait_control = Mock()
            n.tick()
            n.wait_control.assert_called_once()
            self.assertEqual(n.goal, (1,0,0))
        n.last_control_stamp = 11.
        with self.assertRaises(RuntimeError): n.tick()

    def test_control_wait_invalidates_plan_and_stops(self):
        from unittest.mock import Mock
        n=self.node(); n.generation=3; n.goal=(1,0,0)
        n.cmd=Mock(); n.status=Mock(); n.parts=[1]
        n.wait_control(.25)
        self.assertEqual(n.generation,4)
        self.assertEqual(n.state,'WAIT_CONTROL')
        self.assertEqual(n.goal,(1,0,0))
        self.assertEqual(n.parts,[])
        n.cmd.publish.assert_called_once()

    def test_control_recovery_replans_only_after_verified_stop(self):
        from unittest.mock import Mock
        n=self.node(); n.cmd=Mock(); n.halt=Mock()
        n.control_wait_started=18.; n.control_stable_since=18.
        n.control_stop_window=Mock(); n.control_stop_window.update.return_value=True
        n.control_recovery=Mock(); n.control_recovery.update.return_value='wait'
        n.sensors=Mock(); n.authority=Mock(); n.pose=Mock(return_value=(0,0,0))
        n.obstacles=Mock(); n.grid=Mock(); n.grid.free.return_value=True
        n.scan=Mock(); n.actuator_state=lambda:(0,0)
        n.replan_from_current=Mock()
        with patch.object(self.module.time,'time',return_value=20.):
            n.recover_control(.05)
            n.replan_from_current.assert_not_called()
            n.control_recovery.update.return_value='resume'
            n.recover_control(.05)
            n.replan_from_current.assert_called_once()
        with patch.object(self.module.time,'time',return_value=29.):
            n.recover_control(.05)
            n.halt.assert_called_once()

    def test_bad_joint_message_clears_old_feedback(self):
        n = self.node(); n.joint_feedback = (10.,20.,0.,0.)
        n.on_joints(types.SimpleNamespace(name=[],position=[],velocity=[]))
        self.assertIsNone(n.joint_feedback)

    def test_speed_cap_uses_current_direction_segment(self):
        n=self.node(); n.speed_caps=[[.07,.06,0.],[0.,.025,.02,0.]]
        n.part=1; n.index=0
        self.assertEqual(n.active_speed_cap(),.025)
        n.index=2
        self.assertEqual(n.active_speed_cap(),.02)
        n.index=3
        self.assertEqual(n.active_speed_cap(),0.)

    def test_line_stage_advances_only_after_pose_confirmation(self):
        from unittest.mock import Mock
        n = self.node()
        n.line_stage = 'APPROACH_LINE'
        n.maneuver_mode = 'LATERAL'
        n.stage_target = (1., .3, 0.)
        n.line_route = (0, n.stage_target)
        n.zero_cost_line = [((0., 0.), (2., 0.))]
        n.goal = (1.8, .4, 0.)
        n.grid = Mock()
        n.grid.free.return_value = True
        n.status = Mock(); n.start_plan = Mock(); n.replan_from_current = Mock()
        n.finish_stage((.7, .3, 0.))
        n.replan_from_current.assert_called_once()
        n.start_plan.assert_not_called()
        n.replan_from_current.reset_mock()
        n.finish_stage((1.02, .3, .02))
        self.assertEqual(n.line_stage, 'RETREAT_LINE')
        n.start_plan.assert_called_once()

    def test_turning_lateral_confirms_second_line_point_before_final(self):
        from unittest.mock import Mock
        n = self.node()
        n.line_stage = 'APPROACH_LINE'
        n.maneuver_mode = 'LATERAL_TURN_180'
        n.stage_target = (0., 0., math.pi)
        n.line_route = (0, n.stage_target)
        n.zero_cost_line = [((0., 0.), (2., 0.))]
        n.goal = (1.8, .4, 0.)
        n.grid = Mock()
        n.grid.free.return_value = True
        n.status = Mock(); n.start_plan = Mock(); n.replan_from_current = Mock(); n.halt = Mock()
        n.finish_stage((.02, 0., math.pi))
        self.assertEqual(n.line_stage, 'RETREAT_LINE')
        self.assertAlmostEqual(n.stage_target[0], 3.3)
        self.assertEqual(n.start_plan.call_count, 1)
        n.finish_stage((3.31, 0., math.pi))
        self.assertEqual(n.line_stage, 'FINAL')
        self.assertEqual(n.start_plan.call_count, 2)
        n.halt.assert_not_called()

    def test_final_verification_uses_requested_pose_and_halved_limits(self):
        from unittest.mock import Mock
        n = self.node()
        n.line_stage = 'FINAL'; n.goal = (.62, 0., 0.)
        n.maneuver_mode = 'LATERAL'; n.execution_heading = .3
        n.halt = Mock()
        n.finish_stage((.08, 0., 0.))
        self.assertTrue(n.halt.call_args[0][0].startswith('FINAL_TOLERANCE_FAILED:'))
        n.finish_stage((.62-.62*math.cos(.09), -.62*math.sin(.09), .09))
        self.assertTrue(n.halt.call_args[0][0].startswith('FINAL_TOLERANCE_FAILED:'))
        n.finish_stage((.69-.62*math.cos(.08), -.62*math.sin(.08), .08))
        self.assertTrue(n.halt.call_args[0][0].startswith('SUCCEEDED:'))

class LocalizationTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_navigation()

    class Stamp:
        def __init__(self, value): self.value = value
        def to_sec(self): return self.value
        def __sub__(self, other): return type(self)(self.value-other.value)

    def setup_pose(self, odom_time=666.458, map_time=666.358):
        module=self.module
        n=module.Navigator.__new__(module.Navigator);n.ground_truth_test=False
        calls=[];stamp=self.Stamp
        def latest(parent,child):
            return stamp(odom_time if parent==module.P['odom_frame'] else map_time)
        def lookup(parent,child,t):
            calls.append((parent,child,t.to_sec()))
            if parent==module.P['map_frame'] and child==module.P['odom_frame']:
                if t.to_sec()>map_time:raise RuntimeError('future map TF')
                return (1.,2.,0.),math.pi/2
            if parent==module.P['odom_frame'] and child==module.P['base_frame']:
                return (3.,4.,0.),.2
            raise AssertionError('Must not query map->base at newest odometry time')
        n.tf=types.SimpleNamespace(getLatestCommonTime=latest,lookupTransform=lookup)
        return n,calls

    def test_one_scan_map_delay_uses_current_odometry(self):
        m=self.module;n,calls=self.setup_pose()
        with patch.object(m.rospy,'Time',types.SimpleNamespace(now=lambda:self.Stamp(666.492))), \
             patch.object(m.tf,'transformations',types.SimpleNamespace(euler_from_quaternion=lambda q:(0,0,q)),create=True), \
             patch.object(m,'to_rear',lambda p,ref:p):
            pose=n.pose()
        self.assertAlmostEqual(pose[0],-3.)
        self.assertAlmostEqual(pose[1],5.)
        self.assertAlmostEqual(pose[2],math.pi/2+.2)
        self.assertEqual([c[2] for c in calls],[666.358,666.458])

    def test_fresh_odometry_does_not_hide_stale_localization(self):
        m=self.module;n,calls=self.setup_pose(map_time=665.8)
        with patch.object(m.rospy,'Time',types.SimpleNamespace(now=lambda:self.Stamp(666.492))):
            with self.assertRaisesRegex(RuntimeError,'map localization TF stale'):n.pose()
        self.assertEqual(calls,[])

    def test_stale_odometry_still_stops(self):
        m=self.module;n,calls=self.setup_pose(odom_time=665.8)
        with patch.object(m.rospy,'Time',types.SimpleNamespace(now=lambda:self.Stamp(666.492))):
            with self.assertRaisesRegex(RuntimeError,'local odom TF stale'):n.pose()


class ExecutedTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.module=load_navigation()

    def test_wall_returns_cannot_rewrite_validated_command(self):
        m=self.module
        for mode in ('NORMAL','STRAIGHT','TURN_90_LEFT'):
            for left,right in ((1.,1.),(.4,1.2),(1.2,.4)):
                for angle in (.05,-.05):
                    n=m.Navigator.__new__(m.Navigator)
                    n.actuator_state=lambda:(0.,.05)
                    n.grid=object();n.index=0;n.local_radius=1.1;n.control_dt=.05;n.steering_rate=1.
                    n.actuator=dict(steer_rate=2.5,acceleration=.3,braking=.5)
                    n.tracking_options={};n.maneuver_mode=mode
                    n.scan=types.SimpleNamespace(ranges=[right,left],angle_min=-1.,angle_increment=2.,range_min=.05,range_max=25.)
                    candidate=(.1,.05,math.tan(angle)/m.P['wheelbase'],angle,[(0.,0.,0.)])
                    with patch.object(m,'choose',return_value=candidate):
                        self.assertIs(n.local_candidate((0,0,0),[]),candidate)

    def test_no_safe_candidate_remains_no_command(self):
        m=self.module;n=m.Navigator.__new__(m.Navigator)
        n.actuator_state=lambda:(0.,0.)
        n.grid=object();n.index=0;n.local_radius=1.1;n.control_dt=.05;n.steering_rate=1.
        n.actuator=dict(steer_rate=2.5,acceleration=.3,braking=.5);n.tracking_options={}
        with patch.object(m,'choose',return_value=None):self.assertIsNone(n.local_candidate((0,0,0),[]))

if __name__ == '__main__':
    unittest.main()

class ScanFilterTests(unittest.TestCase):
    def test_invalid_returns_skipped_and_free_space_obstacle_kept(self):
        from ackermann_core import Grid
        m=load_navigation()
        class Stamp:
            def __sub__(self, other): return self
            def to_sec(self): return 0.
            def to_nsec(self): return 100
        m.rospy.Time=types.SimpleNamespace(now=lambda:Stamp())
        m.tf.transformations=types.SimpleNamespace(euler_from_quaternion=lambda q:(0,0,0))
        n=m.Navigator.__new__(m.Navigator)
        data=[0]*40000
        g=Grid(200,200,.02,(-2,-2,0),data,.02)
        wall=g.cell(.4,0);data[wall[1]*200+wall[0]]=100
        n.grid=Grid(200,200,.02,(-2,-2,0),data,.02)
        n.scan_timeout=1.
        n.hit_confirmation=types.SimpleNamespace(update=lambda cells, stamp: cells)
        scan=types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(),frame_id='laser'),
             ranges=[float('nan'),float('inf'),.4,.6,.01],range_min=.05,range_max=20.,angle_min=0.,angle_increment=0.)
        n.scans=[scan]
        n.tf=types.SimpleNamespace(canTransform=lambda *a:True,lookupTransform=lambda *a:((0,0,0),(0,0,0,1)))
        with patch.object(m,'smooth_ranges',side_effect=lambda ranges,*a:ranges), patch.dict(m.P,scan_stride=1):
            n.obstacles()
        self.assertEqual(n.grid.dynamic,{n.grid.cell(.6,0.)})

class ScanDeliveryTests(unittest.TestCase):
    def test_scan_callback_does_not_wait_for_navigation_lock(self):
        from collections import deque
        m=load_navigation();n=m.Navigator.__new__(m.Navigator)
        class HeldLock:
            def __enter__(self): raise AssertionError('scan blocked by planner')
            def __exit__(self,*args): pass
        n.lock=HeldLock();n.scans=deque(maxlen=8)
        msg=object()
        n.on_scan(msg)
        self.assertIs(n.scan,msg)
        self.assertEqual(list(n.scans),[msg])

class ArrivalStageTests(unittest.TestCase):
    def test_final_line_stage_uses_final_not_intermediate_tolerance(self):
        m=load_navigation();n=m.Navigator.__new__(m.Navigator)
        for stage in (None,'FINAL'):
            n.line_stage=stage
            self.assertEqual(n.early_arrival_tolerances(),(m.P['final_early_position'],math.radians(m.P['final_early_heading_deg'])))
            self.assertLess(n.early_arrival_tolerances()[0],m.P['final_finish_position'])
        for stage in ('APPROACH_LINE','RETREAT_LINE'):
            n.line_stage=stage
            self.assertEqual(n.early_arrival_tolerances()[0],m.P['stage_early_position'])
