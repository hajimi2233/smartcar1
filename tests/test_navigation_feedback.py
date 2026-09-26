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
    path = Path(__file__).resolve().parents[1] / 'docker/overlay/single_goal_nav.py'
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
