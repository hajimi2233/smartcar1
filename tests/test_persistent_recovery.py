"""Retained mission recovery and real Hybrid A* goal-region regressions."""
import math
import threading
import time
import types
import unittest
from collections import deque
from unittest.mock import Mock, patch
from test_navigation_feedback import load_navigation


class RecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = load_navigation()

    def node(self):
        m = self.m
        n = m.Navigator.__new__(m.Navigator)
        n.persistent_recovery = True; n.ground_truth_test = False
        n.goal = (2., 0., 0.); n.generation = 0
        n.line_stage = None; n.stage_target = None; n.line_route = None
        n.return_context = None; n.return_pending = False; n.relaxed_goal = False
        n.stop_history = deque([(0., 0., 0.)], maxlen=100)
        n.status = Mock(); n.cmd = Mock(); n.path_pub = Mock()
        n.sensors = Mock(); n.authority = Mock(); n.obstacles = Mock()
        n.pose = Mock(return_value=(1., 0., 0.))
        n.stop_pose = Mock(side_effect=lambda p:p)
        n.actuator_state = Mock(return_value=(0., 0.))
        n.scan = types.SimpleNamespace(header=types.SimpleNamespace(stamp=Mock()))
        n.scan.header.stamp.to_nsec.side_effect = range(100)
        n.grid = m.Grid(100,100,.1,(-5.,-5.,0.),[0]*10000)
        n.replan_count = 100
        n.start_plan = Mock()
        return n

    def ready(self, n):
        n.retry_after = 0.
        n.retry_window = Mock(); n.retry_window.update.return_value = True
        for _ in range(5): n.recover_retry(.05)

    def test_sensor_loss_retains_goal_and_never_plans_until_fresh(self):
        n = self.node(); original = n.goal
        n.halt('STOPPED: scan stale')
        self.assertEqual(n.state, 'WAIT_RETRY'); self.assertEqual(n.goal, original)
        n.sensors.side_effect = RuntimeError('stale')
        self.ready(n); n.start_plan.assert_not_called()
        n.sensors.side_effect = None
        self.ready(n); n.start_plan.assert_called_once_with((1.,0.,0.))
        self.assertEqual(n.replan_count, 101)
        self.assertEqual(n.goal, original)

    def test_region_failure_returns_latest_distinct_stop_not_oldest(self):
        n = self.node(); original = n.goal
        n.stop_history.extend([(.5,0.,0.),(1.,0.,0.)])
        n.line_stage = 'FINAL'
        n.halt('PLAN_FAILED: exact goal failed')
        self.assertFalse(n.return_pending)
        n.halt('PLAN_FAILED: regional search exhausted')
        self.ready(n)
        self.assertEqual(n.line_stage, 'RETURN_STOP')
        self.assertEqual(n.stage_target, (.5,0.,0.))
        self.assertEqual(n.return_context, ('FINAL', None))
        self.assertEqual(n.goal, original)
        before = list(n.stop_history)
        n.finish_stage((.5,0.,0.))
        self.assertEqual(n.state, 'WAIT_RETRY')
        self.assertEqual(n.line_stage, 'FINAL')
        self.assertEqual(n.goal, original)
        self.assertEqual(list(n.stop_history), before)

    def test_return_does_not_ping_pong_between_history_points(self):
        n=self.node(); n.relaxed_goal=True
        n.stop_history.extend([(.5,0.,0.),(1.,0.,0.)])
        n.halt('PLAN_FAILED: regional search exhausted'); self.ready(n)
        self.assertEqual(n.stage_target,(.5,0.,0.))
        n.finish_stage((.5,0.,0.))
        n.pose.return_value=(.5,0.,0.)
        n.halt('PLAN_FAILED: regional search exhausted'); self.ready(n)
        self.assertIsNone(n.line_stage)
        self.assertEqual(n.recovery_checkpoint,(.5,0.,0.))
        n.remember_stop((.8,.3,0.))
        self.assertIsNone(n.recovery_checkpoint)

    def test_search_budget_grows_only_for_budget_failure(self):
        n=self.node()
        n.halt('STOPPED: scan stale')
        self.assertEqual(getattr(n,'planning_budget_failures',0),0)
        for expected in (1,2,2,2):
            n.halt('PLAN_FAILED: planning budget exceeded; choose a roomier approach point')
            self.assertEqual(n.planning_budget_failures,expected)
        n.remember_stop((.8,.3,0.))
        self.assertEqual(n.planning_budget_failures,0)

    def test_recovery_accepts_stop_between_eight_and_ten_cm(self):
        n=self.node(); n.maneuver_mode='NORMAL'; n.relaxed_goal=True
        n.remember_stop=Mock(); n.halt=Mock()
        n.finish_stage(self.m.to_rear((2.09,0.,0.),'goal'))
        self.assertTrue(n.halt.call_args.args[0].startswith('SUCCEEDED:'))
        n.finish_stage(self.m.to_rear((2.101,0.,0.),'goal'))
        self.assertTrue(n.halt.call_args.args[0].startswith('FINAL_TOLERANCE_FAILED:'))

    def test_suffix_budget_exhaustion_is_counted(self):
        n=self.node(); n.line_stage='LINE_SUFFIX'
        n.halt('PLAN_FAILED: line suffix search budget exhausted; no complete retreat-to-goal route')
        self.assertEqual(n.planning_budget_failures,1)

    def test_failed_return_keeps_checkpoint_and_never_signals_success(self):
        n = self.node(); n.relaxed_goal = True
        n.line_stage = 'RETURN_STOP'; n.stage_target = (0.,0.,0.)
        n.return_context = ('FINAL', None)
        for _ in range(4):
            n.halt('PLAN_FAILED: return blocked')
            self.ready(n)
            self.assertEqual(n.line_stage, 'RETURN_STOP')
            self.assertEqual(n.stage_target, (0.,0.,0.))
            self.assertEqual(n.goal, (2.,0.,0.))
        self.assertFalse(any(c.args[0].startswith('SUCCEEDED') for c in n.status.call_args_list))

    def test_cancel_is_terminal_even_during_return(self):
        n = self.node(); n.line_stage = 'RETURN_STOP'
        with patch.object(self.m, 'Path') as path:
            path.return_value.header = types.SimpleNamespace()
            n.halt('CANCELLED')
        self.assertIsNone(n.goal); self.assertEqual(n.state, 'IDLE')
        self.assertIsNone(n.return_context)

    def test_no_checkpoint_retries_goal_without_fabricating_return(self):
        n = self.node(); n.stop_history.clear(); n.relaxed_goal = True
        n.halt('PLAN_FAILED: regional search exhausted')
        self.ready(n)
        self.assertIsNone(n.line_stage); self.assertIsNone(n.stage_target)
        n.start_plan.assert_called_once()

    def test_blocked_current_pose_does_not_trigger_motion(self):
        n = self.node(); n.halt('STOPPED: unexpected')
        n.grid.free = Mock(return_value=False)
        self.ready(n); n.start_plan.assert_not_called()
        self.assertEqual(n.state, 'WAIT_RETRY')

    def test_regional_planner_reaches_free_part_when_centre_is_blocked(self):
        m = self.m
        grid = m.Grid(100,100,.1,(-5.,-5.,0.),[0]*10000)
        grid.depth_rules = [dict(id='inspect_1',type='B',x=0.,y=0.,direction=1)]
        original = (0., .45, math.pi/2)
        start = m.to_rear((0., -.5, math.pi/2), 'goal')
        goal = m.to_rear(original, 'goal')
        self.assertFalse(grid.free(goal))
        with self.assertRaises(ValueError):
            m.plan(grid,start,goal,front_goal=original)
        path = m.plan(grid,start,goal,max_seconds=2.,goal_position_tolerance=.1,
                      front_goal=original,goal_region=True)
        end = m.from_rear(path[-1][:3], 'goal')
        self.assertLessEqual(math.hypot(end[0]-original[0],end[1]-original[1]),.1)
        self.assertTrue(all(grid.free(p[:3]) for p in path))
        self.assertLess(end[1],.4)

    def test_recorded_narrow_stop_can_return_with_required_safety_margin(self):
        import json
        from pathlib import Path
        m = self.m
        with open(Path(__file__).resolve().parents[1]/'data/maps/sim_field_narrow/geometry.json') as f:
            data = json.load(f)
        pose = (4.043261728, 2.819000008, -3.050932029)
        checkpoint = (3.016, 2.741, math.pi)
        grid = m.Grid(data['width'],data['height'],data['resolution'],
                      tuple(data['origin']),data['data'],.04)
        self.assertFalse(grid.free(pose))
        grid.margin = .02; grid.cache.clear()
        self.assertTrue(grid.free(pose))
        route = m.plan(grid,pose,checkpoint,max_seconds=2.,goal_position_tolerance=.025,
                       goal_heading_tolerance=math.radians(5))
        self.assertTrue(all(grid.free(p[:3]) for p in route))
        self.assertLessEqual(math.hypot(route[-1][0]-checkpoint[0],route[-1][1]-checkpoint[1]),.025)

        n = self.node(); del n.start_plan
        n.grid = grid; n.margin = .02; n.planning_margin = .04
        n.relaxed_goal = True; n.line_stage = 'RETURN_STOP'; n.stage_target = checkpoint
        n.maneuver_mode = 'NORMAL'; n.goal_heading_tolerance = math.radians(10)
        n.goal_position_tolerance = .04; n.zero_cost_line = []; n.zero_cost_width = .01
        n.radius = 1.3; n.timeout = 2.; n.stop_event = threading.Event()
        n.lock = threading.RLock(); n.publish_path = Mock()
        with patch.object(m.threading, 'Thread') as thread:
            thread.side_effect = lambda **kw: types.SimpleNamespace(start=kw['target'])
            n.start_plan(pose)
        self.assertEqual(n.state, 'DRIVING')
        self.assertEqual(n.line_stage, 'RETURN_STOP')
        self.assertEqual(n.grid.margin, .02)
        self.assertTrue(any('RECOVERY_CLEARANCE:' in c.args[0] for c in n.status.call_args_list))

    def test_zero_route_gets_fresh_stop_confirmation(self):
        m = self.m; n = self.node(); del n.start_plan
        n.maneuver_mode='NORMAL'; n.goal_heading_tolerance=.17
        n.goal_position_tolerance=.04; n.zero_cost_line=[]; n.zero_cost_width=.01
        n.planning_margin=.02; n.margin=.02; n.relaxed_goal=True
        n.radius=1.3; n.timeout=1.; n.stop_event=threading.Event()
        n.lock=threading.RLock(); n.publish_path=Mock()
        n.stop_state='VERIFY_STOP'; n.stop_started=time.time()-100
        old=n.stop_started
        with patch.object(m.threading,'Thread') as thread, patch.object(m,'plan') as planner:
            thread.side_effect=lambda **kw: types.SimpleNamespace(start=kw['target'])
            planner.return_value=[(1.38,0.,0.,0,0.)]
            n.start_plan((1.38,0.,0.))
        self.assertEqual(n.state,'VERIFY_STOP')
        self.assertIsNone(n.stop_state)
        self.assertGreater(n.stop_started,old+90)
        self.assertEqual(n.stop_window.samples,[])
        self.assertFalse(any(c.args[0].startswith('SUCCEEDED:') for c in n.status.call_args_list))

    def test_stop_confirmation_uses_odom_not_map_jitter(self):
        m=self.m; n=self.node(); del n.stop_pose
        n.tf=Mock()
        stamp=types.SimpleNamespace(to_sec=lambda:10.)
        n.tf.getLatestCommonTime.return_value=stamp
        n.tf.lookupTransform.return_value=((1.,2.,0.),(0.,0.,0.,1.))
        now=Mock(); now.__sub__=Mock(return_value=types.SimpleNamespace(to_sec=lambda:.05))
        with patch.object(m.rospy.Time,'now',return_value=now), patch.object(
                m.tf,'transformations',types.SimpleNamespace(euler_from_quaternion=lambda q:(0.,0.,0.)),create=True):
            a=n.stop_pose((10.,10.,.1)); b=n.stop_pose((10.06,10.,.1))
            self.assertEqual(a,b)
            self.assertEqual(a,(1.,2.,0.))
            now.__sub__.return_value=types.SimpleNamespace(to_sec=lambda:1.)
            with self.assertRaises(RuntimeError):n.stop_pose((10.,10.,.1))

    def test_return_uses_original_goal_metadata_and_keeps_depth_rules(self):
        m = self.m; n = self.node()
        del n.start_plan  # Exercise the actual asynchronous planner entry.
        n.maneuver_mode = 'LATERAL'; n.maneuver_heading_tolerance = .2
        n.zero_cost_line = [((0.,0.),(2.,0.))]; n.zero_cost_width = .1
        n.line_stage = 'RETURN_STOP'; n.stage_target = (0.,0.,0.)
        n.relaxed_goal = True; n.goal_heading_tolerance = .17
        n.goal_position_tolerance = .08; n.planning_margin = 0.
        n.radius = 1.3; n.timeout = 1.; n.stop_event = threading.Event()
        n.lock = threading.RLock(); n.publish_path = Mock()
        n.grid.depth_rules = [dict(id='inspect_1',type='B',x=4.,y=4.,direction=1)]
        with patch.object(m.threading, 'Thread') as thread, patch.object(m, 'plan') as planner:
            thread.side_effect = lambda **kw: types.SimpleNamespace(start=kw['target'])
            planner.return_value = [(1.,0.,0.,0,0.),(0.,0.,0.,-1,0.)]
            n.start_plan((1.,0.,0.))
        args = planner.call_args.args
        self.assertEqual(args[2], n.stage_target)
        self.assertEqual(args[8], 'NORMAL')
        self.assertEqual(args[0].depth_rules, n.grid.depth_rules)
        self.assertEqual(n.goal, (2.,0.,0.))
        self.assertEqual(n.line_stage, 'RETURN_STOP')
        self.assertEqual(n.state, 'DRIVING')

    def test_entry_stop_starts_atomic_suffix_then_executes_full_path(self):
        m=self.m; n=self.node()
        n.maneuver_mode='LATERAL'; n.maneuver_heading_tolerance=.2
        n.line_stage='APPROACH_LINE'; n.stage_target=(0.,0.,0.)
        n.locked_line=((-3.,0.),(3.,0.)); n.remember_stop=Mock()
        n.finish_stage((0.,0.,0.))
        self.assertEqual(n.line_stage,'LINE_SUFFIX')
        n.start_plan.assert_called_once_with((0.,0.,0.))
        del n.start_plan
        n.goal_heading_tolerance=.17; n.goal_position_tolerance=.04
        n.zero_cost_line=[n.locked_line]; n.zero_cost_width=.1
        n.planning_margin=.02; n.radius=1.3; n.timeout=5.
        n.stop_event=threading.Event(); n.lock=threading.RLock(); n.publish_path=Mock()
        route=[(0.,0.,0.,0,0.),(-.5,0.,0.,-1,0.),(1.38,0.,0.,1,0.)]
        with patch.object(m.threading,'Thread') as thread, patch.object(m,'plan_line_suffix') as suffix, patch.object(m,'plan') as direct:
            thread.side_effect=lambda **kw:types.SimpleNamespace(start=kw['target'])
            suffix.return_value=(route,(-.5,0.,0.))
            n.start_plan((0.,0.,0.))
        self.assertEqual(suffix.call_args.args[6],15.)
        direct.assert_not_called()
        self.assertEqual(n.line_stage,'FINAL')
        self.assertEqual(n.state,'DRIVING')
        n.publish_path.assert_called_once_with(route)
        self.assertEqual(len(n.parts),2)
        n.line_stage='LINE_SUFFIX'; n.stage_target=(0.,0.,0.)
        n.relaxed_goal=True; n.planning_budget_failures=1
        with patch.object(m.threading,'Thread') as thread, patch.object(m,'plan_line_suffix') as suffix:
            thread.side_effect=lambda **kw:types.SimpleNamespace(start=kw['target'])
            suffix.return_value=(route,(-.5,0.,0.))
            n.start_plan((0.,0.,0.))
        self.assertEqual(suffix.call_args.args[6],20.)
        self.assertEqual(suffix.call_args.args[8],.08)


    def test_suffix_failure_never_publishes_partial_path(self):
        m=self.m; n=self.node(); del n.start_plan
        n.maneuver_mode='LATERAL'; n.maneuver_heading_tolerance=.2
        n.line_stage='LINE_SUFFIX'; n.stage_target=(0.,0.,0.)
        n.locked_line=((-3.,0.),(3.,0.)); n.zero_cost_line=[n.locked_line]
        n.zero_cost_width=.1; n.goal_heading_tolerance=.17; n.goal_position_tolerance=.04
        n.planning_margin=.02; n.radius=1.3; n.timeout=5.
        n.stop_event=threading.Event(); n.lock=threading.RLock(); n.publish_path=Mock()
        n.halt=Mock()
        with patch.object(m.threading,'Thread') as thread, patch.object(m,'plan_line_suffix',side_effect=RuntimeError('no complete route')):
            thread.side_effect=lambda **kw:types.SimpleNamespace(start=kw['target'])
            n.start_plan((0.,0.,0.))
        n.publish_path.assert_not_called()
        n.halt.assert_called_once_with('PLAN_FAILED: no complete route')
        self.assertEqual(n.line_stage,'LINE_SUFFIX')



if __name__ == '__main__': unittest.main()
