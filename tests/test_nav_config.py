"""Non-default geometry must propagate through planning and control math."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/smartcar_navigation/scripts'))
from nav_config import DEFAULTS, P, configure, from_rear, to_rear
from ackermann_core import Grid, advance, plan, line_retreat_target, StopWindow
from actuator_model import predict_arc, decode_joints
from ground_truth_pose import rear_pose
from path_tracking import steering, tracking_command, replay_command
from local_planner import choose


class ConfigTests(unittest.TestCase):
    def setUp(self):
        configure()

    def tearDown(self):
        configure()

    def test_reference_roundtrip_at_rotated_heading(self):
        configure(dict(wheelbase=.83, goal_reference='custom',
                       goal_offset_x=.42, goal_offset_y=-.17))
        for angle in (0., math.pi/2, -2.1):
            rear = (2., 3., angle)
            goal = from_rear(rear, 'goal')
            recovered = to_rear(goal, 'goal')
            for a, b in zip(rear, recovered):
                self.assertAlmostEqual(a, b)
        configure(dict(wheelbase=.83))
        self.assertAlmostEqual(from_rear((0.,0.,0.), 'goal')[0], .83)

    def test_path_reference_does_not_change_goal_or_internal_pose(self):
        configure(dict(path_reference='custom', path_offset_x=.2, path_offset_y=.1))
        rear = (2., 3., math.pi/2)
        published = from_rear(rear, 'path')
        self.assertAlmostEqual(published[0], 1.9)
        self.assertAlmostEqual(published[1], 3.2)
        self.assertAlmostEqual(from_rear(rear, 'goal')[1], 3.62)
        self.assertEqual(rear, (2.,3.,math.pi/2))

    def test_localization_and_truth_share_base_offset(self):
        configure(dict(base_reference='custom', base_offset_x=.45, base_offset_y=.12))
        pose = (2., 3., math.pi/2)
        self.assertEqual(rear_pose(pose, (0.,0.,0.)), to_rear(pose, 'base'))

    def test_wheelbase_changes_feedback_steering_and_prediction_together(self):
        configure(dict(wheelbase=.9, front_track=.5, wheel_radius=.12))
        k = .4
        names = ['front_left_steer_joint','front_right_steer_joint',
                 'rear_left_wheel_joint','rear_right_wheel_joint']
        angles = [math.atan(.9*k/(1-.25*k)), math.atan(.9*k/(1+.25*k)), 0., 0.]
        delta, speed = decode_joints(names, angles, [0.,0.,1.,1.])
        ref = dict(lateral=0., heading=0., curvature=k)
        self.assertAlmostEqual(delta, steering(ref, 1, 1.3))
        self.assertAlmostEqual(speed, .12)
        class FreeGrid:
            def arc(self, pose, distance, curvature):
                return [advance(pose, distance, curvature)]
        result = predict_arc(FreeGrid(), (2.,2.,0.), speed, delta, speed, delta, .2)
        expected = advance((2.,2.,0.), speed*.2, k)
        for a,b in zip(result[0], expected):
            self.assertAlmostEqual(a,b)

    def test_planner_goal_reference_supports_lateral_offset(self):
        configure(dict(goal_reference='custom', goal_offset_x=.3, goal_offset_y=.2))
        grid = Grid(100, 100, .1, (0.,0.,0.), [0]*10000)
        rear_goal = (3., 2., 0.)
        goal = from_rear(rear_goal, 'goal')
        path = plan(grid, (2.,2.,0.), to_rear(goal, 'goal'), front_goal=goal)
        endpoint = from_rear(path[-1][:3], 'goal')
        self.assertLess(math.hypot(endpoint[0]-goal[0], endpoint[1]-goal[1]), .081)

    def test_configured_stage_distance_and_stop_window(self):
        configure(dict(line_retreat_distance=.8, stop_window=.2))
        self.assertAlmostEqual(line_retreat_target(((0.,0.),(5.,0.)), (3.,0.,0.), 0.)[0], 2.2)
        window = StopWindow()
        self.assertFalse(window.update(0., (0.,0.,0.)))
        self.assertTrue(window.update(.21, (0.,0.,0.)))

    def test_speed_caps_and_local_rollout_use_changed_geometry(self):
        configure(dict(wheelbase=.85, tracking_forward_speed=.06,
                       replay_forward_speed=.025, rollout_steps=2))
        segment = [(2.+i*.02,2.,0.,1,0.) for i in range(50)]
        v, delta, ref = tracking_command((2.,2.,0.), segment, 0, 1.3)
        self.assertLessEqual(v, .06)
        self.assertAlmostEqual(replay_command(ref,1)[0], .025)
        grid = Grid(100,100,.1,(0.,0.,0.),[0]*10000)
        result = choose(grid,(2.,2.,0.),segment,0,1.3,0.,current_speed=0.)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[2], math.tan(result[3])/.85)
        self.assertLessEqual(len(result[4]), 2)

    def test_collision_envelope_is_configurable(self):
        data=[0]*10000
        data[20*100+30]=100
        small = Grid(100,100,.1,(0.,0.,0.),data)
        self.assertTrue(small.free((2.,2.,0.)))
        configure(dict(collision_half_length=.9))
        large = Grid(100,100,.1,(0.,0.,0.),data)
        self.assertFalse(large.free((2.,2.,0.)))

    def test_invalid_config_is_atomic(self):
        for overrides in ({'wheelbas':.8}, {'wheelbase':float('nan')},
                          {'scan_stride':1.5}, {'wall_straight_weight':1.1},
                          {'control_period':1.}, {'goal_reference':'unknown'},
                          {'base_frame':'/base'}, {'tracking_max_error':.1}):
            with self.assertRaises(ValueError):
                configure(overrides)
            self.assertEqual(P, DEFAULTS)


if __name__ == '__main__':
    unittest.main()
