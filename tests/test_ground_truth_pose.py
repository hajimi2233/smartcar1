import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/smartcar_navigation/scripts'))
from ground_truth_pose import alignment, rear_pose
from path_tracking import tracking_command, replay_command


class GroundTruthPoseTests(unittest.TestCase):
    def test_rotated_map_alignment_and_rear_axle(self):
        world = (2., 3., math.pi/2)
        transform = alignment((5., 7., math.pi), world)
        actual = rear_pose(world, transform)
        self.assertAlmostEqual(actual[0], 5.31)
        self.assertAlmostEqual(actual[1], 7.)
        self.assertAlmostEqual(abs(actual[2]), math.pi)

    def test_forward_and_reverse_correct_cross_track_error(self):
        for sign in (1, -1):
            segment = [(0., 0., 0., sign, 0.), (sign*1., 0., 0., sign, 0.)]
            speed, steer, reference = tracking_command((0., .05, 0.), segment, 0, 1.3)
            self.assertGreater(speed*sign, 0.)
            self.assertLess(steer, 0.)
            self.assertAlmostEqual(reference['lateral'], .05)

    def test_replay_uses_planned_curvature_without_error_correction(self):
        reference = dict(lateral=.08, heading=.15, curvature=.5, remaining=1.)
        self.assertEqual(replay_command(reference, 1, .07), (.04, .5))
        self.assertEqual(replay_command(reference, -1, .02), (-.02, .5))
        self.assertEqual(replay_command(reference, 1, .14, 2.), (.08, .5))
        self.assertEqual(replay_command(reference, -1, .09, 2.), (-.06, .5))


if __name__ == '__main__':
    unittest.main()
