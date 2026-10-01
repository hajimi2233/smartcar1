import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
from corridor_geometry import classify


class CorridorTests(unittest.TestCase):
    def test_inside_outside_edges_corners(self):
        p = [(0,0),(2,0),(2,2),(0,2)]
        for xy in ((1,1),(0,1),(2,2),(0,0)):
            self.assertEqual(classify(p, xy+(0,)), 'INSIDE')
        for xy in ((-.001,1),(2.001,1),(1,3)):
            self.assertEqual(classify(p, xy+(0,)), 'OUTSIDE')

    def test_rotated_clockwise_and_counterclockwise(self):
        p = [(0,1),(1,0),(2,1),(1,2)]
        for poly in (p,list(reversed(p))):
            self.assertEqual(classify(poly,(1,1,2.5)), 'INSIDE')
            self.assertEqual(classify(poly,(.1,.1,0)), 'OUTSIDE')
            self.assertEqual(classify(poly,(.5,.5,0)), 'INSIDE')

    def test_reference_point_not_body_or_heading(self):
        p = [(0,0),(2,0),(2,2),(0,2)]
        for yaw in (0,math.pi/2,math.pi):
            self.assertEqual(classify(p,(.001,.001,yaw)), 'INSIDE')

    def test_invalid_data(self):
        for p in ([],[(0,0)]*4,[(0,0),(1,1),(0,1),(1,0)]):
            with self.assertRaises(ValueError): classify(p,(0,0,0))
        with self.assertRaises(ValueError):
            classify([(0,0),(2,0),(2,2),(0,2)],(float('nan'),0,0))


if __name__ == '__main__':
    unittest.main()
