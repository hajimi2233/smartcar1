import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'docker/overlay'))
from region_geometry import validate, intersects
from ackermann_core import Grid, plan


class RegionTests(unittest.TestCase):
    def test_invalid_crossed_and_degenerate_corners(self):
        for p in ([(0,0),(1,1),(0,1),(1,0)], [(0,0)]*4):
            with self.assertRaises(ValueError): validate(p)

    def test_rotated_and_reverse_boundary(self):
        p=[(0,1),(1,0),(2,1),(1,2)]
        validate(p); validate(list(reversed(p)))
        self.assertTrue(intersects(p,[(.9,.9),(1.1,.9),(1.1,1.1),(.9,1.1)]))
        self.assertFalse(intersects(p,[(3,3),(4,3),(4,4),(3,4)]))

    def test_whole_body_not_only_reference_point(self):
        g=Grid(240,240,.05,(-6,-6,0),[0]*57600)
        g.region=[(0,-1),(1,-1),(1,1),(0,1)]
        self.assertFalse(g.free((-.5,0,0)))
        self.assertTrue(g.free((-1,0,0)))
        with self.assertRaises(ValueError): plan(g,(-2,0,0),(-.5,0,0))

    def test_sweep_cannot_cross_region(self):
        g=Grid(240,240,.05,(-6,-6,0),[0]*57600)
        g.region=[(0,-1),(.1,-1),(.1,1),(0,1)]
        self.assertIsNone(g.arc((-1,0,0),2,0))


class TightRegionTests(unittest.TestCase):
    def test_rear_body_not_widened_by_front_tires(self):
        g=Grid(240,240,.05,(-6,-6,0),[0]*57600)
        g.region=[(-.08,.30),(.08,.30),(.08,.4),(-.08,.4)]
        self.assertFalse(g.region_blocked((0,0,0)))
        g.region=[(.58,.30),(.68,.30),(.68,.4),(.58,.4)]
        self.assertTrue(g.region_blocked((0,0,0)))

    def test_real_body_still_blocked_with_zero_margin(self):
        g=Grid(240,240,.05,(-6,-6,0),[0]*57600)
        g.region_margin=0
        g.region=[(-.08,.26),(.08,.26),(.08,.4),(-.08,.4)]
        self.assertTrue(g.region_blocked((0,0,0)))
