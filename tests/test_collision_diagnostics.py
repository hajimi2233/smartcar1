import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
from ackermann_core import Grid

class DiagnosticsTests(unittest.TestCase):
    def grid(self, value=0):
        data=[0]*10000
        data[50*100+50]=value
        return Grid(100,100,.05,(-2.5,-2.5,0),data)

    def test_sources_and_overlap(self):
        for value,expected in ((100,'STATIC_MAP'),(-1,'MAP_UNKNOWN')):
            g=self.grid(value);g.dynamic.add((50,50))
            g.region=[(-.1,-.1),(.1,-.1),(.1,.1),(-.1,.1)]
            hits=g.collision((0,0,0),True)
            self.assertEqual(set(hits),set([expected,'LIVE_SCAN','REGION']))
            self.assertFalse(g.free((0,0,0)))
        g=self.grid();g.dynamic.add((50,50))
        self.assertEqual(set(g.collision((0,0,0),True)),{'LIVE_SCAN'})

    def test_bounds_clear_and_sweep(self):
        g=self.grid()
        self.assertTrue(g.free((0,0,0)))
        self.assertIn('MAP_BOUNDS',g.collision((3,0,0),True))
        g.dynamic.add((50,50))
        self.assertIsNone(g.arc((-1,0,0),.5,0))
        detail=g.blocked_detail((-1,0,0),.5,0)
        self.assertIn('LIVE_SCAN@(0.025,0.025)',detail)
        self.assertIn('rear_pose=',detail)
