import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
from inspection_depth import rules_for_goal, blocked, segment_blocked, validate_regions
from ackermann_core import Grid, rear_target, plan
from nav_config import configure


class DepthTests(unittest.TestCase):
    def setUp(self):
        configure()
        self.regions = [dict(id='inspect_%d'%(i+1), type='B' if i==8 else 'A',
                             x=4.-i//2*2., y=1. if i%2==0 else -1.) for i in range(10)]
        self.grid = Grid(200,200,.1,(-10.,-10.,0.),[0]*40000)

    def test_target_a_and_b_have_identical_no_overshoot_rule(self):
        for kind in ('A','B'):
            r=dict(id='inspect_1',type=kind,x=0.,y=0.)
            for direction in (-1,1):
                yaw=direction*math.pi/2
                rules=rules_for_goal([r],(0.,-direction, yaw),(0.,0.,yaw),'inspect_1',{})
                self.assertFalse(blocked((0.,direction*.39),rules))
                self.assertTrue(blocked((0.,direction*.41),rules))
                self.assertFalse(blocked((0.,-direction*2),rules))
                self.assertFalse(blocked((.5,direction*.5),rules))

    def test_leaving_b_keeps_old_depth_and_adds_new_target_constraint(self):
        b=self.regions[8]; a=self.regions[0]
        sides={b['id']:1}
        current=(b['x'],b['y'],math.pi/2)
        rules=rules_for_goal(self.regions,current,(a['x'],a['y'],-math.pi/2),a['id'],sides)
        self.assertEqual(set(r['id'] for r in rules),set([b['id'],a['id']]))
        self.assertTrue(blocked((b['x'],b['y']+.41),rules))
        self.assertFalse(blocked((b['x'],b['y']-1.),rules))
        outside=rules_for_goal(self.regions,(b['x'],b['y']-1.,math.pi/2),(5.,-3.,0.),None,sides)
        self.assertEqual(outside,[])

    def test_departing_a_is_not_restricted(self):
        a=self.regions[0]
        self.assertEqual(rules_for_goal(self.regions,(a['x'],a['y'],math.pi/2),(6.,0.,0.),None,{}),[])

    def test_turning_inside_b_does_not_reverse_boundary(self):
        b=self.regions[8]
        rule=rules_for_goal(self.regions,(b['x'],b['y'],-math.pi/2),(5.,0.,0.),None,{b['id']:1})
        self.assertEqual(rule[0]['direction'],1)

    def test_arc_and_large_step_cannot_skip_forbidden_strip(self):
        r=dict(id='inspect_1',type='B',x=0.,y=0.,direction=1)
        self.grid.depth_rules=[r]
        self.assertTrue(segment_blocked((-1.,.5),(1.,.5),[r]))
        self.assertIsNone(self.grid.arc(rear_target((-1.,.5,0.)),2.,0.))
        self.assertIsNone(self.grid.arc(rear_target((0.,.3,math.pi/2)),.3,0.))
        self.assertIsNotNone(self.grid.arc(rear_target((0.,.3,math.pi/2)),-1.,0.))
        self.assertIn('INSPECTION_DEPTH_inspect_1',self.grid.collision(rear_target((0.,.5,0.))))

    def test_global_planner_can_retreat_but_rejects_goal_beyond_depth(self):
        self.grid.depth_rules=[dict(id='inspect_9',type='B',x=0.,y=0.,direction=1)]
        start=rear_target((0.,0.,math.pi/2))
        goal=rear_target((0.,-1.,math.pi/2))
        path=plan(self.grid,start,goal,max_seconds=2.)
        self.assertTrue(path)
        for a,b in zip(path,path[1:]):
            self.assertTrue(self.grid.free(b[:3]))
        with self.assertRaises(ValueError):
            plan(self.grid,start,rear_target((0.,1.,math.pi/2)))

    def test_metadata_validation_and_goal_binding(self):
        self.assertEqual(len(validate_regions(self.regions)),10)
        with self.assertRaises(ValueError):
            rules_for_goal(self.regions,(8.,0.,0.),(9.,9.,math.pi/2),'inspect_1',{})
        with self.assertRaises(ValueError): validate_regions(self.regions[:9])


if __name__=='__main__': unittest.main()
