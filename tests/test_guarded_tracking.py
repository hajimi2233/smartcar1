import os, sys, math, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src/smartcar_navigation/scripts'))
from ackermann_core import Grid
from local_planner import guarded_track
from actuator_model import predict_arc

class GuardTests(unittest.TestCase):
    def grid(self):
        return Grid(200, 200, .02, (-2., -2., 0.), [0]*40000, .02)
    def path(self, sign=1):
        return [(sign*i*.01, 0., 0., sign, 0.) for i in range(150)]
    def test_clear_space_and_opposite_feedback(self):
        g=self.grid();d={}
        self.assertIsNotNone(guarded_track(g,(0,0,0),self.path(),0,1.1,0.,d,current_speed=0.))
        self.assertEqual(d['controller'],'clear_path')
        self.assertIsNone(guarded_track(g,(0,0,0),self.path(),0,1.1,0.,current_speed=-.03))
    def test_live_obstacle_and_full_braking(self):
        # Moving at .4 m/s cannot stop in the ~8 cm remaining clearance.
        g=self.grid();g.dynamic.add(g.cell(.85,0.))
        d={}
        self.assertIsNone(guarded_track(g,(0,0,0),self.path(),0,1.1,0.,d,current_speed=.4))
        self.assertTrue(d['counts'])
    def test_near_wall_command_can_stop(self):
        g=self.grid();g.dynamic.add(g.cell(.9,0.))
        c=guarded_track(g,(0,0,0),self.path(),0,1.1,0.,current_speed=.05,guard_distance=.3)
        self.assertIsNotNone(c)
        response=predict_arc(g,(0,0,0),.05,0.,c[1],c[3],.3)
        self.assertIsNotNone(response)
        p,v,s,_=response
        self.assertIsNotNone(predict_arc(g,p,v,s,0.,c[3],abs(v)/.5+.02))
    def test_wall_bias_only_straight_mode(self):
        g=self.grid()
        normal=guarded_track(g,(0,0,0),self.path(),0,1.1,0.,wall_steer=.1,wall_weight=1.)
        straight=guarded_track(g,(0,0,0),self.path(),0,1.1,0.,wall_steer=.1,wall_weight=1.,maneuver_mode='STRAIGHT')
        self.assertEqual(normal[3],0.)
        self.assertGreater(straight[3],0.)
    def test_invalid_timing(self):
        with self.assertRaises(ValueError):
            guarded_track(self.grid(),(0,0,0),self.path(),0,1.1,0.,control_dt=2.)

    def test_returned_command_has_collision_free_braking_for_random_obstacles(self):
        import random
        from nav_config import P
        rng=random.Random(41);g=self.grid()
        for _ in range(250):
            direction=rng.choice([-1,1]);speed=direction*rng.uniform(0.,.2)
            steer=rng.uniform(-.45,.45)
            g.dynamic=set(g.cell(rng.uniform(-.8,1.4),rng.uniform(-.6,.6)) for _ in range(4))
            c=guarded_track(g,(0,0,0),self.path(direction),0,1.1,steer,current_speed=speed)
            if c is None:continue
            response=predict_arc(g,(0,0,0),speed,steer,c[1],c[3],P['max_control_dt']+.2)
            self.assertIsNotNone(response)
            p,v,s,_=response
            self.assertIsNotNone(predict_arc(g,p,v,s,0.,c[3],abs(v)/.5+.02))

if __name__=='__main__':unittest.main()
