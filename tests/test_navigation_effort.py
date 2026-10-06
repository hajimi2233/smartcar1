import math,sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
import ackermann_core as core
from nav_config import configure,P
from path_tracking import tracking_command

class EffortTests(unittest.TestCase):
    def setUp(self):configure()
    def grid(self):return core.Grid(240,240,.05,(-6.,-6.,0.),[0]*57600)
    def test_turn_lag_limits_travel_both_directions(self):
        for sign in (1,-1):
            path=[(0.,0.,0.,sign,.7)]
            path += [(p[0],p[1],p[2],sign,.7) for p in [core.advance((0.,0.,0.),sign*i*.02,.7) for i in range(1,51)]]
            v,delta,_=tracking_command((0.,0.,0.),path,0,1.1,-.3)
            self.assertLessEqual(abs(v)*abs(delta+.3),P['tracking_turn_allowance']+1e-9)
            matched,_,_=tracking_command((0.,0.,0.),path,0,1.1,delta)
            self.assertGreater(abs(matched),abs(v))
    def test_time_estimate_penalizes_cusps_and_reverse(self):
        straight=[(0.,0.,0.,0,0.),(1.,0.,0.,1,0.)]
        cusp=[(0.,0.,0.,0,0.),(.5,0.,0.,1,0.),(0.,0.,0.,-1,0.)]
        self.assertGreater(core.route_seconds(cusp),core.route_seconds(straight))
    def test_shifted_candidates_stay_on_nearest_line(self):
        g=self.grid();g.zero_cost_line=[((-4.,2.),(4.,2.)),((-4.,-2.),(4.,-2.))]
        start=(0.,0.,math.pi/2);goal=(1.,1.8,math.pi/2)
        base=core.line_stage_candidates(g,start,goal)[0][3]
        free=g.free
        g.free=lambda p:False if math.hypot(p[0]-base[0],p[1]-base[1])<.1 else free(p)
        rows=core.line_stage_candidates(g,start,goal,offsets=(0.,-.4,.4))
        self.assertTrue(rows)
        self.assertTrue(all(r[1]==0 and abs(r[3][1]-2.)<1e-9 for r in rows))
    def test_entry_compares_candidates_with_shared_budget(self):
        g=self.grid();g.zero_cost_line=[((-4.,2.),(4.,2.))]
        start=(0.,0.,math.pi/2);goal=(1.,1.,math.pi/2)
        nominal=core.line_stage_candidates(g,start,goal,'LATERAL')[0][3]
        calls=[]
        def fake(grid,a,b,*args,**kw):
            if 'max_direction_changes' in kw:raise RuntimeError('budget exhausted')
            calls.append((b,kw['max_seconds']))
            return [tuple(a)+(0,0.),tuple(a)+(-1,0.),tuple(a)+(1,0.),tuple(b)+(-1,0.)]
        with patch.object(core,'plan',side_effect=fake):
            path,route=core.plan_line_approach(g,start,goal,max_seconds=10.,maneuver_mode='LATERAL')
        self.assertEqual(len(calls),3)
        self.assertEqual(set(round(b[0]-nominal[0],3) for b,_ in calls),{-.3,-.6,-.9})
        self.assertLessEqual(calls[0][1],10./3.)
        self.assertLessEqual(calls[-1][1],10.)
        self.assertEqual(route[1],min((b for b,_ in calls),key=lambda b:math.hypot(b[0],b[1])))
        self.assertEqual(path[-1][:3],route[1])

    def test_entry_can_escape_blocked_nominal_without_switching_lines(self):
        g=self.grid();g.zero_cost_line=[((-4.,2.),(4.,2.)),((-4.,-2.),(4.,-2.))]
        start=(0.,0.,math.pi/2);goal=(1.,1.8,math.pi/2)
        nominal=core.line_stage_candidates(g,start,goal,'LATERAL')[0][3]
        free=g.free
        g.free=lambda p: math.hypot(p[0]-nominal[0],p[1]-nominal[1])>.1 and free(p)
        with patch.object(core,'plan',side_effect=lambda grid,a,b,**kw:[tuple(a)+(0,0.),tuple(b)+(-1,0.)]):
            path,route=core.plan_line_approach(g,start,goal,maneuver_mode='LATERAL')
        self.assertEqual(route[0],0)
        self.assertAlmostEqual(abs(route[1][0]-nominal[0]),.3)

    def test_entry_stops_at_first_continuous_route(self):
        g=self.grid();g.zero_cost_line=[((-4.,2.),(4.,2.))]
        start=(0.,0.,math.pi/2);goal=(1.,1.8,math.pi/2)
        calls=[]
        def fake(grid,a,b,**kw):
            calls.append(b)
            if len(calls)==1:
                return [tuple(a)+(0,0.),tuple(a)+(-1,0.),tuple(b)+(1,0.)]
            return [tuple(a)+(0,0.),tuple(b)+(-1,.3)]
        with patch.object(core,'plan',side_effect=fake):
            path,route=core.plan_line_approach(g,start,goal,maneuver_mode='LATERAL')
        self.assertEqual(len(calls),2)
        self.assertEqual(route[1],calls[1])
        self.assertEqual(len(core.segments(path)),1)

    def test_entry_cancel_after_continuous_result_is_not_ignored(self):
        g=self.grid();g.zero_cost_line=[((-4.,2.),(4.,2.))]
        cancelled=[False]
        def fake(grid,a,b,**kw):
            cancelled[0]=True
            return [tuple(a)+(0,0.),tuple(b)+(-1,0.)]
        with patch.object(core,'plan',side_effect=fake),self.assertRaisesRegex(RuntimeError,'cancelled'):
            core.plan_line_approach(g,(0.,0.,math.pi/2),(1.,1.8,math.pi/2),
                                   maneuver_mode='LATERAL',cancel=lambda:cancelled[0])

    def test_entry_prefers_longer_route_with_fewer_gear_changes(self):
        short=[(0.,0.,0.,0,0.),(.1,0.,0.,-1,0.),(.2,0.,0.,1,0.)]
        long=[(0.,0.,0.,0,0.),(-2.,0.,0.,-1,0.)]
        self.assertGreater(core.route_seconds(long),core.route_seconds(short))
        self.assertLess(core.entry_route_effort(long),core.entry_route_effort(short))

    def test_entry_penalizes_steering_oscillation_at_same_gear_count(self):
        smooth=[(0.,0.,0.,0,0.),(-.5,0.,0.,-1,.3),(-1.,0.,0.,-1,.3)]
        oscillating=[(0.,0.,0.,0,0.),(-.5,0.,0.,-1,.3),(-1.,0.,0.,-1,-.3)]
        self.assertLess(core.entry_route_effort(smooth),core.entry_route_effort(oscillating))

    def test_suffix_compares_complete_routes_and_keeps_join(self):
        g=self.grid();line=((-4.,2.),(4.,2.))
        start=(0.,2.,0.);goal=(1.,1.,math.pi/2)
        calls=[]
        def fake(grid,a,b,*args,**kw):
            if 'max_direction_changes' in kw:raise RuntimeError('budget exhausted')
            calls.append((a,b,kw))
            return [tuple(a)+(0,0.),tuple(a)+(-1,0.),tuple(a)+(1,0.),tuple(b)+(-1,0.)]
        with patch.object(core,'plan',side_effect=fake):
            path,retreat=core.plan_line_suffix(g,start,goal,line,0.,1.3,15.,lambda:False,.04,.17,'LATERAL',goal,True)
        self.assertEqual(len(calls),6)
        for i in range(0,6,2):
            self.assertEqual(calls[i][0],start)
            self.assertEqual(calls[i+1][0],calls[i][1])
            self.assertTrue(calls[i+1][2]['goal_region'])
        self.assertEqual(path[0][:3],start)
        self.assertEqual(path[-1][:3],goal)
        self.assertIn(retreat,[p[:3] for p in path])
        self.assertAlmostEqual(retreat[0],-.8)

    def test_rearward_entry_offsets_follow_arrival_heading_not_line_draw_order(self):
        for goal_x in (-1.,1.):
            for reverse in (False,True):
                g=self.grid();line=((-4.,2.),(4.,2.))
                g.zero_cost_line=[line[::-1] if reverse else line]
                start=(0.,0.,math.pi/2);goal=(goal_x,1.8,math.pi/2)
                base=core.line_stage_candidates(g,start,goal)[0][3]
                rows=core.line_stage_candidates(g,start,goal,offsets=(.3,.6,.9),rear_offsets=True)
                self.assertEqual(len(rows),3)
                for row,distance in zip(rows,(.3,.6,.9)):
                    p=row[3]
                    self.assertAlmostEqual(p[0],base[0]-distance*math.cos(base[2]))
                    self.assertAlmostEqual(p[1],base[1])

    def test_suffix_stops_when_both_legs_are_continuous(self):
        g=self.grid();goal=(1.,1.,math.pi/2);start=(0.,2.,0.)
        calls=[]
        def fake(grid,a,b,*args,**kw):
            calls.append(b)
            direction=1 if b==goal else -1
            return [tuple(a)+(0,0.),tuple(b)+(direction,0.)]
        with patch.object(core,'plan',side_effect=fake):
            path,retreat=core.plan_line_suffix(g,start,goal,((-4.,2.),(4.,2.)),0.,1.3,15.,lambda:False,.04,.17,'LATERAL',goal)
        self.assertEqual(len(calls),2)
        self.assertAlmostEqual(retreat[0],-.8)
        self.assertEqual(len(core.segments(path)),2)  # one expected cusp at point 2

    def test_suffix_does_not_accept_easy_retreat_with_failed_final(self):
        g=self.grid();goal=(1.,1.,math.pi/2);calls=[]
        def fake(grid,a,b,*args,**kw):
            calls.append(b)
            if len(calls)==2:raise RuntimeError('final blocked')
            return [tuple(a)+(0,0.),tuple(b)+(-1,0.)]
        with patch.object(core,'plan',side_effect=fake):
            path,retreat=core.plan_line_suffix(g,(0.,2.,0.),goal,((-4.,2.),(4.,2.)),0.,1.3,15.,lambda:False,.04,.17,'LATERAL',goal)
        self.assertEqual(len(calls),4)
        self.assertAlmostEqual(retreat[0],-1.1)
        self.assertEqual(path[-1][:3],goal)

    def test_suffix_prefers_longer_continuous_legs_over_shunting(self):
        anchor=(0.,0.,0.,0,0.)
        short=[anchor,(.1,0.,0.,-1,0.),(.2,0.,0.,1,0.),(.1,0.,0.,-1,0.)]
        long=[anchor,(-2.,0.,0.,-1,0.)]
        final=[(-2.,0.,0.,0,0.),(0.,0.,0.,1,0.)]
        self.assertGreater(core.route_seconds(long),core.route_seconds(short))
        self.assertLess(core.suffix_route_effort(long,final),core.suffix_route_effort(short,final))
        self.assertEqual(core.suffix_route_effort(long,final)[0],1)

    def test_suffix_equal_shunting_prefers_smoother_steering(self):
        smooth=[(0.,0.,0.,0,0.),(-.5,0.,0.,-1,.3),(-1.,0.,0.,-1,.3)]
        oscillating=[(0.,0.,0.,0,0.),(-.5,0.,0.,-1,.3),(-1.,0.,0.,-1,-.3)]
        final=[(-1.,0.,0.,0,0.),(0.,0.,0.,1,0.)]
        self.assertLess(core.suffix_route_effort(smooth,final),core.suffix_route_effort(oscillating,final))

    def test_bounded_gear_search_has_no_hidden_direction_changes(self):
        g=self.grid()
        for goal in ((1.,0.,0.),(-1.,0.,0.)):
            path=core.plan(g,(0.,0.,0.),goal,max_seconds=2.,max_direction_changes=0)
            self.assertEqual(core.entry_route_effort(path)[0],0)
            self.assertTrue(all(g.free(p[:3]) for p in path))
        with self.assertRaises(ValueError):
            core.plan(g,(0.,0.,0.),(1.,0.,0.),max_direction_changes=-1)

    def test_anytime_retains_feasible_path_when_improvement_times_out(self):
        path=[(0.,0.,0.,0,0.),(-.2,0.,0.,-1,0.),(.5,0.,0.,1,0.)]
        with patch.object(core,'plan',side_effect=[path,RuntimeError('planning budget exceeded')]):
            result=core.plan_fewer_changes(self.grid(),(0.,0.,0.),(.5,0.,0.),max_seconds=1.)
        self.assertIs(result,path)

    def test_anytime_adopts_longer_path_with_fewer_changes(self):
        path=[(0.,0.,0.,0,0.),(-.2,0.,0.,-1,0.),(.5,0.,0.,1,0.)]
        better=[path[0],(2.,0.,0.,1,0.),(.5,0.,0.,1,0.)]
        with patch.object(core,'plan',side_effect=[path,better]):
            result=core.plan_fewer_changes(self.grid(),(0.,0.,0.),(.5,0.,0.),max_seconds=1.)
        self.assertIs(result,better)

    def test_anytime_cancel_does_not_execute_incumbent(self):
        path=[(0.,0.,0.,0,0.),(-.2,0.,0.,-1,0.),(.5,0.,0.,1,0.)]
        cancelled=[False]
        def fake(*a,**kw):
            cancelled[0]=True
            return path
        with patch.object(core,'plan',side_effect=fake),self.assertRaisesRegex(RuntimeError,'cancelled'):
            core.plan_fewer_changes(self.grid(),(0.,0.,0.),(.5,0.,0.),cancel=lambda:cancelled[0])

    def test_suffix_rejects_all_partial_routes(self):
        g=self.grid();goal=(1.,1.,0.)
        def fake(grid,a,b,*args,**kw):
            if b==goal:raise RuntimeError('blocked final')
            return [tuple(a)+(0,0.),tuple(b)+(-1,0.)]
        with patch.object(core,'plan',side_effect=fake),self.assertRaises(RuntimeError):
            core.plan_line_suffix(g,(0.,2.,0.),goal,((-4.,2.),(4.,2.)),0.,1.3,15.,lambda:False,.04,.17,'LATERAL',goal)

    def test_cancel_never_installs_partial_route(self):
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            core.plan_line_suffix(self.grid(),(0.,2.,0.),(1.,1.,0.),((-4.,2.),(4.,2.)),0.,1.3,15.,lambda:True,.04,.17,'LATERAL',(1.,1.,0.))

if __name__=='__main__':unittest.main()
