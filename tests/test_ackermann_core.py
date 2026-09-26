"""Deterministic offline geometry/planning/control tests; not Gazebo validation."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docker/overlay'))
from ackermann_core import Grid, advance, plan, plan_prefer_lines, plan_line_approach, line_stage_candidates, line_retreat_target, line_approaches, segments, tracking, speed_profile, rear_target, front_position, wrap, StopWindow


def open_grid():
    return Grid(240,240,.05,(-6.,-6.,0.),[0]*57600)


class NavigationGeometryTests(unittest.TestCase):
    def test_front_rear_round_trip(self):
        for angle in (0,math.pi/2,math.pi,-math.pi/2):
            front=(2.,3.,angle)
            x,y=front_position(rear_target(front))
            self.assertAlmostEqual(x,2.)
            self.assertAlmostEqual(y,3.)

    def test_arc_reversibility(self):
        p=(.2,-.4,1.)
        restored=advance(advance(p,.5,1/1.3),-.5,1/1.3)
        for a,b in zip(p,restored):self.assertAlmostEqual(a,b)

    def test_body_collision_even_if_rear_axle_cell_free(self):
        data=[0]*57600
        g=open_grid();i,j=g.cell(.60,0.)
        data[j*g.w+i]=100
        g=Grid(g.w,g.h,g.res,g.origin,data)
        self.assertFalse(g.occupied(*g.cell(0.,0.)))
        self.assertFalse(g.free((0.,0.,0.)))
        with self.assertRaises(ValueError):plan(g,(0.,0.,0.),(2.,0.,0.))

    def test_swept_arc_detects_obstacle_between_free_endpoints(self):
        g=open_grid();data=list(g.data);i,j=g.cell(1.1,0.);data[j*g.w+i]=100
        g=Grid(g.w,g.h,g.res,g.origin,data)
        self.assertTrue(g.free((0.,0.,0.)))
        self.assertTrue(g.free((2.,0.,0.)))
        self.assertIsNone(g.arc((0.,0.,0.),2.,0.))

    def test_unknown_and_outside_rejected(self):
        g=Grid(40,40,.05,(-1,-1,0),[-1]*1600)
        self.assertFalse(g.free((0.,0.,0.)))
        self.assertFalse(open_grid().free((6.,0.,0.)))

    def test_rotated_map_origin(self):
        g=Grid(40,40,.05,(3.,4.,math.pi/2),[0]*1600)
        self.assertEqual(g.cell(2.475,4.525),(10,10))

    def test_goal_collision_never_shifted(self):
        g=open_grid();g.dynamic.add(g.cell(2.,0.))
        with self.assertRaises(ValueError):plan(g,(0.,0.,0.),(2.,0.,0.))

    def test_radius_bound_and_cancellation(self):
        g=open_grid()
        with self.assertRaises(ValueError):plan(g,(0.,0.,0.),(1.,0.,0.),radius=.8)
        with self.assertRaises(RuntimeError):plan(g,(0.,0.,0.),(1.,0.,0.),cancel=lambda: True)

    def test_detour_around_obstacle_is_checked_for_entire_body(self):
        g=open_grid()
        g.dynamic=set(g.cell(1.5,y/20.) for y in range(-6,7))
        path=plan(g,(0.,0.,0.),(3.,0.,0.),max_seconds=15.)
        self.assertGreater(max(abs(p[1]) for p in path),.5)
        self.assertTrue(all(g.free(p[:3]) for p in path))

    def test_planner_and_closed_loop_reach_front_axle_pose(self):
        # Bicycle simulation with command steering slew; no Gazebo/slip/sensor model.
        goals=[(1.,0.,0.),(-1.,0.,0.),(1.3,1.3,math.pi/2),(0.,0.,math.pi/2)]
        for goal in goals:
            g=open_grid()
            path=plan(g,(0.,0.,0.),goal,max_seconds=15.,goal_position_tolerance=.08,goal_heading_tolerance=.08)
            self.assertLessEqual(math.hypot(path[-1][0]-goal[0],path[-1][1]-goal[1]),.08)
            self.assertLessEqual(abs(wrap(path[-1][2]-goal[2])),.08)
            for previous,p in zip(path,path[1:]):
                self.assertLessEqual(abs(p[4]),1/1.3+1e-9)
                self.assertTrue(g.free(p[:3]))
                if abs(wrap(p[2]-previous[2]))>1e-7:
                    self.assertGreater(math.hypot(p[0]-previous[0],p[1]-previous[1]),0.)
            if goal[0]<0:self.assertTrue(any(p[3]<0 for p in path))
            if goal==(0.,0.,math.pi/2):self.assertGreater(len(segments(path)),1)
            pose=(0.,0.,0.)
            for part in segments(path):
                index,steer=0,0.
                for _ in range(12000):
                    v,k,index,remaining=tracking(pose,part,index)
                    if remaining<.025 and index>=len(part)-8:break
                    desired=math.atan(.62*k)
                    steer=max(steer-.05,min(steer+.05,desired))
                    pose=advance(pose,v*.05,math.tan(steer)/.62)
                else:self.fail('controller failed to reach direction-segment endpoint')
            actual=front_position(pose);target=front_position(goal)
            self.assertLessEqual(math.hypot(actual[0]-target[0],actual[1]-target[1]),.08)
            self.assertLessEqual(abs(wrap(pose[2]-goal[2])),math.radians(4))

if __name__ == '__main__':unittest.main()


class StopWindowTests(unittest.TestCase):
    def test_stationary_lidar_jitter(self):
        window = StopWindow()
        results = [window.update(i*.05, (.006*(-1)**i, .002*(-1)**i, .002*(-1)**i))
                   for i in range(25)]
        self.assertFalse(any(results[:20]))
        self.assertTrue(results[-1])

    def test_creep_rotation_and_oscillation_are_not_stopped(self):
        for poses in ([(i*.05*.035, 0, 0) for i in range(25)],
                      [(0, 0, i*.05*.03) for i in range(25)],
                      [(.025*math.sin(i*.5), 0, 0) for i in range(25)]):
            window = StopWindow()
            results = [window.update(i*.05, p) for i,p in enumerate(poses)]
            self.assertFalse(any(results))

    def test_heading_wrap(self):
        window = StopWindow()
        for i in range(25):
            stopped = window.update(i*.05, (0, 0, wrap(math.pi+.002*(-1)**i)))
        self.assertTrue(stopped)


class SpeedProfileTests(unittest.TestCase):
    def test_curve_and_cusp_caps(self):
        path=[(0,0,0,0,0),(1,0,0,1,0),(1.1,.1,.5,1,.8),(1.1,.1,.5,-1,0)]
        caps=speed_profile(path); self.assertGreater(caps[1],0); self.assertLessEqual(caps[2],caps[1]); self.assertEqual(caps[3],0)


class GoalToleranceTests(unittest.TestCase):
    def test_goal_region_tolerance(self):
        g=open_grid(); p=plan(g,(0,0,0),(.06,0,0),radius=1.3,goal_position_tolerance=.08,goal_heading_tolerance=math.radians(3))
        self.assertLessEqual(math.hypot(p[-1][0]-.06,p[-1][1]),.08)


class PreferredLineTests(unittest.TestCase):
    def test_line_ranking_uses_requested_front_axle_goal(self):
        with patch('ackermann_core.line_stage_candidates', return_value=[]) as candidates:
            plan_line_approach(open_grid(), (0., 0., 0.), (2., 0., 0.),
                               front_goal=(2.62, 0., 0.))
        self.assertEqual(candidates.call_args[0][2], (2.62, 0., 0.))

    def test_staged_route_selects_closest_feasible_line_to_goal(self):
        g = open_grid()
        g.zero_cost_line = [((.4, -1.), (.4, 1.)),
                            ((1.4, -1.), (1.4, 1.))]
        start = (0., 0., 0.)
        with patch('ackermann_core.plan', return_value=[start, (1.4, -1.5, 0., -1, 0.)]) as planner:
            path, route = plan_line_approach(g, start, (1.5, 1., 0.),
                                             max_seconds=5., maneuver_mode='LATERAL')
        self.assertEqual(route[0], 1)
        self.assertAlmostEqual(route[1][0], 1.4)
        self.assertAlmostEqual(route[1][1], -1.5)
        self.assertTrue(planner.call_args[1]['start_reverse_only'])

    def test_lateral_approach_starts_in_reverse(self):
        g = open_grid()
        g.zero_cost_line = [((.4, -2.), (.4, 2.))]
        start = (0., 0., 0.)
        with patch('ackermann_core.plan', return_value=[start, (.4, -1.5, 0., 1, 0.)]):
            path, route = plan_line_approach(g, start, (2., 1., 0.),
                                             max_seconds=5., maneuver_mode='LATERAL')
        self.assertIsNone(route)
        self.assertIsNone(path)

    def test_short_or_blocked_lines_have_no_stage_candidate(self):
        g = open_grid()
        g.zero_cost_line = [((0., .3), (.2, .3)), ((7., 0.), (8., 0.))]
        self.assertEqual(line_stage_candidates(g, (0., 0., 0.), (2., 0., 0.)), [])

    def test_line_endpoint_candidates_are_unique(self):
        g = open_grid()
        g.zero_cost_line = [((.4, -1.), (.4, 1.))]
        candidates = line_stage_candidates(g, (0., 0., 0.), (2., 1., 0.))
        poses = [item[3] for item in candidates]
        self.assertEqual(len(poses), len(set(poses)))

    def test_line_entry_side_is_vehicle_relative_and_ignores_draw_order(self):
        g = open_grid()
        start = (0., 0., 0.)
        for line in (((.4, -.5), (.4, .5)), ((.4, .5), (.4, -.5))):
            g.zero_cost_line = [line]
            for goal_y, mode, expected_y in ((1., 'LATERAL', -1.5),
                                             (-1., 'LATERAL', 1.5),
                                             (1., 'LATERAL_TURN_180', 1.5),
                                             (-1., 'LATERAL_TURN_180', -1.5)):
                candidates = line_stage_candidates(g, start, (2., goal_y, 0.), mode)
                self.assertTrue(candidates)
                self.assertAlmostEqual(candidates[0][3][0], .4)
                self.assertAlmostEqual(candidates[0][3][1], expected_y)
                self.assertAlmostEqual(candidates[0][3][2], 0.)

    def test_second_point_uses_goal_projection_and_first_heading(self):
        for line in (((0., 0.), (2., 0.)), ((2., 0.), (0., 0.))):
            right = line_retreat_target(line, (1.8, .4, 0.), math.pi)
            left = line_retreat_target(line, (.2, -.4, 0.), 0.)
            self.assertAlmostEqual(right[0], 3.3)
            self.assertAlmostEqual(right[1], 0.)
            self.assertAlmostEqual(right[2], math.pi)
            self.assertAlmostEqual(left[0], -1.3)
            self.assertAlmostEqual(left[1], 0.)
            self.assertAlmostEqual(left[2], 0.)
        with self.assertRaises(ValueError):
            line_retreat_target(((0., 0.), (2., 0.)), (1., 0., 0.), math.pi/2)

    def test_line_cost_rewards_matching_heading(self):
        g = open_grid()
        g.zero_cost_line = [((0., 0.), (2., 0.))]
        forward = g.line_bonus((1., 0., 0.))
        crosswise = g.line_bonus((1., 0., math.pi/2))
        reverse = g.line_bonus((1., 0., math.pi))
        self.assertGreater(forward, crosswise)
        self.assertAlmostEqual(forward, reverse)
        self.assertEqual(g.line_bonus((1., .2, 0.)), 0.)

    def test_line_entry_direction_does_not_depend_on_click_order(self):
        g = open_grid()
        g.zero_cost_line = [((1.4, .3), (.4, .3))]
        approaches = line_approaches(g, (0., 0., 0.), (2., 0., 0.))
        self.assertTrue(approaches)
        self.assertLess(abs(wrap(approaches[0][2])), .1)

    def test_special_move_joins_line_then_reaches_goal(self):
        g = open_grid()
        g.zero_cost_line = [((.4, .3), (1.4, .3))]
        path, entry = plan_prefer_lines(g, (0., 0., 0.), (2., 0., 0.),
                                        max_seconds=3., maneuver_mode='LATERAL',
                                        return_line=True)
        self.assertIsNotNone(entry)
        self.assertLess(abs(entry[1]-.3), .08)
        self.assertGreater(max(p[1] for p in path), .2)
        self.assertLess(math.hypot(path[-1][0]-2., path[-1][1]), .08)
        self.assertTrue(all(g.free(p[:3]) for p in path))
        direct = plan_prefer_lines(g, (0., 0., 0.), (2., 0., 0.),
                                   max_seconds=3., maneuver_mode='NORMAL')
        self.assertLess(max(p[1] for p in direct), .2)

    def test_unreachable_line_falls_back_to_direct_path(self):
        g = open_grid()
        g.zero_cost_line = [((7., 0.), (8., 0.))]
        self.assertEqual(line_approaches(g, (0., 0., 0.), (2., 0., 0.)), [])
        path, entry = plan_prefer_lines(g, (0., 0., 0.), (2., 0., 0.),
                                        max_seconds=3., maneuver_mode='TURN_90_LEFT',
                                        return_line=True)
        self.assertIsNone(entry)
        self.assertLess(math.hypot(path[-1][0]-2., path[-1][1]), .08)
