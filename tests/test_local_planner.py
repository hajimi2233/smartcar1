import sys,math,unittest,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/smartcar_navigation/scripts'))
from ackermann_core import Grid,advance
from local_planner import choose
class LocalTests(unittest.TestCase):
 def grid(self):return Grid(160,160,.05,(-4,-4,0),[0]*25600)
 def segment(self,sign=1):return [(sign*i*.0125,0,0,sign,0) for i in range(81)]
 def test_forward_reverse_and_slew(self):
  for sign in (1,-1):
   g=self.grid();out=choose(g,(0,0,0),self.segment(sign),0,1.3,0)
   self.assertIsNotNone(out);self.assertGreater(out[1]*sign,0)
   self.assertLessEqual(abs(out[3]),.05)
   self.assertTrue(all(g.free(p) for p in out[4]))
 def test_blocked_no_command(self):
  g=self.grid();g.region=[(-.1,-1),(1,-1),(1,1),(-.1,1)]
  self.assertIsNone(choose(g,(0,0,0),self.segment(),0,1.3,0))
 def test_soft_margin_not_hard_obstacle(self):
  g=self.grid();g.region=[(-1,.34),(1,.34),(1,1),(-1,1)]
  self.assertTrue(g.free((0,0,0)))
  self.assertTrue(g.collision((0,0,0),extra=.08))
  self.assertIsNotNone(choose(g,(0,0,0),self.segment(),0,1.3,0))
 def test_end_and_cusp_not_crossed(self):
  g=self.grid();seg=self.segment()[:5]
  out=choose(g,(0,0,0),seg,0,1.3,0)
  self.assertIsNotNone(out)
  self.assertLessEqual(max(p[0] for p in out[4]),.05+1e-6)
 def test_sparse_reference_projection(self):
  from local_planner import path_error
  self.assertAlmostEqual(path_error((.5,.02,0),[(0,0,0),(1,0,0)])[0],.02)
 def test_offset_between_old_thresholds_can_recover(self):
  out=choose(self.grid(),(0,.19,0),self.segment(),0,1.3,0)
  self.assertIsNotNone(out)
  self.assertLess(out[4][-1][1],.19)
 def test_no_extra_guard_past_rollout(self):
  g=self.grid();g.dynamic.add(g.cell(1.02,0))
  out=choose(g,(0,0,0),self.segment(),0,1.3,0)
  self.assertIsNotNone(out)
  self.assertTrue(g.arc((0,0,0),.18,out[2]))
 def test_reports_actual_rejection(self):
  g=self.grid();g.region=[(-.1,-1),(1,-1),(1,1),(-.1,1)]
  d={};self.assertIsNone(choose(g,(0,0,0),self.segment(),0,1.3,0,d))
  self.assertGreater(d['counts']['STOPPING_COLLISION'],0)
 def test_progress_on_turn_is_path_distance(self):
  from local_planner import path_progress
  path=[(0,0,0),(1,0,0),(1,1,1.57)]
  self.assertAlmostEqual(path_progress((1,.5,1.57),path),1.5)
 def test_closed_loop_bend_both_directions(self):
  from ackermann_core import tracking
  for sign in (1,-1):
   g=self.grid();p=(0,0,0);segment=[(0,0,0,sign,0)]
   for i in range(65):
    k=0 if i<20 else .6
    p=advance(p,sign*.0125,k);segment.append(p+(sign,k))
   pose=(0,0,0);steer=0;index=0
   for i in range(240):
    _,_,index,remaining=tracking(pose,segment,index,1.3)
    if remaining<.06: break
    result=choose(g,pose,segment,index,1.3,steer)
    self.assertIsNotNone(result)
    _,v,k,steer,_=result
    # 0.1 seconds of ideal kinematic plant, with slew-limited commands.
    pose=advance(pose,v*.1,k)
   self.assertLess(remaining,.06)


class SteeringReversalTests(unittest.TestCase):
 def test_same_direction_adjustment_has_no_reversal_penalty(self):
  g=Grid(160,160,.05,(-4,-4,0),[0]*25600); out=choose(g,(0,0,0),[(i*.0125,0,0,1,0) for i in range(81)],0,1.3,.04)
  self.assertIsNotNone(out)

class WallPreferenceTests(unittest.TestCase):
 def test_nonstraight_modes_ignore_wall_preference(self):
  g=Grid(160,160,.05,(-4,-4,0),[0]*25600)
  segment=[(i*.0125,0,0,1,0) for i in range(81)]
  for mode in ('NORMAL','TURN_90_LEFT','LATERAL'):
   baseline=choose(g,(0,0,0),segment,0,1.1,0,maneuver_mode=mode)
   biased=choose(g,(0,0,0),segment,0,1.1,0,maneuver_mode=mode,wall_steer=.25,wall_weight=.7)
   self.assertEqual(baseline,biased)
 def test_straight_goal_curved_segment_keeps_tracking(self):
  g=Grid(160,160,.05,(-4,-4,0),[0]*25600);p=(0,0,0);segment=[p+(1,.2)]
  for i in range(80):
   p=advance(p,.0125,.2);segment.append(p+(1,.2))
  a=choose(g,(0,0,0),segment,0,1.1,0,maneuver_mode='STRAIGHT')
  b=choose(g,(0,0,0),segment,0,1.1,0,maneuver_mode='STRAIGHT',wall_steer=-.25,wall_weight=.7)
  self.assertEqual(a,b)
 def test_straight_bias_is_slew_limited_and_in_predicted_path(self):
  g=Grid(160,160,.05,(-4,-4,0),[0]*25600);segment=[(i*.0125,0,0,1,0) for i in range(81)]
  result=choose(g,(0,0,0),segment,0,1.1,0,current_speed=.05,maneuver_mode='STRAIGHT',wall_steer=.2,wall_weight=.7)
  self.assertIsNotNone(result);self.assertGreater(result[3],0);self.assertLessEqual(result[3],.05+1e-9)
  self.assertGreater(result[4][0][2],0)
  self.assertTrue(all(g.free(p) for p in result[4]))
