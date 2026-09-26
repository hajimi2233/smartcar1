import sys,time,unittest
sys.path.insert(0,'/home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts')
from single_goal_nav import Navigator
from ackermann_core import Grid
from nav_obstacles import ConsecutiveFailures
class Sink:
 def publish(self,msg):
  assert msg.linear.x == 0 and msg.angular.z == 0
class TrackingTests(unittest.TestCase):
 def node(self):
  n=Navigator.__new__(Navigator);n.cmd=Sink();n.status=lambda s:None
  n.failures=ConsecutiveFailures();n.goal=(1,1,0);n.grid=Grid(100,100,.05,(-2.5,-2.5,0),[0]*10000)
  n.obstacles=lambda:None;n.start_plan=lambda p:setattr(n,'planned',p)
  n.halt=lambda s:(setattr(n,'goal',None),setattr(n,'failure',s))
  n.wait_tracking(.21);return n
 def test_stable_replans_retaining_goal(self):
  n=self.node();t=n.recovery.started
  for i in range(18):
   n.obstacle_stamp=i;n.recover_tracking((0,0,0),t+i*.1)
  self.assertEqual(n.planned,(0,0,0));self.assertEqual(n.goal,(1,1,0))
 def test_no_new_scans_no_replan(self):
  n=self.node();t=n.recovery.started
  for i in range(42):
   n.obstacle_stamp=1;n.recover_tracking((0,0,0),t+i*.1)
  self.assertFalse(hasattr(n,'planned'))
 def test_drift_timeout(self):
  n=self.node();t=n.recovery.started
  for i in range(102):
   n.obstacle_stamp=i;n.recover_tracking((i*.004,0,0),t+i*.1)
  self.assertIsNone(n.goal);self.assertFalse(hasattr(n,'planned'))
 def test_blocked_and_retry_limit(self):
  n=self.node();t=n.recovery.started;n.grid.dynamic.add((50,50))
  for i in range(52):
   n.obstacle_stamp=i;n.recover_tracking((0,0,0),t+i*.1)
  self.assertIsNone(n.goal);self.assertFalse(hasattr(n,'planned'))
  n=self.node();n.wait_tracking(.3)
  self.assertIsNone(n.goal)
unittest.main()
