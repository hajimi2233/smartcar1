import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docker/overlay'))
from nav_obstacles import Recovery,smooth_ranges

class ObstacleTests(unittest.TestCase):
    def test_surface_noise_and_thin_obstacle(self):
        self.assertEqual(smooth_ranges([2.,1.98,2.01],.2,12)[1],2.)
        self.assertEqual(smooth_ranges([2.,.4,2.],.2,12)[1],.4)
        self.assertEqual(smooth_ranges([float('inf'),.4,2.],.2,12)[1],.4)
    def test_same_scan_cannot_resume(self):
        r=Recovery(10)
        for i in range(40): self.assertEqual(r.update(10+i*.1,1,True,True),'wait')
    def test_clear_scans_and_reblock(self):
        r=Recovery(10)
        for i in range(6): result=r.update(10+i*.11,i,True,True)
        self.assertEqual(result,'resume')
        self.assertEqual(r.update(10.7,6,False,True),'wait')
        self.assertEqual(r.clear_frames,0)
    def test_persistent_block_and_unstopped_timeout(self):
        self.assertEqual(Recovery(10).update(15,1,False,True),'replan')
        self.assertEqual(Recovery(10).update(20,1,True,False),'fail')


class ConsecutiveTests(unittest.TestCase):
    def test_same_source_different_coordinates(self):
        from nav_obstacles import ConsecutiveFailures, obstacle_key
        f=ConsecutiveFailures()
        self.assertTrue(f.record(obstacle_key('sources=[REGION]; travel=.1')))
        self.assertFalse(f.record(obstacle_key('sources=[REGION]; travel=.2')))
        self.assertEqual(obstacle_key('sources=[LIVE_SCAN@(1,2), STATIC_MAP@(3,4)]'),
                         obstacle_key('sources=[STATIC_MAP@(9,8), LIVE_SCAN@(7,6)]'))

    def test_different_errors_not_total_limit(self):
        from nav_obstacles import ConsecutiveFailures
        f=ConsecutiveFailures()
        for key in ['REGION','SCAN','TRACKING','REGION','SCAN']:
            self.assertTrue(f.record(key))
        self.assertFalse(f.record('SCAN'))
