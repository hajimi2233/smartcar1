"""Offline behavior tests; no ROS, display or vehicle required."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class TeleopTest(unittest.TestCase):
    def run_keys(self, events):
        samples = []
        clock = [0.0]
        pending = ['']
        iterator = iter(events)
        class Twist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0)
                self.angular = types.SimpleNamespace(z=0.0)
        rospy = types.SimpleNamespace(
            init_node=lambda _: None, is_shutdown=lambda: False,
            Publisher=lambda *a, **k: types.SimpleNamespace(
                publish=lambda m: samples.append((m.linear.x, m.angular.z))))
        stdin = types.SimpleNamespace(isatty=lambda: True, fileno=lambda: 0,
                                      read=lambda _: pending[0])
        def poll(*args):
            t, key = next(iterator)
            clock[0], pending[0] = t, key
            return ([stdin] if key else [], [], [])
        mocks = {
            'rospy': rospy,
            'geometry_msgs': types.ModuleType('geometry_msgs'),
            'geometry_msgs.msg': types.SimpleNamespace(Twist=Twist),
            'termios': types.SimpleNamespace(tcgetattr=lambda _: [],
                       tcsetattr=lambda *a: None, TCSADRAIN=0),
            'tty': types.SimpleNamespace(setcbreak=lambda _: None),
        }
        path = Path(__file__).resolve().parents[1] / 'docker/overlay/terminal_teleop.py'
        with patch.dict(sys.modules, mocks):
            spec = importlib.util.spec_from_file_location('teleop_test_target', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with patch.object(sys, 'stdin', stdin), patch.object(module.select, 'select', poll), \
                 patch.object(module.time, 'time', lambda: clock[0]), \
                 patch.object(module.time, 'sleep', lambda _: None):
                module.main()
        return samples

    def test_speed_gear_and_steering_persist_until_changed(self):
        out = self.run_keys([(0, 'w'), (.1, 'a'), (.2, 'w'), (.3, 'q')])
        self.assertEqual(out[0], (.10, 0))
        self.assertAlmostEqual(out[1][0], .10)
        self.assertAlmostEqual(out[1][1], .10 * math.tan(math.radians(20)) / .62)
        self.assertAlmostEqual(out[2][0], .18)
        self.assertEqual(out[-3:], [(0, 0)] * 3)

    def test_reverse_steering_and_space_stop(self):
        out = self.run_keys([(0, 'a'), (.1, 's'), (.15, ' '), (.2, 'q')])
        self.assertEqual(out[0], (0, 0))
        self.assertEqual(out[1][0], -.10)
        self.assertLess(out[1][1], 0)
        self.assertEqual(out[2], (0, 0))

    def test_reverse_has_four_gears(self):
        out = self.run_keys([(0, 's'), (.1, 's'), (.2, 's'), (.3, 's'), (.4, 's'), (.5, 'q')])
        self.assertEqual([sample[0] for sample in out[:5]], [-.10, -.18, -.26, -.35, -.35])

if __name__ == '__main__':
    unittest.main()
