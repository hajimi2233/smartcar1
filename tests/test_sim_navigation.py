"""Check quick-start reuse/conflicts without disturbing an active simulation."""
import importlib.util
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, mock_open, patch


class QuickNavigationTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/sim_navigation.py'
        spec = importlib.util.spec_from_file_location('sim_navigation', str(path))
        self.nav = importlib.util.module_from_spec(spec)
        self.ros = {name: MagicMock() for name in ('rosgraph', 'rosnode', 'rospy')}
        with patch.dict(sys.modules, self.ros):
            spec.loader.exec_module(self.nav)
        self.map = '/home/hajimi/smartcar/data/maps/sim_field_narrow/map.yaml'
        self.lines = '/home/hajimi/smartcar/data/maps/sim_field_narrow/low_cost_lines.json'
        self.ros['rosnode'].get_node_names.return_value = ['/gazebo']
        self.ros['rospy'].get_param.side_effect = lambda key, default: {
            '/wall_localizer/target_mode': 'full_map',
            '/single_goal_nav/low_cost_lines_file': self.lines,
        }.get(key, default)
        self.launch = self.enterContext(patch.object(self.nav.os, 'execvp'))
        self.enterContext(patch.object(self.nav.os.path, 'isfile', return_value=True))
        self.enterContext(patch.object(self.nav, 'ServerProxy'))
        self.enterContext(patch('builtins.open', mock_open(read_data=(
            'map_server\0' + self.map + '\0').encode())))
        self.enterContext(patch.object(sys, 'argv', ['quick-nav', 'nav', self.map, self.lines]))
        self.enterContext(patch.dict(os.environ, SMARTCAR_ROOT='/home/hajimi/smartcar'))

    def localizer_running(self):
        self.ros['rosnode'].get_node_names.return_value += ['/wall_localizer', '/slam_map_server']

    def test_start_only_launches_localization(self):
        sys.argv[1] = 'start'
        self.nav.main()
        args = self.launch.call_args[0][1]
        self.assertIn('/tmp/smartcar-quick-localization.launch', args)
        self.assertIn('map_file:=' + self.map, args)
        self.assertFalse(any('low_cost_lines' in arg for arg in args))

    def test_nav_requires_localization(self):
        with self.assertRaisesRegex(SystemExit, 'Run sim.sh start'):
            self.nav.main()
        self.launch.assert_not_called()

    def test_matching_localization_is_reused(self):
        self.localizer_running()
        self.nav.main()
        args = self.launch.call_args[0][1]
        self.assertIn('/tmp/smartcar-quick-nav.launch', args)
        self.assertFalse(any('localization' in arg for arg in args))

    def test_start_reuses_localization_without_starting_navigation(self):
        sys.argv[1] = 'start'
        self.localizer_running()
        self.nav.main()
        self.launch.assert_not_called()

    def test_keyboard_blocks_navigation(self):
        self.localizer_running()
        self.ros['rosnode'].get_node_names.return_value += ['/inspection_sim_keyboard']
        with self.assertRaisesRegex(SystemExit, 'Keyboard control'):
            self.nav.main()
        self.launch.assert_not_called()

    def test_matching_navigation_is_not_started_twice(self):
        self.localizer_running()
        self.ros['rosnode'].get_node_names.return_value += ['/single_goal_nav']
        self.nav.main()
        self.launch.assert_not_called()

    def test_wrong_map_is_rejected(self):
        self.localizer_running()
        with patch('builtins.open', mock_open(read_data=b'map_server\0/other/map.yaml\0')):
            with self.assertRaisesRegex(SystemExit, 'different map'):
                self.nav.main()
        self.launch.assert_not_called()

    def test_partial_localization_is_rejected(self):
        self.ros['rosnode'].get_node_names.return_value += ['/slam_map_server']
        with self.assertRaisesRegex(SystemExit, 'incomplete localization'):
            self.nav.main()
        self.launch.assert_not_called()

    def test_different_line_store_is_rejected(self):
        self.localizer_running()
        self.ros['rosnode'].get_node_names.return_value += ['/single_goal_nav']
        self.ros['rospy'].get_param.side_effect = lambda key, default: (
            'full_map' if key.endswith('target_mode') else '/other/lines.json')
        with self.assertRaisesRegex(SystemExit, 'different localization or low-cost-line'):
            self.nav.main()
        self.launch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
