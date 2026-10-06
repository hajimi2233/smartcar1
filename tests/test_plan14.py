"""Compile the real C planner and validate its reduced coordinate contract."""
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/smartcar_mission/scripts'))
from plan_io import POINT_NAMES, load_plan


class Plan14Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.directory = Path(cls.tmp.name)
        cls.binary = cls.directory/'inspection_plan14'
        if os.environ.get('PLAN14_BINARY'):
            cls.binary = Path(os.environ['PLAN14_BINARY'])
            return
        core = ROOT/'src/smartcar_mission/core'
        subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                        str(core/'planner.c'), str(core/'plan14.c'), '-lm', '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def fixture(self, angle=0.):
        xy = [(i//2, 1. if i%2 == 0 else -1.) for i in range(10)]
        xy += [(-1., 2.), (-1., -2.), (5., 2.), (5., -2.), (-1., 2.), (-1., -2.)]
        xy = [(x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle)) for x,y in xy]
        path = self.directory/'points.csv'
        with path.open('w') as stream:
            writer = csv.writer(stream); writer.writerow(['point_id','x','y'])
            for name, pair in zip(POINT_NAMES, xy): writer.writerow([name]+list(pair))
        return path, dict(zip(POINT_NAMES, xy))

    def generate(self, layout, path):
        return subprocess.run([str(self.binary), layout, str(path)], capture_output=True, text=True)

    def test_all_1024_layouts_only_emit_inspections_outer_and_exit(self):
        path, positions = self.fixture()
        for bits in range(1024):
            layout = ''.join(('B' if bits & (1<<i) else 'A')+str(i+1) for i in range(10))
            result = self.generate(layout, path)
            self.assertEqual(result.returncode, 0, result.stderr)
            rows = [json.loads(line) for line in result.stdout.splitlines()]
            labels = [r['target_id'] for r in rows]
            self.assertEqual(labels[-1], 'end')
            self.assertNotIn('start', labels)
            for i in range(1,11): self.assertEqual(labels.count('inspect_%d'%i), 1)
            for row in rows:
                self.assertEqual((row['x'],row['y']), positions[row['target_id']])
                self.assertTrue(math.isfinite(row['yaw_rad']))
                self.assertEqual(row['frame_id'], 'map')
            exported = self.directory/'plan.jsonl'; exported.write_text(result.stdout)
            points, _ = load_plan(str(exported))
            self.assertEqual(len(points), len(rows))

    def test_rotated_channels_and_both_outer_directions(self):
        angle = .43
        path, _ = self.fixture(angle)
        result = self.generate(''.join('B'+str(i) for i in range(1,11)), path)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        for row in rows:
            name = row['target_id']
            if name.startswith('inspect_'):
                i = int(name.split('_')[1])
                expected = angle + (-math.pi/2 if i%2 else math.pi/2)
                self.assertAlmostEqual(math.cos(row['yaw_rad']-expected), 1.)
        outer = [row for row in rows if row['target_id'].startswith('outer_')]
        self.assertTrue(outer)
        self.assertAlmostEqual(math.cos(outer[0]['yaw_rad']-(angle-math.pi/2)), 1.)

    def test_bad_points_and_incomplete_layout_fail_without_output(self):
        layout = ''.join('A'+str(i) for i in range(1,11))
        path, _ = self.fixture()
        self.assertNotEqual(self.generate('A1A2', path).returncode, 0)
        original = path.read_text()
        for bad in (original.replace('inspect_2', 'inspect_1'), original.replace('inspect_1,0.0,1.0', 'inspect_1,nan,1.0'),
                    '\n'.join(original.splitlines()[:-1])):
            path.write_text(bad)
            result = self.generate(layout,path)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(result.stdout,'')

    def test_right_to_left_numbering_selects_physical_outer_end(self):
        path, positions = self.fixture()
        for i in range(1,11):
            x,y = positions['inspect_%d'%i]
            positions['inspect_%d'%i] = (4-x,y)
        with path.open('w') as stream:
            writer=csv.writer(stream); writer.writerow(['point_id','x','y'])
            for name in POINT_NAMES: writer.writerow([name]+list(positions[name]))
        result=self.generate(''.join('B'+str(i) for i in range(1,11)),path)
        self.assertEqual(result.returncode,0,result.stderr)
        rows=[json.loads(line) for line in result.stdout.splitlines()]
        outer=[r['target_id'] for r in rows if r['target_id'].startswith('outer_')]
        self.assertEqual(outer[:2],['outer_left_top','outer_left_bottom'])

    def test_degenerate_channel_is_rejected(self):
        path, _ = self.fixture()
        text = path.read_text().replace('inspect_2,0.0,-1.0','inspect_2,0.0,1.0')
        path.write_text(text)
        self.assertNotEqual(self.generate(''.join('A'+str(i) for i in range(1,11)),path).returncode,0)


if __name__ == '__main__': unittest.main()
