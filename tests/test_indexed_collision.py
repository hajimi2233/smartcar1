import os, sys, random, math, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src/smartcar_navigation/scripts'))
from ackermann_core import Grid
from nav_config import P


class CollisionIndexTests(unittest.TestCase):
    def test_matches_full_cell_scan(self):
        rng = random.Random(71)
        data = [rng.choice([0]*30 + [100, -1]) for _ in range(80*80)]
        g = Grid(80, 80, .05, (-2., -2., .2), data)
        # Restore the full rectangle scan as an independent reference while
        # retaining the original exact geometry and source diagnostics.
        import inspect
        source = inspect.getsource(Grid.collision)
        a = source.index('        stride = self.w+1')
        b = source.index('                dx, dy =', a)
        tail = source[b:source.index('        return hits', b)]
        tail = '\n'.join(line[4:] for line in tail.split('\n'))
        source = source[:a] + "        for i, j in ((i, j) for j in range(jmin, jmax+1) for i in range(imin, imax+1) if self.occupied(i, j)):\n" + tail + '        return hits\n'
        import textwrap, ackermann_core
        namespace = dict(vars(ackermann_core))
        exec(textwrap.dedent(source), namespace)
        reference = namespace['collision']
        for n in range(5000):
            if n % 100 == 0:
                g.dynamic = set((rng.randrange(80), rng.randrange(80)) for _ in range(50))
            if n % 10 == 0:
                g.dynamic.add((rng.randrange(80), rng.randrange(80)))
            p = (rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5), rng.uniform(-math.pi, math.pi))
            extra = rng.choice([0., .02, .10])
            self.assertEqual(reference(g, p, True, extra), g.collision(p, True, extra))
            self.assertEqual(reference(g, p, False, extra), g.collision(p, False, extra))

    def test_wall_filter_does_not_treat_unknown_as_wall(self):
        data = [0]*100
        data[55] = -1
        data[22] = 100
        g = Grid(10, 10, .05, (0, 0, 0), data)
        self.assertFalse(g.known_static_near(5, 5))
        self.assertTrue(g.known_static_near(2, 3))
        self.assertFalse(g.known_static_near(2, 4))

if __name__ == '__main__': unittest.main()
