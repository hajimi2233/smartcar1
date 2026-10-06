"""Map-aligned inspection depth limits measured at the front axle midpoint."""
import math

HALF = .4


def validate_regions(regions):
    if not isinstance(regions, list) or len(regions) != 10:
        raise ValueError('Expected 10 inspection regions')
    result = []
    for i, r in enumerate(regions):
        if r.get('id') != 'inspect_%d' % (i+1) or r.get('type') not in ('A', 'B'):
            raise ValueError('Invalid inspection id/type')
        for k in ('x', 'y'):
            v = r.get(k)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or math.isnan(v) or math.isinf(v):
                raise ValueError('Invalid inspection coordinate')
        result.append(dict(r))
    return result


def inside(point, region):
    return abs(point[0]-region['x']) <= HALF+1e-8 and abs(point[1]-region['y']) <= HALF+1e-8


def entry_sign(yaw):
    s = math.sin(yaw)
    if abs(s) < .5:
        raise ValueError('Inspection heading must point up or down in map coordinates')
    return 1 if s > 0 else -1


def rules_for_goal(regions, current_front, goal, target_id, known_sides):
    """Current B occupancy and target depth constraints coexist for this goal."""
    rules = {}
    for r in regions:
        if r['type'] == 'B' and inside(current_front, r):
            sign = known_sides.get(r['id'])
            if sign is None:
                sign = entry_sign(current_front[2])
                known_sides[r['id']] = sign
            rules[r['id']] = dict(r, direction=sign)
    if target_id is not None:
        matches = [r for r in regions if r['id'] == target_id]
        if not matches:
            raise ValueError('Unknown inspection target')
        r = matches[0]
        if math.hypot(goal[0]-r['x'], goal[1]-r['y']) > 1e-6:
            raise ValueError('Goal does not match inspection center')
        sign = entry_sign(goal[2])
        if r['id'] in rules and rules[r['id']]['direction'] != sign:
            raise ValueError('Cannot reverse depth boundary while inside a B region')
        rules[r['id']] = dict(r, direction=sign)
        # B entry direction remains fixed even if the vehicle later turns inside.
        if r['type'] == 'B': known_sides[r['id']] = sign
    return list(rules.values())


def blocked(point, rules, extra=0.):
    for r in rules:
        if (abs(point[0]-r['x']) <= HALF+extra and
                r['direction']*(point[1]-r['y']) >= HALF-extra):
            return r['id']
    return None


def segment_blocked(a, b, rules, extra=0.):
    """Clip a chord against each closed forbidden half-strip (no sampling gaps)."""
    for r in rules:
        lo, hi = 0., 1.
        # Each linear inequality is f(t) >= 0.
        for f, df in ((a[0]-r['x']+HALF+extra, b[0]-a[0]),
                      (r['x']+HALF+extra-a[0], a[0]-b[0]),
                      (r['direction']*(a[1]-r['y'])-HALF+extra,
                       r['direction']*(b[1]-a[1]))):
            if abs(df) < 1e-15:
                if f < 0: hi = -1.; break
            elif df > 0: lo = max(lo, -f/df)
            else: hi = min(hi, -f/df)
        if lo <= hi: return True
    return False
