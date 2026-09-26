#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-independent rear-axle Hybrid A* and signed path tracking (Python 2/3)."""
from __future__ import division
import heapq
import math
import time
from region_geometry import intersects


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def finite(values):
    return all(not math.isnan(v) and not math.isinf(v) for v in values)


def advance(p, distance, curvature):
    x, y, yaw = p
    turn = distance * curvature
    if abs(curvature) < 1e-9:
        return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw
    return (x + (math.sin(yaw + turn) - math.sin(yaw)) / curvature,
            y + (math.cos(yaw) - math.cos(yaw + turn)) / curvature,
            wrap(yaw + turn))


def rear_target(front, wheelbase=0.62):
    return (front[0] - wheelbase * math.cos(front[2]),
            front[1] - wheelbase * math.sin(front[2]), front[2])


def front_position(rear, wheelbase=0.62):
    return (rear[0] + wheelbase * math.cos(rear[2]),
            rear[1] + wheelbase * math.sin(rear[2]))


class Grid(object):
    """Unknown/outside blocked. Conservative rectangle vs occupied cell disks.

    Body relative to rear axle: [-.09, .71] x [-.275, .275].
    Obstacle cell circumradius added to margin; samples <= resolution/2.
    """
    def __init__(self, width, height, resolution, origin, data, margin=0.04):
        self.w, self.h, self.res = width, height, resolution
        self.origin = origin
        self.data = data
        self.margin = margin
        self.front, self.back, self.half_width = .70, .08, .25
        self.dynamic = set()
        self.region = []
        self.region_margin = .01
        self.zero_cost_line = []
        self.zero_cost_width = .01
        self.cache = {}
        self.prefix = [0] * ((width+1)*(height+1))
        for j in range(height):
            row = 0
            for i in range(width):
                row += int(data[j*width+i] < 0 or data[j*width+i] >= 50)
                self.prefix[(j+1)*(width+1)+i+1] = self.prefix[j*(width+1)+i+1] + row

    def cell(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return int(math.floor((c * dx + s * dy) / self.res)), int(math.floor((-s * dx + c * dy) / self.res))

    def local(self, p):
        dx, dy = p[0] - self.origin[0], p[1] - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - self.origin[2])

    def occupied(self, i, j):
        return (i < 0 or j < 0 or i >= self.w or j >= self.h or
                self.data[j * self.w + i] < 0 or self.data[j * self.w + i] >= 50 or
                (i, j) in self.dynamic)

    def region_blocked(self, pose, extra=0.):
        if not self.region:
            return False
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        m = self.region_margin + extra
        # Body and front tire envelopes separately: the wide steering envelope
        # must not widen the entire rear body. Tire disks enclose all angles.
        rectangles = [(-.09-m, .71+m, -.275-m, .275+m)]
        radius = math.hypot(.09, .035)
        for side in (-.225, .225):
            rectangles.append((.62-radius-m, .62+radius+m,
                               side-radius-m, side+radius+m))
        for left, right, bottom, top in rectangles:
            body = [(pose[0]+x*c-y*sn, pose[1]+x*sn+y*c)
                    for x,y in ((left,bottom),(right,bottom),(right,top),(left,top))]
            if intersects(body, self.region):
                return True
        return False

    def line_bonus(self, pose):
        """Return a bounded, direction-aware preference near drawn lines.

        ``zero_cost_line`` is a list of ``((x1, y1), (x2, y2))`` pairs.
        Drawn lines have no arrow, so either parallel heading is preferred.
        """
        if not self.zero_cost_line:
            return 0.0
        best = 0.0
        for segment in self.zero_cost_line:
            try:
                (x1, y1), (x2, y2) = segment
            except (TypeError, ValueError):
                continue
            dx, dy = x2 - x1, y2 - y1
            den = dx * dx + dy * dy
            if den < 1e-8:
                continue
            t = max(0., min(1., ((pose[0] - x1) * dx + (pose[1] - y1) * dy) / den))
            distance = math.hypot(pose[0] - (x1 + t * dx),
                                  pose[1] - (y1 + t * dy))
            influence = max(.15, self.zero_cost_width)
            proximity = max(0., 1. - distance / influence)
            alignment = abs(math.cos(wrap(pose[2] - math.atan2(dy, dx))))
            best = max(best, proximity * (.25 + .75 * alignment**4))
        return best

    def boundary_cost(self, pose, extra=0.):
        """Progressive cost near the finite map boundary; outside remains hard blocked."""
        if not finite(pose):
            return float('inf')
        x, y, yaw = self.local(pose)
        c, sn = math.cos(yaw), math.sin(yaw)
        m = extra + self.margin + self.res * .707107 + self.res * .5
        a, b = .4 + m, self.half_width + m
        ex, ey = abs(c) * a + abs(sn) * b, abs(sn) * a + abs(c) * b
        clearance = min(x-ex, y-ey, self.w*self.res-x-ex,
                        self.h*self.res-y-ey)
        if clearance < 0.:
            return float('inf')
        # Cost tiers: no edge pressure, then increasing preference for the interior.
        if clearance >= .50: return 0.0
        if clearance >= .25: return .25
        if clearance >= .10: return 1.0
        return 3.0

    def free(self, pose):
        return not self.collision(pose)

    def collision(self, pose, detailed=False, extra=0.):
        """Same geometry for decisions and diagnostics; detailed gathers all sources."""
        hits = {}

        if not finite(pose):
            return {'INVALID_POSE': None}
        if self.region_blocked(pose, extra):
            hits['REGION'] = None
            if not detailed: return hits
        x, y, yaw = self.local(pose)
        c, s = math.cos(yaw), math.sin(yaw)
        # Half a cell diagonal and sampling sweep bound added conservatively.
        m = extra + self.margin + self.res * .707107 + self.res * .5
        center_x, center_y = x + .31 * c, y + .31 * s
        a, b = .4 + m, self.half_width + m
        ex, ey = abs(c) * a + abs(s) * b, abs(s) * a + abs(c) * b
        imin, imax = int(math.floor((center_x-ex)/self.res)), int(math.floor((center_x+ex)/self.res))
        jmin, jmax = int(math.floor((center_y-ey)/self.res)), int(math.floor((center_y+ey)/self.res))
        if imin < 0 or jmin < 0 or imax >= self.w or jmax >= self.h:
            hits['MAP_BOUNDS'] = None
            return hits
        stride = self.w+1
        count = (self.prefix[(jmax+1)*stride+imax+1] - self.prefix[jmin*stride+imax+1]
                 - self.prefix[(jmax+1)*stride+imin] + self.prefix[jmin*stride+imin])
        if count == 0 and not any(imin <= i <= imax and jmin <= j <= jmax for i,j in self.dynamic):
            return hits
        for j in range(jmin, jmax+1):
            for i in range(imin, imax+1):
                if not self.occupied(i, j):
                    continue
                dx, dy = (i+.5)*self.res-center_x, (j+.5)*self.res-center_y
                if abs(c*dx+s*dy) <= a and abs(-s*dx+c*dy) <= b:
                    cell = ((i+.5)*self.res, (j+.5)*self.res)
                    co, so = math.cos(self.origin[2]), math.sin(self.origin[2])
                    point = (self.origin[0]+co*cell[0]-so*cell[1],
                             self.origin[1]+so*cell[0]+co*cell[1])
                    value = self.data[j*self.w+i]
                    if value < 0: hits.setdefault('MAP_UNKNOWN', point)
                    elif value >= 50: hits.setdefault('STATIC_MAP', point)
                    if (i,j) in self.dynamic: hits.setdefault('LIVE_SCAN', point)
                    if not detailed: return hits
        return hits

    def arc(self, pose, distance, curvature):
        # Maximum body-point speed / rear axle speed < 1.7 at radius >= 1.3.
        n = max(1, int(math.ceil(abs(distance) / min(.025, self.res / 4))))
        pts = [advance(pose, distance*i/n, curvature) for i in range(1, n+1)]
        return pts if all(self.free(p) for p in pts) else None

    def blocked_detail(self, pose, distance=0., curvature=0.):
        n = max(1, int(math.ceil(abs(distance) / min(.025, self.res / 4))))
        indices = range(1, n+1) if distance else [0]
        for i in indices:
            travel = distance*i/n
            p = advance(pose, travel, curvature)
            hits = self.collision(p, detailed=True)
            if hits:
                sources = ', '.join(name + ('@(%.3f,%.3f)' % point if point is not None else '')
                                    for name,point in sorted(hits.items()))
                return ('sources=[%s]; travel=%.3fm; rear_pose=(%.3f,%.3f,%.1fdeg); frame=map'
                        % (sources, travel, p[0], p[1], math.degrees(p[2])))
        return 'no collision in checked samples'


def plan(grid, start, goal, radius=1.3, max_seconds=12.0, cancel=lambda: False, goal_position_tolerance=.08, goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL', front_goal=None, start_reverse_only=False):
    """Returns [(x,y,yaw,direction,curvature), ...], no endpoint snapping.

    Weighted Hybrid A*: finite budget, non-optimal; fails if search cannot find
    a valid path. Never shrinks vehicle, shifts goal, or permits pivot turns.
    """
    if (radius < 1.3 or not finite([goal_position_tolerance, goal_heading_tolerance])
            or goal_position_tolerance <= 0 or goal_heading_tolerance <= 0
            or not finite(start+goal) or not grid.free(start) or not grid.free(goal)):
        raise ValueError('start/goal collision, nonfinite pose, or radius below 1.3 m')
    started = time.time()
    def key(p, d):
        return int(round(p[0]/.08)), int(round(p[1]/.08)), int(round(wrap(p[2])/(math.pi/36))) % 72, d
    def goal_error(p):
        xy = front_position(p) if front_goal is not None else p[:2]
        target = front_goal if front_goal is not None else goal
        return math.hypot(xy[0]-target[0], xy[1]-target[1]), abs(wrap(p[2]-target[2]))
    def heuristic(p):
        distance, angle = goal_error(p)
        return max(max(0., distance-goal_position_tolerance),
                   radius*max(0., angle-goal_heading_tolerance))
    # Immutable parent records: updating a discretized state's score must never
    # mutate a previously generated child's geometric ancestor.
    records = [(start, 0, 0., None, [])]
    queue = [(heuristic(start), 0)]
    best = {key(start, 0): 0.}
    while queue:
        if cancel():
            raise RuntimeError('cancelled')
        if time.time()-started > max_seconds or len(records) > 60000:
            raise RuntimeError('planning budget exceeded; choose a roomier approach point')
        _, idx = heapq.heappop(queue)
        p, previous_sign, cost, parent, _ = records[idx]
        if cost > best.get(key(p, previous_sign), float('inf')) + 1e-8:
            continue
        dist = math.hypot(p[0]-goal[0], p[1]-goal[1])
        goal_distance, goal_angle = goal_error(p)
        if goal_distance <= goal_position_tolerance and goal_angle <= goal_heading_tolerance:
            result = []
            while records[idx][3] is not None:
                result.append(records[idx][4]); idx = records[idx][3]
            return [(start[0],start[1],start[2],0,0.)] + [p for chunk in reversed(result) for p in chunk]
        for direction in (1, -1):
            if start_reverse_only and parent is None and direction > 0:
                continue
            lengths = [.30] if dist > .8 else [.10, .25]
            # Aligned terminal straight motion improves precision without a snap.
            along = (goal[0]-p[0])*math.cos(p[2]) + (goal[1]-p[1])*math.sin(p[2])
            across = -(goal[0]-p[0])*math.sin(p[2]) + (goal[1]-p[1])*math.cos(p[2])
            if .015 < abs(along) < .8 and along*direction > 0 and abs(across) < .025:
                lengths = lengths + [abs(along)]
            for length in lengths:
                for curvature in (0., -1./radius, -.5/radius, .5/radius, 1./radius):
                    endpoint = advance(p, direction*length, curvature)
                    switch = .35 if previous_sign and previous_sign != direction else 0.
                    reverse = 1.35 if direction < 0 else 1.0
                    maneuver_bonus = 0.0
                    if maneuver_mode == 'STRAIGHT' and abs(curvature) > 1e-9:
                        maneuver_bonus += .35 * length
                    elif maneuver_mode == 'LATERAL':
                        if abs(curvature) < 1e-9:
                            maneuver_bonus += .20 * length
                        if previous_sign and previous_sign != direction:
                            maneuver_bonus += .20
                    elif maneuver_mode == 'LATERAL_TURN_180':
                        if abs(curvature) < 1e-9:
                            maneuver_bonus += .15 * length
                        if direction > 0:
                            maneuver_bonus -= .08 * length
                    if maneuver_mode in ('TURN_90_LEFT', 'TURN_90_RIGHT'):
                        desired_sign = 1.0 if maneuver_mode.endswith('LEFT') else -1.0
                        travelled = math.hypot(p[0]-start[0], p[1]-start[1])
                        if travelled < .45 and curvature * desired_sign < 0:
                            maneuver_bonus = -.28 * length
                        elif travelled >= .30 and curvature * desired_sign > 0:
                            maneuver_bonus = -.16 * length
                    newcost = cost + length*reverse + switch + maneuver_bonus
                    k = key(endpoint, direction)
                    if newcost >= best.get(k, float('inf')):
                        continue
                    arc = grid.arc(p, direction*length, curvature)
                    if arc is None:
                        continue
                    # Soft clearance: narrow but hard-safe states remain legal.
                    edge_cost = grid.boundary_cost(endpoint)
                    if math.isinf(edge_cost):
                        continue
                    newcost += length * (1.5 if grid.collision(endpoint, extra=.02) else 0.)
                    newcost += length * edge_cost
                    newcost -= .70 * length * grid.line_bonus(endpoint)
                    if newcost >= best.get(k, float('inf')):
                        continue
                    best[k] = newcost
                    segment = [(q[0],q[1],q[2],direction,curvature) for q in arc]
                    records.append((endpoint,direction,newcost,idx,segment))
                    heapq.heappush(queue,(newcost+1.6*heuristic(endpoint),len(records)-1))
    raise RuntimeError('no path found')


def line_approaches(grid, start, goal, limit=4):
    """Rank collision-free entry poses in either direction along drawn lines."""
    candidates = []
    for line in grid.zero_cost_line:
        try:
            (x1, y1), (x2, y2) = line
        except (TypeError, ValueError):
            continue
        dx, dy = x2-x1, y2-y1
        length = math.hypot(dx, dy)
        if length < .3 or not finite((x1, y1, x2, y2)):
            continue
        line_yaw = math.atan2(dy, dx)
        projection = ((start[0]-x1)*dx + (start[1]-y1)*dy)/(length*length)
        fractions = (.2, .5, .8, max(.15, min(.85, projection)))
        for t in fractions:
            for yaw in (line_yaw, wrap(line_yaw+math.pi)):
                pose = (x1+t*dx, y1+t*dy, yaw)
                if not grid.free(pose):
                    continue
                approach = math.hypot(pose[0]-start[0], pose[1]-start[1])
                if approach < .12:
                    continue
                heading = abs(wrap(yaw-start[2]))
                goal_forward = (goal[0]-pose[0])*math.cos(yaw) + (goal[1]-pose[1])*math.sin(yaw)
                score = approach + .35*heading + .5*max(0., -goal_forward)
                candidates.append((score, pose))
    candidates.sort(key=lambda item: item[0])
    result = []
    for _, pose in candidates:
        if all(math.hypot(pose[0]-other[0], pose[1]-other[1]) > .12
               or abs(wrap(pose[2]-other[2])) > .15 for other in result):
            result.append(pose)
        if len(result) >= limit:
            break
    return result


def line_stage_candidates(grid, start, goal, maneuver_mode='LATERAL'):
    """Body-clear entries 1.5 m from the start's closest point on each line."""
    candidates = []
    lateral = (goal[0]-start[0]) * -math.sin(start[2]) + (goal[1]-start[1]) * math.cos(start[2])
    if abs(lateral) < 1e-6:
        return candidates
    desired_side = (1 if lateral > 0 else -1) * (-1 if maneuver_mode == 'LATERAL' else 1)
    for number, line in enumerate(grid.zero_cost_line):
        try:
            (x1, y1), (x2, y2) = line
        except (TypeError, ValueError):
            continue
        if not finite((x1, y1, x2, y2)):
            continue
        dx, dy = x2-x1, y2-y1
        length = math.hypot(dx, dy)
        if length < .3:
            continue
        goal_t = max(0., min(1., ((goal[0]-x1)*dx+(goal[1]-y1)*dy)/(length*length)))
        distance = math.hypot(goal[0]-x1-goal_t*dx, goal[1]-y1-goal_t*dy)
        start_t = max(0., min(1., ((start[0]-x1)*dx+(start[1]-y1)*dy)/(length*length)))
        line_side = -math.sin(start[2])*dx/length + math.cos(start[2])*dy/length
        if abs(line_side) < 1e-6:
            continue
        entry_t = start_t + 1.5/length * desired_side * (1 if line_side > 0 else -1)
        # First-stage heading is defined in the map frame: face toward the
        # same map side as the target.
        map_right = goal[0] > start[0]
        heading = 0. if map_right else math.pi
        entry = (x1+entry_t*dx, y1+entry_t*dy, heading)
        if grid.free(entry):
            candidates.append((distance, number, abs(wrap(heading-start[2])), entry))
    candidates.sort(key=lambda item: (item[0], item[1], item[2]))
    return candidates


def line_retreat_target(line, goal, heading, distance=1.5):
    """Project the front-axle goal onto a line, then step toward the car's rear."""
    (x1, y1), (x2, y2) = line
    dx, dy = x2-x1, y2-y1
    length = math.hypot(dx, dy)
    if length < .3 or not finite((x1, y1, x2, y2, goal[0], goal[1], heading)):
        raise ValueError('invalid low-cost line or target')
    alignment = (dx*math.cos(heading) + dy*math.sin(heading))/length
    if abs(alignment) < .9:
        raise ValueError('line is not parallel to the first-stage heading')
    t = max(0., min(1., ((goal[0]-x1)*dx+(goal[1]-y1)*dy)/(length*length)))
    t -= distance/length * (1 if alignment > 0 else -1)
    return (x1+t*dx, y1+t*dy, heading)


def plan_line_approach(grid, start, goal, radius=1.3, max_seconds=5.,
                       cancel=lambda: False, goal_position_tolerance=.08,
                       goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL',
                       front_goal=None):
    """Choose a reachable entry on the closest feasible line to the goal."""
    deadline = time.time() + max_seconds
    line_goal = front_goal if front_goal is not None else goal
    for _, number, _, entry in line_stage_candidates(grid, start, line_goal, maneuver_mode):
        if cancel():
            raise RuntimeError('cancelled')
        remaining = deadline-time.time()
        if remaining < .3:
            break
        try:
            first = plan(grid, start, entry, radius=radius,
                         max_seconds=min(remaining*.45, 2.), cancel=cancel,
                         goal_position_tolerance=.06,
                         goal_heading_tolerance=math.radians(5),
                         start_reverse_only=maneuver_mode == 'LATERAL')
            if (maneuver_mode == 'LATERAL' and len(first) > 1
                    and first[1][3] >= 0):
                raise RuntimeError('lateral approach must start in reverse')
            return first, (number, entry)
        except (RuntimeError, ValueError):
            continue
    return None, None


def plan_prefer_lines(grid, start, goal, radius=1.3, max_seconds=5.,
                      cancel=lambda: False, goal_position_tolerance=.08,
                      goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL',
                      front_goal=None, return_line=False):
    """For special moves, join a feasible directed line before planning onward."""
    kwargs = dict(radius=radius, cancel=cancel,
                  goal_position_tolerance=goal_position_tolerance,
                  goal_heading_tolerance=goal_heading_tolerance,
                  maneuver_mode=maneuver_mode, front_goal=front_goal)
    # Staged low-cost-line routing is only for the two lateral maneuvers.
    special = maneuver_mode in ('LATERAL', 'LATERAL_TURN_180')
    if special and grid.zero_cost_line:
        approaches = []
        deadline = time.time() + max_seconds
        for entry in line_approaches(grid, start, goal):
            if cancel():
                raise RuntimeError('cancelled')
            remaining = deadline-time.time()
            if remaining <= .1:
                break
            try:
                first = plan(grid, start, entry, radius=radius,
                             max_seconds=min(2., remaining), cancel=cancel,
                             goal_position_tolerance=.08,
                             goal_heading_tolerance=math.radians(8))
            except (RuntimeError, ValueError):
                continue
            effort = 0.
            for a, b in zip(first, first[1:]):
                length = math.hypot(b[0]-a[0], b[1]-a[1])
                effort += length * (1.35 if b[3] < 0 else 1.)
                if a[3] and a[3] != b[3]:
                    effort += .35
            approaches.append((effort, first))
        approaches.sort(key=lambda item: item[0])
        deadline = time.time() + max_seconds
        for _, first in approaches:
            if cancel():
                raise RuntimeError('cancelled')
            remaining = deadline-time.time()
            if remaining <= .1:
                break
            try:
                second = plan(grid, first[-1][:3], goal,
                              max_seconds=remaining, **kwargs)
                route = first + second[1:]
                return (route, first[-1][:3]) if return_line else route
            except (RuntimeError, ValueError):
                continue
    route = plan(grid, start, goal, max_seconds=max_seconds, **kwargs)
    return (route, None) if return_line else route


def speed_profile(path, forward=.07, reverse=.045):
    """Plan speed caps from curvature, direction changes and goal distance."""
    if not path: return []
    caps=[0.0]*len(path)
    for i,p in enumerate(path):
        if i == 0 or i == len(path)-1:
            continue
        if p[3] != path[i-1][3] and path[i-1][3] != 0:
            continue
        base=forward if p[3] > 0 else reverse
        k=abs(p[4])
        caps[i]=base/(1.0+1.8*k)
        # Slow before the next cusp or final point, not after it.
        for j in range(i+1,min(len(path),i+8)):
            if path[j][3] != p[3]:
                caps[i]=min(caps[i],base*max(.35,(j-i)/8.0)); break
    for i in range(len(caps)-2,-1,-1):
        if caps[i] > 0 and caps[i+1] > 0:
            caps[i]=min(caps[i],caps[i+1]+.015)
    # The initial path anchor is not a physical cusp; allow the first
    # direction segment to start moving after WAIT_STOP verification.
    if len(caps) > 1 and caps[0] <= 1e-9:
        caps[0] = caps[1]
    return caps


def segments(path):
    out = []
    for p in path[1:]:
        if not out or p[3] != out[-1][-1][3]:
            anchor = path[0] if not out else out[-1][-1]
            out.append([(anchor[0],anchor[1],anchor[2],p[3],p[4])])
        out[-1].append(p)
    return out


def tracking(pose, segment, index, radius=1.3):
    """Pure pursuit on one signed segment only; cannot look across a cusp."""
    # Bounded progression prevents jumps between crossing sections of a path.
    upper = min(len(segment), index + 35)
    index = min(range(index, upper), key=lambda i: math.hypot(segment[i][0]-pose[0],segment[i][1]-pose[1]))
    target_i, length = index, 0.
    while target_i+1 < len(segment) and length < .22:
        a, b = segment[target_i], segment[target_i+1]
        length += math.hypot(b[0]-a[0], b[1]-a[1]); target_i += 1
    target = segment[target_i]
    dx, dy = target[0]-pose[0], target[1]-pose[1]
    lateral = -math.sin(pose[2])*dx + math.cos(pose[2])*dy
    curvature = 2*lateral/max(dx*dx+dy*dy, .01)
    curvature = max(-1./radius,min(1./radius,curvature))
    end = segment[-1]
    remaining = math.hypot(end[0]-pose[0],end[1]-pose[1])
    v = (min(.15, max(.035, remaining*.4)) if end[3] > 0 else -min(.08,max(.035,remaining*.4)))
    return v, curvature, index, remaining


class StopWindow(object):
    """Bounded pose jitter over a full second, not noisy per-tick derivatives."""
    def __init__(self):
        self.samples = []

    def update(self, now, pose):
        self.samples.append((now, pose))
        # Keep one sample at/before the window boundary.
        while len(self.samples) > 1 and self.samples[1][0] <= now - 1.0:
            self.samples.pop(0)
        if now - self.samples[0][0] < 1.0:
            return False
        poses = [p for _, p in self.samples]
        span = math.hypot(max(p[0] for p in poses)-min(p[0] for p in poses),
                          max(p[1] for p in poses)-min(p[1] for p in poses))
        angles = [wrap(p[2]-poses[0][2]) for p in poses]
        return span <= .02 and max(angles)-min(angles) <= .015
