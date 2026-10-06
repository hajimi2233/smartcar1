#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-independent rear-axle Hybrid A* and signed path tracking (Python 2/3)."""
from __future__ import division
import heapq
import math
import time
import bisect
from region_geometry import intersects
from nav_config import P, from_rear
from inspection_depth import blocked as depth_blocked, segment_blocked


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


def rear_target(front, wheelbase=None):
    wheelbase = P['wheelbase'] if wheelbase is None else wheelbase
    return (front[0] - wheelbase * math.cos(front[2]),
            front[1] - wheelbase * math.sin(front[2]), front[2])


def front_position(rear, wheelbase=None):
    wheelbase = P['wheelbase'] if wheelbase is None else wheelbase
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
        self.front, self.back, self.half_width = P['body_front'], P['body_rear'], P['collision_half_width']
        self.dynamic = set()
        self.static_rows = [[] for _ in range(height)]
        self.region = []
        self.depth_rules = []
        self.region_margin = P['region_margin']
        self.zero_cost_line = []
        self.zero_cost_width = .01
        self.cache = {}
        self.prefix = [0] * ((width+1)*(height+1))
        for j in range(height):
            row = 0
            for i in range(width):
                blocked = data[j*width+i] < 0 or data[j*width+i] >= 50
                row += int(blocked)
                if blocked:
                    self.static_rows[j].append(i)
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

    def static_occupied(self, i, j):
        return (i < 0 or j < 0 or i >= self.w or j >= self.h or
                self.data[j * self.w + i] < 0 or self.data[j * self.w + i] >= 50)

    def known_static_near(self, i, j, radius_cells=1):
        for row in range(max(0, j-radius_cells), min(self.h, j+radius_cells+1)):
            xs = self.static_rows[row]
            k = bisect.bisect_left(xs, i-radius_cells)
            for x in xs[k:bisect.bisect_right(xs, i+radius_cells)]:
                if self.data[row*self.w+x] >= 50:
                    return True
        return False

    def set_dynamic(self, cells):
        self.dynamic = set(cells)

    def region_blocked(self, pose, extra=0.):
        if not self.region:
            return False
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        m = self.region_margin + extra
        # Body and front tire envelopes separately: the wide steering envelope
        # must not widen the entire rear body. Tire disks enclose all angles.
        rectangles = [(-P['body_rear']-m, P['body_front']+m, -P['body_half_width']-m, P['body_half_width']+m)]
        radius = math.hypot(P['wheel_radius'], P['tire_half_width'])
        for side in (-P['front_track']/2., P['front_track']/2.):
            rectangles.append((P['wheelbase']-radius-m, P['wheelbase']+radius+m,
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
        a, b = P['boundary_half_length'] + m, self.half_width + m
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
        front = front_position(pose)
        rule = depth_blocked(front, self.depth_rules, extra)
        if rule:
            hits['INSPECTION_DEPTH_'+rule] = front
            if not detailed: return hits
        if self.region_blocked(pose, extra):
            hits['REGION'] = None
            if not detailed: return hits
        x, y, yaw = self.local(pose)
        c, s = math.cos(yaw), math.sin(yaw)
        # Half a cell diagonal and sampling sweep bound added conservatively.
        m = extra + self.margin + self.res * .707107 + self.res * .5
        center_x, center_y = x + P['collision_center_x'] * c, y + P['collision_center_x'] * s
        a, b = P['collision_half_length'] + m, self.half_width + m
        ex, ey = abs(c) * a + abs(s) * b, abs(s) * a + abs(c) * b
        imin, imax = int(math.floor((center_x-ex)/self.res)), int(math.floor((center_x+ex)/self.res))
        jmin, jmax = int(math.floor((center_y-ey)/self.res)), int(math.floor((center_y+ey)/self.res))
        if imin < 0 or jmin < 0 or imax >= self.w or jmax >= self.h:
            hits['MAP_BOUNDS'] = None
            return hits
        stride = self.w+1
        count = (self.prefix[(jmax+1)*stride+imax+1] - self.prefix[jmin*stride+imax+1]
                 - self.prefix[(jmax+1)*stride+imin] + self.prefix[jmin*stride+imin])
        dynamic_rows = {}
        for i, j in self.dynamic:
            if imin <= i <= imax and jmin <= j <= jmax:
                dynamic_rows.setdefault(j, []).append(i)
        if count == 0 and not dynamic_rows:
            return hits
        for j in range(jmin, jmax+1):
            xs = self.static_rows[j]
            row = xs[bisect.bisect_left(xs, imin):bisect.bisect_right(xs, imax)] if count else []
            if j in dynamic_rows:
                row = sorted(set(row).union(dynamic_rows[j]))
            for i in row:
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
        if self.depth_rules:
            previous = front_position(pose)
            # A bicycle arc's front axle follows a circle; cover chord sagitta.
            theta = abs(distance*curvature/n)
            sagitta = (math.hypot(1./curvature, P['wheelbase'])*(1-math.cos(theta/2))
                        if abs(curvature)>1e-9 else 0.)
            for p in pts:
                current = front_position(p)
                if segment_blocked(previous, current, self.depth_rules, sagitta): return None
                previous = current
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


def plan(grid, start, goal, radius=1.3, max_seconds=12.0, cancel=lambda: False, goal_position_tolerance=.08, goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL', front_goal=None, start_reverse_only=False, goal_region=False, max_direction_changes=None):
    """Returns [(x,y,yaw,direction,curvature), ...], no endpoint snapping.

    Weighted Hybrid A*: finite budget, non-optimal; fails if search cannot find
    a valid path. Never shrinks vehicle, shifts goal, or permits pivot turns.
    """
    if (radius < P['global_min_radius'] or not finite([goal_position_tolerance, goal_heading_tolerance])
            or goal_position_tolerance <= 0 or goal_heading_tolerance <= 0
            or not finite(start+goal) or not grid.free(start) or (not goal_region and not grid.free(goal))):
        raise ValueError('start/goal collision, nonfinite pose, or radius below configured global minimum')
    started = time.time()
    if max_direction_changes is not None and (isinstance(max_direction_changes, bool)
            or not isinstance(max_direction_changes, int) or max_direction_changes < 0):
        raise ValueError('max_direction_changes must be a nonnegative integer or None')
    def key(p, d, changes=0):
        base = (int(round(p[0]/.08)), int(round(p[1]/.08)),
                int(round(wrap(p[2])/(math.pi/36))) % 72, d)
        # A route with remaining gear changes must not be pruned by a route
        # reaching the same pose after consuming that allowance.
        return base if max_direction_changes is None else base+(changes,)
    def goal_error(p):
        xy = from_rear(p, 'goal')[:2] if front_goal is not None else p[:2]
        target = front_goal if front_goal is not None else goal
        return math.hypot(xy[0]-target[0], xy[1]-target[1]), abs(wrap(p[2]-target[2]))
    def heuristic(p):
        distance, angle = goal_error(p)
        return max(max(0., distance-goal_position_tolerance),
                   radius*max(0., angle-goal_heading_tolerance))
    # Immutable parent records: updating a discretized state's score must never
    # mutate a previously generated child's geometric ancestor.
    records = [(start, 0, 0., None, [])]
    direction_changes = [0]
    queue = [(heuristic(start), 0)]
    best = {key(start, 0): 0.}
    last_yield = time.time()
    while queue:
        if cancel():
            raise RuntimeError('cancelled')
        if time.time()-started > max_seconds or len(records) > P['planner_max_records']:
            raise RuntimeError('planning budget exceeded; choose a roomier approach point')
        _, idx = heapq.heappop(queue)
        # Planning is intentionally bounded, but it must not monopolize the
        # interpreter while ROS sensor callbacks deliver fresh scans.
        if time.time()-last_yield >= .01:
            time.sleep(.001)
            last_yield = time.time()
        p, previous_sign, cost, parent, _ = records[idx]
        if cost > best.get(key(p, previous_sign, direction_changes[idx]), float('inf')) + 1e-8:
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
                if goal_region and abs(along)>goal_position_tolerance*.75:
                    lengths.append(abs(along)-goal_position_tolerance*.75)
            for length in lengths:
                for curvature in (0., -1./radius, -.5/radius, .5/radius, 1./radius):
                    endpoint = advance(p, direction*length, curvature)
                    switching = previous_sign and previous_sign != direction
                    changes = direction_changes[idx] + (1 if switching else 0)
                    if max_direction_changes is not None and changes > max_direction_changes:
                        continue
                    switch = P['planner_switch_cost'] if switching else 0.
                    last_k = records[idx][4][-1][4] if records[idx][4] else 0.
                    steer_change = abs(math.atan(P['wheelbase']*curvature)-math.atan(P['wheelbase']*last_k))
                    if switching:
                        run, ancestor = 0., idx
                        while ancestor is not None and records[ancestor][1] == previous_sign and run < .3:
                            record = records[ancestor]
                            prev = records[record[3]][0] if record[3] is not None else record[0]
                            for sample in record[4]:
                                run += math.hypot(sample[0]-prev[0],sample[1]-prev[1]); prev=sample
                            ancestor = record[3]
                        switch += P['planner_short_segment_cost']*max(0.,1.-run/.3)
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
                    newcost = cost + length*reverse + switch + maneuver_bonus + P['planner_steer_cost']*steer_change
                    k = key(endpoint, direction, changes)
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
                    # Failed tracking locations are soft, episode-local preferences.
                    # They never turn a necessary narrow passage into a hard obstacle.
                    for fx,fy in getattr(grid, 'failed_tracking', []):
                        newcost += length * .8 * max(0.,1.-math.hypot(endpoint[0]-fx,endpoint[1]-fy)/.35)
                    newcost -= .70 * length * grid.line_bonus(endpoint)
                    if newcost >= best.get(k, float('inf')):
                        continue
                    best[k] = newcost
                    segment = [(q[0],q[1],q[2],direction,curvature) for q in arc]
                    records.append((endpoint,direction,newcost,idx,segment))
                    direction_changes.append(changes)
                    heapq.heappush(queue,(newcost+1.6*heuristic(endpoint),len(records)-1))
    raise RuntimeError('no path found')


def gear_route_effort(path):
    """Whole-route gear changes first; estimated travel time breaks ties."""
    return (entry_route_effort(path)[0], route_seconds(path))


def improve_gear_route(grid, start, goal, incumbent, radius, max_seconds,
                       cancel, position_tolerance, heading_tolerance,
                       mode='NORMAL', front_goal=None, start_reverse_only=False,
                       goal_region=False):
    """Keep a feasible incumbent through every unsuccessful optimization attempt."""
    deadline = time.time()+max_seconds
    best = incumbent
    initial_changes = gear_route_effort(best)[0]
    for limit in range(initial_changes):
        if cancel():
            raise RuntimeError('cancelled')
        if limit >= gear_route_effort(best)[0]:
            break
        remaining = deadline-time.time()
        if remaining < .1:
            break
        try:
            candidate = plan(grid, start, goal, radius,
                             remaining/(initial_changes-limit), cancel,
                             position_tolerance, heading_tolerance, mode,
                             front_goal, start_reverse_only=start_reverse_only,
                             goal_region=goal_region, max_direction_changes=limit)
            if gear_route_effort(candidate) < gear_route_effort(best):
                best = candidate
        except (ValueError, RuntimeError):
            pass  # A failed optimization must not discard the feasible route.
    if cancel():
        raise RuntimeError('cancelled')
    return best


def plan_fewer_changes(grid, start, goal, radius=1.3, max_seconds=12.,
                       cancel=lambda:False, goal_position_tolerance=.08,
                       goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL',
                       front_goal=None, start_reverse_only=False, goal_region=False):
    deadline=time.time()+max_seconds
    path=plan(grid,start,goal,radius,max_seconds,cancel,goal_position_tolerance,
              goal_heading_tolerance,maneuver_mode,front_goal,
              start_reverse_only=start_reverse_only,goal_region=goal_region)
    return improve_gear_route(grid,start,goal,path,radius,max(0.,deadline-time.time()),
                              cancel,goal_position_tolerance,goal_heading_tolerance,
                              maneuver_mode,front_goal,start_reverse_only,goal_region)


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


def nearest_line_number(lines, goal):
    """Select by final goal-reference position before testing feasibility."""
    x, y = goal[:2]
    ranked = []
    for number, line in enumerate(lines):
        (x1, y1), (x2, y2) = line
        dx, dy = x2-x1, y2-y1
        length2 = dx*dx+dy*dy
        if not finite((x1,y1,x2,y2)) or length2 < 1e-6:
            raise ValueError('invalid low-cost line')
        t = max(0., min(1., ((x-x1)*dx+(y-y1)*dy)/length2))
        ranked.append((math.hypot(x-x1-t*dx, y-y1-t*dy), number))
    if not ranked:
        raise RuntimeError('draw a low-cost line before a lateral maneuver')
    return min(ranked)[1]


def line_stage_candidates(grid, start, goal, maneuver_mode='LATERAL', offsets=(0.,),
                          rear_offsets=False):
    """Only the nearest line may supply an entry, even if it is blocked."""
    candidates = []
    number = nearest_line_number(grid.zero_cost_line, goal)
    lateral = (goal[0]-start[0]) * -math.sin(start[2]) + (goal[1]-start[1]) * math.cos(start[2])
    if abs(lateral) < 1e-6:
        return candidates
    desired_side = (1 if lateral > 0 else -1) * (-1 if maneuver_mode == 'LATERAL' else 1)
    for number, line in [(number, grid.zero_cost_line[number])]:
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
        entry_t = start_t + P['line_entry_distance']/length * desired_side * (1 if line_side > 0 else -1)
        # First-stage heading is defined in the map frame: face toward the
        # same map side as the target.
        map_right = goal[0] > start[0]
        heading = 0. if map_right else math.pi
        entry = (x1+entry_t*dx, y1+entry_t*dy, heading)
        alignment = (dx*math.cos(heading)+dy*math.sin(heading))/length
        rear_sign = -1. if alignment >= 0. else 1.
        for shift in offsets:
            if rear_offsets:
                shift *= rear_sign
            candidate=(entry[0]+shift*dx/length,entry[1]+shift*dy/length,heading)
            if grid.free(candidate):
                candidates.append((distance, number, abs(wrap(heading-start[2])), candidate))
    candidates.sort(key=lambda item: (item[0], item[1], item[2]))
    return candidates


def line_retreat_target(line, goal, heading, distance=None):
    """Project the front-axle goal onto a line, then step toward the car's rear."""
    distance = P['line_retreat_distance'] if distance is None else distance
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
    """Accept the first cusp-free entry; otherwise compare within one budget."""
    deadline = time.time() + max_seconds
    line_goal = front_goal if front_goal is not None else goal
    candidates = line_stage_candidates(
        grid, start, line_goal, maneuver_mode,
        offsets=tuple(P['line_entry_search_span']*scale
                      for scale in (1./3., 2./3., 1.)), rear_offsets=True)
    best = None
    for index, (_, number, _, entry) in enumerate(candidates):
        if cancel():
            raise RuntimeError('cancelled')
        remaining = deadline-time.time()
        if remaining < .3:
            break
        try:
            first = plan(grid, start, entry, radius=radius,
                         max_seconds=remaining/(len(candidates)-index), cancel=cancel,
                         goal_position_tolerance=P['stage_plan_position'],
                         goal_heading_tolerance=math.radians(P['stage_plan_heading_deg']),
                         start_reverse_only=maneuver_mode == 'LATERAL')
            if (maneuver_mode == 'LATERAL' and len(first) > 1
                    and first[1][3] >= 0):
                raise RuntimeError('lateral approach must start in reverse')
            cost = gear_route_effort(first)
            if best is None or cost < best[0]:
                best = (cost, first, (number, entry))
            # A continuous forward/reverse route already meets the entry goal.
            # Stop searching, but keep the cancellation check below the loop.
            if cost[0] == 0:
                break
        except (RuntimeError, ValueError):
            continue
    if cancel():
        raise RuntimeError('cancelled')
    if best is not None:
        improved = improve_gear_route(
            grid, start, best[2][1], best[1], radius, max(0.,deadline-time.time()),
            cancel, P['stage_plan_position'], math.radians(P['stage_plan_heading_deg']),
            start_reverse_only=maneuver_mode == 'LATERAL')
        return improved, best[2]
    return None, None



def entry_route_effort(path):
    """Prefer fewer gear changes, then trade travel time for quieter steering.

    This ranks completed entry candidates only; safety and the planner's
    feasibility search are unchanged. It is not a global optimality proof.
    """
    switches = 0
    previous = 0
    last_delta = 0.
    steering_travel = 0.
    for p in path[1:]:
        direction = p[3]
        if direction and previous and direction != previous:
            switches += 1
        if direction:
            previous = direction
        delta = math.atan(P['wheelbase']*p[4])
        steering_travel += abs(delta-last_delta)
        last_delta = delta
    steering_travel += abs(last_delta)  # straighten at the entry stop
    return (switches, route_seconds(path) +
            P['line_entry_steer_seconds']*steering_travel)


def route_seconds(path):
    """Comparable travel estimate including curvature, steering and cusp stops."""
    total=0.;previous=0;last_delta=0.
    for a,b in zip(path,path[1:]):
        speed=P['forward_speed'] if b[3]>0 else P['reverse_speed']
        total+=math.hypot(b[0]-a[0],b[1]-a[1])*(1.+1.8*abs(b[4]))/speed
        delta=math.atan(P['wheelbase']*b[4])
        total+=abs(delta-last_delta) # nominal 1 rad/s steering
        if previous and previous!=b[3]:total+=P['stop_window']+speed/.5
        previous=b[3];last_delta=delta
    return total


def suffix_route_effort(second, third):
    """Count every actual gear change, including the join at point 2."""
    return gear_route_effort(second + third[1:])


def plan_line_suffix(grid, start, goal, line, heading, radius, max_seconds,
                     cancel, position_tolerance, heading_tolerance, mode,
                     front_goal, goal_region=False):
    """Plan entry -> retreat -> goal atomically on the already locked line.

    Each final leg starts at the actual planned retreat endpoint. Only complete
    candidates prefer fewer internal direction changes and less steering;
    no partial-route fallback.
    """
    deadline = time.time() + max_seconds
    best = None
    shifts = tuple(P['line_search_span']*scale for scale in (1./3., 2./3., 1.))
    for i, shift in enumerate(shifts):
        if cancel():
            raise RuntimeError('cancelled')
        remaining = deadline - time.time()
        if remaining < .1:
            break
        # Share time among candidates, reserving most of each share for the
        # harder final approach. Unused time carries to later candidates.
        allowance = remaining / (len(shifts)-i)
        retreat = line_retreat_target(line, front_goal, heading,
                                     max(.3, P['line_retreat_distance']+shift))
        try:
            second = plan(grid, start, retreat, radius, allowance*.4, cancel,
                          P['stage_plan_position'],
                          math.radians(P['stage_plan_heading_deg']))
            remaining = deadline - time.time()
            if remaining < .1:
                break
            third = plan(grid, second[-1][:3], goal, radius,
                         min(remaining, allowance*.6), cancel,
                         position_tolerance, heading_tolerance, mode, front_goal,
                         goal_region=goal_region)
            route = second + third[1:]
            cost = suffix_route_effort(second, third)
            # A normal change of direction at point 2 is allowed. Neither
            # individual leg may need extra shunting for early acceptance.
            if entry_route_effort(second)[0] == 0 and entry_route_effort(third)[0] == 0:
                best = (cost, route, retreat, second, third)
                break
            if best is None or cost < best[0]:
                best = (cost, route, retreat, second, third)
        except (ValueError, RuntimeError):
            continue
    if cancel():
        raise RuntimeError('cancelled')
    if best is None:
        raise RuntimeError('line suffix search budget exhausted; no complete retreat-to-goal route')
    second, third = best[3], best[4]
    improved = improve_gear_route(
        grid, second[-1][:3], goal, third, radius, max(0.,deadline-time.time()),
        cancel, position_tolerance, heading_tolerance, mode, front_goal,
        goal_region=goal_region)
    route = second + improved[1:]
    return (route if gear_route_effort(route) < best[0] else best[1]), best[2]


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
        while len(self.samples) > 1 and self.samples[1][0] <= now - P['stop_window']:
            self.samples.pop(0)
        if now - self.samples[0][0] < P['stop_window']:
            return False
        poses = [p for _, p in self.samples]
        span = math.hypot(max(p[0] for p in poses)-min(p[0] for p in poses),
                          max(p[1] for p in poses)-min(p[1] for p in poses))
        angles = [wrap(p[2]-poses[0][2]) for p in poses]
        return span <= P['stop_position_span'] and max(angles)-min(angles) <= P['stop_heading_span']
