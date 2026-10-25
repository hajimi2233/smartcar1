#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-independent rear-axle Hybrid A* and signed path tracking (Python 2/3)."""
from __future__ import division
import heapq
import math
import time
import bisect
import json
import numpy as np
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


def load_turn90_trace(path):
    """Load a recorded rear-axle turn as a checked planning candidate."""
    if not path:
        return None
    with open(path) as stream:
        data = json.load(stream)
    samples = []
    last_direction, last_curvature = 0, 0.
    for sample in data.get('samples', []):
        command = sample.get('command', {})
        speed = float(command.get('linear_x', 0.))
        if abs(speed) < .02 and not last_direction:
            continue
        pose = sample.get('local_rear_pose')
        if not pose or len(pose) < 3:
            continue
        if abs(speed) >= .02:
            last_direction = 1 if speed > 0. else -1
            last_curvature = float(command.get('angular_z', 0.)) / speed
        curvature = last_curvature
        if not finite(list(pose[:3]) + [speed, curvature]):
            raise ValueError('nonfinite turn90 sample')
        if samples and math.hypot(pose[0]-samples[-1][0], pose[1]-samples[-1][1]) < .00005:
            continue
        samples.append((float(pose[0]), float(pose[1]), float(pose[2]),
                        last_direction, curvature))
    if len(samples) < 2:
        raise ValueError('turn90 primitive has fewer than two moving samples')
    return dict(samples=samples, heading=float(data.get('heading_change_deg', 0.)),
                source=path, radius=float(data.get('radius', 1.3)))


def transform_turn90_trace(trace, start, desired_sign):
    """Transform a recorded trace to ``start`` and mirror it if necessary."""
    source_sign = 1. if trace['heading'] >= 0. else -1.
    mirror = source_sign != (1. if desired_sign >= 0 else -1.)
    c, s = math.cos(start[2]), math.sin(start[2])
    out = [(start[0], start[1], start[2], 0, 0.)]
    for x, y, yaw, direction, curvature in trace['samples']:
        if mirror:
            y, yaw, curvature = -y, -yaw, -curvature
        out.append((start[0] + c*x - s*y, start[1] + s*x + c*y,
                    wrap(start[2] + yaw), direction, curvature))
    return out


def checked_straight(grid, start, distance, cancel=lambda: False):
    """A signed straight segment, including its anchor and swept checks."""
    if cancel():
        raise RuntimeError('cancelled')
    if not grid.free(start):
        raise ValueError('straight start blocked')
    path = [(start[0], start[1], start[2], 0, 0.)]
    if abs(distance) < 1e-8:
        return path
    points = grid.arc(start, distance, 0.)
    if points is None:
        raise ValueError('straight swept path blocked')
    sign = 1 if distance > 0 else -1
    return path + [(p[0], p[1], p[2], sign, 0.) for p in points]


def checked_trace(grid, path, cancel=lambda: False):
    """Densify measured edges, including braking, and check swept geometry."""
    result = [path[0]]
    reach = math.hypot(P['body_front'], P['body_half_width']) + .2
    step = min(.01, grid.res/4.)
    for a, b in zip(path, path[1:]):
        if cancel():
            raise RuntimeError('cancelled')
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        angle = wrap(b[2]-a[2])
        n = max(1, int(math.ceil((length+reach*abs(angle))/step)))
        for i in range(1, n+1):
            t = float(i)/n
            p = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                 wrap(a[2]+t*angle), b[3], b[4])
            if (not grid.free(p[:3]) or grid.collision(p[:3], extra=step)
                    or segment_blocked(front_position(result[-1]), front_position(p), grid.depth_rules, step)):
                raise ValueError('primitive swept path blocked')
            result.append(p)
    return result


def separated_line_turn(grid, start, goal, trace, heading_tolerance,
                        front_goal=None, pre_extension=.30,
                        post_extension=.30, cancel=lambda: False):
    """Solve straight -> measured turn -> straight; no offset enumeration.

    The two signed straight lengths are solved from the template displacement
    and endpoint headings. The measured turn remains rigid, including cusps.
    """
    delta = wrap(goal[2]-start[2])
    if not math.radians(60) <= abs(delta) <= math.radians(120):
        raise ValueError('heading not compatible with a 90-degree primitive')
    moving_direction = trace['samples'][0][3]
    lead = advance((0., 0., 0.), moving_direction * pre_extension, 0.)
    template = transform_turn90_trace(trace, lead, 1 if delta>0 else -1)
    template.insert(0, (0., 0., 0., moving_direction, 0.))
    exit_direction = template[-1][3]
    tail = advance(template[-1][:3], exit_direction * post_extension, 0.)
    template.append((tail[0], tail[1], tail[2], exit_direction, 0.))
    tx, ty, angle = template[-1][:3]
    end_yaw = wrap(start[2]+angle)
    if abs(wrap(end_yaw-goal[2])) > heading_tolerance:
        raise ValueError('measured primitive heading outside goal tolerance')
    target = goal
    if front_goal is not None:
        offset = from_rear((0.,0.,end_yaw), 'goal')
        target = (front_goal[0]-offset[0], front_goal[1]-offset[1], end_yaw)
    c, s = math.cos(start[2]), math.sin(start[2])
    dx, dy = target[0]-start[0], target[1]-start[1]
    x, y = c*dx+s*dy, -s*dx+c*dy
    after = (y-ty)/math.sin(angle)
    before = x-tx-after*math.cos(angle)
    first = checked_straight(grid, start, before, cancel)
    entry = first[-1][:3]
    turn = checked_trace(grid, transform_turn90_trace(trace, entry, 1 if delta>0 else -1), cancel)
    last = checked_straight(grid, turn[-1][:3], after, cancel)
    return first+turn[1:]+last[1:], entry, before, after


def plan_turn90_trace(grid, start, goal, trace, desired_sign, max_goal_gap=.65,
                      pre_extension=.30, post_extension=.30,
                      cancel=lambda: False):
    """Use the measured turn, then let the normal planner correct the endpoint."""
    if pre_extension < 0. or post_extension < 0.:
        raise ValueError('turn90 straight extensions must be nonnegative')
    moving_direction = trace['samples'][0][3]
    lead = advance(start, moving_direction * pre_extension, 0.)
    path = transform_turn90_trace(trace, lead, desired_sign)
    # The transformed trace starts at ``lead``.  Prepend the straight lead-in
    # from the actual vehicle pose, then extend the exit along its final gear.
    path.insert(0, (start[0], start[1], start[2], moving_direction, 0.))
    exit_direction = path[-1][3]
    tail = advance(path[-1][:3], exit_direction * post_extension, 0.)
    path.append((tail[0], tail[1], tail[2], exit_direction, 0.))
    if len(path) < 3 or math.hypot(path[-1][0]-goal[0], path[-1][1]-goal[1]) > max_goal_gap:
        return None
    previous = path[0]
    for point in path[1:]:
        if cancel() or not grid.free(point[:3]):
            return None
        previous = point
    try:
        suffix = plan(grid, path[-1][:3], goal, radius=max(1.3, trace['radius']),
                      max_seconds=2.0, cancel=cancel, goal_position_tolerance=.08,
                      goal_heading_tolerance=math.radians(12.), maneuver_mode='NORMAL')
    except Exception:
        return None
    return path + suffix[1:]


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
        self.live_points = None
        self.static_rows = [[] for _ in range(height)]
        self.region = []
        self.wheel_collision_enabled = True
        self.depth_rules = []
        self.region_margin = P['region_margin']
        self.zero_cost_line = []
        self.zero_cost_width = .01
        # Optional world-coordinate search window used by Hybrid A*.
        # None keeps the historical full-map behaviour.
        self.search_bounds = None
        # The navigator sets this to NORMAL when a recovery/replan must use
        # the full map, even if the original maneuver was special.
        self.search_mode = None
        self.cache = {}
        # Built once per map revision, shared read-only by planning snapshots.
        self.data = tuple(data)
        array=np.asarray(self.data,dtype=np.int16)
        blocked = ((array < 0) | (array >= 50)).reshape(height,width)
        self.static_rows = tuple(tuple(np.flatnonzero(row).tolist()) for row in blocked)
        self.prefix = tuple(np.pad(blocked.astype(np.int32).cumsum(0).cumsum(1),
                             ((1,0),(1,0)),mode='constant').reshape(-1).tolist())

    def snapshot(self):
        clone=object.__new__(Grid);clone.__dict__=self.__dict__.copy()
        clone.depth_rules=list(self.depth_rules);clone.region=list(self.region)
        clone.zero_cost_line=list(self.zero_cost_line);clone.cache={}
        if hasattr(self,'failed_tracking'):clone.failed_tracking=list(self.failed_tracking)
        return clone

    @property
    def dynamic(self):
        # Prevent mutations that could leave the row index stale.
        return self._dynamic

    @dynamic.setter
    def dynamic(self,cells):
        self._dynamic=frozenset(cells);rows={}
        for i,j in self._dynamic:rows.setdefault(j,[]).append(i)
        self.dynamic_rows={j:tuple(sorted(xs)) for j,xs in rows.items()}
        self.dynamic_row_keys=tuple(sorted(rows))

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

    def set_dynamic(self, cells, points=None):
        checked=None if points is None else tuple(tuple(p) for p in points)
        if checked is not None and any(len(p)!=2 or not finite(p) for p in checked):
            raise ValueError('invalid confirmed live obstacle point')
        self.dynamic = set(cells)
        # None keeps the historical conservative grid interface for planners.
        # A tuple owns a snapshot of confirmed, unquantized world-frame returns.
        self.live_points = checked

    def live_collision(self, pose, extra=0.):
        if self.live_points is None or not self.live_points:return {}
        c,s=math.cos(pose[2]),math.sin(pose[2])
        padding=self.margin+extra+P['live_scan_uncertainty']
        a=P['collision_half_length']+padding;b=self.half_width+padding
        for point in self.live_points:
            dx,dy=point[0]-pose[0],point[1]-pose[1]
            if (abs(c*dx+s*dy-P['collision_center_x'])<=a
                    and abs(-s*dx+c*dy)<=b):
                return {'LIVE_SCAN':point}
        return {}

    def region_blocked(self, pose, extra=0.):
        if not self.region:
            return False
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        m = self.region_margin + extra
        # Body and front tire envelopes separately: the wide steering envelope
        # must not widen the entire rear body. Tire disks enclose all angles.
        rectangles = [(-P['body_rear']-m, P['body_front']+m, -P['body_half_width']-m, P['body_half_width']+m)]
        if self.wheel_collision_enabled:
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
        if self.search_bounds is not None:
            xmin, xmax, ymin, ymax = self.search_bounds
            if (pose[0] < xmin or pose[0] > xmax or
                    pose[1] < ymin or pose[1] > ymax):
                return False
        return not self.collision(pose)

    def set_search_bounds(self, start, goal, margin):
        """Restrict planning states to an expanded start/goal window.

        The margin is applied around both endpoints, so a long route keeps a
        corridor-like rectangle rather than a small circle around the car.
        Collision geometry remains unchanged; only state expansion is bounded.
        """
        if margin is None or margin <= 0.:
            self.search_bounds = None
            return
        margin = float(margin)
        self.search_bounds = (min(start[0], goal[0])-margin,
                              max(start[0], goal[0])+margin,
                              min(start[1], goal[1])-margin,
                              max(start[1], goal[1])+margin)

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
        precise_live = self.live_points is not None
        if precise_live:
            hits.update(self.live_collision(pose,extra))
            if hits and not detailed:return hits
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
        for j in (() if precise_live else self.dynamic_row_keys[bisect.bisect_left(self.dynamic_row_keys,jmin):bisect.bisect_right(self.dynamic_row_keys,jmax)]):
            xs=self.dynamic_rows.get(j)
            if xs is not None:
                selected=xs[bisect.bisect_left(xs,imin):bisect.bisect_right(xs,imax)]
                if len(selected):dynamic_rows[j]=selected
        if count == 0 and not dynamic_rows:
            return hits
        for j in range(jmin, jmax+1):
            xs = self.static_rows[j]
            row = xs[bisect.bisect_left(xs, imin):bisect.bisect_right(xs, imax)] if count else []
            if j in dynamic_rows:
                row = sorted(set(row).union(dynamic_rows[j]))
            if len(row)>=32:
                indices=np.asarray(row)
                dx=(indices+.5)*self.res-center_x;dy=(j+.5)*self.res-center_y
                inside=(np.abs(c*dx+s*dy)<=a)&(np.abs(-s*dx+c*dy)<=b)
                row=indices[inside]
            for i in row:
                dx, dy = (i+.5)*self.res-center_x, (j+.5)*self.res-center_y
                if abs(c*dx+s*dy) <= a and abs(-s*dx+c*dy) <= b:
                    if (getattr(self,'fast_turn90_precise_static_collision',False)
                            and (precise_live or (i,j) not in self.dynamic)):
                        from fast_turn90 import static_cell_overlaps
                        # Keep physical dimensions, configured margin, extra
                        # clearance and half-cell sampling padding unchanged.
                        # Replace isotropic cell-diagonal inflation by exact
                        # rectangle/cell intersection; dynamic cells stay on
                        # the original conservative branch.
                        cell_pad=self.res*.707107
                        # Stationary static footprint has no motion padding.
                        # Grid.arc adds an interval-specific bound via extra.
                        static_pad=self.res*.5 if getattr(self,'fast_turn90_separate_sweep',False) else 0.
                        if not static_cell_overlaps(dx,dy,c,s,a-cell_pad-static_pad,b-cell_pad-static_pad,self.res*.5):
                            continue
                    cell = ((i+.5)*self.res, (j+.5)*self.res)
                    co, so = math.cos(self.origin[2]), math.sin(self.origin[2])
                    point = (self.origin[0]+co*cell[0]-so*cell[1],
                             self.origin[1]+so*cell[0]+co*cell[1])
                    value = self.data[j*self.w+i]
                    if value < 0: hits.setdefault('MAP_UNKNOWN', point)
                    elif value >= 50: hits.setdefault('STATIC_MAP', point)
                    if not precise_live and (i,j) in self.dynamic: hits.setdefault('LIVE_SCAN', point)
                    if not detailed: return hits
        return hits

    def motion_padding(self, distance, curvature):
        """Half-interval bound on every padded body point's displacement.

        Translation speed is one per rear-axle travel; rotation adds reach*k.
        Midpoint checks with this padding cover the complete interval, including
        its endpoints. The reach keeps the original conservative dimensions.
        """
        m=max(self.margin,self.region_margin)+self.res*1.207107
        reach=max(math.hypot(abs(P['collision_center_x'])+P['collision_half_length']+m,
                             self.half_width+m),
                  math.hypot(max(self.front,self.back)+m,P['body_half_width']+m),
                  math.hypot(P['wheelbase']+m, P['front_track']/2.+m))
        if self.wheel_collision_enabled:
            tire=math.hypot(P['wheel_radius'],P['tire_half_width'])
            reach=max(reach,math.hypot(P['wheelbase']+tire+m,P['front_track']/2.+tire+m))
        return abs(distance)*.5*(1.+reach*abs(curvature))

    def arc(self, pose, distance, curvature):
        # Maximum body-point speed / rear axle speed < 1.7 at radius >= 1.3.
        n = max(1, int(math.ceil(abs(distance) / min(.025, self.res / 4))))
        pts = [advance(pose, distance*i/n, curvature) for i in range(1, n+1)]
        # A whole swept footprint proven clear can skip individual pose queries.
        # All original path samples and near-wall checking remain unchanged.
        if not self.depth_rules and not self.region:
            m=self.margin+self.res*1.207107
            reach=math.hypot(abs(P['collision_center_x'])+P['collision_half_length']+m,
                             self.half_width+m)
            bound=abs(distance)*(1.+reach*abs(curvature))
            if not self.collision(pose,extra=bound):return pts
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
        if (self.live_points is not None or getattr(self,'fast_turn90_separate_sweep',False)
                and getattr(self,'fast_turn90_precise_static_collision',False)):
            if not self.free(pose):return None
            if self.search_bounds is not None:
                xmin,xmax,ymin,ymax=self.search_bounds
                if any(not (xmin<=p[0]<=xmax and ymin<=p[1]<=ymax) for p in pts):return None
            interval_padding=self.motion_padding(distance/n,curvature)
            for i in range(n):
                middle=advance(pose,distance*(i+.5)/n,curvature)
                if self.collision(middle,extra=interval_padding):return None
            return pts
        return pts if all(self.free(p) for p in pts) else None

    def blocked_detail(self, pose, distance=0., curvature=0.):
        n = max(1, int(math.ceil(abs(distance) / min(.025, self.res / 4))))
        indices = range(1, n+1) if distance else [0]
        for i in indices:
            diagnostic_padding=0.
            separated=(distance and getattr(self,'fast_turn90_separate_sweep',False)
                       and getattr(self,'fast_turn90_precise_static_collision',False))
            if separated:
                travel=distance*(i-.5)/n
                diagnostic_padding=self.motion_padding(distance/n,curvature)
            else:
                travel=distance*i/n
            p = advance(pose, travel, curvature)
            hits = self.collision(p, detailed=True, extra=diagnostic_padding)
            if hits:
                sources = ', '.join(name + ('@(%.3f,%.3f)' % point if point is not None else '')
                                    for name,point in sorted(hits.items()))
                return ('sources=[%s]; travel=%.3fm; rear_pose=(%.3f,%.3f,%.1fdeg); frame=map'
                        % (sources, travel, p[0], p[1], math.degrees(p[2])))
        return 'no collision in checked samples'


def plan(grid, start, goal, radius=1.25, max_seconds=12.0, cancel=lambda: False, goal_position_tolerance=.08, goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL', front_goal=None, start_reverse_only=False, goal_region=False, max_direction_changes=None, initial_direction=0):
    """Returns [(x,y,yaw,direction,curvature), ...], no endpoint snapping.

    Weighted Hybrid A*: finite budget, non-optimal; fails if search cannot find
    a valid path. Never shrinks vehicle, shifts goal, or permits pivot turns.
    """
    started = time.time()
    if getattr(grid,'fast_turn90_enabled',False):
        from fast_turn90 import planning_tolerances,direction_change_limit
        grid.fast_turn90_start_reference=tuple(start)
        # None is the unrestricted A* feasibility fallback, not a template cap.
        if max_direction_changes is not None:
            max_direction_changes=direction_change_limit(grid,max_direction_changes)
        goal_position_tolerance,goal_heading_tolerance=planning_tolerances(
            grid,goal,radius,goal_position_tolerance,goal_heading_tolerance,front_goal)
        if (getattr(grid,'fast_turn90_request_role',None) in ('stage','final')
                and getattr(grid,'fast_turn90_tolerances_enabled',True)):
            goal_region=True
    # Limit state expansion to the configured start/goal window. This is a
    # planning acceleration only; collision checks and returned geometry are
    # unchanged. A non-positive value preserves full-map search.
    search_mode = getattr(grid, 'search_mode', None) or maneuver_mode
    special_modes = ('STRAIGHT', 'LATERAL', 'LATERAL_TURN_180',
                     'TURN_90_LEFT', 'TURN_90_RIGHT')
    search_margin = (P.get('planner_special_search_margin', 0.)
                     if search_mode in special_modes else 0.)
    grid.set_search_bounds(start, goal, search_margin)
    if (radius < P['global_min_radius'] or not finite([goal_position_tolerance, goal_heading_tolerance])
            or goal_position_tolerance <= 0 or goal_heading_tolerance <= 0
            or not finite(start+goal) or not grid.free(start) or (not goal_region and not grid.free(goal))):
        raise ValueError('start/goal collision, nonfinite pose, or radius below configured global minimum')
    if max_direction_changes is not None and (isinstance(max_direction_changes, bool)
            or not isinstance(max_direction_changes, int) or max_direction_changes < 0):
        raise ValueError('max_direction_changes must be a nonnegative integer or None')
    if getattr(grid, 'fast_turn90_enabled', False):
        from fast_turn90 import plan_turn90_candidates
        allowance = min(getattr(grid, 'fast_turn90_budget', .35),
                        max(0., max_seconds - (time.time() - started)))
        direct = plan_turn90_candidates(
            grid, start, goal, radius, allowance, cancel,
            goal_position_tolerance, goal_heading_tolerance, front_goal,
            start_reverse_only, max_direction_changes, initial_direction)
        if direct is not None:
            return direct
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
    records = [(start, initial_direction, 0., None, [])]
    direction_changes = [0]
    queue = [(heuristic(start), 0)]
    best = {key(start, initial_direction): 0.}
    last_yield = time.time()
    while queue:
        if cancel():
            raise RuntimeError('cancelled')
        if time.time()-started > max_seconds or len(records) > getattr(grid,'planner_max_records',P['planner_max_records']):
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
        terminal = None
        if goal_region and goal_angle <= goal_heading_tolerance and .015 < goal_distance < .8:
            reference = from_rear(p,'goal') if front_goal is not None else p
            requested = front_goal if front_goal is not None else goal
            dx,dy=requested[0]-reference[0],requested[1]-reference[1]
            along=dx*math.cos(p[2])+dy*math.sin(p[2])
            across=-dx*math.sin(p[2])+dy*math.cos(p[2])
            direction=1 if along>0 else -1
            changes=direction_changes[idx]+int(bool(previous_sign and previous_sign!=direction))
            if (abs(across)<=goal_position_tolerance and abs(along)>.015
                    and (parent is not None or not start_reverse_only or direction<0)
                    and (max_direction_changes is None or changes<=max_direction_changes)):
                straight=grid.arc(p,along,0.)
                if straight and all(not math.isinf(grid.boundary_cost(q)) for q in straight):
                    distance_error,angle_error=goal_error(straight[-1])
                    if distance_error<=goal_position_tolerance and angle_error<=goal_heading_tolerance:
                        terminal=[tuple(q)+(direction,0.) for q in straight]
        if terminal is not None or (goal_distance <= goal_position_tolerance and goal_angle <= goal_heading_tolerance):
            result = []
            while records[idx][3] is not None:
                result.append(records[idx][4]); idx = records[idx][3]
            return [(start[0],start[1],start[2],0,0.)] + [p for chunk in reversed(result) for p in chunk] + (terminal or [])
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


def timed_route_search(searches, max_seconds, cancel, elapsed_offset=0., gear_limits=(2,3)):
    """Shared deadline, preferred gear limits and no forced acceptance wait.

    Phase endpoints cap allocations; exhausted searches advance immediately.
    Valid candidates return immediately, subject to cancellation and deadline.
    """
    now=time.time()
    if not finite([elapsed_offset]):raise ValueError('invalid elapsed offset')
    if elapsed_offset<0:raise ValueError('negative elapsed offset')
    started = now-elapsed_offset
    deadline = now + min(20., max(0., max_seconds))
    three_after=P['planner_three_change_after']
    unlimited_after=P['planner_unlimited_after']
    last_limit=None
    last_errors=[]
    for begin, end, limit in ((0., three_after,gear_limits[0]), (three_after, unlimited_after,gear_limits[1]), (unlimited_after, 20., None)):
        if time.time() >= deadline:
            break
        phase_end = min(deadline, started + end)
        if phase_end<=time.time():continue
        last_limit=limit
        last_errors=[]
        for index, search in enumerate(searches):
            if cancel():
                raise RuntimeError('cancelled')
            remaining = phase_end - time.time()
            if remaining <= 0.:
                break
            try:
                route, info = search(remaining / (len(searches)-index), limit)
            except (ValueError, RuntimeError) as exc:
                if cancel():
                    raise RuntimeError('cancelled')
                last_errors.append(str(exc))
                continue
            if cancel():
                raise RuntimeError('cancelled')
            changes = gear_route_effort(route)[0]
            if limit is not None and changes > limit:
                continue
            # Never discard a lower-change candidate merely because it was
            # discovered in a later search phase.
            if cancel():
                raise RuntimeError('cancelled')
            if time.time() <= deadline:
                return route, info
    error=RuntimeError('planning budget exceeded; no feasible route')
    error.search_exhausted=(last_limit is None and bool(searches)
                           and len(last_errors)==len(searches)
                           and all(text.startswith('no path found') for text in last_errors))
    raise error


def plan_fewer_changes(grid, start, goal, radius=1.25, max_seconds=20.,
                       cancel=lambda:False, goal_position_tolerance=.08,
                       goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL',
                       front_goal=None, start_reverse_only=False, goal_region=False, acceptance_offset=0.):
    grid=grid.snapshot()
    grid.fast_turn90_request_role='final' if front_goal is not None else getattr(grid,'fast_turn90_request_role','final')
    quick_handoff_started=time.time()
    if getattr(grid,'fast_turn90_enabled',False):
        from fast_turn90 import quick_connection
        quick_budget=min(max_seconds,getattr(grid,'fast_turn90_budget',.35))
        direct=quick_connection(grid,start,goal,radius,quick_budget,cancel,
                                goal_position_tolerance,goal_heading_tolerance,front_goal,start_reverse_only)
        if direct is not None:return direct
        spent=time.time()-quick_handoff_started
        max_seconds-=spent;acceptance_offset+=spent
        if max_seconds<=0.:raise RuntimeError('planning budget exceeded in quick handoff')
    def search(seconds, limit):
        path = plan(grid, start, goal, radius, seconds, cancel,
                    goal_position_tolerance, goal_heading_tolerance,
                    maneuver_mode, front_goal,
                    start_reverse_only=start_reverse_only, goal_region=goal_region,
                    max_direction_changes=limit)
        return path, None
    from fast_turn90 import preferred_gear_limits
    limits=preferred_gear_limits(grid, goal, radius, front_goal)
    return timed_route_search([search], max_seconds, cancel, acceptance_offset, gear_limits=limits)[0]


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
        from fast_turn90 import stage_distances
        entry_distance,unused=stage_distances(grid,goal,P['global_min_radius'])
        entry_t = start_t + entry_distance/length * desired_side * (1 if line_side > 0 else -1)
        # First-stage heading is defined in the map frame: face toward the
        # same map side as the target.
        map_right = goal[0] > start[0]
        heading = 0. if map_right else math.pi
        if getattr(grid,'line_heading_from_low_cost_line',False):
            # The aisle is parallel to the measured line, not world x.
            # Derive its sign from projected target displacement, so endpoint
            # order and a rotated map cannot reverse the vehicle's heading.
            along_goal=(goal[0]-start[0])*dx+(goal[1]-start[1])*dy
            heading=wrap(math.atan2(dy,dx)+(0. if along_goal>0 else math.pi))
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


def line_candidate_offsets(grid=None, goal=None):
    """Return the entry point plus configured rearward candidates."""
    step = float(P.get('line_entry_candidate_step', .5))
    count = int(P.get('line_entry_candidate_count', 1))
    if grid is not None and getattr(grid,'fast_turn90_enabled',False):
        from fast_turn90 import stage_distances,space_profile
        entry,unused=stage_distances(grid,goal,P['global_min_radius'])
        profile=space_profile(grid,goal,P['global_min_radius'],goal)['profile']
        step=float(getattr(grid,'fast_turn90_'+profile+'_candidate_step',min(.5,entry*.25)))
        count=int(getattr(grid,'fast_turn90_stage_candidate_count',1))
        if not finite((step,)) or not 0<step<=1. or not 0<=count<=3:
            raise ValueError('invalid space entry candidates')
        return (0.,)+tuple(sign*step*i for i in range(1,count+1) for sign in (1,-1))
    return tuple(step*i for i in range(count+1))


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


def plan_line_approach(grid, start, goal, radius=1.25, max_seconds=5.,
                       cancel=lambda: False, goal_position_tolerance=.08,
                       goal_heading_tolerance=.0523598776, maneuver_mode='NORMAL',
                       front_goal=None):
    """Search entry candidates with the shared 10/15/20 s policy."""
    line_goal = front_goal if front_goal is not None else goal
    grid=grid.snapshot()
    grid.fast_turn90_reference_goal=line_goal
    grid.fast_turn90_request_role='stage'
    candidates = line_stage_candidates(
        grid, start, line_goal, maneuver_mode,
        offsets=line_candidate_offsets(grid,line_goal), rear_offsets=True)
    quick_handoff_started=time.time()
    if getattr(grid,'fast_turn90_enabled',False):
        from fast_turn90 import quick_connection
        quick_budget=min(max_seconds,getattr(grid,'fast_turn90_budget',.35))
        quick_deadline=quick_handoff_started+quick_budget;quick_routes=[]
        for index,(_,number,_,entry) in enumerate(candidates):
            remaining=quick_deadline-time.time()
            if remaining<=0.:break
            direct=quick_connection(grid,start,entry,radius,min(1.5,remaining) if index==0 else remaining/max(1,len(candidates)-index),cancel,
                                    P['stage_plan_position'],math.radians(P['stage_plan_heading_deg']),
                                    start_reverse_only=maneuver_mode=='LATERAL')
            if direct is not None:
                effort=gear_route_effort(direct)
                if effort[0]<=1:return direct,(number,entry)
                quick_routes.append((effort,direct,(number,entry)))
        if quick_routes:
            if cancel():raise RuntimeError('cancelled')
            quick_routes.sort(key=lambda item:item[0])
            return quick_routes[0][1],quick_routes[0][2]
        max_seconds-=time.time()-quick_handoff_started
        if max_seconds<=0.:raise RuntimeError('planning budget exceeded in quick handoff')
    searches = []
    for _, number, _, entry in candidates:
        def search(seconds, limit, number=number, entry=entry):
            first = plan(grid, start, entry, radius=radius, max_seconds=seconds,
                         cancel=cancel, goal_position_tolerance=P['stage_plan_position'],
                         goal_heading_tolerance=math.radians(P['stage_plan_heading_deg']),
                         maneuver_mode=maneuver_mode,
                         start_reverse_only=maneuver_mode == 'LATERAL',
                         max_direction_changes=limit)
            if (maneuver_mode == 'LATERAL' and len(first) > 1 and first[1][3] >= 0):
                raise RuntimeError('lateral approach must start in reverse')
            return first, (number, entry)
        searches.append(search)
    if not searches:
        return None, None
    from fast_turn90 import preferred_gear_limits
    limits=preferred_gear_limits(grid, line_goal, radius, front_goal)
    return timed_route_search(searches, max_seconds, cancel, gear_limits=limits)



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


def plan_line_retreat(grid, start, line, front_goal, heading, radius, max_seconds,
                      cancel=lambda: False):
    """Plan only to the line stop; the final turn waits for measured arrival."""
    from fast_turn90 import stage_distances,planning_tolerances
    grid=grid.snapshot()
    grid.fast_turn90_request_role='stage'
    retreat = line_retreat_target(line, front_goal, heading,
                                  stage_distances(grid,front_goal,radius)[1])
    position_limit,heading_limit=planning_tolerances(
        grid,retreat,radius,P['stage_plan_position'],math.radians(P['stage_plan_heading_deg']))
    dx, dy = retreat[0]-start[0], retreat[1]-start[1]
    along = dx*math.cos(start[2])+dy*math.sin(start[2])
    across = -dx*math.sin(start[2])+dy*math.cos(start[2])
    aligned = (abs(across) <= position_limit and
               abs(wrap(heading-start[2])) <= heading_limit)
    if aligned:
        return checked_straight(grid, start, along, cancel), retreat
    # Correct an imperfect line entry to the stop, without searching the final goal.
    route = plan_fewer_changes(grid, start, retreat, radius, max_seconds, cancel,
                               P['stage_plan_position'],
                               math.radians(P['stage_plan_heading_deg']), 'STRAIGHT')
    return route, retreat


def plan_line_suffix(grid, start, goal, line, heading, radius, max_seconds,
                     cancel, position_tolerance, heading_tolerance, mode,
                     front_goal, goal_region=False, turn90_trace=None,
                     turn90_pre_extension=.30, turn90_post_extension=.30,
                     report=lambda message: None):
    """Plan entry -> retreat -> goal atomically on the already locked line.

    Each final leg starts at the actual planned retreat endpoint. Only complete
    candidates prefer fewer internal direction changes and less steering;
    no partial-route fallback.
    """
    if turn90_trace is not None:
        try:
            route, entry, before, after = separated_line_turn(
                grid, start, goal, turn90_trace, heading_tolerance, front_goal,
                turn90_pre_extension, turn90_post_extension, cancel)
            report('TURN90_SEPARATED: straight %.3fm -> measured turn -> straight %.3fm; no offset search' %
                   (before, after))
            return route, entry
        except ValueError as exc:
            report('TURN90_SEPARATED: rejected (%s); fallback at nominal point' % exc)

    from fast_turn90 import stage_distances,preferred_gear_limits
    retreat = line_retreat_target(line, front_goal, heading, stage_distances(grid,front_goal,radius)[1])
    stage_grid=grid.snapshot();stage_grid.fast_turn90_request_role='stage'
    final_grid=grid.snapshot();final_grid.fast_turn90_request_role='final'
    def search(seconds, limit):
        deadline = time.time() + seconds
        dx, dy = retreat[0]-start[0], retreat[1]-start[1]
        along = dx*math.cos(start[2])+dy*math.sin(start[2])
        across = -dx*math.sin(start[2])+dy*math.cos(start[2])
        aligned = (abs(across) <= P['stage_plan_position'] and
                   abs(wrap(heading-start[2])) <= math.radians(P['stage_plan_heading_deg']))
        if aligned:
            second = checked_straight(grid, start, along, cancel)
        else:
            second = plan(stage_grid, start, retreat, radius, seconds*.4, cancel,
                          P['stage_plan_position'], math.radians(P['stage_plan_heading_deg']),
                          'STRAIGHT', max_direction_changes=limit)
        remaining = deadline-time.time()
        if remaining <= 0.:
            raise RuntimeError('planning budget exceeded')
        used = gear_route_effort(second)[0]
        incoming = next((p[3] for p in reversed(second) if p[3]), 0)
        third_limit = None if limit is None else max(0, limit-used)
        third = plan(final_grid, second[-1][:3], goal, radius, remaining, cancel,
                     position_tolerance, heading_tolerance, mode, front_goal,
                     goal_region=goal_region, max_direction_changes=third_limit,
                     initial_direction=incoming)
        return second + third[1:], retreat
    limits=preferred_gear_limits(grid,goal,radius,front_goal)
    return timed_route_search([search], max_seconds, cancel, gear_limits=limits)


def plan_prefer_lines(grid, start, goal, radius=1.25, max_seconds=5.,
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
        caps[i]=base/(1.0+P['curve_speed_gain']*k)
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


def plan_reliability_fallback(grid,start,goal,requested_goal,radius,max_seconds,
                              cancel=lambda:False,mode='NORMAL',stage=None,
                              stage_target=None,line=None,position=.03,
                              heading=math.radians(4.),max_records=300000):
    """Search for feasibility at the current stop before checkpoint recovery.

    Physical collision/unknown/depth constraints remain mandatory. Window,
    template gear cap, exact template stop policy and reverse-first tactics
    must not restrict this final full-map search. Uses one shared deadline.
    """
    g=grid.snapshot();g.fast_turn90_enabled=False;g.fast_turn90_tolerances_enabled=False
    g.search_mode='NORMAL';g.search_bounds=None;g.planner_max_records=max_records
    deadline=time.time()+max_seconds
    from fast_turn90 import stop_policy
    policy=stop_policy(grid,requested_goal,radius)
    stage_position=min(P['stage_plan_position'],.06)
    stage_heading=5.
    if policy is not None:
        stage_position=min(stage_position,policy['stage_position']-.01)
        stage_heading=min(stage_heading,policy['stage_heading_deg']-.5)
    if stage in ('RETURN_STOP','APPROACH_LINE','RETREAT_LINE'):
        target=stage_target
        if stage=='RETREAT_LINE':
            from fast_turn90 import stage_distances
            target=line_retreat_target(line,requested_goal,stage_target[2],stage_distances(grid,requested_goal,radius)[1])
        if target is None:raise ValueError('missing fallback stage target')
        path=plan(g,start,tuple(target),radius,max_seconds,cancel,
                  stage_position,math.radians(stage_heading),'NORMAL',
                  goal_region=True,max_direction_changes=None)
        return path,None,stage,target
    if stage is None and mode in ('LATERAL','LATERAL_TURN_180') and g.zero_cost_line:
        # Preserve the useful staged route if any entry works. No reverse-first
        # hard requirement and no direction-change restriction in the fallback.
        entries=line_stage_candidates(g,start,requested_goal,mode,
                                      offsets=line_candidate_offsets(g,requested_goal),rear_offsets=True)
        entry_deadline=time.time()+min(20.,max_seconds*.35)
        for index,(_,number,_,entry) in enumerate(entries):
            if cancel():raise RuntimeError('cancelled')
            remaining=entry_deadline-time.time()
            if remaining<=0:break
            try:
                path=plan(g,start,entry,radius,remaining/max(1,len(entries)-index),cancel,
                          stage_position,math.radians(stage_heading),'NORMAL',
                          goal_region=True,max_direction_changes=None)
                return path,(number,entry),'APPROACH_LINE',entry
            except (RuntimeError,ValueError):
                if cancel():raise RuntimeError('cancelled')
        # Line-based acceleration is optional; full-map feasibility is not.
    remaining=deadline-time.time()
    if remaining<=0:raise RuntimeError('full-map feasibility fallback budget exceeded')
    path=plan(g,start,goal,radius,remaining,cancel,position,heading,'NORMAL',requested_goal,
              goal_region=True,max_direction_changes=None,start_reverse_only=False)
    return path,None,'FINAL' if stage=='FINAL' else None,requested_goal
