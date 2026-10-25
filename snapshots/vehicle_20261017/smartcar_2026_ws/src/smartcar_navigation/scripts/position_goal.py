"""Position-only front-axle terminal goals; normal pose planning is unchanged."""
from __future__ import division
import math
import time
from ackermann_core import plan, finite
from nav_config import P, from_rear, to_rear


def position_only_payload(data):
    value=data.get('position_only',False)
    if not isinstance(value,bool):raise ValueError('position_only must be boolean')
    if value and data.get('target_id')!='end':
        raise ValueError('position-only arrival is restricted to the mission end')
    return value


def plan_to_position(grid,start,goal,radius,max_seconds,cancel=lambda:False,
                     position_tolerance=.025,max_records=None):
    """Search x/y using the ACTUAL front axle at every candidate heading.

    A pi heading tolerance has no preferred yaw; disable only this goal's
    fixed-heading templates. Collision, scan, region and radius constraints
    are retained on an independent snapshot.
    """
    if (not finite(tuple(start)+tuple(goal)+(radius,max_seconds,position_tolerance))
            or radius<P['global_min_radius'] or max_seconds<=0 or position_tolerance<=0):
        raise ValueError('invalid position-only goal/search bounds')
    deadline=time.time()+max_seconds
    if cancel():raise RuntimeError('cancelled')
    g=grid.snapshot()
    g.fast_turn90_enabled=False;g.fast_turn90_tolerances_enabled=False
    g.search_mode='NORMAL';g.set_search_bounds(start,start,0.)
    if max_records is not None:g.planner_max_records=max_records
    if not g.free(start):raise ValueError('start collision in position-only plan')
    reference=from_rear(start,'goal')
    dx,dy=goal[0]-reference[0],goal[1]-reference[1]
    if math.hypot(dx,dy)<=position_tolerance:
        return [tuple(start[:3])+(0,0.)]
    along=dx*math.cos(start[2])+dy*math.sin(start[2])
    across=-dx*math.sin(start[2])+dy*math.cos(start[2])
    # The common aligned return needs no turn and no Hybrid A* exploration.
    if abs(across)<=position_tolerance and abs(along)>.015:
        arc=g.arc(start,along,0.)
        if arc and all(not math.isinf(g.boundary_cost(p)) for p in arc):
            xy=from_rear(arc[-1],'goal')
            if math.hypot(xy[0]-goal[0],xy[1]-goal[1])<=position_tolerance:
                if cancel():raise RuntimeError('cancelled')
                if time.time()>deadline:raise RuntimeError('position-only planning budget exceeded')
                direction=1 if along>0 else -1
                return [tuple(start[:3])+(0,0.)]+[tuple(p)+(direction,0.) for p in arc]
    remaining=deadline-time.time()
    if remaining<=0:raise RuntimeError('position-only planning budget exceeded')
    # The yaw stored in the point is deliberately replaced by an irrelevant
    # start-yaw hint; goal errors use actual candidate front-axle positions.
    front_goal=(goal[0],goal[1],start[2])
    return plan(g,tuple(start[:3]),to_rear(front_goal,'goal'),radius,remaining,cancel,
                position_tolerance,math.pi,'NORMAL',front_goal,
                goal_region=True,max_direction_changes=None)
