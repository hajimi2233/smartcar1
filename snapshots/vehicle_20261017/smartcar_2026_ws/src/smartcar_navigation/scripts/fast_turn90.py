"""Finite analytic 90-degree candidates; no ROS, endpoint snapping or grid search.

Compatible with Python 2/3. Every returned edge is sampled by Grid.arc, retaining
the deployed footprint, unknown-space, dynamic obstacle and depth checks.
"""
from __future__ import division
import math
import time
import threading
from collections import OrderedDict

_RECIPES=OrderedDict()
_RECIPE_LOCK=threading.RLock()


def maneuver_lateral(start,front_goal):
    """Use the intended lane axis for near-parallel goals despite stop yaw.

    A small yaw error over a long straight must not turn an in-lane goal into
    a lateral mission. This selects a strategy only; geometry uses actual yaw.
    Both points use the configured user-goal reference.
    """
    from nav_config import from_rear
    current=from_rear(start,'goal')
    delta=abs(_wrap(front_goal[2]-start[2]))
    axis=front_goal[2] if delta<=math.radians(15.) or delta>=math.radians(150.) else start[2]
    dx,dy=front_goal[0]-current[0],front_goal[1]-current[1]
    return -dx*math.sin(axis)+dy*math.cos(axis)


def static_cell_overlaps(dx,dy,c,s,half_length,half_width,cell_half):
    """Exact OBB/cell SAT; caller retains margin and sampling sweep padding.

    The entire occupied/unknown cell is an obstacle, never just its centre.
    Touching counts as collision. Four separating axes suffice in 2D.
    """
    projection=cell_half*(abs(c)+abs(s))
    return (abs(dx)<=abs(c)*half_length+abs(s)*half_width+cell_half and
            abs(dy)<=abs(s)*half_length+abs(c)*half_width+cell_half and
            abs(c*dx+s*dy)<=half_length+projection and
            abs(-s*dx+c*dy)<=half_width+projection)


def direction_change_limit(grid,limit=None):
    if limit is not None and (isinstance(limit,bool) or not isinstance(limit,int) or limit<0):
        raise ValueError('invalid direction change limit')
    if not getattr(grid,'fast_turn90_enabled',False):return limit
    cap=int(getattr(grid,'fast_turn90_max_changes',3))
    if getattr(grid,'fast_turn90_many_cusps_enabled',False):
        cap=max(cap,int(getattr(grid,'fast_turn90_small_max_changes',32)))
    if not 0<=cap<=64:raise ValueError('invalid hard direction-change cap')
    return cap if limit is None else min(limit,cap)


def _entry_to_line(grid,point,line):
    """Estimate physical channel wall ends; never equate target depth to space."""
    (x1,y1),(x2,y2)=line;dx,dy=x2-x1,y2-y1;length2=dx*dx+dy*dy
    t=max(0.,min(1.,((point[0]-x1)*dx+(point[1]-y1)*dy)/length2))
    px,py=x1+t*dx,y1+t*dy;distance=math.hypot(px-point[0],py-point[1])
    explicit=getattr(grid,'fast_turn90_entry_distance',None)
    if explicit is not None:
        explicit=float(explicit)
        if not 0<explicit<=5.:raise ValueError('invalid measured entrance distance')
        return explicit,'measured_entry',None
    entries=getattr(grid,'fast_turn90_channel_entries',[])
    if entries:
        entry=min(entries,key=lambda p:math.hypot(p[0]-point[0],p[1]-point[1]))
        ex,ey=entry[:2];q=max(0.,min(1.,((ex-x1)*dx+(ey-y1)*dy)/length2))
        return math.hypot(ex-x1-q*dx,ey-y1-q*dy),'configured_entry',tuple(entry[:2])
    if distance<grid.res:return None,'unknown_entry',None
    width=float(getattr(grid,'fast_turn90_channel_width',.90))
    if not .5<=width<=2.:raise ValueError('invalid channel width')
    cache=getattr(grid,'fast_turn90_entry_cache',None)
    if cache is None:cache={};grid.fast_turn90_entry_cache=cache
    key=(id(grid.data),tuple(grid.origin),tuple(line[0]),tuple(line[1]),round(point[0],4),round(point[1],4),width)
    if key in cache:return cache[key]
    nx,ny=(px-point[0])/distance,(py-point[1])/distance
    ox,oy,oyaw=grid.origin;cs,sn=math.cos(oyaw),math.sin(oyaw)
    def walls(q,side):
        cells=set()
        for lateral in (width/2.-.08,width/2.,width/2.+.08):
            i,j=grid.cell(point[0]+q*nx-side*lateral*ny,point[1]+q*ny+side*lateral*nx)
            if 0<=i<grid.w and 0<=j<grid.h and grid.data[j*grid.w+i]>=50:cells.add((i,j))
        return cells
    if not walls(grid.res*.5,1) or not walls(grid.res*.5,-1):
        result=(None,'unknown_entry',None)
    else:
        furthest=0.;empty=0;found=False
        for sample in range(int(distance/grid.res)+1):
            q=(sample+.5)*grid.res
            if q>=distance:break
            cells=walls(q,1)|walls(q,-1)
            if not cells:
                empty+=1
                if empty>=2:found=True;break
                continue
            empty=0
            for i,j in cells:
                for di,dj in ((0,0),(1,0),(0,1),(1,1)):
                    lx,ly=(i+di)*grid.res,(j+dj)*grid.res
                    wx,wy=ox+cs*lx-sn*ly,oy+sn*lx+cs*ly
                    furthest=max(furthest,(wx-point[0])*nx+(wy-point[1])*ny)
        result=(max(0.,distance-furthest),'map_wall_end',
                (point[0]+furthest*nx,point[1]+furthest*ny)) if found else (None,'unknown_entry',None)
    if len(cache)>256:cache.clear()
    cache[key]=result;return result

# Finish limits bound measured handoff/arrival; plan limits leave stop reserve.
TOLERANCE_DEFAULTS={
    # Observed stage stops reach 9.4cm/7.3deg. These admission limits include
    # reserve; they do not prescribe artificial, smaller test disturbances.
    # Accurate planned endpoints remain separate from achievable actual stops.
    'large':dict(stage_position=.12,stage_heading_deg=10.,final_position=.05,final_heading_deg=5.,
                 stage_plan_position=.03,stage_plan_heading_deg=1.5,final_plan_position=.03,final_plan_heading_deg=1.5,
                 align_heading_deg=.3,align_cross_track=.003),
    'medium':dict(stage_position=.12,stage_heading_deg=10.,final_position=.05,final_heading_deg=5.,
                  stage_plan_position=.03,stage_plan_heading_deg=1.5,final_plan_position=.03,final_plan_heading_deg=1.5,
                  align_heading_deg=.3,align_cross_track=.003),
    'small':dict(stage_position=.05,stage_heading_deg=3.,final_position=.05,final_heading_deg=5.,
                 stage_plan_position=.03,stage_plan_heading_deg=1.,final_plan_position=.025,final_plan_heading_deg=2.,
                 align_heading_deg=3.,align_cross_track=.02)}


def stop_policy(grid,goal,radius):
    if not (getattr(grid,'fast_turn90_enabled',False) and
            getattr(grid,'fast_turn90_tolerances_enabled',True)):return None
    selected=space_profile(grid,goal,radius,goal)
    profile=selected['profile'];result=dict(TOLERANCE_DEFAULTS[profile])
    for field,default in list(result.items()):
        result[field]=float(getattr(grid,'fast_turn90_'+profile+'_'+field,default))
        upper=30. if field.endswith('_deg') else .30
        if not 0<result[field]<=upper or math.isnan(result[field]):
            raise ValueError('invalid space tolerance '+profile+'_'+field)
    for role in ('stage','final'):
        if (result[role+'_plan_position']>=result[role+'_position'] or
                result[role+'_plan_heading_deg']>=result[role+'_heading_deg']):
            raise ValueError('plan tolerance must leave stop reserve')
    result.update(profile=profile,line_distance=selected['distance'])
    entry=selected.get('entry_distance')
    # Physical entrance depth is diagnostic only; it never changes stop limits.
    result.update(entry_distance=entry,classification_basis=selected.get('classification_basis'))
    return result


def planning_tolerances(grid,goal,radius,position,heading,front_goal=None):
    role=getattr(grid,'fast_turn90_request_role',None)
    if role not in ('stage','final'):return position,heading
    reference=getattr(grid,'fast_turn90_reference_goal',None)
    reference=reference if reference is not None else front_goal if front_goal is not None else goal
    policy=stop_policy(grid,reference,radius)
    if policy is None:return position,heading
    # The caller's first-attempt/relaxed positional budget remains an upper cap.
    return min(position,policy[role+'_plan_position']),math.radians(policy[role+'_plan_heading_deg'])


def quick_connection(grid,start,goal,radius,seconds,cancel,position,heading,
                     front_goal=None,start_reverse_only=False):
    """Check all allowed finite connections before spending seconds in A*."""
    position,heading=planning_tolerances(grid,goal,radius,position,heading,front_goal)
    return plan_turn90_candidates(grid,start,goal,radius,seconds,cancel,position,heading,
                                  front_goal,start_reverse_only,direction_change_limit(grid))


def space_profile(grid, goal, radius, front_goal=None):
    """Final goal-reference point to nearest finite low-cost segment, in meters."""
    from nav_config import P,from_rear
    point=getattr(grid,'fast_turn90_reference_goal',None)
    if point is None:point=front_goal if front_goal is not None else from_rear(goal,'goal')
    distances=[]
    for number,line in enumerate(grid.zero_cost_line):
        (x1,y1),(x2,y2)=line
        dx,dy=x2-x1,y2-y1;length2=dx*dx+dy*dy
        if length2<1e-8:continue
        t=max(0.,min(1.,((point[0]-x1)*dx+(point[1]-y1)*dy)/length2))
        distances.append((math.hypot(point[0]-x1-t*dx,point[1]-y1-t*dy),number))
    small=float(getattr(grid,'fast_turn90_small_distance',1.50))
    large=float(getattr(grid,'fast_turn90_large_distance',2.12))
    if not (0<small<large and not math.isinf(large)):
        raise ValueError('invalid space profile thresholds')
    override=getattr(grid,'fast_turn90_profile_override',None)
    if not distances:return dict(profile=override if override in TOLERANCE_DEFAULTS else 'medium',distance=None,line=None,
                                 entry_distance=None,classification_basis='no_low_cost_line')
    distance,number=min(distances)
    entry,coordinate=None,None
    if getattr(grid,'fast_turn90_entry_diagnostics_enabled',False):
        entry,unused,coordinate=_entry_to_line(grid,point,grid.zero_cost_line[number])
    profile='small' if distance<small-1e-8 else 'large' if distance>=large-1e-8 else 'medium'
    return dict(profile=override if override in TOLERANCE_DEFAULTS else profile,
                distance=distance,line=number,entry_distance=entry,entry_coordinate=coordinate,
                classification_basis='final_goal_to_finite_low_cost_line')


def preferred_gear_limits(grid, goal, radius, front_goal=None):
    """Small-space templates need several cusps, without first waiting on 2/3."""
    if (getattr(grid,'fast_turn90_enabled',False) and getattr(grid,'fast_turn90_many_cusps_enabled',False)
            and space_profile(grid,goal,radius,front_goal)['profile']=='small'):
        limit=int(getattr(grid,'fast_turn90_small_max_changes',32))
        if not 2<=limit<=64:raise ValueError('invalid small-space gear limit')
        return limit,limit+2
    return 2,3


def stage_distances(grid,goal,radius):
    """Keep entry offset and final-turn retreat offset independently tuneable."""
    profile=space_profile(grid,goal,radius,goal)['profile']
    defaults={'large':(1.8,1.8),'medium':(1.5,1.5),'small':(.8,.8)}
    entry,retreat=defaults[profile]
    if not getattr(grid,'fast_turn90_enabled',False):
        from nav_config import P
        return P['line_entry_distance'],P['line_retreat_distance']
    entry=float(getattr(grid,'fast_turn90_'+profile+'_entry_distance',entry))
    retreat=float(getattr(grid,'fast_turn90_'+profile+'_retreat_distance',retreat))
    if not all(0<value<=4. and not math.isnan(value) for value in (entry,retreat)):
        raise ValueError('invalid space stage distance')
    return entry,retreat


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _advance(pose, distance, curvature):
    x, y, yaw = pose
    if abs(curvature) < 1e-10:
        return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw
    turn = distance * curvature
    return (x + (math.sin(yaw + turn) - math.sin(yaw)) / curvature,
            y + (math.cos(yaw) - math.cos(yaw + turn)) / curvature,
            _wrap(yaw + turn))


def _effort(legs, initial_direction=0):
    previous, changes, cost = initial_direction, 0, 0.
    first = 0
    for distance, curvature in legs:
        if abs(distance) < 1e-8:
            continue
        direction = 1 if distance > 0 else -1
        if not first:
            first = direction
        if previous and direction != previous:
            changes += 1
        previous = direction
        cost += abs(distance) * (1.35 if direction < 0 else 1.)
    return changes, cost, first


def _csc_candidates(start, goal, radius):
    """Circle tangents for same-gear arc/straight/arc, including gentle alignment.

    Turn directions are geometric travel directions. Reverse travel rotates both
    travel headings by pi and negates both distance and curvature in body space.
    """
    result=[]
    for direction in (1,-1):
        h0=start[2]+(math.pi if direction<0 else 0.)
        h1=goal[2]+(math.pi if direction<0 else 0.)
        for scale in (1.,1.5,2.,3.,4.):
            r=radius*scale
            for first in (1,-1):
                for last in (1,-1):
                    c0=(start[0]-first*r*math.sin(h0),start[1]+first*r*math.cos(h0))
                    c1=(goal[0]-last*r*math.sin(h1),goal[1]+last*r*math.cos(h1))
                    dx,dy=c1[0]-c0[0],c1[1]-c0[1]
                    distance=math.hypot(dx,dy)
                    if distance<1e-9:continue
                    q=(last-first)*r/distance
                    if abs(q)>1.+1e-10:continue
                    tangent=math.atan2(dy,dx)-math.asin(max(-1.,min(1.,q)))
                    a0=(first*(tangent-h0))%(2*math.pi)
                    a1=(last*(h1-tangent))%(2*math.pi)
                    # Remove roundoff near zero, without inventing a full loop.
                    if a0>2*math.pi-1e-8:a0=0.
                    if a1>2*math.pi-1e-8:a1=0.
                    if a0+a1>math.pi+1e-9:continue
                    straight=max(0.,distance*math.cos(math.asin(max(-1.,min(1.,q)))))
                    result.append([(direction*r*a0,direction*first/r),
                                   (direction*straight,0.),
                                   (direction*r*a1,direction*last/r)])
    return result


def _medium_candidates(start, goal, radius, max_changes=None, start_reverse_only=False,
               initial_direction=0):
    """Solve lead/tail lengths from each fixed one/two-arc template.

    All angles sum exactly to goal yaw. Leading and trailing straights may be
    signed; their lengths solve a 2x2 system, rather than an offset grid search.
    """
    delta = _wrap(goal[2] - start[2])
    if abs(delta)>math.radians(120.):
        return []
    c, s = math.cos(start[2]), math.sin(start[2])
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    x, y = c * dx + s * dy, -s * dx + c * dy
    sine, cosine = math.sin(delta), math.cos(delta)
    result, seen = [], set()
    from nav_config import P
    penalty=P.get('planner_switch_cost',.45)
    def rank(item):return item[1]+penalty*item[0],item[0],item[1]
    def add(legs):
        legs=[(d,k) for d,k in legs if abs(d)>1e-8]
        signature=tuple((round(d,8),round(k,8)) for d,k in legs)
        if signature in seen:return
        seen.add(signature)
        changes,cost,first=_effort(legs,initial_direction)
        if start_reverse_only and first>0:return
        if max_changes is not None and changes>max_changes:return
        result.append((changes,cost,legs))
    for legs in _csc_candidates(start,goal,radius):add(legs)
    if abs(delta)<math.radians(30.):
        # Real stops are neither exactly parallel nor exactly centred. Solve a
        # bounded S-C-S-C-S correction including mixed gears, even at delta=0.
        # Unlike dividing by sin(delta), this system stays useful near zero.
        for degrees in (-30.,-15.,-10.,-5.,5.,10.,15.,30.):
            first_angle=math.radians(degrees);second_angle=delta-first_angle
            determinant=math.sin(delta-first_angle)
            if abs(determinant)<.01:continue
            for scale in (1.,2.,4.):
                r=radius*scale
                for first_direction in (1,-1):
                    for last_direction in (1,-1):
                        arc1=(first_direction*r*abs(first_angle),first_direction*(1 if first_angle>0 else -1)/r)
                        arc2=(last_direction*r*abs(second_angle),last_direction*(1 if second_angle>0 else -1)/r)
                        endpoint=_advance(_advance((0.,0.,0.),*arc1),*arc2)
                        for before in (0.,-.10,.10,-.30,.30,-.60,.60,-1.,1.):
                            rx,ry=x-endpoint[0]-before,y-endpoint[1]
                            middle=(rx*sine-ry*cosine)/determinant
                            after=(math.cos(first_angle)*ry-math.sin(first_angle)*rx)/determinant
                            add([(before,0.),arc1,(middle,0.),arc2,(after,0.)])
        result.sort(key=rank)
        return result
    turn_sign = 1 if delta > 0 else -1
    # Correct the measured lane-exit heading BEFORE the long straight and main
    # turn. Adjacent arcs cannot express this: they start the 90-degree turn
    # inside the channel. Solve the internal straight as well as the final one.
    first_angles=[math.radians(d) for d in (-10.,-5.,-2.5,2.5,5.,10.)]
    first_angles.extend((delta-math.pi/2,delta+math.pi/2))
    for first_angle in first_angles:
        if abs(first_angle)>math.radians(25.) or abs(first_angle)<math.radians(.3):continue
        second_angle=delta-first_angle;determinant=math.sin(second_angle)
        if abs(determinant)<.1 or abs(second_angle)>math.radians(120.):continue
        for first_scale,last_scale in ((1.,1.),(2.,1.),(4.,1.),(1.,1.25),(2.,1.5)):
            for first_direction in (1,-1):
                for last_direction in (1,-1):
                    arc1=(first_direction*radius*first_scale*abs(first_angle),
                          first_direction*(1 if first_angle>0 else -1)/(radius*first_scale))
                    arc2=(last_direction*radius*last_scale*abs(second_angle),
                          last_direction*(1 if second_angle>0 else -1)/(radius*last_scale))
                    endpoint=_advance(_advance((0.,0.,0.),*arc1),*arc2)
                    for before in (0.,-.1,.1,-.3,.3,-.6,.6):
                        rx,ry=x-endpoint[0]-before,y-endpoint[1]
                        middle=(rx*sine-ry*cosine)/determinant
                        after=(math.cos(first_angle)*ry-math.sin(first_angle)*rx)/determinant
                        add([(before,0.),arc1,(middle,0.),arc2,(after,0.)])
    # Mixed radii retain the old shared-radius templates and add the larger
    # radius used by the Hybrid A* half-curvature primitive.
    pairs=[(a,b) for a in (1.,1.5,2.) for b in (1.,1.10,1.25,1.5,2.)]
    for scale1,scale2 in pairs:
        r1,r2=radius*scale1,radius*scale2
        for degrees in (0., -5.,5.,-15.,15.,-30.,30.,-45.,45.,60.,75.):
            first_angle = turn_sign * math.radians(degrees)
            for first_direction in (1, -1):
                for second_direction in (1, -1):
                    arcs, endpoint = [], (0., 0., 0.)
                    for angle, direction, r in ((first_angle,first_direction,r1),
                                                (delta-first_angle,second_direction,r2)):
                        if abs(angle) < 1e-9:
                            continue
                        curvature = direction * (1 if angle > 0 else -1) / r
                        distance = direction * r * abs(angle)
                        arcs.append((distance, curvature))
                        endpoint = _advance(endpoint, distance, curvature)
                    after = (y - endpoint[1]) / sine
                    before = x - endpoint[0] - after * cosine
                    add([(before,0.)]+arcs+[(after,0.)])
    # Separate a main 90-degree turn from a small final heading correction by
    # an internal straight. Its length follows from selected lead/tail lengths.
    # This matters at the exit: immediately turning inside a lane is blocked,
    # but leaving the lane, turning, then correcting yaw in the outer corridor
    # can be legal. No target heading is changed.
    first_angle=turn_sign*math.pi/2
    second_angle=delta-first_angle
    if abs(second_angle)>1e-7:
        for scale1 in (1.,1.10):
            for scale2 in (1.,2.,4.):
                r1,r2=radius*scale1,radius*scale2
                for d1 in (1,-1):
                    for d2 in (1,-1):
                        k1=d1*turn_sign/r1
                        k2=d2*(1 if second_angle>0 else -1)/r2
                        arc1=(d1*r1*abs(first_angle),k1)
                        arc2=(d2*r2*abs(second_angle),k2)
                        endpoint=_advance(_advance((0.,0.,0.),*arc1),*arc2)
                        b0=(y-endpoint[1])/sine
                        a0=x-endpoint[0]-b0*cosine
                        bs=-math.sin(first_angle)/sine
                        aa=-math.cos(first_angle)-bs*cosine
                        middles=[0.]
                        if abs(aa)>1e-8:
                            middles.extend((value-a0)/aa for value in
                                           (-1.5,-1.,-.5,0.,.25,.5,.75,1.,1.25,1.5,2.))
                        if abs(bs)>1e-8:
                            middles.extend((value-b0)/bs for value in (-1.,-.5,0.,.5,1.))
                        for middle in middles:
                            before=a0+aa*middle;after=b0+bs*middle
                            add([(before,0.),arc1,(middle,0.),arc2,(after,0.)])
    # A short entry correction and a short exit correction can reduce the long
    # lead/tail excursions of a two-arc family in a narrow passage. These are
    # fixed, finite templates, not a position lattice or unrestricted search.
    for scales in ((1.,1.,1.),(2.,1.,2.),(1.,2.,1.)):
        for first_deg in (-30.,-15.,15.,30.):
            for last_deg in (-30.,-15.,15.,30.):
                first=turn_sign*math.radians(first_deg)
                last=turn_sign*math.radians(last_deg)
                angles=(first,delta-first-last,last)
                if abs(angles[1])>math.pi:continue
                for bits in range(8):
                    pose=(0.,0.,0.);legs=[]
                    for i,(angle,scale) in enumerate(zip(angles,scales)):
                        if abs(angle)<1e-8:continue
                        direction=-1 if bits&(1<<i) else 1
                        r=radius*scale
                        leg=(direction*r*abs(angle),direction*(1 if angle>0 else -1)/r)
                        legs.append(leg);pose=_advance(pose,*leg)
                    after=(y-pose[1])/sine
                    before=x-pose[0]-after*cosine
                    add([(before,0.)]+legs+[(after,0.)])
    result.sort(key=rank)
    return result


def _fit_recipe(start,goal,radius,recipe,max_changes,start_reverse_only,initial_direction):
    delta=_wrap(goal[2]-start[2]);sine=math.sin(delta)
    if abs(sine)<1e-6 or abs(delta)>math.radians(120.):return None
    c,s=math.cos(start[2]),math.sin(start[2])
    dx,dy=goal[0]-start[0],goal[1]-start[1]
    x,y=c*dx+s*dy,-s*dx+c*dy
    pose=(0.,0.,0.);arcs=[]
    for fraction,direction,scale in recipe:
        angle=delta*fraction
        if abs(angle)<1e-9:continue
        r=radius*scale
        if r<radius-1e-9:raise ValueError('recipe below minimum radius')
        leg=(direction*r*abs(angle),direction*(1 if angle>0 else -1)/r)
        arcs.append(leg);pose=_advance(pose,*leg)
    after=(y-pose[1])/sine;before=x-pose[0]-after*math.cos(delta)
    legs=[(d,k) for d,k in [(before,0.)]+arcs+[(after,0.)] if abs(d)>1e-8]
    changes,cost,first=_effort(legs,initial_direction)
    if start_reverse_only and first>0:return None
    if max_changes is not None and changes>max_changes:return None
    return changes,cost,legs


def _small_recipes():
    """Bounded alternating-gear short-arc bank; total heading remains exact."""
    for count in range(3,10):
        for shape in (0,1,2,3):
            weights=[1.]*count
            if shape==1:weights[0]=weights[-1]=.5
            elif shape==2:weights[0]=weights[-1]=1.5
            elif shape==3:
                weights=[1.25 if i%2==0 else .75 for i in range(count)]
            total=sum(weights)
            for first in (1,-1):
                yield tuple((weight/total,first*(-1 if i%2 else 1),1.)
                            for i,weight in enumerate(weights))


def _recipe_from_legs(legs,delta,radius):
    if abs(delta)<1e-6:return None
    arcs=[(d,k) for d,k in legs if abs(k)>1e-8]
    # Internal straights need a different solver; do not cache them as arcs.
    indexes=[i for i,(d,k) in enumerate(legs) if abs(k)>1e-8]
    if not indexes or len(indexes)!=indexes[-1]-indexes[0]+1:return None
    recipe=tuple((d*k/delta,1 if d>0 else -1,1./(abs(k)*radius)) for d,k in arcs)
    return recipe if abs(sum(item[0] for item in recipe)-1.)<1e-7 else None


def _balanced_recipes(grid,start,goal,radius,selection):
    """Short alternating arcs keep the body center near the guide line.

    Solve each pair's split angle from a scalar cosine equation; solve the
    lead/tail lengths again afterwards. No pose lattice or endpoint snapping.
    """
    from nav_config import P
    if selection['line'] is None:return []
    (x1,y1),(x2,y2)=grid.zero_cost_line[selection['line']]
    line_yaw=math.atan2(y2-y1,x2-x1)
    if math.cos(start[2]-line_yaw)<0:line_yaw+=math.pi
    delta=_wrap(goal[2]-start[2]);sign=1 if delta>0 else -1
    if not math.radians(30.)<=abs(delta)<=math.radians(120.):return []
    heading=_wrap(start[2]-line_yaw)
    center=(P['body_front']-P['body_rear'])/2.
    first=min(2*math.atan(center/radius),abs(delta)*.4)
    result=[]
    for step_deg in (8.,5.,3.,2.):
        for order in (1,-1):
            lead=0.;recipe=None
            for iteration in range(4):
                initial=_advance(start,lead,0.)
                normal=-(initial[0]-x1)*math.sin(line_yaw)+(initial[1]-y1)*math.cos(line_yaw)
                theta=heading+sign*first
                normal-=sign*radius*(math.cos(heading)-math.cos(theta))
                recipe=[(first/abs(delta),-1,1.)]
                count=max(1,int(math.ceil((abs(delta)-first)/math.radians(step_deg))))
                increment=(abs(delta)-first)/count
                valid=True
                for number in range(count):
                    end=theta+sign*increment
                    desired=-center*math.sin(end)
                    direction=order*(-1 if number%2 else 1)
                    def displacement(mid):
                        return sign*radius*direction*(math.cos(theta)+math.cos(end)-2*math.cos(mid))
                    lo,hi=theta,end;wanted=desired-normal
                    dlo,dhi=displacement(lo),displacement(hi)
                    if wanted<min(dlo,dhi)-1e-9 or wanted>max(dlo,dhi)+1e-9:
                        valid=False;break
                    for unused in range(32):
                        mid=(lo+hi)/2.;value=displacement(mid)
                        if (value<wanted)==(dlo<dhi):lo=mid
                        else:hi=mid
                    mid=(lo+hi)/2.
                    recipe.extend(((abs(mid-theta)/abs(delta),direction,1.),
                                   (abs(end-mid)/abs(delta),-direction,1.)))
                    normal=desired;theta=end
                if not valid:recipe=None;break
                recipe=tuple(recipe)
                fitted=_fit_recipe(start,goal,radius,recipe,None,False,0)
                if fitted is None:recipe=None;break
                new_lead=fitted[2][0][0] if abs(fitted[2][0][1])<1e-8 else 0.
                if abs(new_lead-lead)<1e-7:break
                lead=new_lead
            if recipe is not None:result.append(recipe)
    return result


def candidates(start,goal,radius,max_changes=None,start_reverse_only=False,
               initial_direction=0,profile='medium',recipes=(),extra_recipes=()):
    from nav_config import P
    delta=_wrap(goal[2]-start[2])
    if abs(delta)>math.radians(120.):return []
    def fitted(bank):
        options=[];seen=set()
        for recipe in bank:
            option=_fit_recipe(start,goal,radius,recipe,max_changes,start_reverse_only,initial_direction)
            if option is None:continue
            key=tuple((round(d,7),round(k,7)) for d,k in option[2])
            if key not in seen:seen.add(key);options.append(option)
        options.sort(key=lambda item:(item[1]+P.get('planner_switch_cost',.45)*item[0],item[0]))
        return options
    cached=fitted(recipes)
    once=fitted([((1.,direction,scale),) for scale in (1.,1.25,1.5,2.) for direction in (1,-1)])
    # "Once" requires no gear changes through lead/arc/tail, including current gear.
    # One steering arc may include a short straight reversal. Do not confuse
    # "one turn" with "no forward/reverse change anywhere".
    small=fitted(list(_small_recipes())+list(extra_recipes)) if abs(delta)>=math.radians(30.) else []
    medium=_medium_candidates(start,goal,radius,max_changes,start_reverse_only,initial_direction)
    groups=(once,cached,medium,small) if profile=='large' else (cached,medium,small)
    result=[];seen=set()
    for group in groups:
        for option in group:
            key=tuple((round(d,7),round(k,7)) for d,k in option[2])
            if key not in seen:seen.add(key);result.append(option)
    # Execution quality wins over template family and warm-cache preference.
    # A cached route with more reversals must not displace a feasible simpler one.
    def steering_runs(legs):
        previous=0;runs=0
        for distance,k in legs:
            sign=1 if k>1e-8 else -1 if k<-1e-8 else 0
            if sign and sign!=previous:runs+=1
            if sign:previous=sign
        return runs
    result.sort(key=lambda item:(item[0],steering_runs(item[2]),item[1]))
    return result


def _checked_legs(grid,start,legs,deadline,cancel):
    def safe(pose):return grid.free(pose) and not math.isinf(grid.boundary_cost(pose))
    pose=tuple(start);route=[tuple(start)+(0,0.)]
    for distance,curvature in legs:
        if cancel():raise RuntimeError('cancelled')
        if time.time()>=deadline:return None
        if not all(safe(_advance(pose,distance*f,curvature)) for f in (.25,.5,.75,1.)):
            return None
        points=grid.arc(pose,distance,curvature)
        if points is None or any(math.isinf(grid.boundary_cost(p)) for p in points):return None
        direction=1 if distance>0 else -1
        route.extend(tuple(point)+(direction,curvature) for point in points)
        pose=tuple(points[-1])
    return route if time.time()<=deadline else None


def _align_small(grid,start,target,radius,selection,deadline,cancel,position_tolerance,
                 heading_tolerance,front_goal,start_reverse_only,max_changes,initial_direction,diagnostics,max_length):
    if selection['line'] is None:return None
    (x1,y1),(x2,y2)=grid.zero_cost_line[selection['line']]
    dx,dy=x2-x1,y2-y1;length=math.hypot(dx,dy)
    line_yaw=math.atan2(dy,dx)
    if math.cos(start[2]-line_yaw)<0:line_yaw=_wrap(line_yaw+math.pi)
    across=-(start[0]-x1)*math.sin(line_yaw)+(start[1]-y1)*math.cos(line_yaw)
    # No alignment recursion for an already aligned pose, nor for a vehicle
    # still in the channel. Exits use the time-reversed short-arc bank.
    if abs(across)>.25 or abs(_wrap(start[2]-line_yaw))>math.radians(25.):return None
    reference=getattr(grid,'fast_turn90_reference_goal',None)
    policy=stop_policy(grid,reference if reference is not None else front_goal if front_goal is not None else target,radius)
    cross_trigger=.003 if policy is None else policy['align_cross_track']
    angle_trigger=.3 if policy is None else policy['align_heading_deg']
    if getattr(grid,'fast_turn90_request_role',None)=='stage' and policy is not None:
        angle_trigger=min(angle_trigger,policy['stage_plan_heading_deg'])
    if abs(across)<cross_trigger and abs(_wrap(start[2]-line_yaw))<math.radians(angle_trigger):return None
    t=max(0.,min(1.,((start[0]-x1)*dx+(start[1]-y1)*dy)/(length*length)))
    projected=(x1+t*dx,y1+t*dy)
    options=[]
    for shift in (.0,.3,-.3,.6,-.6,1.,-1.,1.5,-1.5,2.,-2.):
        aligned=(projected[0]+shift*math.cos(line_yaw),projected[1]+shift*math.sin(line_yaw),line_yaw)
        if not grid.free(aligned):continue
        for legs in _csc_candidates(start,aligned,radius):
            changes,cost,first=_effort(legs,initial_direction)
            if start_reverse_only and first>0:continue
            if max_changes is not None and changes>max_changes:continue
            if sum(abs(d) for d,k in legs)>3.:continue
            options.append((cost,changes,aligned,legs))
    options.sort(key=lambda item:item[0])
    for cost,changes,aligned,legs in options:
        if cancel():raise RuntimeError('cancelled')
        if time.time()>=deadline:break
        prefix=_checked_legs(grid,start,legs,deadline,cancel)
        if not prefix or len(prefix)<2:continue
        remaining_limit=None if max_changes is None else max_changes-changes
        detail={}
        suffix=plan_turn90_candidates(grid,prefix[-1][:3],target,radius,deadline-time.time(),cancel,
                                      position_tolerance,heading_tolerance,front_goal,False,remaining_limit,
                                      prefix[-1][3],detail,allow_alignment=False,allow_goal_offsets=False)
        if suffix is None:continue
        route=prefix+suffix[1:]
        travel=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(route,route[1:]))
        if travel>max_length:continue
        if diagnostics is not None:
            diagnostics.update(detail)
            diagnostics.update(aligned_start=True,changes=changes+detail['changes'],cost=cost+detail['cost'])
        return route
    return None


def _plan_exact(grid, start, goal, radius, max_seconds=.35,
                           cancel=lambda: False, position_tolerance=.025,
                           heading_tolerance=math.radians(1.5), front_goal=None,
                           start_reverse_only=False, max_changes=None,
                           initial_direction=0, diagnostics=None,allow_alignment=True):
    """Return a checked candidate or None, leaving Hybrid A* as the fallback."""
    started=time.time()
    max_changes=direction_change_limit(grid,max_changes)
    from nav_config import P, from_rear
    if not all(not math.isnan(v) and not math.isinf(v)
               for v in list(start) + list(goal) + [radius, max_seconds]):
        raise ValueError('nonfinite turn90 input')
    if radius <= 0 or max_seconds <= 0:
        return None
    if radius < P['global_min_radius']:
        raise ValueError('turn90 radius below configured global minimum')
    if cancel():
        raise RuntimeError('cancelled')
    target = tuple(goal)
    if front_goal is not None:
        offset = from_rear((0., 0., goal[2]), 'goal')
        target = (front_goal[0] - offset[0], front_goal[1] - offset[1], goal[2])
    selection=space_profile(grid,target,radius,front_goal)
    checked_grid = grid.snapshot()
    checked_grid.search_bounds = None
    def safe(pose):
        return checked_grid.free(pose) and not math.isinf(checked_grid.boundary_cost(pose))
    if not safe(start) or not safe(target):return None
    deadline=started+max_seconds
    ratio = getattr(grid, 'fast_turn90_max_detour_ratio', 3.)
    if not (ratio > 0 and not math.isnan(ratio) and not math.isinf(ratio)):
        raise ValueError('invalid fast turn detour ratio')
    max_length = max((6. if selection['profile']=='small' else 4.) * radius,
                     ratio * math.hypot(target[0]-start[0],target[1]-start[1]))
    align_enabled=(selection['profile']=='small' or
                   (selection['profile'] in ('medium','large') and getattr(grid,'fast_turn90_enabled',False)
                    and getattr(grid,'fast_turn90_tolerances_enabled',True)))
    if align_enabled and allow_alignment:
        # Reserve most of the shared budget for direct templates. A failed
        # alignment prefix must not consume the entire quick-candidate slot.
        align_deadline=min(deadline,time.time()+min(.12,max_seconds*.30))
        if selection['profile']=='small' and getattr(grid,'fast_turn90_many_cusps_enabled',False):
            align_deadline=deadline  # Historical experimental library only.
        aligned=_align_small(checked_grid,start,target,radius,selection,align_deadline,cancel,
                             position_tolerance,heading_tolerance,front_goal,start_reverse_only,
                             max_changes,initial_direction,diagnostics,max_length)
        if aligned is not None:
            if cancel():raise RuntimeError('cancelled')
            if time.time()<=deadline:return aligned
        # End alignment is the time-reversed counterpart of start alignment.
        # It handles exits ending at a slanted goal without changing that goal.
        detail={}
        reverse=_align_small(checked_grid,target,start,radius,selection,align_deadline,cancel,
                             position_tolerance,heading_tolerance,None,False,
                             max_changes,0,detail,max_length)
        if reverse is not None:
            route=[tuple(reverse[-1][:3])+(0,0.)]
            for index in range(len(reverse)-2,-1,-1):
                edge=reverse[index+1]
                route.append(tuple(reverse[index][:3])+(-edge[3],edge[4]))
            previous=initial_direction;changes=0
            for point in route[1:]:
                if previous and previous!=point[3]:changes+=1
                previous=point[3]
            end=from_rear(route[-1][:3],'goal') if front_goal is not None else route[-1][:3]
            requested=front_goal if front_goal is not None else goal
            if ((not start_reverse_only or route[1][3]<0) and
                    math.hypot(end[0]-requested[0],end[1]-requested[1])<=position_tolerance and
                    abs(_wrap(end[2]-requested[2]))<=heading_tolerance and
                    (max_changes is None or changes<=max_changes)):
                if cancel():raise RuntimeError('cancelled')
                if time.time()<=deadline:
                    if diagnostics is not None:
                        diagnostics.update(detail);diagnostics.update(aligned_end=True,changes=changes)
                    return route
    cache_key=(selection['profile'],1 if _wrap(target[2]-start[2])>=0 else -1,round(radius,6))
    cache_enabled=getattr(grid,'fast_turn90_cache_enabled',True)
    with _RECIPE_LOCK:
        recipes=list(_RECIPES.get(cache_key,())) if cache_enabled else []
    extra=[]
    if selection['profile']=='small' and getattr(grid,'fast_turn90_many_cusps_enabled',False):
        extra=_balanced_recipes(grid,start,target,radius,selection)
        # Exiting a channel is the time-reversed form of entering it. Re-fit
        # to the real endpoints, preserving steering curvature and gear signs.
        extra.extend(tuple((fraction,-direction,scale) for fraction,direction,scale in reversed(recipe))
                     for recipe in _balanced_recipes(grid,target,start,radius,selection))
    templates = candidates(start, target, radius, max_changes,
                           start_reverse_only, initial_direction,selection['profile'],recipes,extra)
    # A cheap connector must not obtain its speed by choosing an arbitrarily
    # long excursion. Close-position 90-degree moves still need space to turn.
    templates = [option for option in templates
                 if sum(abs(d) for d,k in option[2]) <= max_length]
    if diagnostics is not None:
        diagnostics.update(candidates=len(templates), checked=0, hit=False,
                           profile=selection['profile'],line_distance=selection['distance'],
                           entry_distance=selection.get('entry_distance'),classification_basis=selection.get('classification_basis'),
                           cached_recipes=len(recipes))
    if not templates:
        return None
    # A search window is an acceleration hint, not a collision boundary. Do not
    # mutate the caller's grid or relax any physical/map constraints.
    for changes, cost, legs in templates:
        if cancel():
            raise RuntimeError('cancelled')
        if time.time() >= deadline:
            break
        if diagnostics is not None:
            diagnostics['checked'] += 1
        # Reject definite collisions cheaply before densely sampling the whole
        # candidate. This never accepts a route: the original swept checks below
        # remain mandatory for every surviving leg.
        coarse=tuple(start)
        coarse_clear=True
        for distance,curvature in legs:
            if not all(safe(_advance(coarse,distance*f,curvature))
                       for f in (.25,.5,.75,1.)):
                coarse_clear=False;break
            coarse=_advance(coarse,distance,curvature)
        if not coarse_clear:continue
        pose, route, valid = tuple(start), [tuple(start) + (0, 0.)], True
        for distance, curvature in legs:
            if cancel():
                raise RuntimeError('cancelled')
            if time.time() >= deadline:
                valid = False
                break
            points = checked_grid.arc(pose, distance, curvature)
            if points is None or any(math.isinf(checked_grid.boundary_cost(p)) for p in points):
                valid = False
                break
            direction = 1 if distance > 0 else -1
            route.extend(tuple(point) + (direction, curvature) for point in points)
            pose = tuple(points[-1])
        if not valid:
            continue
        if cancel():
            raise RuntimeError('cancelled')
        if time.time()>deadline:break
        endpoint = from_rear(pose, 'goal') if front_goal is not None else pose
        requested = front_goal if front_goal is not None else goal
        if (math.hypot(endpoint[0] - requested[0], endpoint[1] - requested[1])
                > position_tolerance
                or abs(_wrap(endpoint[2] - requested[2])) > heading_tolerance):
            continue
        if diagnostics is not None:
            found_recipe=_recipe_from_legs(legs,_wrap(target[2]-start[2]),radius)
            def recipe_key(item):return tuple((round(a,6),d,round(r,6)) for a,d,r in item)
            diagnostics.update(hit=True, changes=changes, cost=cost,
                               cache_reused=found_recipe is not None and any(recipe_key(found_recipe)==recipe_key(item) for item in recipes),
                               elapsed_s=max_seconds - max(0., deadline - time.time()))
        recipe=_recipe_from_legs(legs,_wrap(target[2]-start[2]),radius)
        if cache_enabled and recipe is not None:
            with _RECIPE_LOCK:
                previous=list(_RECIPES.pop(cache_key,()))
                _RECIPES[cache_key]=[recipe]+[item for item in previous if item!=recipe][:7]
                while len(_RECIPES)>32:_RECIPES.popitem(last=False)
        return route
    return None


def plan_turn90_candidates(grid,start,goal,radius,max_seconds=.35,cancel=lambda:False,
                           position_tolerance=.025,heading_tolerance=math.radians(1.5),front_goal=None,
                           start_reverse_only=False,max_changes=None,initial_direction=0,diagnostics=None,
                           allow_alignment=True,allow_goal_offsets=True):
    """Fit checked routes inside an explicitly configured positional/yaw region."""
    from nav_config import from_rear
    role=getattr(grid,'fast_turn90_request_role',None)
    use_region=(allow_goal_offsets and role in ('stage','final') and
                getattr(grid,'fast_turn90_tolerances_enabled',True))
    if not use_region:
        return _plan_exact(grid,start,goal,radius,max_seconds,cancel,position_tolerance,
                           heading_tolerance,front_goal,start_reverse_only,max_changes,
                           initial_direction,diagnostics,allow_alignment)
    deadline=time.time()+max(0.,max_seconds)
    requested=tuple(front_goal if front_goal is not None else goal)
    targets=[(0.,0.,goal[2])]
    selected=space_profile(grid,goal,radius,front_goal)
    angles=[goal[2]]
    if selected['line'] is not None:
        (x1,y1),(x2,y2)=grid.zero_cost_line[selected['line']]
        line_angle=math.atan2(y2-y1,x2-x1)
        along=min((line_angle,_wrap(line_angle+math.pi)),key=lambda a:abs(_wrap(a-goal[2])))
        if abs(_wrap(along-goal[2]))<=math.radians(25.):
            change=max(-heading_tolerance,min(heading_tolerance,_wrap(along-goal[2])))
            angles.append(_wrap(goal[2]+change))
    angles.extend(_wrap(goal[2]+sign*heading_tolerance*fraction)
                  for fraction in (.5,1.) for sign in (-1,1))
    targets.extend((0.,0.,angle) for angle in angles[1:])
    step=position_tolerance*.8
    # Position candidates stay inside the caller's circular tolerance, including
    # diagonal samples. Each is solved anew; returned endpoints are never moved.
    if role!='final' or getattr(grid,'fast_turn90_final_position_offsets_enabled',False):
        targets.extend((step*math.cos(a),step*math.sin(a),goal[2])
                       for a in (0.,math.pi/2,math.pi,3*math.pi/2))
    seen=set()
    for index,(dx,dy,yaw) in enumerate(targets):
        if cancel():raise RuntimeError('cancelled')
        left=deadline-time.time()
        if left<=0:break
        key=(round(dx,7),round(dy,7),round(yaw,7))
        if key in seen:continue
        seen.add(key)
        budget=min(1.2,left) if index==0 else left/max(1,len(targets)-index)
        candidate=(goal[0]+dx,goal[1]+dy,yaw)
        candidate_front=None if front_goal is None else (front_goal[0]+dx,front_goal[1]+dy,front_goal[2])
        detail={}
        defer_alignment=(index==0 and selected['profile'] in ('medium','large') and allow_alignment)
        route=_plan_exact(grid,start,candidate,radius,budget,cancel,position_tolerance,
                          heading_tolerance,candidate_front,start_reverse_only,max_changes,
                          initial_direction,detail,allow_alignment and not defer_alignment)
        if route is None and defer_alignment and deadline>time.time():
            route=_plan_exact(grid,start,candidate,radius,min(.12,deadline-time.time()),cancel,
                              position_tolerance,heading_tolerance,candidate_front,start_reverse_only,
                              max_changes,initial_direction,detail,True)
        if route is None:continue
        end=from_rear(route[-1][:3],'goal') if front_goal is not None else route[-1][:3]
        distance=math.hypot(end[0]-requested[0],end[1]-requested[1])
        angle=abs(_wrap(end[2]-requested[2]))
        if distance>position_tolerance+1e-8 or angle>heading_tolerance+1e-8:continue
        if cancel():raise RuntimeError('cancelled')
        if time.time()>deadline:break
        if diagnostics is not None:
            diagnostics.update(detail,goal_position_error=distance,goal_heading_error_deg=math.degrees(angle),
                               goal_region=True)
        return route
    return None
