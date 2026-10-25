"""Direction-aware rear-axle tracking of the existing, unsmoothed global path."""
import math
from ackermann_core import wrap
from nav_config import P


class PreparedPath(tuple):
    """Immutable direction segment with O(1) remaining-distance lookup."""
    def __new__(cls, points):
        obj = tuple.__new__(cls, (tuple(p) for p in points))
        lengths = tuple(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(obj,obj[1:]))
        suffix = [0.] * len(obj)
        for i in range(len(lengths)-1, -1, -1):
            suffix[i] = suffix[i+1] + lengths[i]
        obj.edge_lengths, obj.suffix = lengths, tuple(suffix)
        return obj


def motion_observed(current, previous, dt, wheel_speed):
    """Recognize intentional slow tracking without treating pose jitter as drive."""
    translation = math.hypot(current[0]-previous[0],current[1]-previous[1])/dt
    rotation = abs(wrap(current[2]-previous[2]))/dt
    return abs(wheel_speed) > .003 and (translation > .003 or rotation > .005)


def project(pose, segment, index):
    """Project onto nearby edges in the current direction segment only.

    A path point's curvature describes the edge arriving at that point. Vehicle
    heading is not reversed for reverse travel; signed velocity handles that.
    """
    if len(segment) < 2:
        raise ValueError('tracking needs two path samples')
    index = max(0, min(index, len(segment)-2))
    start, stop = max(0,index-2), min(len(segment)-1,index+35)
    best = None
    for i in range(start, stop):
        a,b = segment[i],segment[i+1]
        dx,dy = b[0]-a[0],b[1]-a[1]
        length = math.hypot(dx,dy)
        if length < 1e-9:
            continue
        t = max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(length*length)))
        x,y = a[0]+t*dx,a[1]+t*dy
        d2 = (pose[0]-x)**2+(pose[1]-y)**2
        if (best is None or d2 < best[0]-1e-12
                or (abs(d2-best[0]) <= 1e-12 and best[2] >= 1.-1e-9 and t <= 1e-9)):
            yaw = a[2]+t*wrap(b[2]-a[2])
            best = (d2,i,t,length,x,y,yaw,b[4])
    if best is None:
        raise ValueError('direction segment has no nonzero edges')
    _,i,t,length,x,y,yaw,k = best
    tail = (segment.suffix[i+1] if isinstance(segment, PreparedPath) else
            sum(math.hypot(b[0]-a[0],b[1]-a[1])
                for a,b in zip(segment[i+1:],segment[i+2:])))
    remaining = (1.-t)*length + tail
    lateral = -(pose[0]-x)*math.sin(yaw)+(pose[1]-y)*math.cos(yaw)
    return dict(index=i, remaining=remaining, lateral=lateral,
                heading=wrap(pose[2]-yaw), curvature=k, fraction=t,
                edge_length=length, distance=math.sqrt(best[0]))


def steering_limits(radius):
    left=max(radius,P['local_left_min_radius'])
    right=max(radius,P['local_right_min_radius'])
    return -math.atan(P['wheelbase']/right),math.atan(P['wheelbase']/left)


def steering(reference, direction, radius, lateral_gain=6.0, heading_gain=4.):
    """Curvature feedforward plus signed Frenet-error feedback.

    With signed speed v, e_y_dot ~= v*e_heading. Multiplying the heading
    damping by direction keeps it stabilizing during reverse motion too.
    """
    ey, eh, k = reference['lateral'], reference['heading'], reference['curvature']
    feedforward = k*math.cos(eh)/max(.25,1.-k*ey)
    requested = feedforward-lateral_gain*ey-direction*heading_gain*math.sin(eh)
    lower,upper=steering_limits(radius)
    requested = max(math.tan(lower)/P['wheelbase'],min(math.tan(upper)/P['wheelbase'],requested))
    return math.atan(P['wheelbase']*requested)


def tracking_steering(ref, segment, radius, lateral_gain=6., heading_gain=4.,
                      steering_response=None, reference_speed=None):
    """One steering target for stationary preparation and moving tracking."""
    sign = segment[-1][3]
    # Last part of the fixed reference: align heading earlier so the front-axle
    # goal does not retain a large lateral error when the rear reaches the end.
    # Blend continuously, preserve all steering/speed bounds and the path.
    weight=max(0.,1.-ref['remaining']/P['terminal_alignment_span'])
    terminal_heading_gain=heading_gain*(1.+weight*(P['terminal_heading_gain_scale']-1.))
    control_ref = dict(ref)
    preview=getattr(steering_response,'preview',None) if steering_response is not None else None
    if preview is not None:
        saved,j,k=preview
        valid=(saved is segment and 1<=j<len(segment) and segment[j][3]==sign
               and segment[j-1][3]==sign and abs(segment[j][4]-k)<1e-9
               and reference_speed is not None and reference_speed*sign>=0.
               and P['steering_preview_distance']>0.)
        if valid:
            remaining=(segment.suffix[j-1] if isinstance(segment,PreparedPath) else
                       sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(segment[j-1:],segment[j:])))
            valid=abs(ref['remaining']-remaining)<=P['steering_preview_distance']+1e-8
        if not valid:
            preview=None;steering_response.preview=None
    # Only advance curvature feedforward. Measured lateral/heading errors and
    # the original path remain unchanged; never look across a gear-change cusp.
    if (steering_response is not None and reference_speed is not None
            and reference_speed*sign >= 0. and P['steering_preview_distance'] > 0.):
        distance = (1.-ref['fraction'])*ref['edge_length']
        original = math.atan(P['wheelbase']*ref['curvature'])
        for j in range(ref['index']+2, len(segment)):
            if distance > P['steering_preview_distance'] or segment[j][3] != sign:
                break
            target = math.atan(P['wheelbase']*segment[j][4])
            if abs(target-original) > .025:
                delay, rate = steering_response.model.timing(target-original)
                horizon = min(P['steering_preview_time'], delay + .5*abs(target-original)/rate)
                travel = min(P['steering_preview_distance'], abs(reference_speed)*horizon)
                # Hold the selected geometric transition, not its final angle:
                # lateral/heading feedback and preload can change that angle.
                # The segment identity and bounded arc distance prevent reuse
                # across a replan, cusp, unrelated transition or distant edge.
                committed=(preview is not None and preview[1]==j and abs(preview[2]-segment[j][4])<1e-9)
                if (reference_speed*sign > 0. and distance <= travel) or committed:
                    control_ref['curvature'] = segment[j][4]
                    steering_response.preview=(segment,j,segment[j][4])
                break
            a,b = segment[j-1],segment[j]
            distance += math.hypot(b[0]-a[0],b[1]-a[1])
    delta = steering(control_ref,sign,radius,lateral_gain,terminal_heading_gain)
    return delta, control_ref['curvature']


def tracking_command(pose, segment, index, radius, actual_steer=0., steer_rate=1.,
                     lateral_gain=6.0, heading_gain=4., preview_distance=.35,steering_response=None,
                     reference_speed=None):
    ref = project(pose,segment,index)
    sign = segment[-1][3]
    delta, command_curvature = tracking_steering(ref,segment,radius,lateral_gain,heading_gain,
                                               steering_response,reference_speed)
    ref['command_curvature'] = command_curvature
    approach=math.sqrt(2.*P['approach_braking']*max(0.,ref['remaining']-P['approach_stop_reserve']))
    base = min(P['tracking_forward_speed'] if sign > 0 else P['tracking_reverse_speed'],max(P['tracking_min_speed'],approach))
    # Reduce travel while the steering catches up or tracking errors grow.
    error_scale = max(.25,1./(1.+10.*abs(ref['lateral'])+3.*abs(ref['heading'])))
    speed = base*error_scale/(1.+3.*abs(delta-actual_steer))
    speed = min(speed,base/(1.+P['curve_speed_gain']*abs(ref['curvature'])))
    # A nominal floor applies only to well-aligned tracking; physical/collision
    # speed caps and the steering-lag bound below can still require lower speed.
    if abs(ref['lateral'])<=.02 and abs(ref['heading'])<=math.radians(3.) and abs(delta-actual_steer)<=.05236:
        speed=max(speed,min(P['tracking_cruise_floor'],base/(1.+P['curve_speed_gain']*abs(ref['curvature']))))
    if (abs(ref['curvature'])>.15 or abs(delta)>.09 or abs(actual_steer)>.09
            or abs(ref['lateral'])>.015 or abs(ref['heading'])>math.radians(3.)):
        speed=min(speed,P['tracking_turn_speed_cap'])
    # Bound travel while measured steering catches up during motion. The real
    # bridge handles zero-speed steering requests separately before departure.
    lag = abs(delta-actual_steer)
    if lag > .08:
        response_time=steering_response.completion_time(delta) if steering_response is not None else lag/steer_rate
        speed = min(speed, P['tracking_turn_allowance']/max(response_time,.05))
    # Slow before a curvature transition; do not move/smooth the global line.
    distance = (1.-ref['fraction'])*ref['edge_length']
    last_delta = math.atan(P['wheelbase']*ref['curvature'])
    for j in range(ref['index']+2,len(segment)):
        if distance > preview_distance:
            break
        next_delta = math.atan(P['wheelbase']*segment[j][4])
        change = abs(next_delta-last_delta)
        if change > .025:
            response_time=steering_response.completion_time(next_delta) if steering_response is not None else change/steer_rate+.25
            speed = min(speed,max(.01,distance/max(response_time,.05)))
        last_delta = next_delta
        a,b = segment[j-1],segment[j]
        distance += math.hypot(b[0]-a[0],b[1]-a[1])
    return sign*speed, delta, ref


def replay_command(reference, direction, speed_cap=None, speed_scale=1.):
    """Replay the selected global edge without steering corrections."""
    base = (P['replay_forward_speed'] if direction > 0 else P['replay_reverse_speed'])*speed_scale
    if speed_cap is not None and speed_cap > 0:
        base = min(base, speed_cap)
    speed = min(base, max(P['replay_min_speed'], reference['remaining']*P['approach_speed_gain']))
    return direction*speed, reference['curvature']
