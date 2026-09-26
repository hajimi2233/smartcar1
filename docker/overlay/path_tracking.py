"""Direction-aware rear-axle tracking of the existing, unsmoothed global path."""
import math
from ackermann_core import wrap


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
    remaining = (1.-t)*length + sum(math.hypot(b[0]-a[0],b[1]-a[1])
                                   for a,b in zip(segment[i+1:],segment[i+2:]))
    lateral = -(pose[0]-x)*math.sin(yaw)+(pose[1]-y)*math.cos(yaw)
    return dict(index=i, remaining=remaining, lateral=lateral,
                heading=wrap(pose[2]-yaw), curvature=k, fraction=t,
                edge_length=length, distance=math.sqrt(best[0]))


def steering(reference, direction, radius, lateral_gain=6.0, heading_gain=4.):
    """Curvature feedforward plus signed Frenet-error feedback.

    With signed speed v, e_y_dot ~= v*e_heading. Multiplying the heading
    damping by direction keeps it stabilizing during reverse motion too.
    """
    ey, eh, k = reference['lateral'], reference['heading'], reference['curvature']
    feedforward = k*math.cos(eh)/max(.25,1.-k*ey)
    requested = feedforward-lateral_gain*ey-direction*heading_gain*math.sin(eh)
    requested = max(-1./radius,min(1./radius,requested))
    return math.atan(.62*requested)


def tracking_command(pose, segment, index, radius, actual_steer=0., steer_rate=1.,
                     lateral_gain=6.0, heading_gain=4., preview_distance=.35):
    ref = project(pose,segment,index)
    sign = segment[-1][3]
    delta = steering(ref,sign,radius,lateral_gain,heading_gain)
    base = min(.15 if sign > 0 else .08,max(.02,ref['remaining']*.4))
    # Reduce travel while the steering catches up or tracking errors grow.
    error_scale = max(.25,1./(1.+10.*abs(ref['lateral'])+3.*abs(ref['heading'])))
    speed = base*error_scale/(1.+3.*abs(delta-actual_steer))
    speed = min(speed,base/(1.+1.8*abs(ref['curvature'])))
    # Slow before a curvature transition; do not move/smooth the global line.
    distance = (1.-ref['fraction'])*ref['edge_length']
    last_delta = math.atan(.62*ref['curvature'])
    for j in range(ref['index']+2,len(segment)):
        if distance > preview_distance:
            break
        next_delta = math.atan(.62*segment[j][4])
        change = abs(next_delta-last_delta)
        if change > .025:
            speed = min(speed,max(.01,distance/(change/steer_rate+.25)))
        last_delta = next_delta
        a,b = segment[j-1],segment[j]
        distance += math.hypot(b[0]-a[0],b[1]-a[1])
    return sign*speed, delta, ref


def replay_command(reference, direction, speed_cap=None, speed_scale=1.):
    """Replay the selected global edge without steering corrections."""
    base = (.04 if direction > 0 else .03)*speed_scale
    if speed_cap is not None and speed_cap > 0:
        base = min(base, speed_cap)
    speed = min(base, max(.012, reference['remaining']*.4))
    return direction*speed, reference['curvature']
