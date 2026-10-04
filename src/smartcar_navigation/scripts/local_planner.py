"""Signed local rollout with shared hard geometry and explainable costs."""
import math
from ackermann_core import wrap
from nav_config import P
from actuator_model import slew, predict_arc
from path_tracking import tracking_command


def path_error(p, reference):
    """Distance to line segments, not distance to sparsely sampled vertices."""
    best = None
    for a,b in zip(reference, reference[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(1e-12,dx*dx+dy*dy)))
        d=math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
        yaw=a[2]+t*wrap(b[2]-a[2])
        if best is None or d<best[0]: best=(d,abs(wrap(yaw-p[2])))
    if best is None:
        a=reference[0];return math.hypot(p[0]-a[0],p[1]-a[1]),abs(wrap(a[2]-p[2]))
    return best


def path_progress(p, reference):
    best, accumulated = None, 0.
    for a,b in zip(reference,reference[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(1e-12,length*length)))
        error=math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
        if best is None or error<best[0]: best=(error,accumulated+t*length)
        accumulated+=length
    return best[1] if best else 0.


def clearance_cost(grid, pose):
    # Multi-band soft cost; never changes hard feasibility.
    return sum(weight for extra,weight in ((.01,.5),(.02,.3),(.04,.2))
               if grid.collision(pose,extra=extra))


def choose(grid, pose, segment, index, radius, previous_steer, diagnostics=None, speed_cap=None,
           control_dt=.05, current_speed=None, steering_rate=1., actuator_steer_rate=2.5,
           acceleration=.30, braking=.50, lateral_gain=6.0, heading_gain=4., preview_distance=.35, maneuver_mode='NORMAL',
           wall_steer=None, wall_weight=0.):
    # previous_steer is measured equivalent steering, NOT the previous command.
    values = (radius, previous_steer, control_dt, steering_rate, actuator_steer_rate,
              acceleration, braking, lateral_gain, heading_gain, preview_distance)
    if any(math.isnan(x) or math.isinf(x) for x in values) or min(radius, control_dt, steering_rate, actuator_steer_rate, acceleration, braking) <= 0 or control_dt > P['max_control_dt']:
        raise ValueError('invalid actuator model or control interval')
    if min(lateral_gain,heading_gain,preview_distance) <= 0:
        raise ValueError('tracking gains and preview distance must be positive')
    rate = min(steering_rate,actuator_steer_rate)
    nominal, preferred, projected = tracking_command(pose,segment,index,radius,previous_steer,rate,
                                                    lateral_gain,heading_gain,preview_distance)
    if not 0 <= wall_weight <= 1 or (wall_steer is not None and (math.isnan(wall_steer) or math.isinf(wall_steer))):
        raise ValueError('invalid wall steering preference')
    def straight_preference(angle, reference):
        # A STRAIGHT goal can still contain curved path segments. Do not dilute
        # their curvature feedforward. Apply any bias BEFORE slew and rollout.
        if maneuver_mode=='STRAIGHT' and wall_steer is not None and abs(reference['curvature'])<1e-6:
            return (1.-wall_weight)*angle+wall_weight*wall_steer
        return angle
    preferred=straight_preference(preferred,projected)
    index,remaining = projected['index'],projected['remaining']
    sign = 1 if nominal > 0 else -1
    limit = math.atan(P['wheelbase']/radius)
    # Small corrections around the feedback command, not a new route search.
    targets = [preferred, preferred-P['steering_sample_offset'], preferred+P['steering_sample_offset']]
    best = None
    reference = segment[max(0,index-15):min(len(segment),index+65)]
    initial_error=path_error(pose,reference)[0]
    initial_progress=path_progress(pose,reference)
    measured_speed = nominal if current_speed is None else current_speed
    if math.isnan(measured_speed) or math.isinf(measured_speed):
        raise ValueError('nonfinite measured speed')
    counts={}; examples={}
    def reject(reason, detail=''):
        counts[reason]=counts.get(reason,0)+1
        if detail: examples.setdefault(reason,detail)
    if measured_speed * sign < -.005:
        reject('DIRECTION_NOT_STOPPED', 'wheel feedback still moving in opposite direction')
        if diagnostics is not None:
            diagnostics.update(counts=counts, examples=examples)
        return None
    cap = abs(speed_cap) if speed_cap is not None else abs(nominal)
    evaluated=set()
    for speed in (min(abs(nominal),cap),min(abs(nominal)*.65,cap),min(abs(nominal)*.4,cap)):
        v=sign*speed
        for target in targets:
            target=max(-limit,min(limit,target))
            first=slew(previous_steer,target,min(steering_rate,actuator_steer_rate),control_dt)
            action=(round(v,12),round(first,12))
            if action in evaluated:
                continue
            evaluated.add(action)
            first_k=math.tan(first)/P['wheelbase']
            # Preserve the previous conservative stopping guard from the CURRENT
            # vehicle pose, not an additional 18 cm beyond a 1-second rollout.
            distance=sign*max(.02,max(speed,abs(measured_speed))*.15)
            if (not grid.free(pose) or grid.arc(pose,distance,first_k) is None
                    or grid.arc(pose,distance,math.tan(previous_steer)/P['wheelbase']) is None):
                reject('STOPPING_COLLISION',grid.blocked_detail(pose,distance,first_k));continue
            p,steer,path,travel=pose,previous_steer,[],0.
            predicted_speed=measured_speed
            errors=[];clearances=[];valid=True
            rollout_index=index
            for step in range(P['rollout_steps']):
                dt=control_dt if step==0 else P['rollout_dt']
                # First command is the sampled action; future commands follow
                # the upcoming curve, rather than extrapolating one fixed arc.
                desired=first
                future_v = v
                if step >= 1:
                    future_v,future_delta,future_ref=tracking_command(p,segment,rollout_index,radius,steer,rate,
                                                                    lateral_gain,heading_gain,preview_distance)
                    rollout_index=future_ref['index']
                    future_delta=straight_preference(future_delta,future_ref)
                    future_v=sign*min(speed,abs(future_v))
                    desired=slew(steer,future_delta,rate,dt)
                predicted=predict_arc(grid,p,predicted_speed,steer,future_v,desired,dt,
                                      acceleration,braking,actuator_steer_rate,max(0.,remaining-travel))
                if predicted is None:
                    reject('ROLLOUT_COLLISION','actuator-limited swept trajectory blocked');valid=False;break
                p,predicted_speed,steer,distance=predicted
                path.append(p);travel+=distance
                errors.append(path_error(p,reference)[0])
                if step%4==0: clearances.append(clearance_cost(grid,p))
                if travel>=remaining-1e-6: break
            if not valid: continue
            deviation,heading=path_error(p,reference)
            # Match the 20 cm controller bound; permit recovery from 18..20 cm
            # only when improving, instead of rejecting every candidate.
            if max(errors)>P['tracking_max_error'] or (initial_error>P['tracking_recover_error'] and deviation>=initial_error-1e-5):
                reject('TRACKING_ENVELOPE','initial=%.3fm end=%.3fm max=%.3fm'%(initial_error,deviation,max(errors)));continue
            progress=path_progress(p,reference)-initial_progress
            # Tracking dominates; avoid suppressing a necessary sign reversal
            # or trading a large tracking error for clearance/progress.
            mode_lateral = 1.0 if maneuver_mode in ('LATERAL','LATERAL_TURN_180') else 4.0
            mode_heading = .15 if maneuver_mode == 'STRAIGHT' else .35
            score=(mode_lateral*sum(errors)/len(errors)+2*deviation+mode_heading*heading
                   +.015*sum(clearances)/max(1,len(clearances))
                   +.04*abs(first-preferred)-.2*progress)
            if best is None or score<best[0]: best=(score,v,first_k,first,path)
    if diagnostics is not None:
        diagnostics.update(counts=counts,examples=examples)
    return best


def explain(diagnostics):
    return '; '.join('%s=%d (%s)'%(key,value,diagnostics['examples'].get(key,''))
                     for key,value in sorted(diagnostics['counts'].items()))


def guarded_track(grid, pose, segment, index, radius, previous_steer, diagnostics=None,
                  speed_cap=None, control_dt=.05, current_speed=None, steering_rate=1.,
                  actuator_steer_rate=2.5, acceleration=.30, braking=.50,
                  lateral_gain=6., heading_gain=4., preview_distance=.35,
                  maneuver_mode='NORMAL', wall_steer=None, wall_weight=0., guard_distance=.10):
    """Track one path; near obstacles validate response plus a complete stop.

    The broad phase encloses the motion of every footprint point through the
    command hold and braking interval. Clear space needs only one query. Near
    walls we test three speeds of the same steering command, never a competing
    local route. Braking is simulated beyond the end of the reference path.
    """
    values = (radius, previous_steer, control_dt, steering_rate, actuator_steer_rate,
              acceleration, braking, lateral_gain, heading_gain, preview_distance, guard_distance)
    if (any(math.isnan(x) or math.isinf(x) for x in values)
            or min(radius, control_dt, steering_rate, actuator_steer_rate, acceleration,
                   braking, lateral_gain, heading_gain, preview_distance) <= 0
            or control_dt > P['max_control_dt'] or guard_distance < 0):
        raise ValueError('invalid tracking actuator model or control interval')
    rate = min(steering_rate, actuator_steer_rate)
    nominal, desired, ref = tracking_command(pose, segment, index, radius, previous_steer,
                                            rate, lateral_gain, heading_gain, preview_distance)
    measured = nominal if current_speed is None else current_speed
    if math.isnan(measured) or math.isinf(measured):
        raise ValueError('nonfinite measured speed')
    if not 0 <= wall_weight <= 1 or (wall_steer is not None and (math.isnan(wall_steer) or math.isinf(wall_steer))):
        raise ValueError('invalid wall steering preference')
    if maneuver_mode == 'STRAIGHT' and wall_steer is not None and abs(ref['curvature']) < 1e-6:
        desired = (1-wall_weight)*desired + wall_weight*wall_steer
    limit = math.atan(P['wheelbase']/radius)
    first = slew(previous_steer, max(-limit, min(limit, desired)), rate, control_dt)
    curvature = math.tan(first)/P['wheelbase']
    sign = 1 if nominal > 0 else -1
    counts, examples = {}, {}
    def reject(reason, detail):
        counts[reason] = counts.get(reason, 0)+1
        examples.setdefault(reason, detail)
        if diagnostics is not None:
            diagnostics.update(counts=counts, examples=examples)
    if sign*measured < -.005:
        reject('DIRECTION_NOT_STOPPED', 'wheel feedback moving in opposite direction')
        return None
    cap = abs(nominal) if speed_cap is None else abs(speed_cap)
    if math.isnan(cap) or math.isinf(cap):
        raise ValueError('nonfinite speed cap')
    hold = max(control_dt, P['max_control_dt']) + .20  # two 10 Hz scans to confirm hits
    for scale in (1., .65, .4):
        v = sign*min(abs(nominal)*scale, cap)
        peak = max(abs(measured), abs(v))
        distance = peak*hold + peak*peak/(2*braking)
        # A rotating footprint point travels further than the rear axle.
        max_k = max(abs(math.tan(previous_steer)), abs(math.tan(first)))/P['wheelbase']
        body_radius = math.hypot(P['collision_center_x']+P['collision_half_length']+grid.margin+grid.res*1.207107,
                                 grid.half_width+grid.margin+grid.res*1.207107)
        guard = max(guard_distance, distance*(1+body_radius*max_k))
        if not grid.collision(pose, extra=guard):
            if diagnostics is not None:
                diagnostics.update(counts=counts, examples=examples, controller='clear_path')
            return (0., v, curvature, first, [pose])
        if not grid.free(pose):
            reject('CURRENT_COLLISION', grid.blocked_detail(pose))
            return None
        response = predict_arc(grid, pose, measured, previous_steer, v, first, hold,
                               acceleration, braking, actuator_steer_rate)
        if response is None:
            reject('RESPONSE_COLLISION', 'command response swept footprint blocked')
            continue
        end, speed, steer, _ = response
        stopped = predict_arc(grid, end, speed, steer, 0., first, abs(speed)/braking+.02,
                              acceleration, braking, actuator_steer_rate)
        if stopped is None:
            reject('BRAKING_COLLISION', 'complete braking swept footprint blocked')
            continue
        if diagnostics is not None:
            diagnostics.update(counts=counts, examples=examples, controller='near_wall')
        return (0., v, curvature, first, [end, stopped[0]])
    return None
