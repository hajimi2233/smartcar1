"""Rear-axle actuator feedback and prediction; no ROS or world-pose input."""
import math
from nav_config import P


class DirectionPersistence(object):
    """Require consecutive fresh opposite-speed samples; duplicate reads add no time."""
    def __init__(self, deadband, duration, max_gap):
        self.deadband, self.duration, self.max_gap = deadband, duration, max_gap
        self.reset()

    def reset(self):
        self.key = self.stamp = self.since = None
        self.confirmed = False

    def update(self, direction, speed, stamp, now, key):
        if (direction not in (-1, 1) or any(math.isnan(x) or math.isinf(x)
                for x in (speed, stamp, now)) or not -.02 <= now-stamp <= self.max_gap):
            self.reset()
            return False
        key = (key, direction)
        if self.key != key or (self.stamp is not None and
                (stamp < self.stamp or stamp-self.stamp > self.max_gap)):
            self.reset()
        self.key = key
        if direction*speed >= -self.deadband:
            self.since = None
            self.confirmed = False
        elif stamp != self.stamp:
            if self.since is None:
                self.since = stamp
            self.confirmed = stamp-self.since >= self.duration-1e-9
        self.stamp = stamp
        return self.confirmed


def matched_motion_progress(samples, now, direction, window, minimum, freshness):
    """Positive proof only: several fresh map matches agree over a time span.

    Missing/duplicate/stale samples and large localization corrections are not
    permission to suppress a direction stop. No wheel pulses or command-speed
    assumptions enter this evidence.
    """
    if direction not in (-1,1):return None
    selected=[]
    for stamp,received,pose in samples:
        if not all(not math.isnan(v) and not math.isinf(v) for v in (stamp,received)+tuple(pose)):return None
        if now-window<=stamp<=now and (not selected or stamp>selected[-1][0]):
            selected.append((stamp,received,pose))
    if len(selected)<3 or now-selected[-1][0]>freshness:return None
    if selected[-1][0]-selected[0][0]<.4:return None
    start=selected[0][2];c,s=math.cos(start[2]),math.sin(start[2])
    steps=[]
    for a,b in zip(selected,selected[1:]):
        if b[0]-a[0]>freshness:return None
        if math.hypot(b[2][0]-a[2][0],b[2][1]-a[2][1])>.15:return None
        delta=(b[2][2]-a[2][2]+math.pi)%(2*math.pi)-math.pi
        if abs(delta)>.15:return None
        step=direction*(c*(b[2][0]-a[2][0])+s*(b[2][1]-a[2][1]))
        if step<-.005:return None
        steps.append(step)
    end=selected[-1][2];dx,dy=end[0]-start[0],end[1]-start[1]
    progress=direction*(c*dx+s*dy)
    lateral=abs(-s*dx+c*dy)
    turn=abs((end[2]-start[2]+math.pi)%(2*math.pi)-math.pi)
    if lateral>.04 or turn>math.radians(10.) or progress<minimum or steps[-1]<.005:return None
    return progress


def slew(current, target, rate, dt):
    return current + max(-rate * dt, min(rate * dt, target - current))


def steering_ready(wait,lag,speed,ready_limit,hold,now,hysteresis=0.):
    """Hysteresis stabilizes confirmation timing; release still uses original limit."""
    if abs(speed)>.005 or (wait.get('ready') is not None and now<wait['ready']):
        wait['ready']=None;return False
    if lag<=ready_limit:
        if wait.get('ready') is None:wait['ready']=now
    elif lag>ready_limit+hysteresis:wait['ready']=None
    return (wait.get('ready') is not None and now-wait['ready']>=hold and lag<=ready_limit)


def decode_joints(names, positions, velocities, wheelbase=None, track=None, wheel_radius=None):
    """Recover equivalent centre steering and rear-axle speed from joints."""
    wheelbase = P['wheelbase'] if wheelbase is None else wheelbase
    track = P['front_track'] if track is None else track
    wheel_radius = P['wheel_radius'] if wheel_radius is None else wheel_radius
    if len(set(names)) != len(names):
        raise ValueError('duplicate joint names')
    indices = dict((name, i) for i, name in enumerate(names))
    curvatures = []
    for name, y in (('front_left_steer_joint', track / 2.),
                    ('front_right_steer_joint', -track / 2.)):
        angle = positions[indices[name]]
        if math.isnan(angle) or math.isinf(angle):
            raise ValueError('nonfinite steering')
        if abs(angle) >= math.pi / 2.:
            raise ValueError('invalid steering angle')
        tangent = math.tan(angle)
        denominator = wheelbase + y * tangent
        if denominator <= 0:
            raise ValueError('invalid steering geometry')
        curvatures.append(tangent / denominator)
    steer = math.atan(wheelbase * sum(curvatures) / 2.)
    speed = wheel_radius * sum(velocities[indices[name]] for name in
                              ('rear_left_wheel_joint', 'rear_right_wheel_joint')) / 2.
    if math.isnan(speed) or math.isinf(speed):
        raise ValueError('nonfinite wheel velocity')
    return steer, speed


def predict_arc(grid, pose, speed, steer, target_speed, target_steer, dt,
                acceleration=.30, braking=.50, steer_rate=2.5, max_travel=float('inf'), steering_response=None,
                pose_guard=None):
    """Small-step bicycle rollout with the plugin's acceleration/steering limits.

    Returns endpoint, achieved speed/steering and travelled distance, or None
    on collision. The caller never treats a target angle as achieved feedback.
    """
    travel = 0.
    while dt > 1e-9 and travel < max_travel - 1e-9:
        step = min(P['prediction_step'], dt)
        rate = acceleration if abs(target_speed) > abs(speed) else braking
        speed = slew(speed, target_speed, rate, step)
        steer = (steering_response.advance(target_steer,step) if steering_response is not None
                 else slew(steer, target_steer, steer_rate, step))
        distance = math.copysign(min(abs(speed) * step, max_travel - travel), speed)
        arc = grid.arc(pose, distance, math.tan(steer) / P['wheelbase'])
        if arc is None:
            return None
        if pose_guard is not None and any(not pose_guard(p) for p in arc):
            return None
        pose = arc[-1]
        travel += abs(distance)
        dt -= step
    return pose, speed, steer, travel
