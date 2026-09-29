"""Rear-axle actuator feedback and prediction; no ROS or world-pose input."""
import math
from nav_config import P


def slew(current, target, rate, dt):
    return current + max(-rate * dt, min(rate * dt, target - current))


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
                acceleration=.30, braking=.50, steer_rate=2.5, max_travel=float('inf')):
    """Small-step bicycle rollout with the plugin's acceleration/steering limits.

    Returns endpoint, achieved speed/steering and travelled distance, or None
    on collision. The caller never treats a target angle as achieved feedback.
    """
    travel = 0.
    while dt > 1e-9 and travel < max_travel - 1e-9:
        step = min(P['prediction_step'], dt)
        rate = acceleration if abs(target_speed) > abs(speed) else braking
        speed = slew(speed, target_speed, rate, step)
        steer = slew(steer, target_steer, steer_rate, step)
        distance = math.copysign(min(abs(speed) * step, max_travel - travel), speed)
        arc = grid.arc(pose, distance, math.tan(steer) / P['wheelbase'])
        if arc is None:
            return None
        pose = arc[-1]
        travel += abs(distance)
        dt -= step
    return pose, speed, steer, travel
