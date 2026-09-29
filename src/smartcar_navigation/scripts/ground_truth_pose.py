"""Frozen planar alignment from Gazebo sim_world to a localized map frame."""
import math
from ackermann_core import wrap, finite
from nav_config import to_rear


def alignment(map_base, world_base):
    if not finite(map_base) or not finite(world_base):
        raise ValueError('nonfinite alignment pose')
    angle = wrap(map_base[2] - world_base[2])
    c, s = math.cos(angle), math.sin(angle)
    return (map_base[0] - c*world_base[0] + s*world_base[1],
            map_base[1] - s*world_base[0] - c*world_base[1], angle)


def rear_pose(world_base, transform):
    if not finite(world_base) or not finite(transform):
        raise ValueError('nonfinite ground truth pose')
    x, y, yaw = world_base
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)
    heading = wrap(yaw + angle)
    return to_rear((tx + c*x - s*y, ty + s*x + c*y, heading), 'base')
