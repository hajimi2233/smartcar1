#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Validate a tuning file offline; no ROS/Gazebo or vehicle connection needed."""
from __future__ import print_function
import math
import os
import sys
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'smartcar_navigation', 'scripts'))
from nav_config import configure


def check(path):
    with open(path, 'r') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('configuration must be a mapping')
    expected = set(('turn_radius local_turn_radius steering_rate tracking_lateral_gain '
                    'tracking_heading_gain tracking_preview_distance collision_margin '
                    'zero_cost_line_width planning_timeout scan_timeout goal_position_tolerance '
                    'goal_heading_tolerance_deg ground_truth_test test_replay_speed_scale '
                    'test_auto_arrive test_auto_arrive_delay cmd_topic joint_topic '
                    'actuator_model_param tuning').split())
    if set(data) != expected:
        raise ValueError('missing keys: %s; unknown keys: %s' %
                         (sorted(expected-set(data)), sorted(set(data)-expected)))
    p = configure(data['tuning'])
    for key in expected - set(('tuning ground_truth_test test_auto_arrive cmd_topic joint_topic actuator_model_param').split()):
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or math.isnan(value) or math.isinf(value) or value < 0:
            raise ValueError(key + ' must be finite and nonnegative')
        if key not in ('zero_cost_line_width','test_auto_arrive_delay') and value == 0:
            raise ValueError(key + ' must be positive')
    for key in ('ground_truth_test','test_auto_arrive'):
        if type(data[key]) is not bool:
            raise ValueError(key + ' must be true or false')
    for key in ('cmd_topic','joint_topic','actuator_model_param'):
        if not data[key] or not isinstance(data[key], str):
            raise ValueError(key + ' must be a nonempty string')
    if not p['physical_min_radius'] <= data['local_turn_radius'] <= data['turn_radius'] or data['turn_radius'] < p['global_min_radius']:
        raise ValueError('inconsistent turn radii')
    if data['collision_margin'] < .02 or data['scan_timeout'] < .4 or not 1 <= data['planning_timeout'] <= 60:
        raise ValueError('collision_margin >= .02; scan_timeout >= .4; planning_timeout in [1,60]')
    if not 0 < data['test_replay_speed_scale'] <= 4:
        raise ValueError('test_replay_speed_scale must be in (0,4]')
    print('OK: %d startup tuning settings; goal=%s, path=%s, base=%s' %
          (len(p),p['goal_reference'],p['path_reference'],p['base_reference']))


if __name__ == '__main__':
    try:
        check(sys.argv[1] if len(sys.argv)>1 else os.path.join(ROOT,'config','navigation.yaml'))
    except (ValueError, TypeError, yaml.YAMLError) as exc:
        sys.exit('Invalid navigation config: ' + str(exc))
