#!/usr/bin/env python
"""Validate a C-generated coordinate queue before any ROS goal is sent."""
import json
import math

POINT_NAMES = ['inspect_%d' % i for i in range(1, 11)] + ['outer_left_top', 'outer_left_bottom', 'outer_right_top', 'outer_right_bottom', 'start', 'end']


def load_plan(path, frame='map'):
    points, labels = [], []
    with open(path) as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            if (row.get('frame_id') != frame or row.get('target_reference') != 'front_axle_midpoint'
                    or row.get('task_id') != len(points)+1 or row.get('target_id') not in POINT_NAMES
                    or row.get('target_id') == 'start'):
                raise ValueError('Invalid plan frame, reference, task order or point name')
            values = tuple(row[key] for key in ('x', 'y', 'yaw_rad'))
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or math.isnan(v) or math.isinf(v) for v in values):
                raise ValueError('Nonfinite or invalid goal coordinate')
            points.append(values)
            labels.append(row['target_id'])
            if row['target_id'].startswith('inspect_') and row.get('inspection_type') not in ('A', 'B'):
                raise ValueError('Inspection plan missing A/B type; regenerate it with the current C planner')
    if not labels or labels[-1] != 'end' or labels.count('end') != 1:
        raise ValueError('Plan must end at the exit')
    if any(labels.count('inspect_%d' % i) != 1 for i in range(1, 11)):
        raise ValueError('Plan must visit each of the 10 inspection points once')
    return points, labels


def load_regions(path):
    load_plan(path)
    with open(path) as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    regions = {r['target_id']: dict(id=r['target_id'], type=r['inspection_type'], x=r['x'], y=r['y'])
               for r in rows if r['target_id'].startswith('inspect_')}
    return [regions['inspect_%d' % i] for i in range(1, 11)]
