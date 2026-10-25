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
    # Also normalize previously generated queues to the current end policy.
    x, y, _yaw = points[-1]
    points[-1] = (x, y, math.radians(-163.5))
    return points, labels


def load_regions(path):
    load_plan(path)
    with open(path) as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    regions = {r['target_id']: dict(id=r['target_id'], type=r['inspection_type'], x=r['x'], y=r['y'])
               for r in rows if r['target_id'].startswith('inspect_')}
    return [regions['inspect_%d' % i] for i in range(1, 11)]


def b_lateral_goals(points,labels,regions):
    """Select B1/B3/B5/B7/B9, entering each from its upper paired point.

    Full ten-region metadata remains intact; this changes only the executed
    sequence and uses actual paired coordinates to derive body heading.
    """
    if len(points)!=len(labels) or len(regions)!=10:
        raise ValueError('B lateral test needs a validated full plan and regions')
    lookup=dict(zip(labels,points));by_id={r['id']:r for r in regions};selected=[];names=[]
    for number in (1,3,5,7,9):
        name='inspect_%d'%number;partner='inspect_%d'%(number+1)
        if name not in lookup or name not in by_id or partner not in by_id:
            raise ValueError('Missing paired B test point '+name)
        region=by_id[name];other=by_id[partner]
        if region['type']!='B':raise ValueError('Selected test region must be B: '+name)
        dx,dy=other['x']-region['x'],other['y']-region['y'];yaw=math.atan2(dy,dx)
        if math.hypot(dx,dy)<.1 or math.sin(yaw)>-.5:
            raise ValueError('B test requires upper odd/lower even paired points')
        if math.hypot(lookup[name][0]-region['x'],lookup[name][1]-region['y'])>1e-6:
            raise ValueError('Plan point differs from inspection region')
        selected.append((region['x'],region['y'],yaw));names.append(name)
    return selected,names
