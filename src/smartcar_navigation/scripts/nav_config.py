"""Startup-only navigation settings, shared by ROS-independent helper modules.

Configure once before starting subscribers/threads. Runtime mutation is not
supported: restart navigation so a path and its executor use one geometry.
"""
import math
try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)

DEFAULTS = dict(
    map_frame='map', odom_frame='odom', base_frame='base_footprint',
    truth_frame='sim_world',
    wheelbase=.62, front_track=.45, wheel_radius=.09,
    # 0.78 m body with a 0.62 m wheelbase: 8 cm front/rear overhang.
    body_front=.70, body_rear=.08, body_half_width=.275,
    collision_half_length=.39, collision_half_width=.25,
    collision_center_x=.31, boundary_half_length=.40, tire_half_width=.035,
    region_margin=.01, physical_min_radius=1.2, global_min_radius=1.25,
    base_reference='custom', base_offset_x=.31, base_offset_y=0.,
    goal_reference='front_axle', goal_offset_x=0., goal_offset_y=0.,
    path_reference='rear_axle', path_offset_x=0., path_offset_y=0.,
    normal_planning_timeout=20.,
    line_entry_distance=1.5, line_retreat_distance=1.5,
    line_entry_search_span=.9, line_entry_timeout=20., line_entry_steer_seconds=30.,
    line_search_span=.9, line_suffix_timeout=20., planner_switch_cost=.45, planner_steer_cost=.02,
    planner_short_segment_cost=.15, tracking_turn_allowance=.015,
    stage_plan_position=.06, stage_plan_heading_deg=5.,
    maneuver_plan_heading_deg=4., stage_finish_position=.12,
    stage_finish_heading_deg=10., final_finish_position=.075,
    final_finish_heading_deg=5., normal_finish_heading_deg=4.,
    stage_early_position=.10, stage_early_heading_deg=8.,
    final_early_position=.06, final_early_heading_deg=4.,
    forward_speed=.07, reverse_speed=.045,
    tracking_forward_speed=.15, tracking_reverse_speed=.08,
    tracking_min_speed=.02, approach_speed_gain=.4,
    replay_forward_speed=.04, replay_reverse_speed=.03, replay_min_speed=.012,
    control_period=.05, max_control_dt=.35,
    feedback_timeout=.30, pose_timeout=.40, truth_timeout=.35,
    stop_window=1., stop_position_span=.02, stop_heading_span=.015,
    stop_timeout=10., progress_timeout=8., max_replans=2,
    prediction_step=.01, rollout_steps=15, rollout_dt=.15,
    steering_sample_offset=.035, tracking_max_error=.20,
    tracking_recover_error=.18, scan_obstacle_range=.80, scan_stride=2,
    wall_straight_weight=.7, wall_maneuver_weight=0., wall_normal_weight=.5,
    wall_gain=.45, wall_max_angle_deg=15.,
    planner_max_records=60000, planner_special_search_margin=1.0,
    line_entry_candidate_step=.5, line_entry_candidate_count=1)

P = dict(DEFAULTS)


def configure(overrides=None):
    """Validate atomically; typo/NaN/inconsistent limits fail before driving."""
    values = dict(DEFAULTS)
    overrides = {} if overrides is None else overrides
    if not isinstance(overrides, dict):
        raise ValueError('tuning must be a dictionary')
    unknown = set(overrides) - set(DEFAULTS)
    if unknown:
        raise ValueError('unknown tuning keys: ' + ', '.join(sorted(unknown)))
    values.update(overrides)
    for key, value in values.items():
        if key.endswith('_frame'):
            if not isinstance(value, string_types) or not value or value.startswith('/') or any(c.isspace() for c in value):
                raise ValueError(key + ' must be a nonempty TF frame without leading slash or spaces')
            continue
        if key.endswith('_reference'):
            if value not in ('rear_axle', 'front_axle', 'custom'):
                raise ValueError(key + ': choose rear_axle, front_axle or custom')
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(key + ' must be numeric')
        if math.isnan(value) or math.isinf(value):
            raise ValueError(key + ' must be finite')
        signed = '_offset_' in key or key == 'collision_center_x'
        zero_ok = key in ('region_margin', 'max_replans') or key.endswith('_weight')
        if not signed and (value < 0 if zero_ok else value <= 0):
            raise ValueError(key + ' has invalid sign')
        if key.endswith('_weight') and value > 1:
            raise ValueError(key + ' must be in [0, 1]')
    for key in ('rollout_steps', 'scan_stride', 'max_replans', 'planner_max_records',
                'line_entry_candidate_count'):
        if int(values[key]) != values[key]:
            raise ValueError(key + ' must be an integer')
        values[key] = int(values[key])
    for lower, upper in (('physical_min_radius', 'global_min_radius'),
                         ('control_period', 'max_control_dt'),
                         ('tracking_recover_error', 'tracking_max_error'),
                         ('stage_early_position', 'stage_finish_position'),
                         ('stage_early_heading_deg', 'stage_finish_heading_deg'),
                         ('final_early_position', 'final_finish_position'),
                         ('final_early_heading_deg', 'final_finish_heading_deg'),
                         ('final_early_heading_deg', 'normal_finish_heading_deg'),
                         ('stop_window', 'stop_timeout')):
        if values[lower] > values[upper]:
            raise ValueError(lower + ' must not exceed ' + upper)
    P.clear()
    P.update(values)
    return dict(P)


def reference_offset(kind):
    reference = P[kind + '_reference']
    if reference == 'rear_axle':
        return 0., 0.
    if reference == 'front_axle':
        return P['wheelbase'], 0.
    return P[kind + '_offset_x'], P[kind + '_offset_y']


def from_rear(pose, kind):
    """Point offset in vehicle axes (+x forward, +y left), vehicle yaw kept."""
    x, y = reference_offset(kind)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0]+c*x-s*y, pose[1]+s*x+c*y, pose[2]


def to_rear(pose, kind):
    x, y = reference_offset(kind)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0]-c*x+s*y, pose[1]-s*x-c*y, pose[2]
