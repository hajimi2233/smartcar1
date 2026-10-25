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
    lateral_entry_finish_position=.20, lateral_entry_finish_heading_deg=20.,
    lateral_entry_early_position=.15, lateral_entry_early_heading_deg=15.,
    final_early_position=.06, final_early_heading_deg=4.,
    forward_speed=.07, reverse_speed=.045,
    tracking_forward_speed=.15, tracking_reverse_speed=.08,
    tracking_min_speed=.02, approach_speed_gain=.4,
    curve_speed_gain=1.8,
    approach_braking=.15, approach_stop_reserve=.02, tracking_cruise_floor=.06,
    steering_preview_distance=.06, steering_preview_time=.60,
    deviation_resume_ratio=.70, deviation_resume_heading_deg=5.,
    line_exit_heading_grace_deg=.5, moving_steer_lag_deg=30.,
    moving_steer_error=.035, moving_steer_heading_deg=5.,
    safety_stop_extra=.03, laser_recovery_enabled=1, laser_recovery_distance=.25,
    laser_recovery_speed=.07, laser_recovery_timeout=8., laser_recovery_scan_age=.30,
    laser_recovery_attempts=2, laser_recovery_grid_step=.02, laser_recovery_range=2.,
    recovery_resume_speed=.05, recovery_resume_distance=.10,
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
    planner_three_change_after=10., planner_unlimited_after=15.,
    local_left_min_radius=1.20, local_right_min_radius=1.20,
    tracking_turn_speed_cap=.30,
    line_entry_candidate_step=.5, line_entry_candidate_count=1,
    direction_speed_deadband=.010,direction_confirm_time=.60,steering_ready_hysteresis_deg=.5,
    direction_map_window=1.2,direction_map_max_age=.55,direction_map_min_progress=.02,live_scan_uncertainty=.01,
    terminal_alignment_span=.40,terminal_heading_gain_scale=1.0)

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
        zero_ok = key in ('region_margin', 'max_replans','steering_ready_hysteresis_deg','steering_preview_distance',
                         'line_exit_heading_grace_deg','moving_steer_lag_deg','safety_stop_extra',
                         'laser_recovery_enabled','curve_speed_gain') or key.endswith('_weight')
        if not signed and (value < 0 if zero_ok else value <= 0):
            raise ValueError(key + ' has invalid sign')
        if key.endswith('_weight') and value > 1:
            raise ValueError(key + ' must be in [0, 1]')
    for key in ('rollout_steps', 'scan_stride', 'max_replans', 'planner_max_records',
                'line_entry_candidate_count','laser_recovery_enabled','laser_recovery_attempts'):
        if int(values[key]) != values[key]:
            raise ValueError(key + ' must be an integer')
        values[key] = int(values[key])
    for lower, upper in (('physical_min_radius', 'global_min_radius'),
                         ('control_period', 'max_control_dt'),
                         ('tracking_recover_error', 'tracking_max_error'),
                         ('stage_early_position', 'stage_finish_position'),
                         ('stage_early_heading_deg', 'stage_finish_heading_deg'),
                         ('lateral_entry_early_position', 'lateral_entry_finish_position'),
                         ('lateral_entry_early_heading_deg', 'lateral_entry_finish_heading_deg'),
                         ('final_early_position', 'final_finish_position'),
                         ('final_early_heading_deg', 'final_finish_heading_deg'),
                         ('final_early_heading_deg', 'normal_finish_heading_deg'),
                         ('stop_window', 'stop_timeout')):
        if values[lower] > values[upper]:
            raise ValueError(lower + ' must not exceed ' + upper)
    if not 0<values['planner_three_change_after']<values['planner_unlimited_after']<20.:
        raise ValueError('planner gear-change schedule must increase within 20 seconds')
    if not 0<values['direction_speed_deadband']<=.02:raise ValueError('direction speed deadband exceeds low-speed noise envelope')
    if not .10<=values['direction_confirm_time']<=1.0:
        raise ValueError('opposite motion confirmation must be 0.10..1.0 seconds')
    if not .6<=values['direction_map_window']<=1.5 or not .015<=values['direction_map_min_progress']<=.04:
        raise ValueError('invalid map direction evidence bounds')
    if not .4<=values['direction_map_max_age']<=.65:
        raise ValueError('direction evidence age must be 0.40..0.65 seconds')
    if not .005<=values['live_scan_uncertainty']<=.03:
        raise ValueError('live scan uncertainty must remain 0.005..0.03 metres')
    if not 0<=values['curve_speed_gain']<=5.:
        raise ValueError('curve speed gain must be in 0..5')
    if not 0<=values['steering_preview_distance']<=.08 or not 0<values['steering_preview_time']<=.8:
        raise ValueError('invalid steering preview bounds')
    if not 0<values['deviation_resume_ratio']<1 or not 0<values['deviation_resume_heading_deg']<=10.:
        raise ValueError('invalid tracking recovery hysteresis')
    if not 0<=values['line_exit_heading_grace_deg']<=1. or not 0<=values['moving_steer_lag_deg']<=35.:
        raise ValueError('invalid stage grace or moving steering lag')
    if not 0<values['moving_steer_error']<=.05 or not 0<values['moving_steer_heading_deg']<=8.:
        raise ValueError('moving steering requires a bounded tracking error')
    if not 0<=values['safety_stop_extra']<=.05 or values['laser_recovery_enabled'] not in (0,1):
        raise ValueError('invalid safety stop/recovery switch')
    if not 0<values['laser_recovery_distance']<=.25 or not 0<values['laser_recovery_speed']<=.10:
        raise ValueError('laser recovery must remain short and slow')
    if not 0<values['laser_recovery_scan_age']<=.40 or not 0<values['laser_recovery_timeout']<=10.:
        raise ValueError('laser recovery freshness/time limit invalid')
    if not 0<values['recovery_resume_speed']<=.10 or not .05<=values['recovery_resume_distance']<=.20:
        raise ValueError('invalid bounded recovery departure settings')
    if not 1<=values['laser_recovery_attempts']<=3 or not .01<=values['laser_recovery_grid_step']<=.03 or not 1<=values['laser_recovery_range']<=3.:
        raise ValueError('invalid laser recovery coverage/settings')
    if not 0<values['approach_braking']<=.5 or not 0<values['approach_stop_reserve']<=.05 or not 0<values['tracking_cruise_floor']<=.15:
        raise ValueError('invalid planned approach speed settings')
    if not 0<values['lateral_entry_finish_position']<=.30 or not 0<values['lateral_entry_finish_heading_deg']<=30.:
        raise ValueError('lateral entry stop tolerance must stay within 0.30m and 30 degrees')
    if not 0<=values['steering_ready_hysteresis_deg']<=2.:raise ValueError('steering readiness hysteresis must be 0..2 degrees')
    if not .1<=values['terminal_alignment_span']<=1. or not 1.<=values['terminal_heading_gain_scale']<=2.:
        raise ValueError('invalid terminal alignment settings')
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
