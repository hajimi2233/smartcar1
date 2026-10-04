# Localization policy comparison (experimental)

This experiment compares division of responsibility with weighted wheel priors.
It does not change the production `wall_localizer` or the default launch mode.

- `tests/ros_localization_policy_suite.py`: isolated Gazebo suite, real ROS AMCL,
  common lidar/scan odometry/map/pulses, six independent policy tracks.
- `scripts/localization_roles.py` under `src/smartcar_localization`: selected-wall
  constraints, full-map completion of weak directions, AMCL-confirmed recovery.
- `scripts/localization_search.py`: bounded startup search using lidar and map,
  followed by local refinement. Search range +/-0.6 m and +/-0.30 rad around the
  provided initial guess. It is not global localization from an unknown pose.
- `tests/replay_localization_search.py`: two additional offline tracks, with
  startup search and optional short outage prediction using signed wheel distance
  and recent lidar-derived angular velocity. Unobserved yaw is marked invalid.

Run the ROS suite only inside a Docker container with `--network none` and
`--isolated-sim`; it publishes movement commands and resets the simulated vehicle.
The runner expects `/project` to contain this repository, `/experiment` as output,
ROS Kinetic and the built simulation packages sourced. Start `drivers_sim.launch`
with the `sim_field_v1` map and `config/simulation.yaml`, then run the suite.
Environment variables: `POLICY_CASES` (comma-separated case names),
`POLICY_REPEATS` (default 3), `POLICY_REPEAT_START` (default 0),
`POLICY_REALTIME_FACTOR` (default 0.4), `POLICY_OUTPUT` (default `/experiment`).

Cases: normal, plus5, minus5, parallel_plus5, initial_offset, scan_outage,
straight_wall_plus5, straight_wall_offset, straight_wall_outage, turn_change_outage.
Original partial-wall selections also cover corner faces. The straight-wall cases
exclude corners to exercise genuinely weak along-wall constraints.

Truth is used only by the signed pulse simulator and the evaluator. The policy,
startup search and outage predictor never receive truth pose, true yaw, IMU,
steering angle or commanded velocity. Ground truth in recorded data must not be
passed to the estimation methods.

Compare maximum planar error across all repeats, including prediction-only and
invalid outputs, not just mean/RMSE. Record unavailable samples and initialization
separately. Replay fills dropped scan timestamps with held baseline estimates and
interpolated evaluation truth, keeping failure peaks in the comparison.

Limitations: fixed short trajectory, ideal map, instantaneous simulated lidar
frames, no wheel slip, fixed AMCL defaults, no high-frequency ROS pose publisher.
The search/outage variants are offline replay, with sensor-timestamp scoring;
this does not demonstrate real-time latency. Startup search holds the known
initialization stage, and real deployment must prevent motion before readiness.
The outage predictor assumes recent angular velocity persists for at most 1.2 s;
without an IMU or steering feedback, changing yaw during lidar loss is unobserved.

Unit checks (ROS environment provides NumPy):

```bash
python -m unittest discover -s tests -p 'test_localization_roles.py'
python -m unittest discover -s tests -p 'test_localization_search.py'
```

## Promoted full-map mode

The normal-operation winner `full_map` is now the default in
`localization.launch`. It runs the installed `wall_localizer.py` with
`target_mode=full_map`, using the same `map_geometry` and `match` functions as
the comparison. No corridor polygon, selected walls, AMCL or wheel input is
required. Initialize with RViz **2D Pose Estimate** near the actual pose.
`wall-status` reports `target_mode: full_map`, `group: full_map`,
`wheel_state: DISABLED`, and `state: MATCHED` for a full-rank match.

Use `localization_mode:=amcl` for the previous AMCL mode, or
`wall_features:=true` for the unchanged selected-wall mode. Automatic switching
between outside full-map matching and inside selected walls is not part of this
change. A full-map launch remains full-map inside and outside the corridor.
