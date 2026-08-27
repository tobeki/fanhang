# Manual Return Planner V1.1

This package implements manual-mode return based on the vehicle's measured
flight history. It validates ordered samples, removes only near-duplicate
positions, reverses the history, applies true 3D Ramer-Douglas-Peucker (RDP),
restores measured samples wherever an RDP chord would cross a meaningful
height change, splits long segments with existing history points, and
publishes the resulting path.

- **V1.0** delivered the algorithm and visualization: record, preprocess,
  Reverse, 3D RDP, max-segment-length constraint, yaw generation, RViz
  visualization, CSV I/O, and the `/manual_return/trigger` service.
- **V1.0.1** adds the missing control loop: a `CommandGate` that makes
  `/quad_0/planning/pos_cmd` have exactly one publisher, a Manual Return
  Executor that publishes a continuous, speed-planned reference trajectory, a
  trigger-time start anchor, a `reset` service, and tracking-error logging for
  the next Safe-RDP stage.
- **V1.0.2** decomposes the tracking error into time-synchronized, cross-track
  (3D / horizontal / vertical) and along-track components, adds per-run output
  subdirectories, a per-run summary, and an offline multi-run analysis tool, so
  Safe-RDP `tracking_margin` can be derived from *spatial* cross-track error
  rather than the time-synchronized norm.
- **Unified benchmark metrics** add an algorithm-agnostic `ReturnMetrics`
  report (`return_metrics.csv` + human-readable `return_summary.txt`) plus a
  batch aggregation script, so different return algorithms can be compared
  fairly on the same set of numbers.
- **V1.0.3** adds a `scenario` label to every run, enhanced RViz visualization
  (history path, return-path line, key-point spheres, home marker, live
  position marker), a benchmark RViz config, and a launch `scenario` argument
  for multi-scenario benchmark runs.

`planManualReturn(...)` remains the only return planning API. Mission return is
intentionally not implemented.

V1.0.1 does **not** implement Historical Corridor validation, PCD collision
checks, vehicle safety envelope checks, shortcut rejection, dynamic obstacle
avoidance, B-Spline smoothing, or production flight safety certification. The
PCD is loaded and visualized only; it is reserved for Safe-RDP V1.1.

## Control chain investigation: `/mandatory_stop_to_planner`

The existing `/mandatory_stop_to_planner` topic is **not** a control-source
switch:

1. Subscriber: `diff_planner_node` (`DiffReplanFSM::mandatoryStopCallback`,
   via the `~mandatory_stop` remap in `single_drone.xml`).
2. Message type: `std_msgs/Empty`.
3. Behavior: sets `mandatory_stop_`, `need_hover_stop_`,
   `flag_escape_emergency_` and forces the planner FSM into `EMERGENCY_STOP` —
   it only stops the planner from generating new trajectories.
4. `traj_server` does **not** subscribe to it and keeps publishing
   `PositionCommand` at 100 Hz (forwarding the last trajectory or a hover
   command).
5. Therefore `/quad_0/planning/pos_cmd` keeps its old publisher and the topic
   cannot guarantee a single control source.

Conclusion: `/mandatory_stop_to_planner` is kept only as an auxiliary signal to
stop normal re-planning; the real control guarantee is the `CommandGate`.

## Control topology

```
traj_server  ──► /manual_return/normal_pos_cmd ──┐
                                                  ├─► CommandGate ─► /quad_0/planning/pos_cmd ─► cascadePID
ManualReturnExecutor ─► /manual_return/return_pos_cmd ─┘
```

`CommandGate` is the **only** publisher of `/quad_0/planning/pos_cmd`, in both
normal flight and return flight. In NORMAL mode it forwards `normal_pos_cmd`
and drops `return_pos_cmd`; in MANUAL_RETURN mode it forwards `return_pos_cmd`
and drops `normal_pos_cmd`. The switch is a deliberate service call and never
an implicit reaction to an incoming command, so a stale diff_planner/traj_server
stream cannot re-take control.

## Components

- `CommandGateCore` (`include/manual_return_planner/command_gate.h`): pure,
  thread-safe, header-only policy (NORMAL / MANUAL_RETURN, forward-or-drop).
  Unit-tested without a running ROS graph.
- `command_gate_node`: the ROS gate node. Subscribes `normal_pos_cmd` and
  `return_pos_cmd`, publishes the controller topic, exposes
  `/manual_return/gate/set_manual_return` and `/manual_return/gate/set_normal`
  (`std_srvs/Trigger`), and publishes `/manual_return/control_source`
  (`std_msgs/String`).
- `ManualReturnNode`: owns the ROS boundary — 10 Hz recorder, trigger/reset
  services, visualization/path/PCD publishers, and the return executor.
- `TrajectoryPreprocessor`, `RdpSimplifier`, `CsvTrajectoryReader`,
  `ManualReturnPlanner`: unchanged pure algorithm/data components.

## State machine

```
RECORDING → PLANNING → WAITING_FOR_TAKEOVER → RETURNING → FINISHED
                    ↘ FAILED
```

- `RECORDING`: sampling odometry at `record_frequency`.
- `PLANNING`: history frozen, plan computed.
- `WAITING_FOR_TAKEOVER`: waiting `control_takeover_delay` before switching the
  gate.
- `RETURNING`: gate switched to MANUAL_RETURN; executor streams the reference.
- `FINISHED`: Home reached; the gate **stays** in MANUAL_RETURN and the
  executor keeps publishing a Home hold command (it does **not** auto-restore
  NORMAL).
- `FAILED`: any validation or control-switch failure.

## Trigger sequence

1. Capture the latest odometry as the trigger-time position/velocity.
2. Stop appending to the outbound history.
3. Force the trigger pose to be the final history point, so after reversal it
   is the return start anchor (even if it is within `min_point_spacing` of the
   last 10 Hz sample).
4. Run `planManualReturn` (Reverse + 3D RDP + max-segment + yaw).
5. Basic checks: at least 2 waypoints, finite positions, plausible
   `return_start_error` (target `< 0.10 m`).
6. Publish `/mandatory_stop_to_planner` (auxiliary only).
7. Enter `WAITING_FOR_TAKEOVER`, then switch the gate to MANUAL_RETURN.
8. Execute a continuous, speed-planned, piecewise-linear reference.
9. On reaching Home (`position error < home_position_tolerance` **and**
   `speed < return_finish_speed_threshold`) enter `FINISHED` and keep holding.

## Velocity planning

The executor integrates arc length along the RDP polyline with a conservative
trapezoidal profile:

- `v <= return_cruise_speed`, `|a| <= max_return_acceleration`;
- acceleration ramp at start and deceleration ramp near Home;
- short paths use a triangular profile;
- reference position/velocity/yaw are interpolated continuously along the
  segments (never jumping between waypoints).

`max_return_acceleration` is now actually used (it was previously only a
stored parameter).

## Topics and services

Return planning/visualization (existing):

```text
/manual_return/raw_path
/manual_return/preprocessed_path
/manual_return/reversed_path
/manual_return/rdp_path
/manual_return/home
/manual_return/map_cloud
/manual_return/status
```

Benchmark visualization (new in V1.0.3):

```text
/manual_return/history_path      (nav_msgs/Path, 10 Hz recorded history)
/manual_return/return_path_line  (Marker LINE_STRIP, simplified return polyline)
/manual_return/key_points        (MarkerArray, one sphere per simplified waypoint)
/manual_return/home_marker       (Marker, Home position)
```

Control switching (new in V1.0.1):

```text
/manual_return/normal_pos_cmd      (traj_server output -> gate input)
/manual_return/return_pos_cmd      (executor output -> gate input)
/manual_return/control_source      (gate status: NORMAL | MANUAL_RETURN)
/quad_0/planning/pos_cmd          (gate output -> cascadePID, sole publisher)
```

Services:

```text
/manual_return/trigger                     (std_srvs/Trigger)
/manual_return/reset                       (std_srvs/Trigger)
/manual_return/gate/set_manual_return      (std_srvs/Trigger)
/manual_return/gate/set_normal             (std_srvs/Trigger)
```

## CSV format and outputs

The exact input header remains:

```text
timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw
```

Units are seconds, metres, metres/second and radians. No timestamp sorting is
performed. Use `trajectory_csv` to load a file and `offline_csv_test=true` to
plan it at startup. Outputs are written under `output_dir`:

```text
raw_path.csv
preprocessed_path.csv
reversed_path.csv
return_path.csv                (the simplified return polyline)
return_tracking_log.csv
return_tracking_analysis.csv   -> diagnostics/
executed_return_path.csv
return_metrics.csv             (unified benchmark metrics, one row)
return_summary.txt             (human-readable report)
diagnostics/                   (analysis detail + worst-case report)
```

`return_tracking_log.csv` columns (V1.0.2 extended):

```text
timestamp,ref_x,ref_y,ref_z,actual_x,actual_y,actual_z,error_x,error_y,error_z,
error_norm,matched_segment,projection_x,projection_y,projection_z,
actual_progress_s,reference_progress_s,along_track_error,cross_track_3d,
cross_track_xy,vertical_error,return_phase
```

The mean, P95 and max tracking error are logged at completion and are intended
to calibrate Safe-RDP `tracking_margin`.

## Parameters

Shipped defaults (`config/manual_return.yaml`):

| parameter | default | meaning |
|---|---:|---|
| `record_frequency` | 10.0 Hz | history sampling frequency |
| `min_point_spacing` | 0.03 m | near-duplicate filter |
| `rdp_epsilon` | 0.05 m | 3D RDP tolerance |
| `vertical_preserve_threshold` | 0.05 m | maximum history z span that an RDP segment may skip |
| `max_segment_length` | 5.0 m | max return segment |
| `max_reasonable_speed` | 2.25 m/s | history jump warning |
| `return_cruise_speed` | 0.5 m/s | return cruise speed |
| `max_return_acceleration` | 1.5 m/s² | return accel (now used) |
| `home_position_tolerance` | 0.30 m | Home arrival tolerance |
| `landing_handoff_height` | 0.25 m | final height above arm/Home before landing takes over |
| `return_finish_speed_threshold` | 0.15 m/s | finish speed threshold |
| `control_takeover_delay` | 1.5 s | gate-switch delay |
| `command_frequency` | 100.0 Hz | executor publish rate |
| `enable_return_command_output` | false | publish return commands |
| `return_command_topic` | `/manual_return/return_pos_cmd` | executor output |
| `gate_set_manual_return_service` | `/manual_return/gate/set_manual_return` | gate arm |
| `gate_set_normal_service` | `/manual_return/gate/set_normal` | gate disarm |

`enable_return_command_output=false` is the offline/visualization-only default.
The new launch sets it to `true` so the return actually executes through the
gate.

## Build, launch and operation

Build the package (together with its message dependency) in the existing
workspace:

```bash
cd ~/fanhang
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="mars_quadrotor_msgs;manual_return_planner"
source devel/setup.bash
```

Launch the full, single-control-source simulation:

```bash
roslaunch manual_return_planner manual_return_mid360.launch
```

Set an RViz goal on `/goal`, let the existing EGO/diff-planner simulation fly,
then trigger return:

```bash
rosservice call /manual_return/trigger "{}"
```

Verify the single publisher (normal and return phases alike):

```bash
rostopic info /quad_0/planning/pos_cmd
# Publishers: must list only the CommandGate node.
```

Confirm the gate status:

```bash
rostopic echo /manual_return/control_source
```

After a FINISHED/FAILED run, reset for the next trial:

```bash
rosservice call /manual_return/reset "{}"
```

`manual_return_mid360.launch` recombines the existing `single_drone_mid360`
stack through `single_drone_gated.xml`, which is a copy of `single_drone.xml`
with a single change: `traj_server`'s `~position_cmd` is remapped to
`/manual_return/normal_pos_cmd` instead of the controller topic. No existing
package source or launch file is modified.

## Tests

`test/test_manual_return.cpp` covers nearly-collinear compression, a 90-degree
corner, 3D height deviation, an S-curve, endpoint reversal/segment bounds,
auxiliary-field warnings, and timestamp rejection.

`test/test_command_gate.cpp` covers CommandGate NORMAL/RETURN forwarding, the
no-implicit-switch-back guarantee, and the trigger-anchor-becomes-return-start
behavior.

Run them with:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="mars_quadrotor_msgs;manual_return_planner" run_tests_manual_return_planner
```

All 11 tests (7 original + 4 new) pass. The supplied 393-point sample still
produces 316 preprocessed points and 22 RDP waypoints at `rdp_epsilon=0.05 m`
(raw/preprocessed/RDP lengths 43.591/43.528/43.503 m, max RDP deviation
0.0472 m).

## V1.0.2 tracking error decomposition

The raw tracking metric `error_norm = |actual(t) - reference(t)|` mixes
along-track lag/lead, time-synchronization and true lateral offset, so it is
**not** a valid Safe-RDP `tracking_margin`. V1.0.2 additionally computes, for
every sample, the projection of the actual position onto the planned RDP
polyline (with monotonic arc-length progress so U-turns/self-crossings cannot
snap back to a wrong segment):

- `cross_track_3d` = |actual - projection| (the spatial offset Safe-RDP cares
  about);
- `cross_track_xy` = horizontal component of the same offset;
- `vertical_error` = |dz| relative to the projection;
- `along_track_error` = reference arc length - projected arc length (positive =
  lagging);
- `return_phase` = TAKEOVER / ACCELERATION / CRUISE / TURN / DECELERATION /
  HOME_HOLD (turn detection driven by `turn_angle_threshold_deg`).

Each run now writes its own timestamped subdirectory under `output_dir`
(e.g. `~/fanhang/logs/manual_return_output/20260820_161439/`) so successive runs do not
overwrite each other, containing `raw_path.csv`, `preprocessed_path.csv`,
`reversed_path.csv`, `rdp_return_path.csv`, `return_tracking_log.csv` (extended
columns), `return_tracking_analysis.csv` (compact), `executed_return_path.csv`
and `return_summary.txt`.

Offline multi-run aggregation (pure standard library, no numpy):

```bash
python3 scripts/analyze_tracking_runs.py --base ~/fanhang/logs/manual_return_output \
    --out aggregated_tracking_summary.csv
```

### First successful run (5 m straight, 3 RDP points)

| metric | mean | P95 | P99 | max |
|---|---:|---:|---:|---:|
| time-synchronized | 0.117 m | 0.676 m | — | 0.807 m |
| cross-track 3D | 0.024 m | 0.120 m | 0.148 m | 0.151 m |
| cross-track XY | 0.015 m | 0.102 m | — | 0.123 m |
| vertical | 0.010 m | 0.088 m | — | 0.151 m |

The time-synchronized P95 (0.676 m) is dominated by along-track offset that
peaks in the cruise phase (~0.8 m along-path at t≈6 s), not by lateral
deviation: the true 3D cross-track P95 is only **0.120 m** (P99 0.148 m, max
0.151 m). The worst cross-track occurs in the final vertical descent to Home
(vertical component 0.151 m). Final home error was 0.063 m. This is the basis
for a simulation `tracking_margin` of order **~0.15 m** rather than the
misleading 0.676 m time-sync P95. Multi-scenario confirmation (long straight,
90° turn, S-curve, 3D) is still required before fixing the value.

## Unified benchmark metrics

To compare return algorithms (RDP, Safe-RDP, shortcut optimization, ...) on an
equal footing, every successful run writes a single-row `return_metrics.csv`
plus a human-readable `return_summary.txt` under its run directory.  The metric
names are algorithm-agnostic (e.g. `simplified_points`, `return_path`, never
`rdp_points`).

| group | fields |
|---|---|
| Input scale | `original_points`, `original_length_m` |
| Compression | `simplified_points`, `point_reduction_percent`, `return_length_m`, `length_change_percent` |
| Path fidelity | `max_deviation_m`, `mean_deviation_m`, `p95_deviation_m` |
| Safety | `min_clearance_m`, `clearance_available`, `unsafe_segments`, `validated_segments`, `collision_check_count` |
| Tracking | `cross_track_p95`, `cross_track_max`, `along_track_p95`, `vertical_error_p95` |
| Completion | `final_home_error_m`, `return_duration_s` |
| Performance | `planning_time_ms`, `memory_usage_mb`, `pointcloud_size`, `voxelized_cloud_size` |

Definitions:

- `original_length_m` / `return_length_m`: sum of 3D distances between
  consecutive points.
- `point_reduction_percent = (original_points - simplified_points) /
  original_points * 100`.
- `length_change_percent = (return_length - original_length) /
  original_length * 100`.
- `*_deviation_m`: distance (3D, never just XY) from the simplified return
  path back to the original flown history, reported as mean / P95 / max.
- `min_clearance_m`: when no point-cloud collision check is available,
  `clearance_available=false` and this field is reported as `nan`
  (`not_available` in the text report) — never a sentinel like `-1`.
- `cross_track_*` / `along_track_*` / `vertical_error_*`: spatial
  decomposition of the tracking error (see the V1.0.2 section), NOT the
  time-synchronized `error_norm`.
- `final_home_error_m`: distance from the last actual position to Home.

Batch aggregation across runs:

```bash
python3 scripts/analyze_return_runs.py --base ~/fanhang/logs/manual_return_output
```

produces `return_all_runs_summary.csv` (one row per run, with an editable
`scenario` column).

### Interpreting the numbers correctly

1. **`length_change_percent` alone does not rank algorithms.**  A shorter
   return path is only better if it stays inside the flown corridor; always
   weigh it together with `*_deviation_m` and `min_clearance_m`.
2. **Path shortening must be paired with `max_deviation_m` and
   `min_clearance_m`.**  Cutting a corner lowers `return_length_m` but raises
   `max_deviation_m` and can violate clearance.
3. **Tracking safety uses `cross_track_*`, not `time_error`.**  The
   time-synchronized `error_norm` mixes along-track lead/lag with true lateral
   offset; Safe-RDP must use the spatial cross-track error.

## V1.0.3 scenario label and benchmark visualization

### Scenario label

`ReturnMetrics` now carries a `scenario` string.  It is read from the ROS
parameter `/manual_return/scenario` (falling back to the node-private
`scenario` parameter, default `unknown`), written as the second column of
`return_metrics.csv` and shown as a `Scenario:` line in `return_summary.txt`.

Set it per run via the launch argument:

```bash
roslaunch manual_return_planner manual_return_mid360.launch scenario:=turn90
```

### Benchmark RViz config

`config/manual_return_benchmark.rviz` (loaded by default) shows:

| display | topic | meaning |
|---|---|---|
| HistoryPath | `/manual_return/history_path` | blue line, recorded input history |
| ReturnPathLine | `/manual_return/return_path_line` | red line strip, simplified return polyline |
| KeyPoints | `/manual_return/key_points` | yellow spheres, one per simplified waypoint |
| HomeMarker | `/manual_return/home_marker` | Home position |
| GlobalCloud | `/map_generator/global_cloud` | map point cloud |

The `key_points` sphere count equals `simplified_points`, so the compression
(e.g. 657 raw points to 13 waypoints) is visible at a glance.

### Home and landing handoff

The first recorded point supplies the horizontal arm/Home position.  The
planner deliberately ends at `landing_handoff_height` (0.25 m by default)
above that point, rather than at the ground or at px4ctrl's approximately 1 m
takeoff-hover Home.  The endpoint is included before Reverse, RDP, and
segment-length enforcement, so it remains the final return waypoint.  The
landing state owns the descent after this low-altitude handoff.

### Z-axis original-route guard

RDP is allowed to compress a history interval only when that interval's
complete z range is at most `vertical_preserve_threshold` (0.05 m by default).
If a candidate RDP segment skips samples spanning more than that height, the
planner restores every preprocessed historical sample between its endpoints
before max-segment enforcement and collision validation.  Consequently:

- a vertical climb/descent is replayed through its measured samples;
- a climb/descent with slight pilot drift follows that same 3D trace;
- RDP cannot turn a stepped or curved height transition into a new diagonal;
- level-flight centimetre-scale z noise remains eligible for compression.

The 0.05 m value is a noise deadband, not permission to invent a vertical
shortcut: any output segment spanning more height is either an adjacent
measured segment or has all skipped measured samples restored.

## V1.0.3.1 benchmark data flow

The benchmark pipeline is: `scenario` launch arg -> ROS parameter -> node ->
`ReturnMetrics` -> `return_metrics.csv` / `return_summary.txt`, then aggregated
by the analysis scripts.

Experiment workflow:

1. Start a scenario:
   ```bash
   roslaunch manual_return_planner manual_return_mid360.launch scenario:=turn90
   ```
2. Let the drone complete its flight.
3. Trigger return: `rosservice call /manual_return/trigger "{}"`.
4. A run directory is created under `logs/manual_return_output/<run_id>/`.

After collecting several runs, generate the comparison tables and check the data:

```bash
python3 scripts/analyze_return_runs.py   --base ~/fanhang/logs/manual_return_output
python3 scripts/analyze_tracking_runs.py --base ~/fanhang/logs/manual_return_output
python3 scripts/check_benchmark_runs.py  --base ~/fanhang/logs/manual_return_output
```

- `analyze_return_runs.py` reads each `return_metrics.csv` and writes
  `return_all_runs_summary.csv` (one row per run, `scenario` column filled from
  the run's own metrics).
- `analyze_tracking_runs.py` reads each `return_tracking_log.csv` and writes
  `aggregated_tracking_summary.csv` (cross-track / along-track / vertical
  decomposition; prints a warning if a run is missing its tracking log).
- `check_benchmark_runs.py` verifies every run has `return_metrics.csv`,
  `return_tracking_log.csv`, `return_summary.txt` and non-empty key fields
  (`scenario`, `original_points`, `simplified_points`, `max_deviation_m`,
  `cross_track_p95`), writing `benchmark_check_report.txt`.

## Known limitations

- The full workspace builds once `libglfw3-dev` is installed (`sudo apt
  install -y libglfw3-dev`); `swarm_bridge` additionally needs `libzmqpp-dev`
  (bundled as .debs under `third_party/zmqpp/`).
- The `manual_return_mid360.launch` closed-loop flight has been validated for a
  short straight return. Longer / multi-turn / 3D scenarios still need to be
  run to calibrate Safe-RDP.
- The RDP epsilon and all speeds are simulation starting points, not safety
  guarantees. Map-aware Safe-RDP checks static accumulated PCD structure, not
  live dynamic obstacles or a ray-traced known-free/unknown volume.


## V1.1 Safe-RDP (whole-map shortcut fallback)

V1.1 adds a PCD-constrained step on top of the Reverse + 3D RDP pipeline.  It
is **not** real-time avoidance, dynamic planning, or A*: it validates only
non-adjacent chords introduced by simplification.  An unsafe chord is locally
replaced by the measured history samples it skipped, so other clear sections
can remain compressed and return planning stays available.

Components:

- `include/manual_return_planner/safe_rdp.h` + `src/safe_rdp.cpp`:
  `SafeRdpPlanner::restoreUnsafeShortcuts(candidate, history, map, config)`.
  It voxel-downsamples the PCD (VoxelGrid), builds a `pcl::KdTreeFLANN`, and
  samples every shortcut at `collision_check_resolution`, measuring the
  nearest obstacle distance.  A shortcut is restored if any sample is closer
  than `R_safe`.  The lower-level `validate(...)` API remains available for
  isolated clearance tests and benchmarks.
- `SafeRdpConfig` (in `manual_return_planner.h`): master switch plus the safety
  envelope.  `R_safe = uav_radius + tracking_margin + extra_margin`
  (= 0.25 + 0.15 + 0.10 = **0.50 m** by default).
- `planManualReturn(...)` runs the fallback when `safe_rdp.enabled`.  With no
  PCD, `clearance_available` remains false and the Reverse + RDP result is
  unchanged.  A mapped collision no longer changes the run to
  `NO_SAFE_PATH`; only that filtering decision is cancelled.

New configuration (`config/manual_return.yaml`):

```yaml
safe_rdp:
  enabled: true
  uav_radius: 0.25
  tracking_margin: 0.15
  extra_margin: 0.10
  voxel_resolution: 0.05
  collision_check_resolution: 0.05
```

New metrics in `return_metrics.csv` / `return_summary.txt`:

```text
safe_rdp_enabled, safe_path_points, original_rdp_points, shortcut_count
```

`unsafe_segments` now counts rejected/restored shortcut candidates;
`validated_segments` and `shortcut_count` count accepted candidates.  The
existing clearance, collision-query, safe-point, and voxelized-cloud metrics
remain populated.

New RViz topic:

```text
/manual_return/safe_return_path   (green LINE_STRIP, after map fallback)
```

Safe-RDP deliberately does **not** implement detour, A*, or new-path search.
It makes a binary decision for each existing RDP chord: keep the chord or
restore the flown interval.

Tests cover strict clearance, obstacle conversion, unsafe-shortcut restoration,
clear-shortcut retention, and empty-map degradation.  Offline benchmark tool:
`src/safe_rdp_benchmark.cpp` (`safe_rdp_benchmark <scenario> <csv> <pcd>`).


## V1.0.4 Waypoint heading control

The return executor previously interpolated yaw linearly **across each whole
segment** (in `locateSegment`), so on a turn (e.g. 0 deg -> 90 deg) the drone
kept rotating while flying a straight line.  V1.0.4 changes the heading
behaviour so the drone holds the current segment heading and only turns near a
waypoint.

New configuration (`config/manual_return.yaml`):

```yaml
yaw_mode: waypoint_hold        # tangent_interpolation | waypoint_hold
yaw_transition_radius: 0.2     # [m] turn happens within this radius of a waypoint
```

- `tangent_interpolation`: legacy behaviour (kept for comparison).
- `waypoint_hold` (default): the reference yaw stays at the current segment
  heading `atan2(dy, dx)` during flight, then transitions to the next segment
  heading within `yaw_transition_radius` of the waypoint.  The return **path
  geometry is unchanged** (same waypoint count and path length); only the yaw
  reference changes.

New outputs:

- `return_yaw_log.csv`: `timestamp,current_x,current_y,current_z,current_yaw,
  target_yaw,yaw_error,current_segment_id,yaw_mode`.
- RViz topic `/manual_return/yaw_marker` (yellow arrow) showing the current
  heading reference.


## V2.0 Historical trajectory understanding (detection only)

V2.0 adds a **read-only** structural analysis of the forward history
(Home -> current position).  It does NOT delete points, does NOT change the
return path, and does NOT touch RDP / Safe-RDP / CommandGate.

New module `trajectory_analysis/` (headers in
`include/manual_return_planner/trajectory_analysis/`, sources in
`src/manual_return_planner/trajectory_analysis/`):

- `TrajectorySegmenter`: splits the forward history on direction changes
  (`segment_angle_threshold_deg`) and sudden speed changes.
- `DwellDetector`: finds maximal runs of consecutive slow points that are long
  enough (`dwell_min_time`) and spatially small (`dwell_radius`).
- `BacktrackDetector`: geometric + time dead-end spur detection (entry ~= exit,
  detour ratio, excursion, opposite direction, dead-end check).
- `OverlapDetector`: repeated-traversal detection between non-adjacent segments
  (spatial distance + direction alignment).
- `TrajectoryAnalysisManager`: orchestrates the four detectors and labels
  segments (`NORMAL` / `DWELL` / `BACKTRACK_CANDIDATE` / `OVERLAP_CANDIDATE`).

Configuration (`config/manual_return.yaml`):

```yaml
trajectory_analysis:
  segment_angle_threshold_deg: 30.0
  segment_speed_change_threshold: 0.5
  dwell_max_speed: 0.1
  dwell_radius: 0.15
  dwell_min_time: 2.0
  backtrack_epsilon_entry: 0.5
  backtrack_min_spur_ratio: 3.0
  backtrack_min_spur_reach: 0.5
  overlap_distance_threshold: 0.3
  overlap_angle_threshold_deg: 30.0
  overlap_min_segment_length: 0.5
```

Outputs (written per run):

```text
trajectory_analysis.csv          (per-segment: id, range, type, length, duration, confidence, reason)
trajectory_edit_candidates.csv   (per-candidate: id, type, range, confidence, reason, metrics)
```

RViz topic `/manual_return/trajectory_analysis_marker` (MarkerArray, one
LINE_STRIP per segment): blue = NORMAL, yellow = DWELL, red =
BACKTRACK_CANDIDATE, purple = OVERLAP_CANDIDATE.

Offline tool: `trajectory_analysis_benchmark <scenario> <csv>`.
Tests: `test/test_trajectory_analysis.cpp` (straight line is NORMAL, dead-end
spur is BACKTRACK, round-trip is OVERLAP, stationary hover is DWELL).


### V2.0 detector calibration (2026-08-24 real-flight fixes)

The first V2.0 build passed hand-built fixtures but silently detected **zero**
backtrack and **zero** overlap on a real 52.6 m flight that clearly contained
both.  Four defects were found and fixed:

1. **Dead-end check rejected every real spur.**  The check required that no
   interior point of `[i, j]` has any neighbour outside `[i, j]`.  On real data
   this can never hold: near the spur "neck" the outbound and return legs of the
   spur are inherently close to each other, which is precisely what defines a
   backtrack.  687 candidates passed all four geometric tests and all 687 were
   rejected.  Now only the **spur apex** (the point furthest from the entry) is
   tested for revisits outside `[i, j]`, controlled by
   `backtrack_apex_revisit_radius`.
2. **Overlap threshold was ~3x too small.**  `overlap_distance_threshold = 0.3`
   was a guessed initial value.  The outbound and return legs are planned
   independently, so their measured lateral offset is ~1.0 m.  Recalibrated to
   `1.2`.
3. **A pure distance test cannot reject collinear segments that merely touch
   end-to-end**, so simply relaxing the threshold would create false positives.
   Added `overlap_min_fraction` (0.5): a large fraction of points on **both**
   segments must be close, not just the average distance.
4. **`trajectory_analysis.csv` reported inconsistent reasons.**  The reason
   lookup used an *intersect* rule while the manager labels with a
   *fully-within* rule, so `NORMAL` segments were reported with a dwell reason.
   The lookup now uses fully-within **and** requires the candidate's implied
   label to equal the segment's label (`segmentTypeForEdit`), because labelling
   is by severity while confidence is independent.

Additionally, a dwell has a spatial extent of only a few millimetres, so its
`LINE_STRIP` was invisible at map scale.  Each dwell now also publishes a yellow
`SPHERE` at its centroid plus a `TEXT_VIEW_FACING` label with its metrics.

Verification after the fixes:

| trajectory | dwell | backtrack | overlap |
|---|---:|---:|---:|
| real flight (52.6 m, spur + reverse leg) | 4 | 2 | 2 |
| straight_long / straight_short / turn90 / s_curve / height_change | 1-3 | 0 | 0 |

`test/data/real_flight_backtrack_overlap.csv` is kept as a regression fixture:
`TrajectoryAnalysisRealFlight.*` asserts that real backtrack and overlap are
detected, so a future change cannot silently regress to "fixtures pass, real
data misses everything".
