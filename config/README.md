# Configuration

The common file contains parameters shared by V-DWB, A-DWA, J-DWA, F-DWA,
and the MPPI comparison baseline.
Method files should contain only the plugin and native-dynamics differences.

`enable_certification: true` adds the common terminal-stop certificate and
retained-backup revalidation. It can be disabled through the research launch
for an ablation, while the common 70 ms nominal command-history rollout remains
enabled.
`v_dwb.yaml` selects a behavior-preserving adapter for Nav2's
`LimitedAccelGenerator`; the separate `v_dwb_standard.yaml` selects the
corresponding `StandardTrajectoryGenerator` adapter. The adapters add no
rollout behavior. They export the Nav2 generators from the same plugin library
as `CertifiedDWBLocalPlanner`, avoiding a duplicate class-loader factory
registration after that library has already linked the generators.

The common dispatch-state parameters are:

- `odom_topic`: measured robot odometry used by the Controller Server and
  `StoppedGoalChecker` (`/whill/odom` on WHILL)
- `command_dispatch_topic`: stamped, sequenced robot-facing software dispatch
  events (`f_dwa_controller/msg/CommandDispatch`)
- `transport_valid_topic`: latched transport validity
- `require_command_dispatch_state`: reject planning before observable state
- `nominal_delay_preview_seconds`: future nominal delay; no future jitter sample
- `terminal_stop_command_delay_seconds`: delay included before terminal stop
- `terminal_stop_velocity_threshold`: common numerical capture threshold for
  velocity, controller-internal acceleration, and F-DWA raw-input history;
  internal state is cleared only after every applicable value is within it
- `stop_capture_velocity`: planned-stop completion threshold
- `planning_deadline_seconds`: 0.03 s deadline at 33.333 Hz

The unmodified Nav2 goal checker first requires measured translational and
angular velocity to be at or below 0.01. Standard Controller Server then sends
its normal task-completion zero through the delayed FIFO; it is not a
high-speed emergency override. The research runner does not close a run on the
FollowPath result alone: it waits until the FIFO is empty and the last applied
command is within the same stopped threshold, then rechecks pose, yaw, and
measured odometry. If delayed commands moved the robot outside the goal
condition, the runner resends the same saved Path within the original run
deadline. A new setPlan resets stateful DWB components before control resumes.
The research-common `RotateToGoal.xy_goal_tolerance_release_margin` is 0.03 m:
the rotate-only latch entered at 0.20 m is released beyond 0.23 m, inside the
0.25 m stopped-goal acceptance boundary. Its upstream-compatible default is
-1.0 (no automatic release), and the goal acceptance tolerance remains
0.25 m. This behavior is exported by
`f_dwa_controller::HysteresisRotateToGoalCritic`; Navigation2 source remains
unchanged.
The dispatch publisher is Reliable during a run and retains only its latest
state for late joiners. This gives a newly started planner or bag recorder an
initial observable state without replaying dispatches from an earlier trial.
The constructor's initial zero/valid/stopped publication is the sole startup
failsafe exception to Timer ownership. During a run and at trial reset, the
robot-facing command and dispatch are published only by the Timer callback,
which re-anchors its next period immediately after the robot-facing handoff.
An early callback less than 30 ms after the previous handoff neither consumes
the FIFO nor publishes.

Simulation uses `command_zero_threshold: 0.0` in the transport. The legacy
`zero_threshold` parameter remains a deprecated compatibility alias. It must
not be set to the 0.01 stop-capture value for the F-8 main comparison.
The 70 ms stop-command delay is conservatively rounded upward to three complete
30 ms command ticks for V/A/J/F certification.

`mppi.yaml` keeps the common 33.333 Hz, 2.4 s horizon, velocity and
acceleration limits, footprint, costmaps, and delayed command transport. Its
sampling distribution and critics are method-specific. MPPI does not inherit
the DWB terminal-stop certificate, so certification comparisons must report
that distinction explicitly.

`f_dwa.yaml` selects the named `f8` design by default and does not store a
coefficient vector. Before Nav2 starts, the research launch asks
`f_dwa_controller.fir_filter_design` to generate the coefficients and inserts
them only into its temporary runtime parameter file. A direct coefficient
vector in source YAML is rejected.

The Python design registry is the single place to set the tap count, design
sample frequency, low-pass cutoff or attenuation bands, window, and
minimum-phase conversion. The named F-7/F-8/F-9 profiles deliberately retain
the 20 Hz sample frequency used to generate the ROS 1 vectors. The ROS 2
controller still runs at 33.333 Hz; changing the design frequency defines a
new profile rather than silently changing F-8.

The paper-design profile-name pattern is kept as comments in `f_dwa.yaml`.
Define its complete design atomically in the Python registry before selecting
it. Coefficients are generated once before controller startup and are not
changed during an experiment run.

The primary comparison keeps `prefer_previous_selected_candidate: false`, so
V/A/J/F use the canonical DWB candidate order. Setting it to `true` is a
performance-only warm-start ablation and must be reported separately.

The runtime optimizations are exact reuse, not candidate pruning. F-DWA
prepares one free/unit held-FIR response per axis and forms all 11 axis
rollouts by affine composition. The fixed 80-element time-step vector is
created once, and nominal and terminal-stop heading trigonometry is cached per
angular rollout and shared by the 11 linear rollouts. Candidate count and
order, integration steps, critics, certification, and command-selection
semantics are unchanged. Regression tests compare all 121 nominal and stop
trajectories with the previous integration; angle integration is bit-exact and
FIR affine arithmetic differs from direct convolution by at most `1e-12`.
The scoring path reuses trajectory and critic-score storage between candidates.
On ticks without candidate-evaluation output it swaps only a newly selected
best trajectory into the result; evaluation ticks retain the same complete
121-candidate message. Safety-only checks materialize stop poses without the
unused velocity sequence, while the selected best still materializes its full
stop commands and FIR states for dispatch and backup retention. The 91-tap
held-unit terminal-stop response, which depends only on the frozen coefficient
vector and control period, is computed once at controller initialization and
shared by the 22 axis stop sequences. Cached and uncached F-8 stop inputs and
states are bit-exact in randomized regression tests. The costmap prefix
workspace also retains its allocation and overwrites every interior cell on
the next update. These changes preserve critic order, arithmetic order,
short-circuit behavior, certification, and selected-command semantics.

Every supported F-DWA launch reloads the method Config and regenerates the
selected Python profile. It never reuses a coefficient cache or a previous
temporary parameter file. Restart the complete launch after changing a Python
design; lifecycle deactivate/activate inside the same run deliberately retains
the frozen coefficients and FIR history.

For a continuous parameter-search batch, keep the launch alive and call
`/controller_server/FollowPath/reset_trial_state` followed by
`/command_delay_transport/reset_trial_state` at every stopped run boundary.
The transport service schedules its reset for the next command Timer tick;
wait for the fresh no-sequence dispatch, applied zero, valid=true, and
stopped=true before resetting the simulated pose. These
reset only run-specific state; the generated coefficient parameter remains
unchanged for the batch.
