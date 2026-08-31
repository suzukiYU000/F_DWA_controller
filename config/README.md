# Configuration

The common file contains parameters shared by V-DWB, A-DWA, J-DWA, F-DWA,
and the MPPI comparison baseline.
Method files should contain only the plugin and native-dynamics differences.

`enable_certification` is `false` in every normal controller configuration.
Setting it to `true` is reserved for an explicit terminal-stop-certificate
ablation; normal experiments do not reject candidates with that certificate.
The common Config retains a method-native stop suffix once its predicted end
is within the GoalChecker's complete 0.20 m position window. This prevents the stopped
GoalChecker from losing the final Path point while a fast candidate is still
crossing the goal. The
common nominal command-history rollout remains enabled.
DWB-derived native-input generators keep the selected candidate's
internal acceleration, jerk, or FIR state correlated with the observable
dispatch ledger in either mode; disabling certification changes neither the
candidate dynamics nor state ownership.
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
- `allow_safety_command_reduction`: real-mode-only permission to correlate one
  same-sequence FIFO-head command that a downstream safety monitor only reduced
  or zeroed; simulation keeps this `false` and requires exact command equality
- `nominal_delay_preview_seconds`: future nominal delay; no future jitter sample
- `use_observed_velocity_for_activation_state`: starts each dynamic window from
  Controller Server odometry while retaining dispatch/native ledgers for known
  future transport and A/J/F internal state; this avoids treating a dispatched
  command as measured wheel speed
- `enable_velocity_response_prediction`: common activation-state predictor;
  starts from measured odometry and approaches the latest robot-facing command
  with the identified dead-time plus first-order WHILL response instead of
  reanchoring every cycle to delayed odometry or treating command as velocity
- `velocity_response_prediction_seconds`: prediction horizon from the current
  measurement; the real experiment uses 0.12 s
- `*_velocity_response_dead_time_seconds`,
  `*_velocity_response_time_constant_seconds`, and
  `*_velocity_response_gain`: independently identified linear/angular plant
  parameters, common to V/A/J/F; simulation applies the same response before
  Gazebo and uses it for candidate activation
- `enable_stop_admissibility`: standard Dynamic Window physical stopping
  condition; a candidate is legal only when its method-native deceleration can
  stop the physical footprint on the current costmap, without the optional
  certificate reserve
- `FootprintClearance.motion_uncertainty_seconds`: timing-uncertainty horizon
  used only by the soft clearance rank; the common configuration uses 0.04 s,
  rounded above the measured dispatch p05--p95 spread without duplicating the
  nominal activation preview
- `FootprintClearance.maximum_motion_margin`: upper bound on the extra soft
  margin produced by generated translation and footprint-corner rotation;
  0.05 m in the common comparison configuration
- `FootprintClearance.localization_uncertainty_margin`: worst-case
  translational pose error outside the measured body; the common 0.10 m band
  extends one continuous clearance-risk curve down to physical contact and
  never makes a trajectory illegal by itself
- `localization_uncertainty_footprint_inset`: bounded localization-error
  recovery. Bringup maps the measured pose-error setting (normally `0.10 m`)
  here. It is admissible only when every current Obstacle/Voxel observation
  layer certifies the inward uncertainty core. Ordinary rollout scoring
  requires overlap in the outer error band to be non-growing and self-clearing.
  A retained method-native recovery prefix may re-enter only that outer band;
  its complete inward core and all current observation layers remain hard-
  certified, so this exception does not bypass J-DWA jerk or F-DWA FIR state.
- `FootprintClearance.uniform_sequence_period`: period of the generated
  method-native stopping poses used to recover their sweep rate; 0.05 s at the
  common 20 Hz Controller rate
- `terminal_stop_command_delay_seconds`: delay included before terminal stop
- `terminal_stop_velocity_threshold`: common numerical capture threshold for
  velocity, controller-internal acceleration, and F-DWA raw-input history;
  internal state is cleared only after every applicable value is within it
- `terminal_stop_maximum_time`: upper bound for constructing a certified stop,
  not a commanded stop duration; the common 12 s bound accommodates the F-8
  filter-history drain and each sequence ends as soon as capture is reached
- `terminal_stop_goal_capture_distance`: 0.20 m common method-native terminal
  capture set, matching GoalChecker; each cycle revalidates the retained
  deceleration suffix
- `stop_capture_velocity`: planned-stop completion threshold
- `planning_deadline_seconds`: 0.05 s deadline at the common 20 Hz control rate

`forward_prune_distance` is 4 m for every DWB-derived method. This exceeds the
maximum nominal travel of a 0.6 m/s, 2.4 s candidate (1.44 m) while remaining
inside the 10 m square local costmap. The path critics therefore see a real
Path segment rather than an artificial 2 m transformed-Path endpoint.

The unmodified Nav2 goal checker first requires measured translational and
angular velocity to be at or below 0.01. Standard Controller Server then sends
its normal task-completion zero through the delayed FIFO; it is not a
high-speed emergency override. The research runner does not close a run on the
FollowPath result alone: it waits until the FIFO is empty and the last applied
command is within the same stopped threshold, then rechecks pose, yaw, and
measured odometry. If delayed commands moved the robot outside the goal
condition, the runner resends the same saved Path within the original run
deadline. A new setPlan resets stateful DWB components before control resumes.
The research-common stopping critic, GoalChecker, and retained method-native
stop use the same 0.20 m position window so Controller Server cannot finish
outside the method-native stop latch. Its
`RotateToGoal.xy_goal_tolerance_release_margin` is -1.0, so terminal braking
remains latched after the robot first enters the acceptance boundary and is
cleared only when a new Path resets the critic. This prevents delayed motion
from releasing the brake phase and accelerating away from the final Path
point. This behavior is exported by
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
The transport Timer, sampled command delay, input-interval check, and pacing
gate use the steady wall clock because they represent software/actuator
latency and the upstream Controller Server also produces commands in wall
time. ROS timestamps remain on dispatch evidence, and the simulated WHILL
dead-time and first-order response continue to advance in Gazebo ROS time.
This separation prevents a Real Time Factor below 1.0 from turning a healthy
20 Hz producer into an artificial FIFO overflow.
Upstream Controller Server arrival intervals are not used to certify this
robot-facing boundary: an early producer callback is retained in the bounded
FIFO, while queue overflow still invalidates the trial.

Simulation uses `command_zero_threshold: 0.0` in the transport. The legacy
`zero_threshold` parameter remains a deprecated compatibility alias. It must
not be set to the 0.01 stop-capture value for the F-8 main comparison.
The 70 ms stop-command delay is conservatively rounded upward to two complete
50 ms controller ticks for V/A/J/F certification. The independent
robot-facing transport still publishes at no more than 33.333 Hz.

`mppi.yaml` keeps the common 20 Hz, 2.4 s horizon, velocity and
acceleration limits, footprint, costmaps, and delayed command transport. Its
sampling distribution and critics are method-specific. MPPI does not inherit
the DWB terminal-stop certificate, so certification comparisons must report
that distinction explicitly.

`f_dwa.yaml` selects the named `f8` design by default and does not store a
coefficient vector. Before Nav2 starts, the research launch asks
`f_dwa_controller.fir_filter_design` to generate the coefficients and inserts
them only into its temporary runtime parameter file. A direct coefficient
vector in source YAML is rejected.
`max_linear_raw_input` bounds the FIR input, which has acceleration units.
Increasing it does not bypass the per-step velocity and acceleration
projection. In the ground-truth Env2 diagnostic, 1.8 and 2.4 did not produce a
repeatable valid improvement over 1.2, so 1.2 remains the comparison default.

The Python design registry is the single place to set the tap count, design
sample frequency, low-pass cutoff or attenuation bands, window, and
minimum-phase conversion. The named F-7/F-8/F-9 profiles deliberately retain
the 20 Hz sample frequency used to generate the ROS 1 vectors. The ROS 2
Controller Server also runs at 20 Hz; changing the design frequency defines a
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
rollouts by affine composition. The fixed 48-element time-step vector is
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
