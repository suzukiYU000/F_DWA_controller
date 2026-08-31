# F_DWA_controller

Nav2 controller plugins for a controlled comparison of acceleration-constrained
DWA (A-DWA), jerk-constrained DWA (J-DWA), and FIR-constrained DWA (F-DWA).

The package derives from Nav2 `dwb_core::DWBLocalPlanner` so that plan handling,
trajectory critics, costmap access, lifecycle behavior, and controller-server
integration remain common across the compared methods.
Research-specific goal-window hysteresis and 5 Hz evaluation throttling are
implemented in this package. The pinned Navigation2 source remains unmodified.

## Current status

The DWB-derived A-DWA, J-DWA, and F-DWA controller class names and native-input
trajectory generators are registered. The common source configuration uses an
11 x 15 candidate budget and a 1.6 s rollout; the experiment GUI applies its
centralized planning defaults when it launches a run. Each cycle builds one
immutable planning snapshot from the latest TF pose, the last robot-facing
software dispatch, locally issued commands, and the configured 70 ms nominal
delay. Odometry velocity is not a nominal-state input.

The clearance critics first apply result-preserving caches. Physical-footprint
boundary samples and padded polygons are rebuilt only when the exact footprint,
costmap resolution, clearance margin, sampling resolution, or band count
changes. When the primary and trigger `FootprintClearanceCritic` instances have
exactly equal transformed paths and fixed-distance path parameters, the primary
critic builds the sampled risk path once and both critics score that same pose
sequence against their own distance fields. Scores, weights, thresholds,
short-circuit order, and candidate enumeration remain independent and
unchanged. Periodic and trial-end logs report this as
`clearance_risk_path_cache`; `avoided_path_builds` is the number of duplicate
path constructions skipped since the last trial reset.

Pose scoring also skips the physical-footprint probe loop only when a strict
distance-field lower bound proves that the original score is exactly `0.0`.
The bound includes map-edge clearance, the complete physical-footprint radius,
Costmap cell extents, and the maximum unsampled boundary interval. Any pose near
an obstacle or map edge, or any numerically inconclusive pose, follows the
original probe-by-probe implementation. Penalized-cell mask comparison is fused
into the mandatory source-grid scan; the exact mask still controls whether the
Euclidean distance field is rebuilt.

The soft clearance exposure of a terminal stopping rollout is additionally
sampled by traveled distance and heading change. The configured spatial
resolution and a 0.10 rad angular limit determine the sampled poses, while the
integral retains the skipped time interval. This is an intentional approximation
of candidate ranking, not a result-preserving cache. Hard stop admissibility and
collision checks still inspect every original rollout pose, so the optimization
cannot make a colliding stop rollout legal merely by skipping a soft-cost sample.

The configured soft band also gains a bounded motion-uncertainty allowance.
For each already-generated candidate, the critic derives maximum boundary
sweep speed from pose translation plus physical-footprint-radius times heading
change. The common 0.04 s uncertainty bound can add at most 0.05 m to the soft
margin. The same calculation uses the 0.05 s samples of the method-native stop
sequence. It reads generated motion and never modifies sampled acceleration,
jerk, FIR state, rollout poses, or the returned command. Entering this expanded
band remains a finite score; physical swept-footprint and complete-stop checks
remain the only hard gates.

`/controller/command_dispatch` is an
`f_dwa_controller/msg/CommandDispatch` event published only when a command is
handed to the robot-facing publisher. Its stamp is observable software dispatch
time, not an estimate of the physical motor-actuation instant. Its sequence ID
detects dropped, reordered, or ambiguous duplicate-velocity events. The FIFO
reception event also carries a same-host monotonic-clock timestamp; this is the
causal key used to exclude numerically identical Controller results issued only
after the command entered the transport. ROS time is retained for diagnostics,
but is not used for this comparison because accelerated `/clock` samples are
not synchronized across processes. The controller correlates each event with
its FIFO command ledger. A/J state follows
that ledger; F-DWA carries the selected raw input and FIR state as metadata.
Inverse FIR reconstruction from differentiated velocity is not used by the
certified path. An unmatched nonzero dispatch invalidates the certificate
instead of inventing state.

The common controller replays already issued commands to the next nominal
activation time and certifies that shared delay trajectory once. Candidate
certification then includes the candidate held over the configured stop-command
delay, its dynamically feasible stop sequence, and retained-backup
revalidation. It checks the complete padded footprint interior, rejects unknown
or off-costmap poses, and interpolates swept motion at no more than half the
0.05 m costmap resolution.

Command quantization and stop capture are separate. Simulation defaults to
`command_zero_threshold: 0.0`, while `stop_capture_velocity: 0.01` is used only
to finish a planned stop and clear its native state. This preserves F-8's small
startup increments. Recovery candidates that start inside the certificate
margin and move outward remain pending and are disabled by default.
F-DWA defaults to a numeric 1.2 Hz low-pass cutoff, which preserves the ROS 1
F-8 design. Its coefficients are generated deterministically by Python before
Nav2 starts, so no source YAML contains a coefficient vector. The GUI accepts
the cutoff in Hz; the established tap count and 20 Hz design rate stay fixed.
The nominal F-DWA rollout uses a two-control-cycle finite-pulse action
primitive: the sampled raw input is active for 0.10 s and is then zero for the
remainder of the scoring horizon. This is the shortest 20 Hz pulse that
completed the fixed room-entrance replay; a one-cycle 0.05 s pulse did not
produce enough predicted progress. The historical ROS 1 0.15 s pulse remains an
explicit ablation; using it while dispatching only the first step can
repeatedly overpredict a future filtered turn. Every horizon state is still
checked against the velocity and acceleration limits, while the independent
stop certificate continues through delayed activation and complete stopping.
Setting
`fir_prediction_pulse_duration: 0.0` restores the former full-horizon held
input as an explicit ablation.

The design registry in `python/f_dwa_controller/fir_filter_design.py` owns the
tap count, design sample frequency, cutoff or attenuation bands, window, and
minimum-phase conversion. F-8 retains its historical 20 Hz design frequency,
which also matches the common Nav2 Controller Server frequency. A design using
a different frequency must receive a new name for experiment traceability.

Every F-DWA research-launch startup regenerates the selected numeric design. No
coefficient cache or previous temporary parameter file is reused. Coefficients
remain frozen for the complete run. Between runs, the experiment GUI may set
the cutoff, coefficients, and generated marker atomically; the trial-reset
boundary then adopts them while no candidate evaluation or motion is active.

## Continuous simulation batches

A parameter-search batch may keep `controller_server` and the command-delay
transport alive. FIR coefficients are generated and set once when the batch
launch starts. At each run boundary, first cancel and await completion of the
old FollowPath action, then pause and reset the simulated robot. Call these
services in order:

```text
/controller_server/FollowPath/reset_trial_state
/command_delay_transport/reset_trial_state
```

Both use `std_srvs/srv/Trigger`. The controller reset clears the current
candidates, retained stop backup, critic state, global path, and A/J/F native
state, while retaining the configured FIR coefficients. The transport reset
service schedules its FIFO, last-applied command, sequence, validity, and random
generator reset for the next command Timer tick. At that boundary it publishes
the robot-facing zero, applied zero, no-sequence reset dispatch, valid=true, and
stopped=true together. Service success means "scheduled"; wait for those fresh
Timer-boundary states before resetting the Gazebo pose or submitting the next
saved Path.

The order is intentional. The Controller ledger is cleared before the
transport establishes the next applied-state epoch, avoiding a stale reset
dispatch being interpreted as part of the previous ledger.
Calling the Controller reset while the old action or robot is still moving
violates this trial-boundary precondition.

Set `/command_delay_transport.random_seed` before its reset when a run needs a
different seed. Reusing the same seed intentionally reproduces the same jitter
sequence for fair method or parameter comparisons.

The package also provides `command_delay_transport`, a simulation-only command
transport. It receives Nav2 commands, samples an independent truncated-normal
delay for every command, preserves FIFO order, and applies at most one queued
command per 33.333 Hz steady-clock timer tick. The default sampled distribution is
bounded to 5--35 ms with mean 20 ms and standard deviation 5 ms. Together with
the Timer phase this reproduces the measured 11--71 ms real dispatch range
independently of the Gazebo Real Time Factor.

When `enable_velocity_response_model` is enabled, the transport then applies
the identified axis-specific dead-time plus first-order WHILL response before
sending velocity to Gazebo. `/controller/applied_cmd_vel` and the stamped
`/controller/command_dispatch` event retain the actuator target, matching real
mode; simulator odometry reports the resulting motion. A queue overflow publishes
`/dwa_experiment/transport_valid = false`, records queue and last-command data
on `/diagnostics`, clears the queue, and publishes zero thereafter. Such a run
is a transport-invalid run, not an algorithm failure, and must be excluded and
retried by the experiment runner.

FIR axes are rolled out 11 times for translation and 11 times for rotation,
then combined into the 121 pose candidates. A full-horizon projected FIR input
profile is affine-sampled without per-step re-projection. Numerical constraint
violations invalidate the candidate rather than clipping its velocity. Critics
run before the terminal-stop certificate, and only candidates capable of
improving the best certified score receive that expensive certificate. Every configured
number of cycles, `planning_timing` logs p50/p95/p99/maximum and the cumulative
50 ms deadline-miss count using a steady clock. Trial-end
`planning_detail_timing` records candidate generation, stop-rollout generation,
the three clearance stages, weighted critic scoring, terminal-stop safety, and
each critic's measured call time. The stage timers overlap by design and their
shares must not be summed. `certificate_rejections`
separates terminal-stop infeasibility from invalid-input, off-costmap,
lethal-obstacle, and unknown-space failures. Trial reset emits the final run
summary before clearing these metrics.

## Planned hierarchy

```text
dwb_core::DWBLocalPlanner
└── f_dwa_controller::CertifiedDWBLocalPlanner
    ├── f_dwa_controller::ADwaController
    ├── f_dwa_controller::JDwaController
    └── f_dwa_controller::FDwaController
```

V-DWB uses `f_dwa_controller::CertifiedDWBLocalPlanner` so the nominal delay
preview remains common even when certification is disabled. Its default
trajectory generator is Nav2's `dwb_plugins::LimitedAccelGenerator`. A separate
configuration selects `dwb_plugins::StandardTrajectoryGenerator` and enables
`limit_vel_cmd_in_traj` so the command sent to the robot is its first
acceleration-limited executable sample.

## Source compatibility

Research builds pin the Navigation2 source tree to:

```text
f0a10d95be09c72e45e4856019b377bbc3b0ee70
```

The external Dynamic Window Pure Pursuit controller is retained as a packaging
and plugin-integration reference. Its Pure Pursuit algorithm is not copied into
this package.

## License

This repository is licensed under the MIT License. Navigation2 DWB remains a
separate BSD-licensed dependency. Any future source-derived changes must retain
the applicable upstream copyright and license notices.
