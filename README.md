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
a single cutoff number for a low-pass, or ordered attenuation bands such as
`[0.8, 1.6], [2.0, inf]`; `inf` means through the 10 Hz Nyquist frequency.
The selected effective tap count and 20 Hz design rate stay fixed.
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

### Input-duration sampling (opt-in F-DWA ablation)

`FollowPath.fir_prediction_pulse_durations` optionally appends durations in
seconds to the scalar `fir_prediction_pulse_duration` baseline. Its default
is an empty double array, preserving the original candidate set and order.
For example, with the scalar at `0.10`, `[0.20, 0.40]` evaluates three pulse
durations: up to `11 * 15 * 3 = 495` candidates. Twelve durations give up to
1980 candidates without replacing the existing amplitude grid. Actual counts
can be smaller when the horizon-feasible input interval is empty/collapsed.

- By default each duration is shared by the linear and angular input.
  `fir_independent_pulse_durations: true` additionally pairs different
  durations on the two axes: three durations give up to `11 * 15 * 3 * 3 =
  1485` candidates, from the same 78 axis responses as synchronized pairing.
  This permits, for example, continuing acceleration while ending the turn
  input earlier. Synchronized candidates remain first in canonical order.
  The prediction horizon is identical for all candidates. Zero means the full
  horizon, not a stop command.
- Durations round up to controller ticks. Duplicate tick counts are removed;
  negative, non-finite, or longer-than-horizon values are rejected.
- Each duration has its own horizon-feasible amplitude interval. Axis
  responses are computed `D * (Nv + Nw)` times and shared across `D * Nv * Nw`
  pose rollouts (`D * D * Nv * Nw` for independent pairing).
  Footprint/critic evaluation still scales with the pose-rollout count.
- Equal first native inputs share complete-stop axis responses and angular
  integration caches within one planning iteration, even if their nominal
  durations differ. Cache keys use exact input equality, separate axes, and
  the existing stop-policy arguments; caches do not cross state updates.
- A raw input is zero after its pulse but the FIR history continues normally.
  Only the first filtered command is dispatched, followed by replanning.
  Equal first commands with different future trajectories are not duplicates.
- The parameter is adopted at initialization or the stopped trial-reset
  boundary. Jerk constraints, V/A/J generation, emergency stops, and the
  independent complete-stop certificate are unchanged. Candidate metadata
  records the effective `linear_prediction_input_duration` and
  `angular_prediction_input_duration`.

Long pulses can overpredict a turn that repeated replanning defers. This option
therefore remains disabled in the default experiment configuration until
closed-loop tests establish progress, clearance, and timing. Treat it as a
separate F-DWA sampling ablation, not an unlabelled V/A/J/F comparison.

### Equal-effect, maximum-duration sampling (opt-in method C)

`FollowPath.fir_equal_effect_max_duration_sampling: true` enables the
simulation research condition labelled method C. For every raw-input
amplitude `q`, it finds the longest full-tail-feasible pulse
`q,...,q,beta*q,0,...` and measures the signed displacement over the nominal
horizon. Sampling uses two stages. For 11 linear samples it first evaluates 6
uniform `q` values, then assigns the remaining 5 evaluations greedily to the
coarse interval with the largest predicted effect gap
`abs(s[i+1]-s[i]) / (k[i]+1)`. The assigned points divide that `q` interval
uniformly. The 15 angular samples use the same 8+7 split. Thus the two axes
require exactly 11+15=26 pulse evaluations before their Cartesian product
forms 165 trajectories.

This bounded allocation approximates equal effect spacing from the coarse
secants; it does not invert exact effect targets or search for unobserved
branches. The coarse even-sized grids do not contain `q=0`, and the refinement
allocation may or may not place a point there. Both configured sample counts
remain odd so that the two stages have `ceil(n/2)` coarse and `floor(n/2)`
refined evaluations.

This mode cannot be combined with recovery, extra pulse-duration banks, or
independent linear/angular duration pairing. Candidate metadata records the
fractional last input, exact equivalent duration, and, when sampled, the
unbounded-duration label for `q=0`. If any of the 26 axis evaluations is
infeasible, the generator emits a throttled warning and uses the configured
fixed-duration bank for that iteration. The option is off by default and is
not evidence of closed-loop or real-WHILL superiority.

The opt-in `DISABLED_FirSamplingBenchmark` gtest compares the baseline,
495-duration, 1767-amplitude, 1827-hybrid, 1485/2079-independent, and
1980-duration candidate sets at a 2.5 s
horizon and 20 Hz. Supply comma-separated taps from
`design_fir_coefficients_for_cutoff` in `F_DWA_BENCHMARK_COEFFICIENTS` and run
with `--gtest_also_run_disabled_tests --gtest_filter=*FirSamplingBenchmark`.
It measures axis preparation, nominal pose integration, and complete-stop
generation separately, using five warmups and thirty measured iterations.
It does **not** measure costmap/critics, ROS transport, rendering, or
closed-loop navigation success.

Measured on 2026-09-05 in the ROS Jazzy development container, using the
Python-designed 0.8 Hz / 46-tap filter, initial `(v, w) = (0.4, 0.1)`, zero
FIR history, velocity limits `(0.8, 0.8)`, acceleration/raw-input limits
`(1.2, 1.57)`, 0.05 s steps, and a 2.5 s horizon:

| Sampling | Candidates | Generation + stop p50 [ms] | p95 [ms] |
|---|---:|---:|---:|
| 11 x 15, scalar 0.10 s (unchanged default) | 165 | 1.165 | 1.372 |
| 11 x 15, three synchronized durations | 495 | 2.070 | 2.424 |
| 31 x 57, scalar 0.10 s | 1767 | 5.001 | 5.801 |
| 21 x 29, three synchronized durations | 1827 | 4.387 | 5.056 |
| 11 x 15, three independent durations | 1485 | 2.553 | 2.879 |
| 11 x 21, three independent durations | 2079 | 3.063 | 3.865 |
| 11 x 15, twelve synchronized durations | 1980 | 12.141 | 13.243 |

Three durations are `[0.10, 0.20, 0.40]`. Independent pairing is the preferred
next closed-loop ablation: it increases temporal diversity without computing
many separate duration responses. The 2079 case uses 96 axis responses,
compared with 312 for twelve synchronized durations. These are microbenchmark
times, not complete Controller Server deadlines or evidence of navigation
success. No real run or AMCL/Gazebo completion is established by these tests.
Defaults and the running real controller are not switched to these profiles.

The candidate regression suite passes 80 tests, including scalar/bank
trajectory and stop equivalence at rest, moving, and near saturation;
independent-duration FIR convolution at every prediction point; pulse tick
deduplication; and atomic stopped-boundary parameter reload.

Run from the ROS workspace in the development container after building the
test target. Point the loader at the build library to avoid mixing new C++
headers/tests with an older experiment-install library:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92
export LD_LIBRARY_PATH="$PWD/build/f_dwa_controller:$LD_LIBRARY_PATH"
export F_DWA_BENCHMARK_COEFFICIENTS=$(PYTHONPATH="$PWD/src/third_party/F_DWA_controller/python" python3 -c 'from f_dwa_controller.fir_filter_design import design_fir_coefficients_for_cutoff; print(",".join(map(str, design_fir_coefficients_for_cutoff(0.8))))')
build/f_dwa_controller/test_native_input_trajectory_generator \
  --gtest_also_run_disabled_tests --gtest_filter='*FirSamplingBenchmark'
```

The design registry in `python/f_dwa_controller/fir_filter_design.py` owns the
tap count, design sample frequency, cutoff or attenuation bands, window, and
minimum-phase conversion. F-8 retains its historical 20 Hz design frequency,
which also matches the common Nav2 Controller Server frequency. A design using
a different frequency must receive a new name for experiment traceability.

Every F-DWA research-launch startup regenerates the selected filter design. No
coefficient cache or previous temporary parameter file is reused. Coefficients
remain frozen for the complete run. Between runs, the experiment GUI may set
the canonical filter specification, coefficients, and generated marker
atomically; the trial-reset
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

With `enable_no_valid_control_deceleration_fallback`, an all-invalid obstacle
cycle remains inside the selected V/A/J/F-DWA dynamics. The planner first uses
only collision-certified stop or short-horizon recovery responses. If the live
obstacle already makes every native response invalid at the next control step,
it dispatches for one cycle the original native candidate with the latest
predicted collision, breaking ties by the ordinary weighted objective with the
failed `ObstacleFootprint` gate omitted. This prevents Controller Server from
injecting an algorithm-external zero while preserving J-DWA jerk history and
F-DWA FIR history. The final response is tagged
`MethodNativeLeastViolationRecovery` and is not collision-certified;
independent emergency stopping remains authoritative.

## Exact footprint evaluation reuse

`HorizonObstacleFootprintCritic` reuses Nav2 edge scores when the complete
ordered list of footprint vertices maps to identical Costmap cells. It uses
the original Nav2 transform, rasterizer, numeric costs, and failure ordering.
This is not pose quantization: the continuous polygon/cell and swept-body
certificates still use their original coordinates and interpolation samples.

Repeated cell/layer diagnostic text is reused for identical cell indices,
cost, bit-identical world coordinates, and diagnostic prefix. Pose diagnostics
and margin recovery still run separately for each complete candidate. Both caches are cleared in every
`prepare()` under the planner's locked Costmap snapshot, including failed
preparations. Each holds at most 4096 entries. Cache misses, capacity limits,
off-grid vertices, and footprints exceeding the raster key capacity use the
unchanged checks. No cost weight, safety margin, FIR history, jerk bound,
sampling interval, or command dispatch rule is changed.

Intermediate nominal-footprint checks can also be omitted when the existing
hazard-prefix grid proves that the full swept segment's enclosing AABB is free
of both lethal and unknown cells. The bound uses the maximum vertex radius,
so it includes rear-corner rotation, with outward rounding and an extra cell
of padding. Unknown/off-grid/inconclusive regions use the original checks;
finite-input validation and subdivision limits remain unchanged.

Continuous polygon/cell certification also reuses the immediately preceding
pose result when the complete local footprint, pose coordinates, and obstacle
policy are exactly equal. This skips repeated transforms in FIR stop tails
without hashing or accumulating a large per-cycle table. One entry is allocated
lazily per workspace and invalidated with the locked Costmap snapshot. Changed
map geometry or unknown-space policy disables reuse; overlap-depth requests
always run the original depth calculation. Larger footprints use the uncached
path. No neighboring poses are merged.

Separating-axis preparation is deferred until an actual hazard cell overlaps
the polygon's exact continuous AABB. Axis normalization and overlap-depth
division are omitted only when the caller does not request a depth; the
original intersection inequalities are unchanged. Release builds use `-O3`
only for `trajectory_certifier.cpp`, without fast-math or native-CPU flags.
FIR and jerk arithmetic keep their existing compilation options.

The opt-in offline benchmarks include an 8100-case narrow-corridor corpus and
isolated Footprint scoring of saved candidate messages. Comparisons preserve
full scores, rejection diagnostics, swept-body certificates, and margin recovery
results. ROS message fields are compared directly, including floating-point
bits; unused serialization padding is not treated as controller output.

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
