# F_DWA_controller

Nav2 controller plugins for a controlled comparison of acceleration-constrained
DWA (A-DWA), jerk-constrained DWA (J-DWA), and FIR-constrained DWA (F-DWA).

The package derives from Nav2 `dwb_core::DWBLocalPlanner` so that plan handling,
trajectory critics, costmap access, lifecycle behavior, and controller-server
integration remain common across the compared methods.
Research-specific goal-window hysteresis and 10 Hz evaluation throttling are
implemented in this package. The pinned Navigation2 source remains unmodified.

## Current status

The DWB-derived A-DWA, J-DWA, and F-DWA controller class names and native-input
trajectory generators are registered. They preserve the 11 x 11 candidate
budget and use the common 2.4 s DWB rollout. Each cycle builds one immutable
planning snapshot from the latest TF pose, the last robot-facing software
dispatch, locally issued commands, and the configured 70 ms nominal delay.
Odometry velocity is not a nominal-state input.

`/controller/command_dispatch` is an
`f_dwa_controller/msg/CommandDispatch` event published only when a command is
handed to the robot-facing publisher. Its stamp is observable software dispatch
time, not an estimate of the physical motor-actuation instant. Its sequence ID
detects dropped, reordered, or ambiguous duplicate-velocity events. The
controller correlates each event with its FIFO command ledger. A/J state follows
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
F-DWA defaults to the named ROS 1 F-8 low-pass design. Its coefficients are
generated deterministically by Python before Nav2 starts, so no source YAML
contains a coefficient vector. Alternative design names remain as commented
patterns in `config/f_dwa.yaml`.

The design registry in `python/f_dwa_controller/fir_filter_design.py` owns the
tap count, design sample frequency, cutoff or attenuation bands, window, and
minimum-phase conversion. F-8 retains its historical 20 Hz design frequency,
which also matches the common Nav2 Controller Server frequency. A design using
a different frequency must receive a new name for experiment traceability.

Every F-DWA research-launch startup regenerates the selected profile. No
coefficient cache or previous temporary parameter file is reused. Coefficients
remain frozen for the complete run; applying a changed design requires a fresh
launch rather than an in-run lifecycle reactivation.

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
command per 33.333 Hz ROS-clock timer tick. The default distribution is bounded
to 60--80 ms with mean 70 ms and standard deviation 3.333 ms.

The transport publishes the command actually sent to the simulator on
`/controller/applied_cmd_vel` and publishes a stamped change event on
`/controller/command_dispatch`. A queue overflow publishes
`/dwa_experiment/transport_valid = false`, records queue and last-command data
on `/diagnostics`, clears the queue, and publishes zero thereafter. Such a run
is a transport-invalid run, not an algorithm failure, and must be excluded and
retried by the experiment runner.

FIR axes are rolled out 11 times for translation and 11 times for rotation,
then combined into the 121 pose candidates. A full-horizon projected FIR input
is applied without per-step re-projection; numerical constraint violations
invalidate the candidate rather than clipping its velocity. Critics run before
the terminal-stop certificate, and only candidates capable of improving the
best certified score receive that expensive certificate. Every configured
number of cycles, `planning_timing` logs p50/p95/p99/maximum and the cumulative
50 ms deadline-miss count using a steady clock. Trial reset emits the final
run summary before clearing these metrics.

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
