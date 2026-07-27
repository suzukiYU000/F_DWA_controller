# F_DWA_controller

Nav2 controller plugins for a controlled comparison of acceleration-constrained
DWA (A-DWA), jerk-constrained DWA (J-DWA), and FIR-constrained DWA (F-DWA).

The package derives from Nav2 `dwb_core::DWBLocalPlanner` so that plan handling,
trajectory critics, costmap access, lifecycle behavior, and controller-server
integration remain common across the compared methods.

## Current status

The DWB-derived A-DWA, J-DWA, and F-DWA controller class names are registered.
A-DWA and J-DWA also have native-input trajectory generators. They preserve the
11 x 11 candidate budget, use the common 2.4 s DWB rollout, and project each
native input into a remaining-horizon feasible interval. J-DWA reconstructs its
acceleration state from `/controller/applied_cmd_vel`.

Candidate-level collision certification, terminal stop suffixes, and retained
backup revalidation are still in progress. Consequently the A/J configurations
set `enable_certification: true` and fail closed during controller
configuration. Setting it to false is limited to generator development and is
not a certified comparison run. F-DWA remains without a trajectory generator
until the exact F-8 coefficient definition is selected.

The package also provides `command_delay_transport`, a simulation-only command
transport. It receives Nav2 commands, samples an independent truncated-normal
delay for every command, preserves FIFO order, and applies at most one queued
command per 33.333 Hz ROS-clock timer tick. The default distribution is bounded
to 60--80 ms with mean 70 ms and standard deviation 3.333 ms.

The transport publishes the command actually sent to the simulator on
`/controller/applied_cmd_vel`. A queue overflow publishes
`/dwa_experiment/transport_valid = false`, records queue and last-command data
on `/diagnostics`, clears the queue, and publishes zero thereafter. Such a run
is a transport-invalid run, not an algorithm failure, and must be excluded and
retried by the experiment runner.

## Planned hierarchy

```text
dwb_core::DWBLocalPlanner
└── f_dwa_controller::CertifiedDWBLocalPlanner
    ├── f_dwa_controller::ADwaController
    ├── f_dwa_controller::JDwaController
    └── f_dwa_controller::FDwaController
```

V-DWB uses `dwb_core::DWBLocalPlanner` directly when common certification is
disabled. When certification is enabled, it uses
`f_dwa_controller::CertifiedDWBLocalPlanner` with Nav2's
`dwb_plugins::LimitedAccelGenerator`. A separate configuration can select
`dwb_plugins::StandardTrajectoryGenerator`.

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
