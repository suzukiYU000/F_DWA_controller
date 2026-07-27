# F_DWA_controller

Nav2 controller plugins for a controlled comparison of acceleration-constrained
DWA (A-DWA), jerk-constrained DWA (J-DWA), and FIR-constrained DWA (F-DWA).

The package derives from Nav2 `dwb_core::DWBLocalPlanner` so that plan handling,
trajectory critics, costmap access, lifecycle behavior, and controller-server
integration remain common across the compared methods.

## Current status

The DWB-derived A-DWA, J-DWA, and F-DWA controller class names and native-input
trajectory generators are registered. They preserve the 11 x 11 candidate
budget, use the common 2.4 s DWB rollout, and project each native input into a
remaining-horizon feasible interval. J-DWA reconstructs its acceleration state
from `/controller/applied_cmd_vel`. F-DWA reconstructs its FIR input history
from the same stream; a transport-induced hold is retained as disturbance
history while future selected raw inputs remain bounded.

The common controller applies the 70 ms nominal activation preview before both
DWB cost evaluation and safety certification. Certification connects a
dynamics-feasible stop sequence after the first executable sample, checks the
complete padded footprint interior on the local costmap, rejects unknown or
off-costmap poses, and interpolates the sweep at no more than half the 0.05 m
costmap resolution. The selected stop suffix is retained. If no ordinary
candidate is legal on a later cycle, the remaining suffix is rebuilt from the
current preview pose and revalidated against the current costmap before use.

The 0.01 velocity capture tube is a deliberate hybrid transition: the simulator
transport converts components inside it to exact zero and the associated
controller state is cleared. Recovery candidates that start inside the
certificate margin and move outward remain pending and are disabled by default.
F-DWA defaults to the exact named ROS 1 F-8 low-pass coefficient vector.
Alternative filter designs are recorded as commented, atomic
coefficient-replacement patterns in `config/f_dwa.yaml`.

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
