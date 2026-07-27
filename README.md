# F_DWA_controller

Nav2 controller plugins for a controlled comparison of acceleration-constrained
DWA (A-DWA), jerk-constrained DWA (J-DWA), and FIR-constrained DWA (F-DWA).

The package derives from Nav2 `dwb_core::DWBLocalPlanner` so that plan handling,
trajectory critics, costmap access, lifecycle behavior, and controller-server
integration remain common across the compared methods.

## Current status

This branch contains the initial buildable plugin scaffold.

Only `f_dwa_controller::CertifiedDWBLocalPlanner` is registered. With
`enable_certification: false`, it delegates to stock DWB. Enabling certification
fails during configuration because candidate-level certification, terminal stop
suffixes, and retained backups have not yet been implemented.

A-DWA, J-DWA, and F-DWA are intentionally not registered yet. This prevents an
incomplete controller from publishing motion commands under a research-method
name.

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
