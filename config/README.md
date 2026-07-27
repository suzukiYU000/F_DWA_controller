# Configuration

The common file contains parameters shared by V-DWB, A-DWA, J-DWA, and F-DWA.
Method files should contain only the plugin and native-dynamics differences.

`enable_certification: true` adds the common terminal-stop certificate and
retained-backup revalidation. It can be disabled through the research launch
for an ablation, while the common 70 ms nominal pose preview remains enabled.
`v_dwb.yaml` selects Nav2's `LimitedAccelGenerator`; the separate
`v_dwb_standard.yaml` selects `StandardTrajectoryGenerator`.

`f_dwa.yaml` uses the named ROS 1 `f-8.yaml` low-pass coefficients by default.
The paper-design replacement pattern is kept as comments beside that
coefficient block so the four-method comparison does not need another config
file. Replace the complete coefficient vector atomically; mixing design
metadata and coefficients from different filters is not a valid experiment.
