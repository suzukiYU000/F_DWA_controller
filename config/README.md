# Configuration

The common file contains parameters shared by V-DWB, A-DWA, J-DWA, and F-DWA.
Method files should contain only the plugin and native-dynamics differences.

`enable_certification: true` adds the common terminal-stop certificate and
retained-backup revalidation. It can be disabled through the research launch
for an ablation, while the common 70 ms nominal pose preview remains enabled.
`v_dwb.yaml` selects Nav2's `LimitedAccelGenerator`; the separate
`v_dwb_standard.yaml` selects `StandardTrajectoryGenerator`.

F-DWA configuration is intentionally not installed until the F-8 coefficient
definition is confirmed.
