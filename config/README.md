# Configuration

Controller configurations will be added with the A-DWA, J-DWA, F-DWA, and
candidate-level certification implementations.

Until then, `f_dwa_controller::CertifiedDWBLocalPlanner` must be configured with
`enable_certification: false`. It then delegates trajectory generation and
scoring to Nav2 DWB without changing the selected command.
