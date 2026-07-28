# Configuration

The common file contains parameters shared by V-DWB, A-DWA, J-DWA, F-DWA,
and the MPPI comparison baseline.
Method files should contain only the plugin and native-dynamics differences.

`enable_certification: true` adds the common terminal-stop certificate and
retained-backup revalidation. It can be disabled through the research launch
for an ablation, while the common 70 ms nominal command-history rollout remains
enabled.
`v_dwb.yaml` selects Nav2's `LimitedAccelGenerator`; the separate
`v_dwb_standard.yaml` selects `StandardTrajectoryGenerator`.

The common dispatch-state parameters are:

- `command_dispatch_topic`: stamped, sequenced robot-facing software dispatch
  events (`f_dwa_controller/msg/CommandDispatch`)
- `transport_valid_topic`: latched transport validity
- `require_command_dispatch_state`: reject planning before observable state
- `nominal_delay_preview_seconds`: future nominal delay; no future jitter sample
- `terminal_stop_command_delay_seconds`: delay included before terminal stop
- `stop_capture_velocity`: planned-stop completion threshold
- `planning_deadline_seconds`: 0.03 s deadline at 33.333 Hz

Simulation uses `command_zero_threshold: 0.0` in the transport. The legacy
`zero_threshold` parameter remains a deprecated compatibility alias. It must
not be set to the 0.01 stop-capture value for the F-8 main comparison.
The 70 ms stop-command delay is conservatively rounded upward to three complete
30 ms command ticks for V/A/J/F certification.

`mppi.yaml` keeps the common 33.333 Hz, 2.4 s horizon, velocity and
acceleration limits, footprint, costmaps, and delayed command transport. Its
sampling distribution and critics are method-specific. MPPI does not inherit
the DWB terminal-stop certificate, so certification comparisons must report
that distinction explicitly.

`f_dwa.yaml` selects the named `f8` design by default and does not store a
coefficient vector. Before Nav2 starts, the research launch asks
`f_dwa_controller.fir_filter_design` to generate the coefficients and inserts
them only into its temporary runtime parameter file. A direct coefficient
vector in source YAML is rejected.

The Python design registry is the single place to set the tap count, design
sample frequency, low-pass cutoff or attenuation bands, window, and
minimum-phase conversion. The named F-7/F-8/F-9 profiles deliberately retain
the 20 Hz sample frequency used to generate the ROS 1 vectors. The ROS 2
controller still runs at 33.333 Hz; changing the design frequency defines a
new profile rather than silently changing F-8.

The paper-design profile-name pattern is kept as comments in `f_dwa.yaml`.
Define its complete design atomically in the Python registry before selecting
it. Coefficients are generated once before controller startup and are not
changed during an experiment run.

Every supported F-DWA launch reloads the method Config and regenerates the
selected Python profile. It never reuses a coefficient cache or a previous
temporary parameter file. Restart the complete launch after changing a Python
design; lifecycle deactivate/activate inside the same run deliberately retains
the frozen coefficients and FIR history.

For a continuous parameter-search batch, keep the launch alive and call
`/command_delay_transport/reset_trial_state` followed by
`/controller_server/FollowPath/reset_trial_state` at every stopped run
boundary. These
reset only run-specific state; the generated coefficient parameter remains
unchanged for the batch.
