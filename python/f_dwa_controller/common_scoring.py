"""Shared V/A/J/F weighted costs and physical-footprint policy."""

SOFT_CRITICS = ('PathDeviation', 'TrajectoryProgress', 'PathAlign',
                'MeanSpeed', 'FootprintClearance')
ACTIVE_CRITICS = ('ObstacleFootprint', *SOFT_CRITICS)
PHYSICAL_COLLISION_SCALE = 0.4


def apply_compact_cost_policy(parameters):
    """Five soft costs, a physical collision gate, and native safe stopping."""
    oscillation_enabled = 'Oscillation' in parameters.get('critics', ())
    parameters['critics'] = list(ACTIVE_CRITICS)
    parameters['ObstacleFootprint.scale'] = PHYSICAL_COLLISION_SCALE
    if oscillation_enabled:
        parameters['critics'].insert(1, 'Oscillation')
    # Retain the user's one-metre corridor and replace the fixed boundary
    # jump by a continuous excess penalty. The interior term replaces the
    # separate grid-based path distance and alignment critics.
    parameters['PathDeviation.maximum_path_distance'] = 1.0
    parameters['PathDeviation.deviation_penalty'] = 0.0
    parameters['PathDeviation.path_distance_scale'] = 32.0
    parameters['PathDeviation.path_distance_tolerance'] = min(
        1.0, parameters['FootprintClearance.localization_uncertainty_margin'])
    parameters['PathDeviation.heading_recovery_scale'] = 0.0
    parameters['TrajectoryProgress.lateral_distance_weight'] = 0.0
    parameters['TrajectoryProgress.lookahead_distance'] = 1.25
    parameters['MeanSpeed.class'] = 'f_dwa_controller::MeanSpeedCritic'
    parameters['MeanSpeed.target_speed'] = parameters['max_vel_x']
    parameters.setdefault('MeanSpeed.scale', 13.0)
    parameters['enable_stop_admissibility'] = True
    parameters['terminal_stop_goal_distance_scale'] = 0.0
    # A valid candidate is selected by these five weighted costs alone.
    # Zero disables the existing post-selection progress/escape override;
    # all-invalid native stopping and the physical collision gate remain.
    parameters['clearance_constraint_minimum_subgoal_distance_progress'] = 0.0
    parameters['clearance_constraint_minimum_subgoal_heading_progress'] = 0.0
    # With zero Costmap padding the hard footprint is the complete measured
    # body. An inward recovery footprint would then shrink that body, so all
    # inset-based exceptions must be disabled together.
    parameters['enable_initial_overlap_recovery'] = False
    parameters['enable_transient_boundary_margin_recovery'] = False
    parameters['localization_uncertainty_footprint_inset'] = 0.0
