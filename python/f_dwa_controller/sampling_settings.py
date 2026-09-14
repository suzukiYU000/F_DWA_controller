"""Shared validation for the experiment's F-DWA sampling bank."""

import math


def normalize_dwa_sampling(settings):
    """Validate the common velocity/acceleration/jerk sample grid."""
    keys = {'vx_samples', 'vtheta_samples'}
    if not isinstance(settings, dict) or set(settings) != keys:
        raise ValueError('DWA sampling must specify exactly ' + ', '.join(sorted(keys)))
    for key, value in settings.items():
        if type(value) is not int or value < 1:
            raise ValueError(f'DWA {key} must be a positive integer')
    return dict(settings)


def normalize_fir_sampling(settings, prediction_time, control_period=0.05):
    """
    Validate a bank and cap extra pulses to the actual rollout horizon.

    Zero denotes a full-horizon raw-input pulse, not a zero output command.
    The scalar pulse is kept separately by the generator and always comes first.
    """
    required_keys = {
        'vx_samples', 'vtheta_samples', 'fir_prediction_pulse_durations',
        'fir_independent_pulse_durations',
    }
    optional_keys = {'fir_equal_effect_max_duration_sampling'}
    if (not isinstance(settings, dict)
            or not required_keys.issubset(settings)
            or not set(settings).issubset(required_keys | optional_keys)):
        raise ValueError(
            'F-DWA sampling must specify ' + ', '.join(sorted(required_keys))
            + '; optional: ' + ', '.join(sorted(optional_keys))
        )
    if (not math.isfinite(prediction_time) or prediction_time <= 0.0
            or not math.isfinite(control_period) or control_period <= 0.0):
        raise ValueError('F-DWA sampling requires a positive finite horizon and period')
    result = dict(settings)
    result.setdefault('fir_equal_effect_max_duration_sampling', False)
    for key in ('vx_samples', 'vtheta_samples'):
        if type(result[key]) is not int or result[key] < 1:
            raise ValueError(f'F-DWA {key} must be a positive integer')
    if type(result['fir_independent_pulse_durations']) is not bool:
        raise ValueError('F-DWA fir_independent_pulse_durations must be boolean')
    if type(result['fir_equal_effect_max_duration_sampling']) is not bool:
        raise ValueError(
            'F-DWA fir_equal_effect_max_duration_sampling must be boolean'
        )
    durations = result['fir_prediction_pulse_durations']
    if not isinstance(durations, list):
        raise ValueError('F-DWA fir_prediction_pulse_durations must be a list')
    normalized = []
    for duration in durations:
        if (type(duration) not in (int, float) or not math.isfinite(duration)
                or duration < 0.0):
            raise ValueError('F-DWA sampling pulse durations must be finite and nonnegative')
        steps = duration / control_period
        if abs(steps - round(steps)) > 1.0e-9:
            raise ValueError('F-DWA sampling pulse durations must be control-period multiples')
        # A shorter GUI horizon must not fail because an extra default pulse is longer.
        effective = 0.0 if duration >= prediction_time else float(duration)
        if effective not in normalized:
            normalized.append(effective)
    result['fir_prediction_pulse_durations'] = normalized
    if result['fir_equal_effect_max_duration_sampling']:
        if (result['vx_samples'] % 2 == 0
                or result['vtheta_samples'] % 2 == 0):
            raise ValueError(
                'Equal-effect F-DWA sampling requires odd axis sample counts'
            )
        if normalized or result['fir_independent_pulse_durations']:
            raise ValueError(
                'Equal-effect F-DWA sampling uses one per-axis duration and '
                'cannot be combined with a duration bank'
            )
    return result
