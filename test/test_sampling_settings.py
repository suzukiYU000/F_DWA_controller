import copy

from f_dwa_controller.sampling_settings import (
    normalize_dwa_sampling,
    normalize_fir_sampling,
)
import pytest


def test_common_grid_is_validated_without_mutation():
    grid = {'vx_samples': 10, 'vtheta_samples': 19}
    result = normalize_dwa_sampling(grid)
    assert result == grid
    assert result is not grid


@pytest.mark.parametrize('grid', [
    None, {}, {'vx_samples': 10},
    {'vx_samples': 10, 'vtheta_samples': 19, 'unknown': 1},
    {'vx_samples': True, 'vtheta_samples': 19},
    {'vx_samples': 10.0, 'vtheta_samples': 19},
    {'vx_samples': 10, 'vtheta_samples': 0},
    {'vx_samples': -1, 'vtheta_samples': 19},
])
def test_rejects_invalid_common_grid(grid):
    with pytest.raises(ValueError):
        normalize_dwa_sampling(grid)


def bank():
    return {
        'vx_samples': 13,
        'vtheta_samples': 19,
        'fir_prediction_pulse_durations': [0.2],
        'fir_independent_pulse_durations': False,
        'fir_equal_effect_max_duration_sampling': False,
    }


def test_normalizes_without_mutating_defaults():
    settings = bank()
    original = copy.deepcopy(settings)
    assert normalize_fir_sampling(settings, 2.5) == settings
    assert normalize_fir_sampling(settings, 0.15)['fir_prediction_pulse_durations'] == [0.0]
    assert settings == original


def test_deduplicates_full_horizon_and_control_ticks():
    settings = bank()
    settings['fir_prediction_pulse_durations'] = [0, 0.1, 0.1, 0.3, 0.4]
    assert normalize_fir_sampling(settings, 0.25)['fir_prediction_pulse_durations'] == [0.0, 0.1]


@pytest.mark.parametrize('key,value', [
    ('vx_samples', 0), ('vx_samples', 13.5), ('vx_samples', True),
    ('vtheta_samples', -1), ('fir_independent_pulse_durations', 'false'),
    ('fir_equal_effect_max_duration_sampling', 'true'),
    ('fir_prediction_pulse_durations', '0.2'),
    ('fir_prediction_pulse_durations', [float('nan')]),
    ('fir_prediction_pulse_durations', [float('inf')]),
    ('fir_prediction_pulse_durations', [-0.1]),
    ('fir_prediction_pulse_durations', [True]),
    ('fir_prediction_pulse_durations', [0.13]),
])
def test_rejects_invalid_values(key, value):
    settings = bank()
    settings[key] = value
    with pytest.raises(ValueError):
        normalize_fir_sampling(settings, 2.5)


@pytest.mark.parametrize('settings', [None, {}, {'vx_sampels': 13}])
def test_rejects_missing_or_unknown_keys(settings):
    with pytest.raises(ValueError):
        normalize_fir_sampling(settings, 2.5)


@pytest.mark.parametrize('horizon', [0.0, -1.0, float('nan'), float('inf')])
def test_rejects_invalid_horizon(horizon):
    with pytest.raises(ValueError):
        normalize_fir_sampling(bank(), horizon)


def test_equal_effect_sampling_requires_odd_single_bank():
    settings = bank()
    settings.update({
        'vx_samples': 11,
        'vtheta_samples': 15,
        'fir_prediction_pulse_durations': [],
        'fir_equal_effect_max_duration_sampling': True,
    })
    result = normalize_fir_sampling(settings, 2.5)
    assert result['fir_equal_effect_max_duration_sampling'] is True
    for key, value in (
        ('vx_samples', 10),
        ('vtheta_samples', 14),
        ('fir_prediction_pulse_durations', [0.2]),
        ('fir_independent_pulse_durations', True),
    ):
        invalid = dict(settings)
        invalid[key] = value
        with pytest.raises(ValueError):
            normalize_fir_sampling(invalid, 2.5)
