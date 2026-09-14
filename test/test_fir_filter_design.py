# Copyright (c) 2026 suzukiYU000
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from copy import deepcopy
from pathlib import Path

from f_dwa_controller.fir_filter_design import (
    canonical_filter_specification,
    coefficient_fingerprint,
    design_fir_coefficients,
    design_fir_coefficients_for_cutoff,
    design_fir_coefficients_for_specification,
    design_fir_coefficients_from_spec,
    filter_design_for_specification,
    FirFilterDesign,
    inject_fir_coefficients,
)
import pytest
from scipy import signal
import yaml


CONFIG_PATH = Path(__file__).parent.parent / 'config' / 'f_dwa.yaml'
ROS1_F8_FINGERPRINT_12_DECIMALS = (
    '44d909e5aad77f8d5233c04aa0feeec233608210bb3813f842bb54e95a4937e3'
)


def _load_f_dwa_parameters():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))


def test_source_yaml_contains_numeric_cutoff_not_coefficients():
    parameters = _load_f_dwa_parameters()
    follow_path = parameters['controller_server']['ros__parameters'][
        'FollowPath'
    ]

    assert follow_path['fir_cutoff_frequency_hz'] == pytest.approx(1.2)
    assert 'fir_design_profile' not in follow_path
    assert 'fir_coefficients' not in follow_path
    assert 'fir_coefficients_generated' not in follow_path


def test_numeric_1_2_hz_filter_is_generated_before_controller_start():
    parameters = _load_f_dwa_parameters()

    report = inject_fir_coefficients(parameters)
    follow_path = parameters['controller_server']['ros__parameters'][
        'FollowPath'
    ]
    coefficients = follow_path['fir_coefficients']

    assert report is not None
    assert report.profile_name == 'cutoff_1.2_hz'
    assert report.requested_taps == 91
    assert report.effective_taps == 46
    assert report.sample_frequency_hz == pytest.approx(20.0)
    assert report.mode == 'lowpass'
    assert report.cutoff_hz == pytest.approx(1.2)
    assert report.attenuation_bands_hz == ()
    assert report.step_response_rise_time_seconds == pytest.approx(0.35)
    assert report.step_response_overshoot_percent == pytest.approx(
        14.872788936733,
    )
    assert len(coefficients) == 46
    assert sum(coefficients) == pytest.approx(1.0, abs=1.0e-12)
    assert follow_path['fir_coefficients_generated'] is True
    assert 'fir_design_profile' not in follow_path
    assert follow_path['fir_cutoff_frequency_hz'] == pytest.approx(1.2)
    assert (
        coefficient_fingerprint(coefficients)
        == ROS1_F8_FINGERPRINT_12_DECIMALS
    )
    assert report.fingerprint == ROS1_F8_FINGERPRINT_12_DECIMALS


@pytest.mark.parametrize('profile_name', ['f7', 'f8', 'f9'])
def test_named_ros1_profiles_have_unit_dc_gain(profile_name):
    coefficients = design_fir_coefficients(profile_name)

    assert len(coefficients) == 46
    assert sum(coefficients) == pytest.approx(1.0, abs=1.0e-12)
    assert abs(coefficients[0]) > 1.0e-12


def test_f9_open_loop_step_response_is_faster_but_overshoots_more():
    f8_parameters = _load_f_dwa_parameters()
    f8_report = inject_fir_coefficients(f8_parameters)
    f9_parameters = _load_f_dwa_parameters()
    f9_parameters['controller_server']['ros__parameters']['FollowPath'][
        'fir_cutoff_frequency_hz'
    ] = 2.0
    f9_report = inject_fir_coefficients(f9_parameters)

    assert f8_report is not None
    assert f9_report is not None
    assert (
        f9_report.step_response_rise_time_seconds
        < f8_report.step_response_rise_time_seconds
    )
    assert (
        f9_report.step_response_overshoot_percent
        > f8_report.step_response_overshoot_percent
    )


def test_attenuation_bands_can_be_defined_in_python():
    coefficients = design_fir_coefficients_from_spec(
        FirFilterDesign(
            num_taps=91,
            sample_frequency_hz=33.333333333333333,
            mode='bandstop',
            attenuation_bands_hz=((2.0, 3.0),),
        )
    )

    assert len(coefficients) == 46
    assert sum(coefficients) == pytest.approx(1.0, abs=1.0e-12)


@pytest.mark.parametrize(
    ('specification', 'mode', 'cutoff_hz', 'bands', 'canonical'),
    [
        ('1.0', 'lowpass', 1.0, (), '1'),
        ('[0.8, 1.6]', 'bandstop', None, ((0.8, 1.6),), '[0.8, 1.6]'),
        (
            '[0.8, 1.6], [2.0, inf]',
            'bandstop',
            None,
            ((0.8, 1.6), (2.0, float('inf'))),
            '[0.8, 1.6], [2, inf]',
        ),
    ],
)
def test_operator_filter_specification_is_parsed_and_canonicalized(
    specification, mode, cutoff_hz, bands, canonical,
):
    design = filter_design_for_specification(specification)

    assert design.mode == mode
    assert design.cutoff_hz == cutoff_hz
    assert design.attenuation_bands_hz == bands
    assert canonical_filter_specification(design) == canonical


@pytest.mark.parametrize(
    'specification',
    [
        '', 'inf', '[0.8]', '[0.8, 1.6] [2.0, inf]',
        '[0, 1.0]', '[1.6, 0.8]', '[0.8, 1.6], [1.5, 2.0]',
        '[0.8, inf], [2.0, 3.0]', '[0.8, 10.0]', '[10.0, inf]',
    ],
)
def test_invalid_operator_filter_specification_is_rejected(specification):
    with pytest.raises(ValueError):
        filter_design_for_specification(specification)


def test_open_ended_attenuation_band_generates_unit_dc_gain_coefficients():
    coefficients = design_fir_coefficients_for_specification(
        '[0.8, 1.6], [2.0, inf]'
    )

    assert len(coefficients) == 46
    assert sum(coefficients) == pytest.approx(1.0, abs=1.0e-12)
    _, response = signal.freqz(
        coefficients,
        worN=[0.2, 1.2, 3.0, 8.0],
        fs=20.0,
    )
    assert abs(response[0]) > 0.9
    assert all(abs(value) < 0.03 for value in response[1:])


def test_operator_attenuation_bands_are_injected_as_auditable_metadata():
    parameters = _load_f_dwa_parameters()
    active = parameters['controller_server']['ros__parameters']['FollowPath']
    active.pop('fir_cutoff_frequency_hz')
    active['fir_filter_specification'] = '[0.8, 1.6], [2.0, inf]'

    report = inject_fir_coefficients(parameters)

    assert report.mode == 'bandstop'
    assert report.cutoff_hz is None
    assert report.attenuation_bands_hz == (
        (0.8, 1.6), (2.0, float('inf')),
    )
    assert active['fir_filter_specification'] == '[0.8, 1.6], [2, inf]'
    assert 'fir_cutoff_frequency_hz' not in active
    assert len(active['fir_coefficients']) == 46


def test_each_startup_regenerates_the_current_numeric_cutoff():
    first_parameters = _load_f_dwa_parameters()
    first_report = inject_fir_coefficients(first_parameters)
    first_coefficients = first_parameters['controller_server'][
        'ros__parameters'
    ]['FollowPath']['fir_coefficients']

    next_parameters = _load_f_dwa_parameters()
    next_parameters['controller_server']['ros__parameters']['FollowPath'][
        'fir_cutoff_frequency_hz'
    ] = 1.0
    next_report = inject_fir_coefficients(next_parameters)
    next_coefficients = next_parameters['controller_server'][
        'ros__parameters'
    ]['FollowPath']['fir_coefficients']

    assert first_report is not None
    assert next_report is not None
    assert first_coefficients != next_coefficients
    assert first_report.fingerprint != next_report.fingerprint


def test_numeric_cutoff_matches_legacy_f8_coefficients():
    assert design_fir_coefficients_for_cutoff(1.2) == pytest.approx(
        design_fir_coefficients('f8'), abs=1.0e-15
    )


@pytest.mark.parametrize('cutoff', [0.33, 0.66, 1.0, 1.5, 2.0, 2.5, 3.0])
def test_double_effective_taps_are_regenerated_with_unit_dc_gain(cutoff):
    parameters = _load_f_dwa_parameters()
    active = parameters['controller_server']['ros__parameters']['FollowPath']
    active.update(fir_cutoff_frequency_hz=cutoff, fir_effective_taps=92)
    report = inject_fir_coefficients(parameters)
    assert report.requested_taps == 183
    assert report.effective_taps == active['fir_effective_taps'] == 92
    assert len(active['fir_coefficients']) == 92
    assert sum(active['fir_coefficients']) == pytest.approx(1.0, abs=1e-12)
    assert report.fingerprint != coefficient_fingerprint(
        design_fir_coefficients_for_cutoff(cutoff)
    )


@pytest.mark.parametrize('taps', [0, 1, -46, 46.5, '92', True])
def test_invalid_effective_tap_count_is_rejected(taps):
    with pytest.raises(ValueError, match='fir_effective_taps'):
        design_fir_coefficients_for_cutoff(1.0, taps)


def test_direct_yaml_coefficients_are_rejected():
    parameters = _load_f_dwa_parameters()
    invalid_parameters = deepcopy(parameters)
    invalid_parameters['controller_server']['ros__parameters']['FollowPath'][
        'fir_coefficients'
    ] = [1.0]

    with pytest.raises(ValueError, match='must not contain fir_coefficients'):
        inject_fir_coefficients(invalid_parameters)
