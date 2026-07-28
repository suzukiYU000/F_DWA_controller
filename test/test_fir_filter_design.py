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
from dataclasses import replace
from pathlib import Path

from f_dwa_controller.fir_filter_design import (
    coefficient_fingerprint,
    design_fir_coefficients,
    design_fir_coefficients_from_spec,
    FIR_FILTER_DESIGNS,
    FirFilterDesign,
    inject_fir_coefficients,
)
import pytest
import yaml


CONFIG_PATH = Path(__file__).parent.parent / 'config' / 'f_dwa.yaml'
ROS1_F8_FINGERPRINT_12_DECIMALS = (
    '44d909e5aad77f8d5233c04aa0feeec233608210bb3813f842bb54e95a4937e3'
)


def _load_f_dwa_parameters():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))


def test_source_yaml_contains_design_name_not_coefficients():
    parameters = _load_f_dwa_parameters()
    follow_path = parameters['controller_server']['ros__parameters'][
        'FollowPath'
    ]

    assert follow_path['fir_design_profile'] == 'f8'
    assert 'fir_coefficients' not in follow_path
    assert 'fir_coefficients_generated' not in follow_path


def test_named_f8_is_generated_before_controller_start():
    parameters = _load_f_dwa_parameters()

    report = inject_fir_coefficients(parameters)
    follow_path = parameters['controller_server']['ros__parameters'][
        'FollowPath'
    ]
    coefficients = follow_path['fir_coefficients']

    assert report is not None
    assert report.profile_name == 'f8'
    assert report.requested_taps == 91
    assert report.effective_taps == 46
    assert report.sample_frequency_hz == pytest.approx(20.0)
    assert len(coefficients) == 46
    assert sum(coefficients) == pytest.approx(1.0, abs=1.0e-12)
    assert follow_path['fir_coefficients_generated'] is True
    assert 'fir_design_profile' not in follow_path
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


def test_each_startup_regenerates_the_current_python_profile(monkeypatch):
    first_parameters = _load_f_dwa_parameters()
    first_report = inject_fir_coefficients(first_parameters)
    first_coefficients = first_parameters['controller_server'][
        'ros__parameters'
    ]['FollowPath']['fir_coefficients']

    monkeypatch.setitem(
        FIR_FILTER_DESIGNS,
        'f8',
        replace(FIR_FILTER_DESIGNS['f8'], cutoff_hz=1.0),
    )
    next_parameters = _load_f_dwa_parameters()
    next_report = inject_fir_coefficients(next_parameters)
    next_coefficients = next_parameters['controller_server'][
        'ros__parameters'
    ]['FollowPath']['fir_coefficients']

    assert first_report is not None
    assert next_report is not None
    assert first_coefficients != next_coefficients
    assert first_report.fingerprint != next_report.fingerprint


def test_direct_yaml_coefficients_are_rejected():
    parameters = _load_f_dwa_parameters()
    invalid_parameters = deepcopy(parameters)
    invalid_parameters['controller_server']['ros__parameters']['FollowPath'][
        'fir_coefficients'
    ] = [1.0]

    with pytest.raises(ValueError, match='must not contain fir_coefficients'):
        inject_fir_coefficients(invalid_parameters)
