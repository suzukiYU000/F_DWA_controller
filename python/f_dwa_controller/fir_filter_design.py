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

"""Deterministic launch-time FIR design for F-DWA."""

from dataclasses import dataclass
import hashlib
import math
from typing import Any, MutableMapping, Optional, Sequence, Tuple

import numpy as np
from scipy import signal


@dataclass(frozen=True)
class FirFilterDesign:
    """Parameters from which a causal unit-DC-gain FIR is generated."""

    num_taps: int
    sample_frequency_hz: float
    mode: str
    cutoff_hz: Optional[float] = None
    attenuation_bands_hz: Tuple[Tuple[float, float], ...] = ()
    window: str = 'hamming'
    minimum_phase: bool = True
    minimum_phase_method: str = 'homomorphic'
    minimum_phase_fft_length: int = 32768


# These profiles intentionally retain the 20 Hz design sampling frequency used
# to generate the named ROS 1 F-7/F-8/F-9 coefficient sets. The ROS 2 research
# controller also runs at 20 Hz. A different control/design rate defines a
# different filter and must therefore use a new profile name.
FIR_FILTER_DESIGNS = {
    'f7': FirFilterDesign(
        num_taps=91,
        sample_frequency_hz=20.0,
        mode='lowpass',
        cutoff_hz=0.5,
    ),
    'f8': FirFilterDesign(
        num_taps=91,
        sample_frequency_hz=20.0,
        mode='lowpass',
        cutoff_hz=1.2,
    ),
    'f9': FirFilterDesign(
        num_taps=91,
        sample_frequency_hz=20.0,
        mode='lowpass',
        cutoff_hz=2.0,
    ),
    # A future attenuation-band design should be added atomically, for example:
    # 'paper': FirFilterDesign(
    #     num_taps=91,
    #     sample_frequency_hz=33.333333333333333,
    #     mode='bandstop',
    #     attenuation_bands_hz=((2.0, 3.0),),
    # ),
}


@dataclass(frozen=True)
class DesignReport:
    """Auditable description of coefficients inserted into ROS parameters."""

    profile_name: str
    requested_taps: int
    effective_taps: int
    sample_frequency_hz: float
    mode: str
    cutoff_hz: Optional[float]
    attenuation_bands_hz: Tuple[Tuple[float, float], ...]
    step_response_rise_time_seconds: float
    step_response_overshoot_percent: float
    fingerprint: str


def _validate_design(design: FirFilterDesign) -> None:
    if design.num_taps <= 0:
        raise ValueError('num_taps must be positive')
    if not math.isfinite(design.sample_frequency_hz) or (
        design.sample_frequency_hz <= 0.0
    ):
        raise ValueError('sample_frequency_hz must be finite and positive')
    if design.minimum_phase and design.minimum_phase_fft_length <= 0:
        raise ValueError('minimum_phase_fft_length must be positive')
    if design.minimum_phase_method not in {'homomorphic', 'hilbert'}:
        raise ValueError(
            'minimum_phase_method must be homomorphic or hilbert'
        )

    nyquist_hz = 0.5 * design.sample_frequency_hz
    if design.mode == 'lowpass':
        if design.cutoff_hz is None or not (
            0.0 < design.cutoff_hz < nyquist_hz
        ):
            raise ValueError('lowpass cutoff_hz must lie below Nyquist')
        if design.attenuation_bands_hz:
            raise ValueError(
                'lowpass does not accept attenuation_bands_hz'
            )
        return

    if design.mode != 'bandstop':
        raise ValueError('mode must be lowpass or bandstop')
    if design.cutoff_hz is not None:
        raise ValueError('bandstop does not accept cutoff_hz')
    if not design.attenuation_bands_hz:
        raise ValueError('bandstop requires attenuation_bands_hz')

    previous_upper_hz = 0.0
    for lower_hz, upper_hz in design.attenuation_bands_hz:
        if not (
            math.isfinite(lower_hz)
            and math.isfinite(upper_hz)
            and previous_upper_hz < lower_hz < upper_hz < nyquist_hz
        ):
            raise ValueError(
                'attenuation bands must be ordered, disjoint, and below '
                'Nyquist'
            )
        previous_upper_hz = upper_hz


def _linear_phase_coefficients(design: FirFilterDesign) -> np.ndarray:
    if design.mode == 'lowpass':
        cutoff: Any = design.cutoff_hz
        pass_zero = 'lowpass'
    else:
        cutoff = [
            edge_hz
            for band_hz in design.attenuation_bands_hz
            for edge_hz in band_hz
        ]
        pass_zero = 'bandstop'

    return signal.firwin(
        numtaps=design.num_taps,
        cutoff=cutoff,
        window=design.window,
        pass_zero=pass_zero,
        fs=design.sample_frequency_hz,
    )


def design_fir_coefficients_from_spec(
    design: FirFilterDesign,
) -> list[float]:
    """Generate coefficients from a fully specified Python design."""
    _validate_design(design)
    coefficients = _linear_phase_coefficients(design)
    if design.minimum_phase:
        coefficients = signal.minimum_phase(
            coefficients,
            method=design.minimum_phase_method,
            n_fft=design.minimum_phase_fft_length,
        )
        expected_taps = (design.num_taps + 1) // 2
        if len(coefficients) != expected_taps:
            raise ValueError(
                'minimum-phase conversion returned an unexpected tap count'
            )

    dc_gain = float(np.sum(coefficients))
    if not math.isfinite(dc_gain) or abs(dc_gain) <= 1.0e-12:
        raise ValueError('generated FIR coefficients have invalid DC gain')
    coefficients = coefficients / dc_gain
    if not np.all(np.isfinite(coefficients)):
        raise ValueError('generated FIR coefficients are not finite')
    if abs(float(coefficients[0])) <= 1.0e-12:
        raise ValueError('generated FIR first coefficient is zero')
    return [float(value) for value in coefficients]


def design_fir_coefficients(profile_name: str) -> list[float]:
    """Generate one named filter without storing its coefficients in YAML."""
    try:
        design = FIR_FILTER_DESIGNS[profile_name]
    except KeyError as error:
        choices = ', '.join(sorted(FIR_FILTER_DESIGNS))
        raise ValueError(
            f'unknown FIR design profile {profile_name!r}; choose: {choices}'
        ) from error
    return design_fir_coefficients_from_spec(design)


def lowpass_design_for_cutoff(cutoff_hz: float) -> FirFilterDesign:
    """Return the established 20 Hz/91-tap design at a numeric cutoff."""
    return FirFilterDesign(
        num_taps=91,
        sample_frequency_hz=20.0,
        mode='lowpass',
        cutoff_hz=float(cutoff_hz),
    )


def design_fir_coefficients_for_cutoff(cutoff_hz: float) -> list[float]:
    """Generate the F-DWA FIR from an operator-provided cutoff in Hz."""
    return design_fir_coefficients_from_spec(
        lowpass_design_for_cutoff(cutoff_hz)
    )


def coefficient_fingerprint(coefficients: Sequence[float]) -> str:
    """Return a tolerance-stable SHA-256 fingerprint for experiment logs."""
    canonical = ';'.join(f'{value:.12f}' for value in coefficients)
    return hashlib.sha256(canonical.encode('ascii')).hexdigest()


def _step_response_metrics(
    coefficients: Sequence[float],
    sample_frequency_hz: float,
) -> tuple[float, float]:
    """Return first-crossing 10--90% rise time and positive overshoot."""
    step_response = np.cumsum(np.asarray(coefficients, dtype=float))
    ten_percent_index = int(np.flatnonzero(step_response >= 0.1)[0])
    ninety_percent_index = int(np.flatnonzero(step_response >= 0.9)[0])
    rise_time_seconds = (
        ninety_percent_index - ten_percent_index
    ) / sample_frequency_hz
    overshoot_percent = max(
        0.0,
        (float(np.max(step_response)) - 1.0) * 100.0,
    )
    return rise_time_seconds, overshoot_percent


def inject_fir_coefficients(
    parameters: MutableMapping[str, Any],
    controller_name: str = 'controller_server',
    plugin_name: str = 'FollowPath',
) -> Optional[DesignReport]:
    """Replace an F-DWA cutoff (or legacy profile) with runtime parameters."""
    try:
        plugin_parameters = parameters[controller_name]['ros__parameters'][
            plugin_name
        ]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f'{controller_name}.{plugin_name} parameters are missing'
        ) from error

    generator_name = str(
        plugin_parameters.get('trajectory_generator_name', '')
    )
    uses_fir = generator_name.endswith('FirTrajectoryGenerator')
    profile_name = plugin_parameters.pop('fir_design_profile', None)
    cutoff_hz = plugin_parameters.get('fir_cutoff_frequency_hz')
    if not uses_fir:
        if profile_name is not None or cutoff_hz is not None:
            raise ValueError(
                'FIR design parameters are only valid for '
                'FirTrajectoryGenerator'
            )
        return None

    if 'fir_coefficients' in plugin_parameters:
        raise ValueError(
            'source YAML must not contain fir_coefficients; use a Python '
            'FIR_FILTER_DESIGNS profile'
        )
    if cutoff_hz is not None:
        if profile_name is not None:
            raise ValueError(
                'Specify fir_cutoff_frequency_hz or legacy '
                'fir_design_profile, not both'
            )
        try:
            cutoff_hz = float(cutoff_hz)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'fir_cutoff_frequency_hz must be a number'
            ) from error
        design = lowpass_design_for_cutoff(cutoff_hz)
        coefficients = design_fir_coefficients_from_spec(design)
        profile_name = f'cutoff_{cutoff_hz:g}_hz'
    else:
        if not isinstance(profile_name, str) or not profile_name:
            raise ValueError(
                'FirTrajectoryGenerator requires fir_cutoff_frequency_hz'
            )
        coefficients = design_fir_coefficients(profile_name)
        design = FIR_FILTER_DESIGNS[profile_name]
    (
        step_response_rise_time_seconds,
        step_response_overshoot_percent,
    ) = _step_response_metrics(
        coefficients,
        design.sample_frequency_hz,
    )
    plugin_parameters['fir_coefficients'] = coefficients
    plugin_parameters['fir_coefficients_generated'] = True
    return DesignReport(
        profile_name=profile_name,
        requested_taps=design.num_taps,
        effective_taps=len(coefficients),
        sample_frequency_hz=design.sample_frequency_hz,
        mode=design.mode,
        cutoff_hz=design.cutoff_hz,
        attenuation_bands_hz=design.attenuation_bands_hz,
        step_response_rise_time_seconds=(
            step_response_rise_time_seconds
        ),
        step_response_overshoot_percent=(
            step_response_overshoot_percent
        ),
        fingerprint=coefficient_fingerprint(coefficients),
    )
