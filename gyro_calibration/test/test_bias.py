"""The start-up bias on synthetic gyro readings, where the bias is known."""

from __future__ import annotations

import numpy as np
import pytest
from gyro_calibration.bias import BiasEstimator, Calibration, corrected

CALIBRATION = Calibration(duration=2.0, max_rate=0.02)
BIAS = (0.0004, -0.0002, 0.0012)  # rad/s, about what the simulated gyro draws
NOISE = 0.0009  # rad/s per reading at 100 Hz, as the simulated gyro has


def _readings(seconds: float, rate: tuple[float, float, float], start: float = 0.0, seed: int = 0):
    rng = np.random.default_rng(seed)
    for i in range(int(seconds * 100) + 1):
        noisy = tuple(r + rng.normal(0.0, NOISE) for r in rate)
        yield start + i / 100, noisy


def test_settings_must_be_positive():
    with pytest.raises(ValueError):
        Calibration(duration=0.0, max_rate=0.02)
    with pytest.raises(ValueError):
        Calibration(duration=2.0, max_rate=-1.0)


def test_no_bias_before_the_duration():
    estimator = BiasEstimator(CALIBRATION)
    for stamp, rates in _readings(1.5, BIAS):
        assert not estimator.add(stamp, rates)
    assert estimator.bias is None


def test_bias_at_rest_is_the_mean_reading():
    estimator = BiasEstimator(CALIBRATION)
    for stamp, rates in _readings(2.0, BIAS):
        estimator.add(stamp, rates)
    # 201 readings: the mean is off by NOISE / sqrt(201), about 0.00006 rad/s.
    assert estimator.bias == pytest.approx(BIAS, abs=4 * NOISE / np.sqrt(201))


def test_a_turn_starts_the_average_over():
    estimator = BiasEstimator(CALIBRATION)
    for stamp, rates in _readings(1.0, BIAS):
        estimator.add(stamp, rates)
    for stamp, rates in _readings(0.5, (0.0, 0.0, 0.3), start=1.0):
        assert not estimator.add(stamp, rates)
    assert estimator.restarts == 1
    for stamp, rates in _readings(2.0, BIAS, start=1.5, seed=1):
        estimator.add(stamp, rates)
    assert estimator.bias == pytest.approx(BIAS, abs=4 * NOISE / np.sqrt(201))


def test_the_bias_stays_once_measured():
    estimator = BiasEstimator(CALIBRATION)
    for stamp, rates in _readings(2.0, BIAS):
        estimator.add(stamp, rates)
    measured = estimator.bias
    assert estimator.add(3.0, (0.0, 0.0, 0.5))
    assert estimator.bias == measured


def test_corrected_takes_the_bias_out():
    assert corrected((0.1, -0.2, 0.3), (0.01, -0.02, 0.03)) == pytest.approx((0.09, -0.18, 0.27))
