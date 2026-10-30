"""A gyro's start-up bias, measured while the robot stands still, without ROS.

A low-cost MEMS gyro reads a small rate even at rest, and a different one every time it powers up.
A filter that integrates the rate turns that bias into a heading error that grows with time. At
rest the true rate is zero, so the average reading over a few seconds is the bias, give or take the
noise divided by the square root of the number of readings. The TurtleBot3's OpenCR board, for
one, calibrates its gyro this way at power-up.

A reading above a threshold means the robot is turning, and the average starts over.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Calibration:
    """How the bias is measured.

    Attributes:
        duration: Seconds of readings to average, by their stamps.
        max_rate: A reading above this on any axis (rad/s) means the robot moved, and the
            average starts over. It sits well above the bias and the noise, and well below any
            turn a person would drive.
    """

    duration: float
    max_rate: float

    def __post_init__(self) -> None:
        if self.duration <= 0:
            raise ValueError(f"duration must be positive, got {self.duration}")
        if self.max_rate <= 0:
            raise ValueError(f"max_rate must be positive, got {self.max_rate}")


# ---------------------------------------------------------------------------
# Measuring
# ---------------------------------------------------------------------------


class BiasEstimator:
    """Averages gyro readings from the first one until ``duration`` has passed at rest.

    Attributes:
        bias: The rates about x, y and z to subtract (rad/s), once measured, else None.
        restarts: How many times a reading above ``max_rate`` started the average over.
    """

    def __init__(self, calibration: Calibration) -> None:
        self.calibration = calibration
        self.bias: tuple[float, float, float] | None = None
        self.restarts = 0
        self._start: float | None = None
        self._readings: list[tuple[float, float, float]] = []

    def add(self, stamp: float, rates: tuple[float, float, float]) -> bool:
        """Take one reading. Returns True once the bias is measured, on this reading or before."""
        if self.bias is not None:
            return True
        if max(abs(r) for r in rates) > self.calibration.max_rate:
            if self._readings:
                self.restarts += 1
            self._start, self._readings = None, []
            return False
        if self._start is None:
            self._start = stamp
        self._readings.append(rates)
        if stamp - self._start >= self.calibration.duration:
            self.bias = tuple(float(v) for v in np.mean(self._readings, axis=0))
            self._readings = []
        return self.bias is not None


def corrected(
    rates: tuple[float, float, float], bias: tuple[float, float, float]
) -> tuple[float, float, float]:
    """The rates with the bias taken out."""
    return rates[0] - bias[0], rates[1] - bias[1], rates[2] - bias[2]
