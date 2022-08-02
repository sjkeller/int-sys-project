from collections import deque


class WindEstimator:
    """Poll readings from wind direction sensor periodically from the
    APM board and estimate the wind direction by processing, e.g. averaging,
    multiple sensor readings.
    Since this is the relative wind direction, we need to consider the compass direction
    for computing the absolute wind direction.
    """

    def __init__(self):
        self._last_readings = deque([], 10)
        self._last_estimation = None

    def start(self):
        pass

    def get_absolute_wind_direction(self):
        # TODO: Calculate absolute wind direction from compass direction (ahrs.yaw -> from apm) and
        # relative wind direction
        # return self._last_estimation
