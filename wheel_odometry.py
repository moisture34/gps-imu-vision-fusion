"""Helpers for planar differential-drive odometry from the RANGER wheel channels.

The parking01 RANGER channels for the AgileX Ranger MINI3 are grouped as:
  channels[0] (column 1) = right-front
  channels[1] (column 2) = left-front
  channels[2] (column 3) = right-rear
  channels[3] (column 4) = left-rear
"""

from __future__ import annotations

import numpy as np


def compute_wheel_odometry(
    wheel_channels: np.ndarray, track_width_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return mean forward speed and differential yaw rate from four wheel speeds.

    The parking01 RANGER channels are grouped as right-front, left-front,
    right-rear, left-rear. Speeds are in m/s and track_width_m is in meters.
    """
    if wheel_channels.ndim != 2 or wheel_channels.shape[1] < 4:
        raise ValueError("RANGER odometry must contain four wheel-speed channels")
    if not np.isfinite(track_width_m) or track_width_m <= 0:
        raise ValueError("wheel track width must be a finite positive value")

    right_speed = np.mean(wheel_channels[:, [0, 2]], axis=1)
    left_speed = np.mean(wheel_channels[:, [1, 3]], axis=1)
    forward_speed = np.mean(wheel_channels[:, :4], axis=1)
    yaw_rate = (right_speed - left_speed) / track_width_m
    return forward_speed, yaw_rate
