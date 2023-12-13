#!/usr/bin/env python3

import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(
[[  60.,  80.,  100.,  80.],
    [ 60., 135., 60., 130.],
 [110., 75., 110., 65.]]
)

MIN = np.array(
[[  0.,  0.,  0.,  0.],
 [ 0., 0., 0., 0.],
 [0., 0., 0., 0.]]
)

MAX = np.array(
[[  180.,  180.,  180.,  180.],
 [ 180., 180., 180., 180.],
 [180., 180., 180., 180.]]
)
