__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

import numpy as np


def find_nearest(A, value):
    idx = (np.abs(np.array(A) - value)).argmin()
    return A[idx]


def find_nearest_index(A, value):
    idx = (np.abs(np.array(A) - value)).argmin()
    return idx
