"""Pure Python reference sampler; no ROS imports, independently testable."""
import numpy as np
from common import JOINTS


def sample_points(times, positions, t):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if times.ndim != 1 or len(times) == 0 or positions.ndim != 2 or len(positions) != len(times):
        raise ValueError('Invalid reference trajectory')
    if not np.isfinite(times).all() or not np.isfinite(positions).all() or np.any(np.diff(times) < 0):
        raise ValueError('Nonfinite or unordered reference trajectory')
    return np.array([np.interp(t, times, positions[:, i]) for i in range(positions.shape[1])])


def combined_reference(state, references, time_s, lookahead=0.2):
    result = np.asarray(state, dtype=float).copy()
    for names, start, times, positions in references:
        values = sample_points(times, positions, time_s + lookahead - start)
        if len(values) != len(names):
            raise ValueError('Reference joint/position dimensions differ')
        for name, value in zip(names, values):
            result[JOINTS.index(name)] = value
    return result
