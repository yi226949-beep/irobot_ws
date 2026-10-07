"""관절공간 quintic(최소 jerk) 궤적."""

import numpy as np


def quintic_duration(q0, q1, vmax, vel_scale, t_min, t_max):
    """최고속도가 vel_scale*vmax 이하가 되는 지속시간. quintic의 최고속도는 1.875*Δq/T."""
    dq = np.abs(np.asarray(q1) - np.asarray(q0))
    t = float(np.max(1.875 * dq / (vel_scale * np.asarray(vmax))))
    return float(np.clip(t, t_min, t_max))


class Quintic:
    def __init__(self, q0, q1, duration):
        self.q0 = np.asarray(q0, float).copy()
        self.q1 = np.asarray(q1, float).copy()
        self.T = float(duration)

    def __call__(self, t):
        tau = np.clip(t / self.T, 0.0, 1.0)
        s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
        return self.q0 + (self.q1 - self.q0) * s

    def done(self, t):
        return t >= self.T
