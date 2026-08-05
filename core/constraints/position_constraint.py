from core.constraints import BaseTerminalConstraint
import numpy as np
from core.type import TargetType

# ============================================================
# FIXED POSITION CONSTRAINT
# ============================================================

class PositionConstraint(BaseTerminalConstraint):

    def __init__(self, target):
        if target.target_type == TargetType.POSITION:
            self.target_position = np.asarray(target.desired_position)
        else:
            self.target_position = np.asarray(target, dtype=float)

    def residual(
            self,
            drift,
            drift_dot,
            reach,
            reach_dot
    ):
        """
        G(t) = ||drift|| - reach
        """

        dist = np.linalg.norm(drift)

        unit = drift / (dist + 1e-9)

        dist_dot = np.dot(unit, drift_dot)

        G = dist - reach

        Gd = dist_dot - reach_dot

        return G, Gd

    def optimal_control(
            self,
            drift,
            u_max
    ):

        norm_drift = np.linalg.norm(drift)

        if norm_drift < 1e-9:
            return np.zeros(3)

        return (- drift / norm_drift) * u_max