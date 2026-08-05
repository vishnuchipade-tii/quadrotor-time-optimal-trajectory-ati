
from core.constraints import BaseTerminalConstraint
import numpy as np
from core.targets import SphereTarget

class SphereConstraint(BaseTerminalConstraint):

    def __init__(self, target: SphereTarget):
        self.target = target

    def residual(
            self,
            drift,
            drift_dot,
            reach,
            reach_dot
    ):
        """
        G(t) = ||drift|| - reach - radius
        """

        dist = np.linalg.norm(drift)

        unit = drift / (dist + 1e-9)

        dist_dot = np.dot(unit, drift_dot)

        G = dist - reach - self.target.radius

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

        return (-drift / norm_drift) * u_max