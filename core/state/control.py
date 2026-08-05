from dataclasses import dataclass, field
from typing import Optional
import numpy as np

@dataclass
class ControlInput3D:
    # For our case, this is Jerk (u)
    # For double integrator case, it would be acceleration
    value: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def copy(self):
        """
        Returns a new State3D instance with copied numpy arrays.
        Essential for accurate history logging.
        """
        return ControlInput3D(self.value.copy())

@dataclass
class QuadControl:
    thrust: Optional[float] = None

    # useful for DI trajectories
    accel: Optional[np.ndarray] = None  #(3,)