from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class FlatState:
    p: Optional[np.ndarray] = None      #(3,)
    v: Optional[np.ndarray] = None      #(3,)
    a: Optional[np.ndarray] = None      #(3,)
    j: Optional[np.ndarray] = None      #(3,)

    yaw: Optional[float] = None
    yaw_rate: Optional[float] = None