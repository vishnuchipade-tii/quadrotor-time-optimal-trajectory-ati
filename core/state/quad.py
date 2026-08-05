from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class QuadState:
    p: Optional[np.ndarray] = None      #(3,)
    v: Optional[np.ndarray] = None      #(3,)

    R: Optional[np.ndarray] = None      #(3,3)
    q: Optional[np.ndarray] = None      #(4,)
    omega: Optional[np.ndarray] = None  #(3,)