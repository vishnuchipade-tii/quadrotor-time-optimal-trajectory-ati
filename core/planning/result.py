from dataclasses import dataclass, field
from typing import List
import numpy as np

@dataclass
class PlanningResult:
    success: bool = False
    final_time: float = 0.0
    times: List[float] = field(default_factory=list)
    states: List[np.ndarray] = field(default_factory=list)
    controls: List[np.ndarray] = field(default_factory=list)