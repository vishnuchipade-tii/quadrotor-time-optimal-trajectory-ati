from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np

from core.state import FlatState, QuadState
from core.state import QuadControl

@dataclass
class QuadTrajectoryPoint:
    flat: Optional[FlatState] = None
    state: Optional[QuadState] = None
    control: Optional[QuadControl] = None


@dataclass
class QuadTrajectoryBundle:
    time: List[float]
    trajectory: List[QuadTrajectoryPoint]
    label: str
    color: str = "blue"
    linestyle: str = "-"