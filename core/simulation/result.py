from dataclasses import dataclass
from typing import List, Union

from core.state import State3D, DIState3D, ControlInput3D

@dataclass
class SimulationResult:
    time_history: List[float]
    state_history: List[Union[State3D, DIState3D]]
    control_history: List[ControlInput3D]
    label: str
    line_style: str = '-'
    color: str = None  # Optional: Let the animator choose if not provided
