from dataclasses import dataclass
from core.dynamics.models import DynamicsType
from core.type import TargetType
from core.type import SolverType

@dataclass(frozen=True)
class SolverKey:
    dynamics: DynamicsType
    target: TargetType
    solver: SolverType
