from dataclasses import dataclass
from typing import Optional

from core.experiment import ExperimentCase
from core.quadrotor import Quadrotor
from core.simulation import SimulationResult

@dataclass
class RunResult:
    name: str
    case: ExperimentCase
    quadrotor: Quadrotor
    simulation_result: SimulationResult
    compute_time: float
    quad_traj_bundle: Optional[QuadTrajectoryBundle] = None