from dataclasses import dataclass, replace
from typing import Any
import numpy as np
from core.dynamics.models import BaseDynamicsModel
from core.targets import BaseTarget
from core.config.solver_config import SolverConfig

@dataclass
class PlanningRequest:
    # Initial state
    initial_state: np.ndarray

    # Dynamics model
    dynamics: "BaseDynamicsModel"

    # Terminal target
    target: "BaseTarget"

    # Bounds
    state_bounds: Any
    input_bounds: Any

    # Solver config
    solver_config: "SolverConfig" = None

    def update(
        self,
        state=None,
        target=None
    ):
        return replace(
            self,
            initial_state=(
                state
                if state is not None
                else self.initial_state
            ),
            target=(
                target
                if target is not None
                else self.target
            )
        )