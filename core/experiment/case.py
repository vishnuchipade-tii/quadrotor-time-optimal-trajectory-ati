from dataclasses import dataclass, field
from typing import Optional, Any, List
from core.type import RunnerType

@dataclass
class ExperimentCase:

    name: str

    dynamics_model: Any
    planner: Any
    params: dict
    quad_config: dict

    initial_state: Any
    target: Any
    sim_metadata: Any

    runner_type: RunnerType = RunnerType.INTERNAL

    color: str = "blue"
    linestyle: str = "-"

    tags: dict = None
    experiment_config: dict = None