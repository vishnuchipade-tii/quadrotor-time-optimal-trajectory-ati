from dataclasses import dataclass, field
from typing import Optional, Any, List

from core.experiment import RunResult

from core.type import TargetType
from core.experiment.case import ExperimentCase

@dataclass
class ExperimentScenario:

    name: str
    scenario_set_name: str
    mode: str

    initial_state: Any
    target: Any
    target_type: TargetType

    cases: List[ExperimentCase] = field(
        default_factory=list
    )

    results: List[RunResult] = field(
        default_factory=list
    )