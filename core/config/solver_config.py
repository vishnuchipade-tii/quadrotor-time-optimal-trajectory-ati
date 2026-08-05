from dataclasses import dataclass, field

from core.type.solver_type import SolverType
import yaml

@dataclass
class SolverConfig:

    solver_type: SolverType

    options: dict = field(
        default_factory=dict
    )

    @classmethod
    def from_config(cls, solver_spec):

        try:
            solver_type = SolverType.from_string(
                solver_spec["type"]
            )
        except ValueError as e:
            raise ValueError(
                f"Unsupported solver_type '{solver_spec['type']}'"
            ) from e

        config = solver_spec.get("config")

        if config is None:
            options = {}

        elif isinstance(config, str):
            with open(config, "r") as f:
                options = yaml.safe_load(f)

        elif isinstance(config, dict):
            options = config

        else:
            raise TypeError(
                f"Unsupported solver config type: {type(config)}"
            )

        return cls(
            solver_type=solver_type,
            options=options,
        )