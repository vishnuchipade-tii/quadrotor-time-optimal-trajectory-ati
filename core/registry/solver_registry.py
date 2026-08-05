from core.registry.base_registry import BaseRegistry


class SolverRegistry(BaseRegistry):
    """
    Registry for solver implementations.

    Key type:
        SolverKey(
            dynamics: DynamicsType,
            target: TargetType,
            solver: SolverType,
        )
    """

    _registry = {}