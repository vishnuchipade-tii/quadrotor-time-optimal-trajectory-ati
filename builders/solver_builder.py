from core.registry.solver_registry import SolverRegistry
from core.keys.solver_key import SolverKey

class SolverBuilder:
    @staticmethod
    def build(
        dynamics_type,
        target_type,
        solver_type,
    ):
        solver_cls = SolverRegistry.get(
            SolverKey(
                dynamics=dynamics_type,
                target=target_type,
                solver=solver_type,
            )
        )

        return solver_cls()