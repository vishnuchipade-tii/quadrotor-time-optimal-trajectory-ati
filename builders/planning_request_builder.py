from core.planning import PlanningRequest
from core.config.solver_config import SolverConfig

class PlanningRequestBuilder:

    @staticmethod
    def build(
        initial_state,
        dynamics,
        target,
        input_bounds,
        planner_spec,
    ):
        """
        Build a PlanningRequest template.
        """

        solver_spec = planner_spec.get("solver")

        solver_config = SolverConfig.from_config(solver_spec)

        return PlanningRequest(

            initial_state=initial_state,

            dynamics=dynamics,

            target=target,

            state_bounds=None,

            input_bounds=input_bounds,

            solver_config=solver_config,
        )