
from builders.solver_builder import SolverBuilder
import numpy as np

from core.dynamics.models import DynamicsType
from planners.base_planner import BasePlanner
from core.type.planner_type import PlannerType
from core.config.planner_config import PlannerConfig
from core.registry.planner_registry import PlannerRegistry

@PlannerRegistry.register(PlannerType.TIME_OPT_TRAJ)
class TimeOptimalTrajectoryPlanner(BasePlanner):

    def __init__(
        self,
        planning_request,
        planner_config: PlannerConfig,
        planner_id="time_opt_traj_planner"
    ):
        super().__init__(planner_id)

        planner_config.validate()

        self.planning_request_template = planning_request

        self.planner_config = planner_config

        self.replan = planner_config.replan
        self.replan_interval = planner_config.replan_interval

        self.last_plan_time = None
        self.last_state = None
        self.last_target = None

        self.sim_metadata = None

        self.solver_config = planning_request.solver_config

        dynamics_type = DynamicsType.from_string(planning_request.dynamics.info.model_id)

        target_type = planning_request.target.target_type

        self.solver = SolverBuilder.build(dynamics_type=dynamics_type, target_type=target_type, solver_type=self.solver_config.solver_type)

        self.solver_id = self.solver.solver_id

    def plan(
        self,
        initial_state,
        target,
        sim_metadata,
        current_time=None,
    ):

        self.sim_metadata = sim_metadata

        if target is None:
            target = self.planning_request_template.target

        self.last_state = initial_state

        self.last_target = target

        if current_time is not None:
            self.last_plan_time = current_time

        planning_request = self.planning_request_template.update(
            state=initial_state,
            target=target
        )

        print(f"[TimeOptimalTrajectoryPlanner] Solving planning request")
        self.result = self.solver.solve(planning_request)

        return True

    def control(self, t, state):

        if (
            self.replan
            and self.last_plan_time is not None
            and t - self.last_plan_time >= self.replan_interval
        ):
            self.plan(
                initial_state=state,
                target=self.last_target,
                sim_metadata=self.sim_metadata,
                current_time=t,
            )

        if self.result is None:
            return np.zeros(3)

        if t >= self.result.final_time:
            return self.result.controls[-1]

        idx = np.searchsorted(
            self.result.times,
            t
        )

        idx = np.clip(
            idx,
            0,
            len(self.result.controls)-1
        )

        return self.result.controls[idx]