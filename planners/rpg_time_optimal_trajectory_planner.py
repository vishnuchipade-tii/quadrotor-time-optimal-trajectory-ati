from planners.base_planner import BasePlanner
from core.type.planner_type import PlannerType
from core.registry.planner_registry import PlannerRegistry

from external.rpg_time_opt_traj_planner.backend import RPGTimeOptimalTrajectoryPlannerBackend

@PlannerRegistry.register(PlannerType.RPG_TIME_OPT_TRAJ)
class RPGTimeOptimalTrajectoryPlanner(BasePlanner):

  def __init__(self, backend: RPGTimeOptimalTrajectoryPlannerBackend, planner_id="rpg_time_opt_traj_planner"):

    super().__init__(planner_id)

    self.backend = backend

    self.compute_time = None

  def plan(self, case, output_root):
    quad_config = case.quad_config["models"]["6dof"]
    experiment_config = case.experiment_config

    bundle, compute_time = self.backend.solve(
                              quad_config=quad_config,
                              experiment_config=experiment_config,
                              output_root=output_root,
                            )

    self.result = bundle
    self.compute_time = compute_time

    return True