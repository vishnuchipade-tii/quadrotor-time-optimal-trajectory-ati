import time

from core.simulation import (SimulationResult)
from core.quadrotor import Quadrotor
from core.experiment import ExperimentCase, RunResult
from runners import BaseRunner
from utils.state_mapping import QuadTrajBundleFactory

from core.type import RunnerType
from core.registry import RunnerRegistry
from runners.simulation_runner import run_simulation

@RunnerRegistry.register(RunnerType.INTERNAL)
class InternalCaseRunner(BaseRunner):

  def __init__(self):
    super().__init__(
      RunnerType.INTERNAL
    )

  def run(
      self,
      case: ExperimentCase,
      return_quad_traj_bundle=True,
  ) -> RunResult:
    uav = Quadrotor(
      id=0,
      initial_state=case.initial_state.copy(),
      dynamics_model=case.dynamics_model,
      params=case.params,
      name=case.name,
    )

    #Run the simulation
    t0 = time.time()

    state_hist, control_hist, time_hist = run_simulation(
      uav=uav,
      planner=case.planner,
      target=case.target,
      sim_metadata=case.sim_metadata,
    )

    compute_time = time.time() - t0

    #Gather the results
    traj_label = case.dynamics_model.display_name

    sim_result = SimulationResult(
      time_history=time_hist,
      state_history=state_hist,
      control_history=control_hist,
      label=traj_label,
      color=case.color,
      line_style=case.linestyle,
    )

    quad_traj_bundle = None
    if return_quad_traj_bundle:
      quad_traj_bundle = QuadTrajBundleFactory.from_simulation_result(
        sim_result=sim_result,
        quadrotor=uav,
      )

    return RunResult(
      name=case.name,
      case=case,
      quadrotor=uav,
      simulation_result=sim_result,
      compute_time=compute_time,
      quad_traj_bundle=quad_traj_bundle,
    )