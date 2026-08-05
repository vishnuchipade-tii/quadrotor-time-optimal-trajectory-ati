from pathlib import Path
from core.experiment import ExperimentCase, RunResult

from core.type import RunnerType
from core.registry.runner_registry import RunnerRegistry
from runners.base_runner import BaseRunner


@RunnerRegistry.register(RunnerType.EXTERNAL_RPG)
class RPGCaseRunner(BaseRunner):

  def __init__(self, output_root):
    super().__init__(
      RunnerType.EXTERNAL_RPG
    )
    self.output_root = Path(output_root)

  def run(
      self,
      case: ExperimentCase,
      return_quad_traj_bundle=True,
  ) -> RunResult:

    case.planner.plan(
      case=case,
      output_root=self.output_root,
    )

    return RunResult(
            name=case.name,
            case=case,
            quadrotor=None,
            simulation_result=None,
            compute_time=case.planner.compute_time,
            quad_traj_bundle=case.planner.result,
        )
