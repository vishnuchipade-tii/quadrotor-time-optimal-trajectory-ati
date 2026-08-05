from core.type.runner_type import RunnerType
from core.registry.runner_registry import RunnerRegistry

class RunnerBuilder:

  @staticmethod
  def build(
      runner_type: RunnerType,
      **kwargs
  ):
    runner_cls = RunnerRegistry.get(
      runner_type
    )

    return runner_cls(**kwargs)