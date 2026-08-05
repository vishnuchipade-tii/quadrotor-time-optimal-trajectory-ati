from core.type.planner_type import PlannerType
from core.registry.planner_registry import PlannerRegistry

class PlannerBuilder:

  @staticmethod
  def build(planner_type: PlannerType, **kwargs):
    planner_cls = PlannerRegistry.get(planner_type)

    return planner_cls(**kwargs)