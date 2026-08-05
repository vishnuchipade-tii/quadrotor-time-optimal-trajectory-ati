from abc import ABC, abstractmethod


class BasePlanner(ABC):

    def __init__(self, planner_id):
        self.planner_id = planner_id
        self.result = None

    @abstractmethod
    def plan(self, *args, **kwargs):
        pass