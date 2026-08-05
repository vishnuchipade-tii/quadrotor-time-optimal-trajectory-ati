from abc import ABC, abstractmethod


class BaseRunner(ABC):

    def __init__(self, runner_type):
        self.runner_type = runner_type
        self.runner_id = runner_type.value

    def __repr__(self):
        return f"{self.__class__.__name__}(type={self.runner_type.value})"

    @abstractmethod
    def run(
        self,
        case,
        return_quad_traj_bundle=True,
    ):
        pass