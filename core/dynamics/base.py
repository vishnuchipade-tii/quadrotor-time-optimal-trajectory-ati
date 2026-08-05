from abc import ABC, abstractmethod

from core.dynamics.model_info import ModelInfo


class BaseDynamicsModel(ABC):
  info: ModelInfo

  @property
  def model_id(self):
    return self.info.model_id

  @property
  def display_name(self):
    return self.info.display_name

  @property
  def directory_name(self):
    return self.info.directory_name

  @abstractmethod
  def dim_state(self) -> int:
    pass

  @abstractmethod
  def rhs(self, state, control):
    """
    Returns right hand side f(x,u) of the system dynamics x_dot = f(x,u), i.e., the derivative x_dot at a given state and control input
    """
    pass

  @abstractmethod
  def step(self, state, control, dt):
    """
    Returns next state (not derivative).
    """
    pass