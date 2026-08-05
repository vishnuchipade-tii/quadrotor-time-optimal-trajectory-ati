class BaseTerminalConstraint:
  """
  Base class for terminal constraints.
  """

  def residual(
      self,
      drift,
      drift_dot,
      reach,
      reach_dot
  ):
    raise NotImplementedError

  def optimal_control(
      self,
      drift,
      u_max
  ):
    raise NotImplementedError