

import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization import ExtractedTrajectoryData

class TrajectoryExtractor:
  @staticmethod
  def _extract_array(traj, getter):
    """
    Extract stacked numpy array from trajectory.
    Returns None if any sample is missing.
    """

    values = []

    for s in traj:

      try:
        v = getter(s)

        if v is None:
          return None

        values.append(v)

      except (AttributeError, TypeError):
        return None

    return np.array(values)

  @staticmethod
  def _extract_scalar(traj, getter):

    values = []

    for s in traj:

      try:
        v = getter(s)

        if v is None:
          return None

        values.append(v)

      except (AttributeError, TypeError):
        return None

    return np.array(values)

  def extract(self, bundle) -> ExtractedTrajectoryData:

    traj = bundle.trajectory

    # =====================================================
    # Core translational states
    # =====================================================

    pos = self._extract_array(
      traj,
      lambda s: s.state.p
    )

    vel = self._extract_array(
      traj,
      lambda s: s.state.v
    )

    # =====================================================
    # Attitude-related
    # =====================================================

    R_hist = self._extract_array(
      traj,
      lambda s: s.state.R
    )

    quat = self._extract_array(
      traj,
      lambda s: s.state.q
    )

    omega = self._extract_array(
      traj,
      lambda s: s.state.omega
    )

    # =====================================================
    # Control
    # =====================================================

    thrust = self._extract_scalar(
      traj,
      lambda s: s.control.thrust
    )

    # =====================================================
    # Flat outputs
    # =====================================================

    flat_acc = self._extract_array(
      traj,
      lambda s: s.flat.a
    )

    flat_jerk = self._extract_array(
      traj,
      lambda s: s.flat.j
    )

    yaw = self._extract_scalar(
      traj,
      lambda s: s.flat.yaw
    )

    yaw_rate = self._extract_scalar(
      traj,
      lambda s: s.flat.yaw_rate
    )

    # =====================================================
    # Euler angles
    # =====================================================

    euler = None

    if quat is not None:

      N = len(quat)

      euler = np.zeros((N, 3))

      for i in range(N):
        rot = R.from_quat(quat[i])

        euler[i] = rot.as_euler(
          'xyz',
          degrees=False
        )

    return ExtractedTrajectoryData(
      t=bundle.time,

      label=bundle.label,
      color=bundle.color,
      linestyle=bundle.linestyle,

      pos=pos,
      vel=vel,

      R_hist=R_hist,
      quat=quat,

      omega=omega,

      thrust=thrust,

      flat_acc=flat_acc,
      flat_jerk=flat_jerk,

      yaw=yaw,
      yaw_rate=yaw_rate,

      euler=euler
    )