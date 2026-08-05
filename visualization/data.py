
from dataclasses import dataclass
import numpy as np
from typing import Optional


@dataclass
class ExtractedTrajectoryData:

    t: np.ndarray

    label: str
    color: str
    linestyle: str

    pos: np.ndarray
    vel: np.ndarray

    R_hist: np.ndarray
    quat: np.ndarray

    omega: np.ndarray

    thrust: np.ndarray

    flat_acc: Optional[np.ndarray]
    flat_jerk: Optional[np.ndarray]

    yaw: Optional[np.ndarray]
    yaw_rate: Optional[np.ndarray]

    euler: np.ndarray

