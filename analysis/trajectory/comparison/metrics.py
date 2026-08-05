from dataclasses import dataclass
import numpy as np

@dataclass
class SignalMetric:

    rms: float
    max: float
    mean: float
    final: float

    history: np.ndarray


@dataclass
class AttitudeMetric:

    rms_deg: float
    max_deg: float
    mean_deg: float
    final_deg: float

    history_deg: np.ndarray