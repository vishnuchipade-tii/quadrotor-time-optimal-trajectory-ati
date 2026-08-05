import pandas as pd
import numpy as np
from utils.strings import clean_string

LABEL_MAP = {
    "compute time": "Compute Time [s]",
    "duration": "Trajectory Time [s]",
    "trajectory duration": "Trajectory Time [s]",
    "max velocity": "Max Velocity [m/s]",
    "max acceleration": "Max Acceleration [m/s$^2$]",
    "max jerk": "Max Jerk [m/s$^3$]",
    "terminal position error": "Final Position Error [m]",
    "attitude max error": "Attitude Max Error [deg]",
    "attitude rms error": "Attitude RMS Error [deg]",
    "trajectory duration difference": "Trajectory Duration Difference [s]",
    "omega max error": "Angular Speed Max Error [rad/s]",
    "omega rms error": "Angular Speed RMS Error [rad/s]",
    "p max error": "Position Max Error [m]",
    "p rms error": "Position RMS Error [m]",
    "v max error": "Velocity Max Error [m/s]",
    "v rms error": "Velocity RMS Error [m/s]",
    "a max error": "Acceleration Max Error [m/s]",
    "a rms error": "Acceleration RMS Error [m/s]",
    "terminal velocity error": "Terminal Velocity Error [m/s]",
    "rmse x": "Trajectory Tracking RMS Error x [m]",
    "rmse y": "Trajectory Tracking RMS Error y [m]",
    "rmse z": "Trajectory Tracking RMS Error z [m]",
    "total rmse": "Trajectory Tracking RMS Error [m]",
    "max track error": "Max Tracking Error [m]",
    "track rms error": "RMS Tracking Error [m]"

}

def apply_label_map(x, label_map):
    """
    Replace value using LABEL_MAP if exists.
    Otherwise return cleaned original label.
    """

    if x is None:
        return x

    x_clean = clean_string(x).lower()

    return label_map.get(x_clean, clean_string(x))