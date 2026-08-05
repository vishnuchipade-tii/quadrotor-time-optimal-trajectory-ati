import pandas as pd
import numpy as np

from core.state import (
    QuadState,
    QuadControl,
    FlatState
)

from core.trajectory import (QuadTrajectoryBundle, QuadTrajectoryPoint)

from scipy.spatial.transform import Rotation as R


def load_rpg_csv(
        csv_path,
        label="RPG",
        color="green",
        linestyle="-."
):

    df = pd.read_csv(csv_path)

    trajectory = []

    # =========================================================
    # Build trajectory
    # =========================================================

    for _, row in df.iterrows():

        # -----------------------------------------------------
        # Position
        # -----------------------------------------------------

        p = np.array([
            row["p_x"],
            row["p_y"],
            row["p_z"]
        ])

        # -----------------------------------------------------
        # Velocity
        # -----------------------------------------------------

        v = np.array([
            row["v_x"],
            row["v_y"],
            row["v_z"]
        ])

        #Acceleration
        a = np.array([
            row["a_lin_x"],
            row["a_lin_y"],
            row["a_lin_z"]
        ])

        # -----------------------------------------------------
        # Quaternion
        #
        # scipy expects:
        # [x,y,z,w]
        # -----------------------------------------------------

        q = np.array([
            row["q_x"],
            row["q_y"],
            row["q_z"],
            row["q_w"]
        ])

        # -----------------------------------------------------
        # Rotation matrix
        # -----------------------------------------------------

        Rwb = R.from_quat(q).as_matrix()

        # -----------------------------------------------------
        # Body rates
        # -----------------------------------------------------

        omega = np.array([
            row["w_x"],
            row["w_y"],
            row["w_z"]
        ])

        # -----------------------------------------------------
        # Controls
        #
        # total thrust
        # -----------------------------------------------------

        thrust = (
            row["u_1"] +
            row["u_2"] +
            row["u_3"] +
            row["u_4"]
        )

        # -----------------------------------------------------
        # Build objects
        # -----------------------------------------------------

        state = QuadState(
            p=p,
            v=v,
            R=Rwb,
            q=q,
            omega=omega
        )

        control = QuadControl(
            thrust=thrust
        )

        flat = FlatState(p=p, v=v, a=a)

        point = QuadTrajectoryPoint(
            flat=flat,
            state=state,
            control=control
        )

        trajectory.append(point)

    # =========================================================
    # Time vector
    # =========================================================

    time = df["t"].to_numpy()

    # =========================================================
    # Bundle
    # =========================================================

    bundle = QuadTrajectoryBundle(
        time=time,
        label=label,
        trajectory=trajectory,
        color=color,
        linestyle=linestyle
    )

    return bundle

import json
from pathlib import Path

def load_rpg_result(
        result_csv_path,
        metadata_path=None,
        label="6DOF",
        color="green",
        linestyle="-."
):
    """
    Load RPG trajectory bundle + metadata.
    """

    bundle = load_rpg_csv(
        result_csv_path,
        label=label,
        color=color,
        linestyle=linestyle
    )

    compute_time = None

    if metadata_path is None:
        metadata_path = (
            Path(result_csv_path)
            .parent
            / "metadata.json"
        )

    if Path(metadata_path).exists():

        with open(metadata_path, "r") as f:
            metadata = json.load(f)

        compute_time = metadata.get(
            "compute_time_s",
            None
        )

    return bundle, compute_time