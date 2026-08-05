
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interp1d
from dataclasses import dataclass
from typing import List, Union, Optional

from pathlib import Path
import json
from utils.import_utils import load_csvs

from analysis.trajectory.tracking import TrackingSegment

PathLike = Union[str, Path]

class TrajectoryTrackingAnalyzer:

  def __init__(
      self,
      reference_csv: Union[PathLike, List[PathLike]],
      actual_csv: Union[PathLike, List[PathLike]],
      event_name: str = "track_traj",
      time_scale: float = 1e-9  # for ns timestamps
  ):
    self.ref_df = load_csvs(reference_csv)
    self.act_df = load_csvs(actual_csv)

    self.event_name = event_name
    self.time_scale = time_scale

    self.segment: Optional[TrackingSegment] = None
    self.results = None

  # --------------------------------------------------------
  # Extract tracking segment using events
  # --------------------------------------------------------
  def extract_tracking_segment(
      self,
      event_start="ACTION_START",
      event_end="ACTION_COMPLETE",
      end_trim_sec: float = 0.0
  ):

    self.act_df.columns = self.act_df.columns.str.strip()

    start_rows = self.act_df[
      (self.act_df["event_type"] == event_start) &
      (self.act_df["event_name"] == self.event_name)
      ]

    end_rows = self.act_df[
      (self.act_df["event_type"] == event_end) &
      (self.act_df["event_name"] == self.event_name)
      ]

    if start_rows.empty or end_rows.empty:
      raise RuntimeError(
        f"Could not find {event_start}/{event_end} "
        f"for event '{self.event_name}'"
      )

    start_t_ns = start_rows["timestamp_ns"].iloc[0]
    end_t_ns = end_rows["timestamp_ns"].iloc[0]

    if end_trim_sec > 0.0:
      end_t_ns -= int(end_trim_sec * 1e9)

    if end_t_ns <= start_t_ns:
      raise RuntimeError(
        "Trimmed end time occurs before start time"
      )

    df = self.act_df[
      (self.act_df["timestamp_ns"] > start_t_ns) &
      (self.act_df["timestamp_ns"] < end_t_ns)
      ].copy()

    # time normalized to tracking start
    df["t"] = (
                  df["timestamp_ns"] - start_t_ns
              ) * self.time_scale

    self.segment = TrackingSegment(
      start_t=start_t_ns * self.time_scale,
      end_t=end_t_ns * self.time_scale,
      df=df
    )

    return df

  # --------------------------------------------------------
  # Align trajectories
  # --------------------------------------------------------
  def compute_alignment(self):

    if self.segment is None:
      self.extract_tracking_segment()

    act = self.segment.df
    ref = self.ref_df.copy()

    # ensure time exists
    if "t" not in ref.columns:
      raise ValueError("Reference CSV must contain column 't'")

    t_ref = ref["t"].values
    ref_xyz = ref[["x", "y", "z"]].values

    # interpolation of reference
    fx = interp1d(t_ref, ref_xyz[:, 0], fill_value="extrapolate")
    fy = interp1d(t_ref, ref_xyz[:, 1], fill_value="extrapolate")
    fz = interp1d(t_ref, ref_xyz[:, 2], fill_value="extrapolate")

    t_act = act["t"].values
    act_xyz = act[["x", "y", "z"]].values

    ref_interp = np.stack([fx(t_act), fy(t_act), fz(t_act)], axis=1)

    error = act_xyz - ref_interp

    rmse = np.sqrt(np.mean(error ** 2, axis=0))
    total_rmse = np.sqrt(np.mean(np.sum(error ** 2, axis=1)))

    error_norm = np.linalg.norm(error, axis=1)  # 2-norm per timestep

    max_error_xyz = np.max(np.abs(error), axis=0)
    max_error_2norm = np.max(error_norm)

    self.results = {
      "t": t_act,
      "actual": act_xyz,
      "reference": ref_interp,
      "error": error,
      "rmse": rmse,
      "total_rmse": total_rmse,
      "max_error_xyz": max_error_xyz,
      "max_error_2norm": max_error_2norm
    }

    return self.results

  # --------------------------------------------------------
  # Metrics
  # --------------------------------------------------------
  def print_metrics(self):
    if self.results is None:
      self.compute_alignment()

    print("\n===== Tracking Performance =====")
    print(f"RMSE X: {self.results['rmse'][0]:.4f}")
    print(f"RMSE Y: {self.results['rmse'][1]:.4f}")
    print(f"RMSE Z: {self.results['rmse'][2]:.4f}")
    print(f"TOTAL RMSE: {self.results['total_rmse']:.4f}")
    print(f"MAX ERROR X: {self.results['max_error_xyz'][0]:.4f}")
    print(f"MAX ERROR Y: {self.results['max_error_xyz'][1]:.4f}")
    print(f"MAX ERROR Z: {self.results['max_error_xyz'][2]:.4f}")
    print(f"MAX 2-NORM ERROR: {self.results['max_error_2norm']:.4f}")

  # --------------------------------------------------------
  # 3D Plot
  # --------------------------------------------------------
  def plot_3d(self):
    if self.results is None:
      self.compute_alignment()

    act = self.results["actual"]
    ref = self.results["reference"]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], label="Reference", linewidth=2)
    ax.plot(act[:, 0], act[:, 1], act[:, 2], label="Actual", linewidth=2)

    ax.set_title("Trajectory Tracking (3D)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    plt.show()

  # --------------------------------------------------------
  # Time series plot
  # --------------------------------------------------------
  def plot_time_series(self):
    if self.results is None:
      self.compute_alignment()

    t = self.results["t"]
    act = self.results["actual"]
    ref = self.results["reference"]
    err = self.results["error"]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    labels = ["X", "Y", "Z"]

    for i in range(3):
      axs[i].plot(t, ref[:, i], label="Reference")
      axs[i].plot(t, act[:, i], label="Actual")
      axs[i].set_ylabel(labels[i])
      axs[i].grid()
      axs[i].legend()

    axs[-1].set_xlabel("Time (s)")
    plt.suptitle("Trajectory Tracking Comparison")
    plt.show()

    # error norm
    plt.figure()
    plt.plot(t, np.linalg.norm(err, axis=1))
    plt.title("Tracking Error Norm")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.grid()
    plt.show()

  # --------------------------------------------------------
  # Full pipeline
  # --------------------------------------------------------
  def run_all(self):
    self.extract_tracking_segment()
    self.compute_alignment()
    self.print_metrics()
    self.plot_3d()
    self.plot_time_series()