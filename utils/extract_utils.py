import numpy as np
from utils.math_utils import safe_norm
import json

def extract_absolute_metrics(run_result, run_data_dir: Optional[Path] = None):
  bundle = run_result.quad_traj_bundle

  exp_case = run_result.case

  # ----------------------------------------------------
  # Identify model type
  # ----------------------------------------------------
  if exp_case.dynamics_model is not None:
    model_id = exp_case.dynamics_model.model_id
  else:
    model_id = "6DOF"

  trajectory = bundle.trajectory

  # helper safe accessors
  def has_flat(field):
    return hasattr(trajectory[0], "flat") and getattr(trajectory[0].flat, field, None) is not None

  def has_state(field):
    return hasattr(trajectory[0].state, field)

  # ----------------------------------------------------
  # Common metrics (ALL models)
  # ----------------------------------------------------

  max_velocity = np.nan

  if has_state("v"):
    max_velocity = np.max([
      safe_norm(pt.state.v)
      for pt in trajectory
    ])

  # ----------------------------------------------------
  # Acceleration (flat models only)
  # ----------------------------------------------------

  max_acceleration = np.nan

  if has_flat("a"):
    max_acceleration = np.max([
      safe_norm(pt.flat.a)
      for pt in trajectory
      if pt.flat is not None and pt.flat.a is not None
    ])

  # ----------------------------------------------------
  # Jerk (only where defined)
  # ----------------------------------------------------

  max_jerk = np.nan

  if has_flat("j"):
    max_jerk = np.max([
      safe_norm(pt.flat.j)
      for pt in trajectory
      if pt.flat is not None and pt.flat.j is not None
    ])

  # ----------------------------------------------------
  # Duration (always valid)
  # ----------------------------------------------------

  duration = bundle.time[-1] if bundle.time is not None else np.nan

  # ----------------------------------------------------
  # Compute time (always valid)
  # ----------------------------------------------------

  compute_time = run_result.compute_time if run_result.compute_time is not None else np.nan

  # Check if the trajectory tracking results are available if so then extract them

  tracking_metrics = {}

  if run_data_dir is not None:

    metrics_file = (
        run_data_dir
        / "traj_track_analysis"
        / "metrics.json"
    )

    if metrics_file.exists():
      with open(metrics_file, "r") as f:
        tracking_metrics = json.load(f)

  # ----------------------------------------------------
  # Return unified dict
  # ----------------------------------------------------

  return {
    "compute_time": compute_time,
    "trajectory_duration": duration,

    "max_velocity": max_velocity,
    "max_acceleration": max_acceleration,
    "track_rms_error": tracking_metrics.get("total_rmse"),
    "max_track_error": tracking_metrics.get("max_error_2norm")
  }


def extract_comparison_metrics(comp_result):
  """
  From TrajectoryComparisonResult
  """

  out = {}

  # timing differences
  out["trajectory_duration_difference"] = comp_result.duration_difference

  # terminal errors
  out["terminal_position_error"] = comp_result.terminal_position_error
  out["terminal_velocity_error"] = comp_result.terminal_velocity_error

  # signal errors
  for sig, m in comp_result.signals.items():
    out[f"{sig}_rms_error"] = m.rms
    out[f"{sig}_max_error"] = m.max

  # attitude
  if comp_result.attitude is not None:
    out["attitude_rms_error"] = comp_result.attitude.rms_deg
    out["attitude_max_error"] = comp_result.attitude.max_deg

  return out


def extract_absolute_metrics_scenario(scenario, data_dir):
  """
  Return absolute metrics for the results in the scenario
  """

  abs_metrics_rows = []

  case_name = scenario.name

  # ----------------------------------------------------
  # Absolute metrics
  # ----------------------------------------------------
  for run in scenario.results:
    model_dir_label = run.case.dynamics_model.info.directory_name
    run_dir = data_dir / f"{model_dir_label}"

    abs_metrics = extract_absolute_metrics(run, run_data_dir=run_dir)

    model_label = run.case.dynamics_model.info.display_name
    abs_metrics_rows.append({
      "case_study": case_name,
      "model": model_label,
      **abs_metrics
    })

  return abs_metrics_rows


def extract_comparison_metrics_scenario(scenario, comparator_result, data_dir):
  """
  Return comparison metrics for the results in the scenario
  """

  comp_metrics_rows = []

  case_name = scenario.name

  # ----------------------------------------------------
  # Comparator metrics
  # ----------------------------------------------------
  for algo_name, comp_result in comparator_result.pair_results.items():
    rel_metrics = extract_comparison_metrics(comp_result)

    model_label = comp_result.run_result.case.dynamics_model.info.display_name
    comp_metrics_rows.append({
      "case_study": case_name,
      "model": model_label,
      **rel_metrics
    })

  return comp_metrics_rows