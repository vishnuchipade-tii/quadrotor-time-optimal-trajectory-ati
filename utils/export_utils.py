# export_utils.py

from core.experiment import RunResult

from pathlib import Path
import json
import numpy as np
import pandas as pd

import pickle

from typing import Union, Optional


def export_run_result(run: RunResult, out_dir: str):
    """
    Exports a single RunResult into its own folder.

    Gracefully handles:
      - missing quad_traj_bundle
      - missing simulation_result
      - partially-populated trajectory points
    """

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ==========================================================
    # 1. Export trajectory
    # ==========================================================
    bundle = getattr(run, "quad_traj_bundle", None)

    if bundle is not None:

        csv_path = out_dir / "trajectory.csv"

        with open(csv_path, "w") as f:
            f.write("t,x,y,z,vx,vy,vz,ax,ay,az,yaw\n")

            for t, pt in zip(bundle.time, bundle.trajectory):

                flat = getattr(pt, "flat", None)

                p = (
                    flat.p
                    if flat is not None and flat.p is not None
                    else np.zeros(3)
                )

                v = (
                    flat.v
                    if flat is not None and flat.v is not None
                    else np.zeros(3)
                )

                a = (
                    flat.a
                    if flat is not None and flat.a is not None
                    else np.zeros(3)
                )

                yaw = (
                    flat.yaw
                    if flat is not None and flat.yaw is not None
                    else 0.0
                )

                f.write(
                    f"{t:.6f},"
                    f"{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},"
                    f"{v[0]:.6f},{v[1]:.6f},{v[2]:.6f},"
                    f"{a[0]:.6f},{a[1]:.6f},{a[2]:.6f},"
                    f"{yaw:.6f}\n"
                )

    # ==========================================================
    # 2. Export simulation metadata (if available)
    # ==========================================================
    sim = getattr(run, "simulation_result", None)

    if sim is not None:

        dt = None

        if (
            hasattr(sim, "time_history")
            and sim.time_history is not None
            and len(sim.time_history) > 1
        ):
            dt = sim.time_history[1] - sim.time_history[0]

        sim_data = {
            "label": getattr(sim, "label", None),
            "dt": dt,
            "num_steps": (
                len(sim.time_history)
                if hasattr(sim, "time_history") and sim.time_history is not None
                else None
            ),
        }

        with open(out_dir / "simulation.json", "w") as f:
            json.dump(sim_data, f, indent=2)

    # ==========================================================
    # 3. Export run metadata
    # ==========================================================
    meta = {
        "name": getattr(run, "name", None),
        "compute_time": getattr(run, "compute_time", None),
    }

    if hasattr(run, "case") and run.case is not None:
        meta["case"] = str(run.case)

    with open(out_dir / "run_meta.json", "w") as f:
        json.dump(meta, f, indent=2)


def export_experiment_scenario(scenario, out_dir: Union[str, Path]):
    """
    Exports full ExperimentScenario into structured directory.
    """

    # -------------------------
    # 1. Save scenario metadata
    # -------------------------
    scenario_meta = {
        "name": scenario.name,
        "target_type": scenario.target_type.value,
        "num_cases": len(scenario.cases),
        "num_results": len(scenario.results),
    }

    with open(out_dir / "scenario_meta.json", "w") as f:
        json.dump(scenario_meta, f, indent=2)

    # -------------------------
    # 2. Export each run
    # -------------------------
    for i, run in enumerate(scenario.results):
        #case_name = getattr(run.case, "name", f"case_{i}")

        model_label = run.case.dynamics_model.info.directory_name
        #get_model_label(run, label_type="directory")

        run_dir = out_dir / f"{model_label}"

        export_run_result(run, run_dir)

def save_experiment_scenario(scenario, output_dir: Path):
    scenario_file = output_dir / "scenario.pkl"

    with open(scenario_file, "wb") as f:
        pickle.dump(scenario, f)

    print(f"[INFO] Saved scenario to {scenario_file}")
