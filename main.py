"""
Module: main.py
Project: Time-Optimal Trajectory for Quadrotor
"""

from pathlib import Path
from json.decoder import NaN
import yaml

import runners
import planners
import solvers
import analysis

from utils.state_mapping import QuadTrajBundleFactory

from core.experiment import ExperimentScenario
from analysis import ExperimentScenarioComparator

from utils.strings import clean_and_capitalize_string
from utils.export_utils import export_experiment_scenario, save_experiment_scenario
from utils.import_utils import load_scenario
from utils.extract_utils import extract_comparison_metrics_scenario, extract_absolute_metrics_scenario

from builders.experiments_builder import build_experiments
from builders import RunnerBuilder

from core.type import RunnerType

from visualization import MultiQuadTrajectoryPlotter

from visualization.config import configure_matplotlib

configure_matplotlib()

#plt.style.use("visualization/styles/paper.mplstyle")
#

from analysis.report import export_metrics_table
from analysis.report import PdfReport

def run_experiment_scenario(scenario: ExperimentScenario):

    print(f"\n=== Running scenario: {scenario.name} ===")

    scenario_set_name = scenario.scenario_set_name
    for case in scenario.cases:

        print(f"Running {case.name}")

        runner_kwargs = {}

        if case.runner_type == RunnerType.EXTERNAL_RPG:
            rpg_output_root = Path("scenario/generated_rpg")

            vehicle_id = case.quad_config["vehicle_id"]

            runner_kwargs["output_root"] = (
                rpg_output_root
                / f"{vehicle_id}"
                / scenario_set_name
            )

        runner = RunnerBuilder.build(
            case.runner_type,
            **runner_kwargs
        )

        print(f"Using runner: {runner}")

        result = runner.run(
            case,
            return_quad_traj_bundle=True,
        )

        print(f"Compute time: {result.compute_time:.3f}s")

        scenario.results.append(result)

def plot_experiment_scenario(scenario: ExperimentScenario, output_dir=None, show_plots=None, save_plots=None):

    bundles = []

    for r in scenario.results:
        if r.quad_traj_bundle is None:
            bundle = QuadTrajBundleFactory.from_simulation_result(sim_result=r.simulation_result, quadrotor=r.quadrotor)
        else:
            bundle = r.quad_traj_bundle
        bundles.append(bundle)

    plotter = MultiQuadTrajectoryPlotter(bundles=bundles, target=scenario.results[0].case.target, output_dir=output_dir, save_plots=save_plots, show_plots=show_plots, default_linewidth=4)
    plot_title_3d_traj = clean_and_capitalize_string(scenario.name)
    plotter.plot({
        "position": {},
        "velocity": {},
        "attitude": {"degrees": True},
        "trajectory_3d": {"show_orientation": True, "view": "isometric", "elev": 25, "azim":-66, "output_filename":"trajectory_3d", "plot_title": plot_title_3d_traj},
    })


import argparse


def parse_args():

    parser = argparse.ArgumentParser(
        description="Time-optimal quadrotor experiment runner."
    )

    parser.add_argument(
        "scenario_files",
        nargs="+",
        help="Scenario YAML file(s)."
    )

    parser.add_argument(
        "--scenario",
        help="Run only the named scenario."
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results"),
        help="Output directory."
    )

    parser.add_argument(
        "--no-run",
        action="store_true",
        help="Load cached scenarios instead of running simulations."
    )

    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Disable plot generation."
    )

    parser.add_argument(
        "--show-plots",
        action="store_true",
        help="Show plots interactively."
    )

    return parser.parse_args()

# ---------------------------------------
# MAIN
# ---------------------------------------

def main(args):

    RESULTS_DIR = args.output
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Global results directory: {RESULTS_DIR.resolve()}")

    save_plots = not args.no_plots
    show_plots = args.show_plots
    run_simulations = not args.no_run

    #Experiments to run
    scenario_set_files = args.scenario_files

    for scenario_set_file in scenario_set_files:

        scenario_set_path = Path(scenario_set_file)
        scenario_set_name = scenario_set_path.stem

        print(f"\n[INFO] Processing experiment file: {scenario_set_file}")

        # -------------------------------
        # Load experiment YAML
        # -------------------------------
        with open(scenario_set_path, "r") as f:
            exp_cfg = yaml.safe_load(f)

        # -------------------------------
        # Load quad config FROM experiment file
        # -------------------------------
        quad_config_path = Path(exp_cfg["quad_config"])


        with open(quad_config_path, "r") as f:
            quad_cfg = yaml.safe_load(f)

        vehicle_id = quad_cfg["vehicle_id"]

        # -------------------------------
        # Output directory (now includes quad + experiment)
        # -------------------------------
        output_dir = RESULTS_DIR / vehicle_id / scenario_set_name
        output_dir.mkdir(parents=True, exist_ok=True)

        latex_output_dir = RESULTS_DIR/ "latex" / vehicle_id / scenario_set_name
        latex_output_dir.mkdir(parents=True, exist_ok=True)

        print(f"[INFO] Saving results to: {output_dir}")

        scenarios = build_experiments(scenario_set_file, quad_cfg)

        if args.scenario:
            scenarios = [
                s for s in scenarios
                if s.name == args.scenario
            ]

        all_abs_metrics_rows = []
        all_comp_metrics_rows = []

        for scenario in scenarios:

            scenario_output_dir = Path(output_dir) / scenario.name
            scenario_output_dir.mkdir(parents=True, exist_ok=True)

            scenario_pickle_file = scenario_output_dir / "scenario.pkl"

            if run_simulations or not scenario_pickle_file.exists():
                print(f"[INFO] Running scenario: {scenario.name}")

                run_experiment_scenario(scenario)

                save_experiment_scenario(scenario, scenario_output_dir)

                export_experiment_scenario(scenario, scenario_output_dir)

            else:
                print(f"[INFO] Loading cached scenario: {scenario.name}")

                scenario = load_scenario(scenario_output_dir)

            #Plotting scenario results
            plot_experiment_scenario(scenario, output_dir=scenario_output_dir, save_plots=save_plots, show_plots=show_plots)

            #Also save to a separate latex directory
            latex_plot_output_dir = Path(latex_output_dir , scenario.name)
            plot_experiment_scenario(scenario, output_dir=latex_plot_output_dir, save_plots=save_plots, show_plots=show_plots)

            # Extract the absolute metrics from scenario result
            abs_metrics_rows = extract_absolute_metrics_scenario(scenario, data_dir=scenario_output_dir)
            all_abs_metrics_rows.extend(abs_metrics_rows)

            #Comparison of the experiment cases in the scenario, if requested
            if scenario.mode == "compare":
                scenario_comparator = ExperimentScenarioComparator(scenario)

                comparison_result = scenario_comparator.compare_to_reference(
                    reference_name="RPG",
                    signals=("p", "v", "a", "omega")
                )

                comp_metrics_rows = extract_comparison_metrics_scenario(scenario, comparison_result, data_dir=scenario_output_dir)
                all_comp_metrics_rows.extend(comp_metrics_rows)

                for name, pair_comparison_result in comparison_result.pair_results.items():
                    print(f"\n{name}")
                    pair_comparison_result.print_summary()


        #Save the absolute metrics for each trajectory
        df_abs_metrics, latex_abs_metrics = export_metrics_table(
            metrics_rows=all_abs_metrics_rows,
            output_dir=Path(output_dir),
            latex_output_dir=Path(latex_output_dir),

            filename="table_abs_metrics.tex",
            csv_filename="all_abs_metrics.csv",

            caption="Performance of the dynamics models",
            label="tab:performance",

            second_caption=r"\small{(6D: Full 6-DOF motion model, DI: Double integrator with drag, TI$^*$: Proposed triple integrator with drag)}"
        )

        # Save the comparative metrics for each trajectory with RPG as the refence
        df_comp_metrics, latex_comp_metrics = export_metrics_table(
            metrics_rows=all_comp_metrics_rows,
            output_dir=Path(output_dir),
            latex_output_dir=Path(latex_output_dir),

            filename="table_comp_metrics.tex",
            csv_filename="all_comp_metrics.csv",

            caption="Comparison of the dynamics models and planners with 6DOF model with RPG Planner",
            label="tab:comparison",

            second_caption=r"\small{(DI: Double integrator with drag, TI$^*$: Proposed triple integrator with drag)}"
        )


        report = PdfReport(
            output_dir=Path(output_dir),
            export_dirs=[Path(latex_output_dir)],
            title="Trajectory Analysis"
        )

        report.add_section(
            "Absolute Metrics",
            latex_abs_metrics,
        )

        report.add_section(
            "Comparative Metrics",
            latex_comp_metrics,
        )

        pdf = report.build()

    print('DONE')

# ---------------------------------------
# ENTRY POINT
# ---------------------------------------

if __name__ == "__main__":
    main(parse_args())