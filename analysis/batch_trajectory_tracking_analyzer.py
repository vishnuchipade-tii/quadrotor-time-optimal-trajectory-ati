from pathlib import Path
import json

from analysis.trajectory import TrajectoryTrackingAnalyzer

class BatchTrajectoryTrackingAnalyzer:

    def __init__(
        self,
        reference_root,
        actual_root,
        output_dir_name="traj_track_analysis"
    ):
        self.reference_root = Path(reference_root)
        self.actual_root = Path(actual_root)
        self.output_dir_name = output_dir_name

    # --------------------------------------------------
    # discover reference trajectories
    # --------------------------------------------------

    def find_reference_csvs(self):

        return sorted(
            self.reference_root.rglob("trajectory.csv")
        )

    # --------------------------------------------------
    # derive expected test identifier
    # --------------------------------------------------

    def reference_id(self, csv_path):

        rel = csv_path.relative_to(self.reference_root)

        return "_".join(rel.parts[:-1])

    # --------------------------------------------------
    # locate matching actual csv
    # --------------------------------------------------

    def find_actual_csv(self, reference_csv):

        ref_id = self.reference_id(reference_csv)

        candidates = list(
            self.actual_root.rglob("gz_ground_truth.csv")
        )

        for candidate in candidates:

            candidate_str = str(candidate)

            if ref_id in candidate_str:
                return candidate

        return None

    # --------------------------------------------------
    # analyze one pair
    # --------------------------------------------------

    def analyze_pair(
        self,
        reference_csv,
        actual_csv,
        end_trim_sec = 0.0
    ):

        analyzer = TrajectoryTrackingAnalyzer(
            reference_csv=reference_csv,
            actual_csv=actual_csv,
            event_name="track_traj"
        )

        analyzer.extract_tracking_segment(end_trim_sec=end_trim_sec)
        analyzer.compute_alignment()

        return analyzer

    # --------------------------------------------------
    # save outputs
    # --------------------------------------------------

    def save_results(
        self,
        analyzer,
        reference_csv
    ):

        out_dir = (
            reference_csv.parent
            / self.output_dir_name
        )

        out_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        metrics = {
            "rmse_x":
                float(analyzer.results["rmse"][0]),
            "rmse_y":
                float(analyzer.results["rmse"][1]),
            "rmse_z":
                float(analyzer.results["rmse"][2]),
            "total_rmse":
                float(analyzer.results["total_rmse"]),
            "max_error_x": float(analyzer.results["max_error_xyz"][0]),
            "max_error_y": float(analyzer.results["max_error_xyz"][1]),
            "max_error_z": float(analyzer.results["max_error_xyz"][2]),
            "max_error_2norm": float(analyzer.results["max_error_2norm"]),
        }

        with open(
            out_dir / "metrics.json",
            "w"
        ) as f:
            json.dump(metrics, f, indent=2)

        self.save_3d_plot(
            analyzer,
            out_dir / "trajectory_3d.png"
        )

        self.save_time_series_plot(
            analyzer,
            out_dir / "time_series.png"
        )

    # --------------------------------------------------
    # save 3d plot
    # --------------------------------------------------

    def save_3d_plot(
        self,
        analyzer,
        output_png
    ):

        import matplotlib.pyplot as plt

        act = analyzer.results["actual"]
        ref = analyzer.results["reference"]

        fig = plt.figure()

        ax = fig.add_subplot(
            111,
            projection="3d"
        )

        ax.plot(
            ref[:, 0],
            ref[:, 1],
            ref[:, 2],
            label="Reference"
        )

        ax.plot(
            act[:, 0],
            act[:, 1],
            act[:, 2],
            label="Actual"
        )

        ax.legend()

        plt.savefig(
            output_png,
            bbox_inches="tight"
        )

        plt.close(fig)

    # --------------------------------------------------
    # save time series
    # --------------------------------------------------

    def save_time_series_plot(
        self,
        analyzer,
        output_png
    ):

        import matplotlib.pyplot as plt
        import numpy as np

        t = analyzer.results["t"]
        act = analyzer.results["actual"]
        ref = analyzer.results["reference"]

        fig, axs = plt.subplots(
            3,
            1,
            figsize=(10, 8),
            sharex=True
        )

        labels = ["x", "y", "z"]

        for i in range(3):

            axs[i].plot(
                t,
                ref[:, i],
                label="Reference"
            )

            axs[i].plot(
                t,
                act[:, i],
                label="Actual"
            )

            axs[i].set_ylabel(labels[i])

            axs[i].grid()

        axs[-1].set_xlabel("time [s]")

        plt.savefig(
            output_png,
            bbox_inches="tight"
        )

        plt.close(fig)

    # --------------------------------------------------
    # run all
    # --------------------------------------------------

    def run(self, end_trim_sec=0.0):

        refs = self.find_reference_csvs()

        print(
            f"Found {len(refs)} reference trajectories"
        )

        for ref_csv in refs:

            actual_csv = self.find_actual_csv(
                ref_csv
            )

            if actual_csv is None:

                print(
                    f"WARNING: no actual log found for "
                    f"{ref_csv}"
                )

                continue

            print()
            print("Reference:", ref_csv)
            print("Actual:", actual_csv)

            try:

                analyzer = self.analyze_pair(
                    ref_csv,
                    actual_csv,
                    end_trim_sec
                )

                self.save_results(
                    analyzer,
                    ref_csv
                )

            except Exception as ex:

                print(
                    f"FAILED: {ref_csv}"
                )

                print(ex)

# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":

    # reference = "/home/vishnuc/Simulation_Testing/Automated_Testing_of_Multirotor_in_Offboard_Mode/ws_aurora/src/ros2_tests/config/trajectory/trajectory.csv"
    # actual = "/home/vishnuc/Simulation_Testing/Automated_Testing_of_Multirotor_in_Offboard_Mode/ws_aurora/src/ros2_tests/log/2026-06-15T12-32-58Z/x500/Offboard_traj_track_mission_ros2::x500/gz_ground_truth.csv"
    #
    # analyzer = TrajectoryTrackingAnalyzer(
    #     reference_csv=reference,
    #     actual_csv=actual,
    #     event_name="track_traj"
    # )
    #
    # analyzer.run_all()

    batch = BatchTrajectoryTrackingAnalyzer(
        reference_root=
        "/home/vishnuc/Research/Codes/time_optimal_trajectory_quadrotor/results/quad_x500/scenario_set4",

        actual_root=
        "/home/vishnuc/Simulation_Testing/Automated_Testing_of_Multirotor_in_Offboard_Mode/ws_aurora/src/ros2_tests/log/x500"
    )

    batch.run(end_trim_sec=10.0)