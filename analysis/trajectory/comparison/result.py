from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
import numpy as np
from core.experiment import RunResult
from analysis.trajectory.comparison.metrics import SignalMetric, AttitudeMetric

@dataclass
class TrajectoryComparisonResult:

    #Trajectory results to be compared, given as full RunResult objects
    #run_result is compared against the reference trajectory result
    run_result: Optional[RunResult] = None
    reference_run_result: Optional[RunResult] = None

    # signal-wise metrics
    signals: Dict[str, SignalMetric] = field(default_factory=dict)

    # special metrics
    attitude: Optional[AttitudeMetric] = None

    # timing
    trajectory_duration: Optional[float] = None
    reference_trajectory_duration: Optional[float] = None
    duration_difference: Optional[float] = None

    # terminal state mismatch
    terminal_position_error: Optional[float] = None
    terminal_velocity_error: Optional[float] = None

    # normalized alignment grid
    normalized_time: Optional[np.ndarray] = None

    # scalar summary table
    summary: Dict[str, float] = field(default_factory=dict)

    # extra experiment data
    extra: Dict[str, Any] = field(default_factory=dict)

    skipped_metrics: List[str] = field(
        default_factory=list
    )

    @property
    def traj_label(self):
        if self.run_result is not None and self.run_result.quad_traj_bundle is not None:
            return self.run_result.quad_traj_bundle.label
        else:
            return None

    @property
    def reference_traj_label(self):
        if self.reference_run_result is not None and self.reference_run_result.quad_traj_bundle is not None:
            return self.reference_run_result.quad_traj_bundle.label
        else:
            return None

    @property
    def comparison_name(self):

        if (
                self.traj_label is not None
                and self.reference_traj_label is not None
        ):
            return (
                f"{self.traj_label} vs "
                f"{self.reference_traj_label}"
            )

        return "Unnamed Comparison"

    def __str__(self):

        lines = []

        if self.traj_label and self.reference_traj_label:

            lines.append(
                f"TrajectoryComparisonResult: {self.traj_label} vs {self.reference_traj_label}"
            )

        else:

            lines.append(
                "TrajectoryComparisonResult"
            )

        for sig, metric in self.signals.items():
            lines.append(
                f"{sig}: RMS={metric.rms:.4f}"
            )

        if self.attitude is not None:
            lines.append(
                f"attitude: RMS={self.attitude.rms_deg:.3f} deg"
            )

        if self.skipped_metrics:
            lines.append(
                "skipped="
                + ", ".join(self.skipped_metrics)
            )

        return "\n".join(lines)

    def print_summary(
            self,
            title: str = "Trajectory Comparison Summary"
    ):

        print("\n" + "=" * 60)
        print(title)
        print("=" * 60)

        if self.traj_label is not None:
            print(f"Trajectory 1 : {self.traj_label}")

        if self.reference_traj_label is not None:
            print(f"Trajectory 2 : {self.reference_traj_label}")

        if (
                self.traj_label is not None
                and self.reference_traj_label is not None
        ):
            print(
                f"Comparison   : "
                f"{self.traj_label} vs {self.reference_traj_label}"
            )

        # --------------------------------------------------
        # Signal metrics
        # --------------------------------------------------

        if self.signals:

            print("\nSignals")

            for sig, metric in self.signals.items():
                print(
                    f"{sig:10s}"
                    f" RMS={metric.rms:10.4f}"
                    f" MAX={metric.max:10.4f}"
                    f" MEAN={metric.mean:10.4f}"
                    f" FINAL={metric.final:10.4f}"
                )

        # --------------------------------------------------
        # Attitude
        # --------------------------------------------------

        if self.attitude is not None:
            print("\nAttitude [deg]")

            print(
                f"{'attitude':10s}"
                f" RMS={self.attitude.rms_deg:10.3f}"
                f" MAX={self.attitude.max_deg:10.3f}"
                f" MEAN={self.attitude.mean_deg:10.3f}"
                f" FINAL={self.attitude.final_deg:10.3f}"
            )

        # --------------------------------------------------
        # Terminal mismatch
        # --------------------------------------------------

        print("\nTerminal Errors")

        if self.terminal_position_error is not None:
            print(
                f"position: "
                f"{self.terminal_position_error:.4f}"
            )

        if self.terminal_velocity_error is not None:
            print(
                f"velocity: "
                f"{self.terminal_velocity_error:.4f}"
            )

        # --------------------------------------------------
        # Timing
        # --------------------------------------------------

        print("\nTiming")

        if self.trajectory_duration is not None:
            print(
                f"trajectory_duration: "
                f"{self.trajectory_duration:.4f} s"
            )

        if self.reference_trajectory_duration is not None:
            print(
                f"reference_trajectory_duration: "
                f"{self.reference_trajectory_duration:.4f} s"
            )

        if self.duration_difference is not None:
            print(
                f"duration_difference: "
                f"{self.duration_difference:.4f} s"
            )

        # --------------------------------------------------
        # Skipped metrics
        # --------------------------------------------------

        if self.skipped_metrics:

            print("\nSkipped Metrics")

            for item in self.skipped_metrics:
                print(f"  - {item}")

        print()
