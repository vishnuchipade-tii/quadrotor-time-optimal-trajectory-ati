import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import interp1d

from analysis.trajectory.comparison.metrics import SignalMetric, AttitudeMetric
from analysis.trajectory.comparison.result import TrajectoryComparisonResult


# ============================================================
# Metric registries
# ============================================================

VECTOR_SIGNALS = {
    "p",
    "v",
    "a",
    "j",
    "omega",
}

SPECIAL_METRICS = {
    "attitude",
}

# ============================================================
# Comparator
# ============================================================

class TrajectoryComparator:

    # ========================================================
    # Constructor
    # ========================================================

    def __init__(
        self,
        run_result,
        reference_run_result,

        signals=(
            "p",
            "v",
            "a",
            "j",
            "omega",
        ),

        special_metrics=(
            "attitude",
        ),

        N=1000,
    ):

        self.run_result = run_result
        self.reference_run_result = reference_run_result

        self.bundle = run_result.quad_traj_bundle
        self.reference_bundle = reference_run_result.quad_traj_bundle

        self.signals = signals

        self.special_metrics = special_metrics

        self.N = N

        self.result = None

    # ========================================================
    # Utilities
    # ========================================================

    def _rotation_angle(
        self,
        R: np.ndarray,
        R_ref: np.ndarray,
    ) -> float:
        """
        Geodesic SO(3) distance in radians
        """

        R_rel = R.T @ R_ref

        cos_theta = (np.trace(R_rel) - 1.0) / 2.0

        cos_theta = np.clip(
            cos_theta,
            -1.0,
            1.0
        )

        return np.arccos(cos_theta)

    def _extract_signal(
        self,
        bundle,
        field: str
    ):

        vals = []

        for pt in bundle.trajectory:

            if field == "p":
                v = pt.state.p

            elif field == "v":
                v = pt.state.v

            elif field == "a":
                v = pt.flat.a

            elif field == "j":
                v = pt.flat.j

            elif field == "omega":
                v = pt.state.omega

            elif field == "R":
                v = pt.state.R

            elif field == "q":
                v = pt.state.q

            else:
                raise ValueError(
                    f"Unknown field: {field}"
                )

            if v is None:
                return None

            vals.append(v)

        return np.asarray(vals)

    def _signal_available(
            self,
            bundle,
            field
    ):
        return (
                self._extract_signal(
                    bundle,
                    field
                ) is not None
        )

    def _resample_signal(
        self,
        time_old,
        signal_old,
        s_new
    ):

        t_old = np.array(time_old)

        T = t_old[-1]

        s_old = t_old / T

        signal_old = np.asarray(signal_old)

        # ----------------------------------------------------
        # Vector signal
        # ----------------------------------------------------

        if signal_old.ndim == 2:

            out = np.zeros(
                (len(s_new), signal_old.shape[1])
            )

            for d in range(signal_old.shape[1]):
                f = interp1d(
                    s_old,
                    signal_old[:, d],
                    kind="linear",
                    fill_value="extrapolate")

                out[:, d] = f(s_new)

            return out

        # ----------------------------------------------------
        # Scalar signal
        # ----------------------------------------------------

        elif signal_old.ndim == 1:

            f = interp1d(
                s_old,
                signal_old,
                kind="linear",
                fill_value="extrapolate"
            )

            return f(s_new)

        else:

            raise ValueError(
                "Unsupported signal dimension"
            )

    # ========================================================
    # Metric computations
    # ========================================================

    def _compute_vector_metrics(
        self,
        x,
        x_ref
    ) -> SignalMetric:

        err = np.linalg.norm(
            x - x_ref,
            axis=1
        )

        return SignalMetric(

            rms=np.sqrt(np.mean(err**2)),
            max=np.max(err),
            mean=np.mean(err),
            final=err[-1],

            history=err
        )

    def _compute_attitude_metrics(
        self,
        R_seq,
        R_ref_seq
    ) -> AttitudeMetric:

        errors = np.zeros(len(R_seq))

        for i in range(len(R_seq)):

            errors[i] = self._rotation_angle(
                R_seq[i],
                R_ref_seq[i]
            )

        errors_deg = np.rad2deg(errors)

        return AttitudeMetric(

            rms_deg=np.sqrt(
                np.mean(errors_deg**2)
            ),

            max_deg=np.max(errors_deg),

            mean_deg=np.mean(errors_deg),

            final_deg=errors_deg[-1],

            history_deg=errors_deg
        )

    # ========================================================
    # Main computation
    # ========================================================

    def compute(
        self,
        signals=None,

        special_metrics=None,

        N=None,
    ):
        if signals is None:
            signals = self.signals

        if special_metrics is None:
            special_metrics = self.special_metrics

        if N is None:
            N = self.N

        result = TrajectoryComparisonResult(
            run_result=self.run_result,
            reference_run_result=self.reference_run_result,
        )

        # ----------------------------------------------------
        # Normalized time
        # ----------------------------------------------------

        s = np.linspace(0.0, 1.0, N)

        result.normalized_time = s

        # ----------------------------------------------------
        # Vector metrics
        # ----------------------------------------------------

        for sig in signals:

            if sig not in VECTOR_SIGNALS:
                continue

            x1 = self._extract_signal(
                self.bundle,
                sig
            )

            x2 = self._extract_signal(
                self.reference_bundle,
                sig
            )

            if x1 is None or x2 is None:
                result.skipped_metrics.append(sig)
                continue

            x1r = self._resample_signal(
                self.bundle.time,
                x1,
                s
            )

            x2r = self._resample_signal(
                self.reference_bundle.time,
                x2,
                s
            )

            result.signals[sig] = (
                self._compute_vector_metrics(
                    x1r,
                    x2r
                )
            )

        # ----------------------------------------------------
        # Attitude metric
        # ----------------------------------------------------

        if "attitude" in special_metrics:

            R1 = self._extract_signal(
                self.bundle,
                "R"
            )

            R2 = self._extract_signal(
                self.reference_bundle,
                "R"
            )

            if R1 is not None and R2 is not None:
                idx1 = np.linspace(
                    0,
                    len(R1) - 1,
                    N
                ).astype(int)

                idx2 = np.linspace(
                    0,
                    len(R2) - 1,
                    N
                ).astype(int)

                result.attitude = (
                    self._compute_attitude_metrics(
                        R1[idx1],
                        R2[idx2]
                    )
                )
            else:
                result.skipped_metrics.append("attitude")
        # ----------------------------------------------------
        # Duration
        # ----------------------------------------------------

        T1 = self.bundle.time[-1]
        T2 = self.reference_bundle.time[-1]

        result.trajectory_duration = T1
        result.reference_trajectory_duration = T2

        result.duration_difference = abs(
            T1 - T2
        )

        # ----------------------------------------------------
        # Terminal state errors
        # ----------------------------------------------------

        pT1 = self.bundle.trajectory[-1].state.p
        pT2 = self.reference_bundle.trajectory[-1].state.p

        vT1 = self.bundle.trajectory[-1].state.v
        vT2 = self.reference_bundle.trajectory[-1].state.v

        result.terminal_position_error = np.linalg.norm(pT1 - pT2)

        result.terminal_velocity_error = np.linalg.norm(vT1 - vT2)

        # ----------------------------------------------------
        # Summary table
        # ----------------------------------------------------

        for sig, metric in result.signals.items():

            result.summary[
                f"{sig}_rms"
            ] = metric.rms

            result.summary[
                f"{sig}_max"
            ] = metric.max

        if result.attitude is not None:

            result.summary[
                "attitude_rms_deg"
            ] = result.attitude.rms_deg

            result.summary[
                "attitude_max_deg"
            ] = result.attitude.max_deg

        result.summary[
            "duration_difference"
        ] = result.duration_difference

        # ----------------------------------------------------
        # Store
        # ----------------------------------------------------

        self.result = result

        return result

    # ========================================================
    # Summary printing
    # ========================================================

    def summary(self):

        if self.result is None:

            raise RuntimeError(
                "Run compute() first"
            )

        r = self.result

        print("\n===================================")
        print("Trajectory Comparison Summary")
        print("===================================\n")

        for sig, metric in r.signals.items():

            print(
                f"{sig:10s}"
                f" RMS={metric.rms:.4f}"
                f" MAX={metric.max:.4f}"
                f" FINAL={metric.final:.4f}"
            )

        if r.attitude is not None:

            print(
                f"\nAttitude [deg]"
                f" RMS={r.attitude.rms_deg:.3f}"
                f" MAX={r.attitude.max_deg:.3f}"
                f" FINAL={r.attitude.final_deg:.3f}"
            )

        print(
            f"\nDuration difference:"
            f" {r.duration_difference:.4f} s"
        )

    # ========================================================
    # Plot signal error histories
    # ========================================================

    def plot_signal_errors(self):

        if self.result is None:

            raise RuntimeError(
                "Run compute() first"
            )

        r = self.result

        n = len(r.signals)

        if n == 0:
            return

        fig, axs = plt.subplots(
            n,
            1,
            figsize=(10, 3*n),
            sharex=True
        )

        if n == 1:
            axs = [axs]

        for ax, (sig, metric) in zip(
            axs,
            r.signals.items()
        ):

            ax.plot(
                r.normalized_time,
                metric.history,
                linewidth=2
            )

            ax.set_ylabel(sig)

            ax.set_title(
                f"{sig}"
                f" RMS={metric.rms:.4f}"
            )

            ax.grid(True)

        axs[-1].set_xlabel(
            "Normalized Time"
        )

        plt.tight_layout()
        plt.show()

    # ========================================================
    # Plot attitude error
    # ========================================================

    def plot_attitude_error(self):

        if self.result is None:

            raise RuntimeError(
                "Run compute() first"
            )

        if self.result.attitude is None:
            return

        r = self.result

        plt.figure(figsize=(10, 4))

        plt.plot(
            r.normalized_time,
            r.attitude.history_deg,
            linewidth=2
        )

        plt.xlabel("Normalized Time")

        plt.ylabel("Attitude Error [deg]")

        plt.title(
            f"Attitude Error"
            f" RMS={r.attitude.rms_deg:.3f} deg"
        )

        plt.grid(True)

        plt.tight_layout()
        plt.show()

    # ========================================================
    # Plot scalar metric bars
    # ========================================================

    def plot_metric_bars(self):

        if self.result is None:

            raise RuntimeError(
                "Run compute() first"
            )

        r = self.result

        labels = []
        values = []

        for sig, metric in r.signals.items():

            labels.append(sig)
            values.append(metric.rms)

        if r.attitude is not None:

            labels.append("attitude_deg")
            values.append(
                r.attitude.rms_deg
            )

        plt.figure(figsize=(8, 4))

        plt.bar(
            labels,
            values
        )

        plt.ylabel("RMS Error")

        plt.title(
            "Trajectory Comparison Metrics"
        )

        plt.grid(True, axis='y')

        plt.tight_layout()
        plt.show()

    # ========================================================
    # Plot all
    # ========================================================

    def plot_all(self):

        self.plot_signal_errors()

        self.plot_attitude_error()

        self.plot_metric_bars()