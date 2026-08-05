import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from core.targets import PositionTarget, SphereTarget, StateTarget

from dataclasses import dataclass
from typing import List, Optional
from pathlib import Path
import datetime

from dataclasses import dataclass
from typing import List

import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from typing import Optional


from visualization import TrajectoryExtractor

# ============================================================
# Main plotter
# ============================================================

class MultiQuadTrajectoryPlotter:

    def __init__(
            self,
            bundles: List,
            target = None,
            output_dir=None,
            save_plots=False,
            show_plots=True,
            default_linewidth = 2.0
    ):

        self.bundles = bundles
        self.target = target

        self.save_plots = save_plots
        self.show_plots = show_plots

        self.output_dir = (
            Path(output_dir)
            if output_dir is not None
            else None
        )

        extractor = TrajectoryExtractor()

        self.data = [
            extractor.extract(bundle)
            for bundle in bundles
        ]

        self.plot_registry = {
            "position": self.plot_position,
            "velocity": self.plot_velocity,
            "flat_acceleration": self.plot_flat_acceleration,
            "flat_jerk": self.plot_flat_jerk,
            "attitude": self.plot_attitude,
            "body_rates": self.plot_body_rates,
            "yaw": self.plot_yaw,
            "thrust": self.plot_thrust,
            "trajectory_xy": self.plot_xy_trajectory,
            "trajectory_yz": self.plot_yz_trajectory,
            "trajectory_xz": self.plot_xz_trajectory,
            "trajectory_3d": self.plot_3d_trajectory,
            "summary": self.plot_summary
        }

        self.default_linewidth = default_linewidth

        self.VIEW_PRESETS = {
            "isometric": (30, -60),
            "top": (90, -90),
            "bottom": (-90, -90),
            "front": (0, -90),
            "rear": (0, 90),
            "left": (0, 180),
            "right": (0, 0),
        }

    @staticmethod
    def create_output_folder(base_dir="results"):

        timestamp = datetime.time().strftime(
            "%Y%m%d_%H%M%S"
        )

        output_dir = Path(base_dir) / f"comparison_{timestamp}"

        output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        return output_dir


    @staticmethod
    def _safe_hasattr(obj, attr):
        return obj is not None and hasattr(obj, attr)


    def _has_any_data(self, attribute_name):

        return any(
            getattr(traj, attribute_name) is not None
            for traj in self.data
        )

    # =========================================================
    # Generic dispatcher
    # =========================================================

    def plot(self, plot_config):

        """
        Example
        -------

        plotter.plot({

            "position": {},

            "attitude": {
                "degrees": True
            },

            "trajectory_3d": {
                "show_orientation": True
            }
        })
        """

        for item, kwargs in plot_config.items():

            if item not in self.plot_registry:

                raise ValueError(
                    f"Unknown plot type: {item}\n"
                    f"Available:\n"
                    f"{list(self.plot_registry.keys())}"
                )

            print(f"[Plotter] Plotting: {item}")

            self.plot_registry[item](**kwargs)

    # =========================================================
    # Plot all
    # =========================================================

    def plot_all(self):

        self.plot({
            # Planar trajectories
            "trajectory_xy": {},
            "trajectory_yz": {},
            "trajectory_xz": {},

            # Full 3D trajectory
            "trajectory_3d": {
                "show_orientation": False,
                "orientation_scale": 0.2,
                "step": 50,
                "view": "isometric",
                "plot_title": "3D Trajectory Comparison",
            },

            # State evolution
            "position": {},
            "velocity": {},

            # Flat-space signals (if available)
            "flat_acceleration": {},
            "flat_jerk": {},

            # Attitude + angular dynamics
            "attitude": {
                "degrees": True
            },

            "body_rates": {
                "degrees": True
            },

            # Control input
            "thrust": {},

            # Summary dashboard
            "summary": {}
        })


    # Plot Saver/Viewer helpers
    def _finalize_figure(
            self,
            fig,
            plot_name
    ):
        """
        Save/show/close figure.
        """

        if self.save_plots and self.output_dir:
            self.output_dir.mkdir(
                parents=True,
                exist_ok=True
            )

            plot_file = self.output_dir / f"{plot_name}.pdf"
            fig.savefig(
                plot_file,
                dpi=300,
                bbox_inches="tight"
            )
            print(f"Saved figure {plot_file}")

        if self.show_plots:
            plt.show()
        else:
            plt.close(fig)

    # =========================================================
    # Generic vector comparison helper
    # =========================================================

    def _set_1d_ylim(
            self,
            ax,
            pad=0.1,
            min_span=1.0,
    ):
        """
        Robust y-axis scaling for time series plots.

        Prevents:
        - collapsing near zero
        - overly tight or misleading limits
        """

        y_min, y_max = ax.get_ylim()

        y_range = y_max - y_min

        # prevent degenerate scaling
        y_range = max(y_range, min_span)

        y_pad = pad * y_range

        ax.set_ylim(
            y_min - y_pad,
            y_max + y_pad
        )

    def _set_axis_equal_2d(
            self,
            ax,
            x_data,
            y_data,
            pad=0.1,
            min_span=1.0,
    ):
        """
        Robust 2D axis scaling that:
        - avoids collapsing near zero
        - keeps aspect ratio consistent
        - adds padding
        """

        x_data = np.asarray(x_data)
        y_data = np.asarray(y_data)

        x_min, x_max = np.min(x_data), np.max(x_data)
        y_min, y_max = np.min(y_data), np.max(y_data)

        x_range = x_max - x_min
        y_range = y_max - y_min

        # prevent degenerate scaling
        x_range = max(x_range, min_span)
        y_range = max(y_range, min_span)

        x_pad = pad * x_range
        y_pad = pad * y_range

        ax.set_xlim(x_min - x_pad, x_max + x_pad)
        ax.set_ylim(y_min - y_pad, y_max + y_pad)

        ax.set_aspect('equal', adjustable='box')

    def _plot_styles(self, traj, linewidth=None):
        return dict(
            label=traj.label,
            color=traj.color,
            linestyle=traj.linestyle,
            linewidth=self.default_linewidth if linewidth is None else linewidth,
        )

    def _plot_vector_overlay(
            self,
            attribute_name,
            labels,
            title,
            ylabel_units=""
    ):

        fig, axs = plt.subplots(
            3, 1,
            figsize=(12, 8),
            sharex=True
        )


        for traj in self.data:

            data = getattr(traj, attribute_name)

            if data is None:
                continue

            for i in range(3):

                axs[i].plot(traj.t, data[:, i], **self._plot_styles(traj))

                axs[i].set_ylabel(
                    f"{labels[i]} {ylabel_units}"
                )

                axs[i].grid(True)

        #Set robust limit scaling
        for ax in axs:
            self._set_1d_ylim(
                ax,
                pad=0.1,
                min_span=1.0
            )

        axs[-1].set_xlabel("Time [s]")

        axs[0].legend()

        fig.suptitle(title)

        plt.tight_layout()
        self._finalize_figure(fig, attribute_name)

    def _plot_2d_projection_overlay(
            self,
            axis_i: int,
            axis_j: int,
            xlabel: str,
            ylabel: str,
            title: str
    ):
        """
        Generic 2D projection plotter.

        axis_i, axis_j:
            indices into position vector (0=x,1=y,2=z)
        """

        fig, ax = plt.subplots(figsize=(8, 8))

        all_x = []
        all_y = []
        for traj in self.data:
            x_arr = traj.pos[:, axis_i]
            y_arr = traj.pos[:, axis_j]

            all_x.append(x_arr)
            all_y.append(y_arr)

            plt.plot(traj.pos[:, axis_i], traj.pos[:, axis_j], **self._plot_styles(traj))

            # start
            plt.scatter(traj.pos[0, axis_i], traj.pos[0, axis_j], color=traj.color, marker='o', s=70)

            # end
            plt.scatter(traj.pos[-1, axis_i], traj.pos[-1, axis_j], color=traj.color, marker='x', s=80)

        all_x = np.concatenate(all_x)
        all_y = np.concatenate(all_y)

        self._set_axis_equal_2d(
            ax,
            all_x,
            all_y,
            pad=0.1,
            min_span=1.0
        )

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)

        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        self._finalize_figure(fig, title)

    def _set_axes_equal_3d(self, ax):
        """
        Set equal scaling for 3D axes.
        Matplotlib does not do this by default.
        """

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        y_range = abs(y_limits[1] - y_limits[0])
        z_range = abs(z_limits[1] - z_limits[0])

        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)

        plot_radius = 0.5 * max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def _resolve_view_angles(
            self,
            view="isometric",
            elev=None,
            azim=None,
    ):
        """
        Resolve camera angles.

        Priority:
            explicit elev/azim > preset
        """

        if view not in self.VIEW_PRESETS:
            raise ValueError(
                f"Unknown view preset '{view}'. "
                f"Available: {list(self.VIEW_PRESETS.keys())}"
            )

        preset_elev, preset_azim = self.VIEW_PRESETS[view]

        if elev is None:
            elev = preset_elev

        if azim is None:
            azim = preset_azim

        return elev, azim

    def _plot_target_3d(self, ax):

        if self.target is None:
            return
        target_color = 'black'
        # Point Target
        if isinstance(self.target, PositionTarget):

            p = self.target.desired_position

            ax.scatter(
                p[0], p[1], p[2],
                color=target_color,
                marker='*',
                s=250,
                label='Target Position'
            )

        # -------------------------------------------------
        # SphereTarget
        # -------------------------------------------------

        elif isinstance(self.target, SphereTarget):

            c = self.target.center
            r = self.target.radius

            u = np.linspace(0, 2 * np.pi, 40)
            v = np.linspace(0, np.pi, 20)

            x = c[0] + r * np.outer(np.cos(u), np.sin(v))
            y = c[1] + r * np.outer(np.sin(u), np.sin(v))
            z = c[2] + r * np.outer(np.ones_like(u), np.cos(v))

            ax.plot_surface(
                x, y, z,
                color=target_color,
                alpha=0.15,
                linewidth=0
            )

            ax.scatter(
                c[0], c[1], c[2],
                color=target_color,
                marker='*',
                s=200,
                label='Target Region Center'
            )

        # -------------------------------------------------
        # StateTarget
        # -------------------------------------------------

        elif isinstance(self.target, StateTarget):

            p = self.target.desired_state.pos

            ax.scatter(
                p[0], p[1], p[2],
                color=target_color,
                marker='*',
                s=250,
                label='Target State Position'
            )

    def _plot_3d_trajectory_overlay(
            self,
            show_orientation=False,
            orientation_scale=0.2,
            step=50,
            view="isometric",
            elev=None,
            azim=None,
            output_filename=None,
            plot_title="3D Trajectory Comparison",
    ):

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        elev, azim = self._resolve_view_angles(
            view=view,
            elev=elev,
            azim=azim
        )

        for traj in self.data:

            ax.plot(
                traj.pos[:, 0],
                traj.pos[:, 1],
                traj.pos[:, 2],
                **self._plot_styles(traj)
            )

            # start point
            ax.scatter(
                traj.pos[0, 0],
                traj.pos[0, 1],
                traj.pos[0, 2],

                color=traj.color,
                s=60
            )

            # end point
            ax.scatter(
                traj.pos[-1, 0],
                traj.pos[-1, 1],
                traj.pos[-1, 2],

                color=traj.color,
                s=60,
                marker='x'
            )

            # orientation frames
            if show_orientation and traj.R_hist is not None:

                for k in range(0, len(traj.pos), step):
                    p = traj.pos[k]
                    Rwb = traj.R_hist[k]

                    b1 = Rwb[:, 0]
                    b2 = Rwb[:, 1]
                    b3 = Rwb[:, 2]

                    ax.quiver(p[0], p[1], p[2],
                              b1[0], b1[1], b1[2],
                              color='r',
                              length=orientation_scale)

                    ax.quiver(p[0], p[1], p[2],
                              b2[0], b2[1], b2[2],
                              color='g',
                              length=orientation_scale)

                    ax.quiver(p[0], p[1], p[2],
                              b3[0], b3[1], b3[2],
                              color='b',
                              length=orientation_scale)

        # draw target
        self._plot_target_3d(ax)

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")

        ax.set_title(plot_title)

        ax.legend(loc="upper right", bbox_to_anchor=(0.92, 0.85))

        self._set_axes_equal_3d(ax)

        ax.view_init(
            elev=elev,
            azim=azim
        )

        plt.tight_layout()

        if output_filename is None:
            filename = f"trajectory_3d_{view}"

            # custom angles supplied
            if elev is not None or azim is not None:
                filename = (
                    f"trajectory_3d_"
                    f"e{int(round(elev))}_"
                    f"a{int(round(azim))}"
                )
        else:
            filename = output_filename

        self._finalize_figure(fig, filename)


    # =========================================================
    # Position
    # =========================================================

    def plot_position(self):

        if not self._has_any_data("pos"):
            print("[Plotter] No position data available.")
            return

        self._plot_vector_overlay(
            "pos",
            ['x', 'y', 'z'],
            'Position Comparison',
            '[m]'
        )

    def plot_xy_trajectory(self):

        if not self._has_any_data("pos"):
            print("[Plotter] No position data available.")
            return

        self._plot_2d_projection_overlay(
            axis_i=0,
            axis_j=1,
            xlabel="X [m]",
            ylabel="Y [m]",
            title="XY Trajectory Comparison"
        )

    def plot_yz_trajectory(self):

        if not self._has_any_data("pos"):
            print("[Plotter] No position data available.")
            return

        self._plot_2d_projection_overlay(
            axis_i=1,
            axis_j=2,
            xlabel="Y [m]",
            ylabel="Z [m]",
            title="YZ Trajectory Comparison"
        )

    def plot_xz_trajectory(self):

        if not self._has_any_data("pos"):
            print("[Plotter] No position data available.")
            return

        self._plot_2d_projection_overlay(
            axis_i=0,
            axis_j=2,
            xlabel="X [m]",
            ylabel="Z [m]",
            title="XZ Trajectory Comparison"
        )

    # =========================================================
    # Velocity
    # =========================================================

    def plot_velocity(self):

        if not self._has_any_data("vel"):
            print("[Plotter] No velocity data available.")
            return

        self._plot_vector_overlay(
            "vel",
            ['vx', 'vy', 'vz'],
            'Velocity Comparison',
            '[m/s]'
        )

    # =========================================================
    # Flat acceleration
    # =========================================================

    def plot_flat_acceleration(self):
        if not self._has_any_data("flat_acc"):
            print("[Plotter] No flat_acc data available.")
            return

        self._plot_vector_overlay(
            "flat_acc",
            ['ax', 'ay', 'az'],
            'Flat Acceleration Comparison',
            '[m/s²]'
        )

    # =========================================================
    # Flat jerk
    # =========================================================

    def plot_flat_jerk(self):

        if not self._has_any_data("flat_jerk"):
            print("[Plotter] No flat_jerk data available.")
            return

        self._plot_vector_overlay(
            "flat_jerk",
            ['jx', 'jy', 'jz'],
            'Flat Jerk Comparison',
            '[m/s³]'
        )

    # =========================================================
    # Attitude
    # =========================================================

    def plot_attitude(self, degrees=True):

        fig, axs = plt.subplots(
            3, 1,
            figsize=(12, 8),
            sharex=True
        )

        labels = ['roll', 'pitch', 'yaw']

        for traj in self.data:

            if traj.euler is None:
                continue

            data = traj.euler.copy()

            units = '[rad]'

            if degrees:

                data = np.rad2deg(data)

                units = '[deg]'

            for i in range(3):

                axs[i].plot(
                    traj.t,
                    data[:, i],
                    **self._plot_styles(traj)
                )

                axs[i].set_ylabel(
                    f"{labels[i]} {units}"
                )

                axs[i].grid(True)

        axs[-1].set_xlabel('Time [s]')

        axs[0].legend()

        fig.suptitle("Attitude Comparison")

        plt.tight_layout()
        self._finalize_figure(fig, "attitude")

    # =========================================================
    # Body rates
    # =========================================================

    def plot_body_rates(self, degrees=True):

        fig, axs = plt.subplots(
            3, 1,
            figsize=(12, 8),
            sharex=True
        )

        labels = [
            r'$\omega_x$',
            r'$\omega_y$',
            r'$\omega_z$'
        ]

        for traj in self.data:

            if traj.omega is None:
                continue

            data = traj.omega.copy()

            units = '[rad/s]'

            if degrees:

                data = np.rad2deg(data)

                units = '[deg/s]'

            for i in range(3):

                axs[i].plot(
                    traj.t,
                    data[:, i],
                    **self._plot_styles(traj)
                )

                axs[i].set_ylabel(
                    f"{labels[i]} {units}"
                )

                axs[i].grid(True)

        axs[-1].set_xlabel('Time [s]')

        axs[0].legend()

        fig.suptitle("Body Rate Comparison")

        plt.tight_layout()
        self._finalize_figure(fig, "body_rates")

    # =========================================================
    # Yaw
    # =========================================================

    def plot_yaw(self, degrees=True):

        fig, axs = plt.subplots(
            2, 1,
            figsize=(12, 6),
            sharex=True
        )

        for traj in self.data:

            if traj.yaw is None:
                continue

            yaw = traj.yaw.copy()
            yaw_rate = traj.yaw_rate.copy()

            if degrees:

                yaw = np.rad2deg(yaw)
                yaw_rate = np.rad2deg(yaw_rate)

                yaw_unit = '[deg]'
                rate_unit = '[deg/s]'

            else:

                yaw_unit = '[rad]'
                rate_unit = '[rad/s]'

            axs[0].plot(
                traj.t,
                yaw,
                **self._plot_styles(traj)
            )

            axs[1].plot(
                traj.t,
                yaw_rate,
                **self._plot_styles(traj)
            )

            axs[0].set_ylabel(
                f'Yaw {yaw_unit}'
            )

            axs[1].set_ylabel(
                f'Yaw Rate {rate_unit}'
            )

            axs[0].grid(True)
            axs[1].grid(True)

        axs[1].set_xlabel('Time [s]')

        axs[0].legend()

        fig.suptitle("Yaw Comparison")

        plt.tight_layout()
        self._finalize_figure(fig, "yaw")

    # =========================================================
    # Thrust
    # =========================================================

    def plot_thrust(self):

        fig = plt.figure(figsize=(12, 4))

        for traj in self.data:

            if traj.thrust is None:
                continue

            plt.plot(
                traj.t,
                traj.thrust,
                **self._plot_styles(traj)
            )

        plt.xlabel('Time [s]')

        plt.ylabel('Thrust [N]')

        plt.title('Collective Thrust Comparison')

        plt.grid(True)

        plt.legend()

        plt.tight_layout()

        self._finalize_figure(fig, "thrust")

    # =========================================================
    # 3D trajectory
    # =========================================================

    def plot_3d_trajectory(
            self,
            show_orientation=False,
            orientation_scale=0.2,
            step=50,
            view="isometric",
            elev=None,
            azim=None,
            output_filename=None,
            plot_title="3D Trajectory Comparison",
    ):

        self._plot_3d_trajectory_overlay(show_orientation, orientation_scale, step, view=view, elev=elev, azim=azim, output_filename=output_filename, plot_title=plot_title)

    # =========================================================
    # Summary
    # =========================================================

    def plot_summary(self):

        fig, axs = plt.subplots(
            4, 1,
            figsize=(12, 12),
            sharex=True
        )

        for traj in self.data:

            # position norm
            pos_norm = np.linalg.norm(
                traj.pos,
                axis=1
            )

            axs[0].plot(
                traj.t,
                pos_norm,
                **self._plot_styles(traj)
            )

            # velocity norm
            vel_norm = np.linalg.norm(
                traj.vel,
                axis=1
            )

            axs[1].plot(
                traj.t,
                vel_norm,
                **self._plot_styles(traj)
            )

            # omega norm
            omega_norm = np.linalg.norm(
                traj.omega,
                axis=1
            )

            axs[2].plot(
                traj.t,
                np.rad2deg(omega_norm),
                **self._plot_styles(traj)
            )

            # thrust
            axs[3].plot(
                traj.t,
                traj.thrust,
                **self._plot_styles(traj)
            )

        axs[0].set_ylabel('||p|| [m]')
        axs[1].set_ylabel('||v|| [m/s]')
        axs[2].set_ylabel('||ω|| [deg/s]')
        axs[3].set_ylabel('Thrust [N]')

        for ax in axs:
            ax.grid(True)

        axs[-1].set_xlabel('Time [s]')

        axs[0].legend()

        fig.suptitle(
            "Trajectory Summary Comparison"
        )

        plt.tight_layout()

        self._finalize_figure(fig, "summary")