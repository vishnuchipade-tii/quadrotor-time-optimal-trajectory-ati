
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation
from core.targets import PositionTarget, SphereTarget, StateTarget
from core.simulation import SimMetadata, SimulationResult
from typing import List
from pathlib import Path
import datetime

class TrajectoryAnimator:
    def __init__(self, results: List[SimulationResult], sim_metadata: SimMetadata, target: SphereTarget):
        self.results = results
        self.dt = sim_metadata.dt
        self.target = target

        # Pre-extract positions for all results
        self.pos_data = [np.array([s.pos for s in res.state_history]) for res in self.results]
        # Set animation length to the longest history
        self.max_frames = max([len(h) for h in self.pos_data]) if self.pos_data else 0

    def animate(self):
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        self._draw_hemisphere(ax)

        lines = []
        # Default color cycle if colors aren't specified in SimulationResult
        default_colors = plt.cm.get_cmap('tab10').colors

        for i, res in enumerate(self.results):
            color = res.color if res.color else default_colors[i % 10]

            ln, = ax.plot([], [], [],
                          lw=2,
                          color=color,
                          ls=res.line_style,
                          label=res.label)
            lines.append(ln)

            # Plot arrows and markers using the packaged history
            self._plot_metadata(ax, res.state_history, color)

        self._set_axes_equal(ax)
        ax.legend(loc='upper right')

        def update(num):
            for i, ln in enumerate(lines):
                # Clamp index to the end of each specific history
                idx = min(num, len(self.pos_data[i]) - 1)
                ln.set_data(self.pos_data[i][:idx, 0], self.pos_data[i][:idx, 1])
                ln.set_3d_properties(self.pos_data[i][:idx, 2])
            return lines

        ani = FuncAnimation(fig, update, frames=self.max_frames, interval=20, blit=True)
        plt.show()

    def _plot_metadata(self, ax, history, color):
        # Initial State
        h = history
        p0 = h[0].pos
        v0 = h[0].vel

        # Start point marker
        ax.scatter(p0[0], p0[1], p0[2], color=color, s=40, marker='o', edgecolors='black')

        # Velocity vector
        v_norm = np.linalg.norm(v0)
        if v_norm > 0.1:
            ax.quiver(p0[0], p0[1], p0[2], v0[0], v0[1], v0[2],
                      length=max(1.0, v_norm * 0.1), color=color, alpha=0.6, pivot='tail', arrow_length_ratio=0.3)

        # Final terminal point
        pf = h[-1].pos
        ax.scatter(pf[0], pf[1], pf[2], color=color, s=100, marker='X', edgecolors='white', linewidth=1)

    def _draw_hemisphere(self, ax):
        u, v = np.linspace(0, 2 * np.pi, 30), np.linspace(0, np.pi / 2, 15)
        x = self.target.radius * np.outer(np.cos(u), np.sin(v)) + self.target.center[0]
        y = self.target.radius * np.outer(np.sin(u), np.sin(v)) + self.target.center[1]
        z = self.target.radius * np.outer(np.ones(np.size(u)), np.cos(v)) + self.target.center[2]
        # Plot target center
        ax.scatter(self.target.center[0], self.target.center[1], self.target.center[2],
                   color='gray', s=100, label='Target Area Center', marker='*')
        #plot the surface
        ax.plot_surface(x, y, z, color='gray', alpha=0.1, linewidth=0)
        ax.plot_wireframe(x, y, z, color='gray', alpha=0.2, linewidth=0.5)

    def _set_axes_equal(self, ax):
        # Combine all history points to find global limits
        all_pts = np.vstack(self.pos_data) if self.pos_data else np.array([[0, 0, 0]])
        x_lim = [all_pts[:, 0].min(), all_pts[:, 0].max()]
        y_lim = [all_pts[:, 1].min(), all_pts[:, 1].max()]
        z_lim = [0, all_pts[:, 2].max()]

        x_range, y_range = abs(x_lim[1] - x_lim[0]), abs(y_lim[1] - y_lim[0])
        plot_radius = max([x_range, y_range, z_lim[1]]) / 2.0

        x_mid, y_mid = np.mean(x_lim), np.mean(y_lim)
        ax.set_xlim3d([x_mid - plot_radius, x_mid + plot_radius])
        ax.set_ylim3d([y_mid - plot_radius, y_mid + plot_radius])
        ax.set_zlim3d([0, plot_radius * 2.0])