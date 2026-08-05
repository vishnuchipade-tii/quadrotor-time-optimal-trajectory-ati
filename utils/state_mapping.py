import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

from core.quadrotor import Quadrotor
from core.state import State3D, DIState3D, FlatState, QuadState
from core.state import QuadControl
from core.simulation import SimulationResult
from abc import ABC, abstractmethod
from core.trajectory import QuadTrajectoryPoint, QuadTrajectoryBundle

# ============================================================
# Triple integrator history to Flat state mapper
# ============================================================
# ============================================================
# Helper: yaw extraction (simple velocity-based)
# ============================================================

def compute_yaw_from_velocity(v: np.ndarray, eps=1e-6):
    vx, vy = v[0], v[1]
    if np.linalg.norm(v[:2]) < eps:
        return 0.0
    return np.arctan2(vy, vx)


def compute_yaw_rate(yaw_prev, yaw_curr, dt):
    # simple finite difference (robust enough for logging)
    return (yaw_curr - yaw_prev) / dt

# ============================================================
# MAIN CONVERTER
# ============================================================

def triple_integrator_history_to_flat(
        state_history: List['State3D'],
        control_history: List['ControlInput3D'],
        dt: float,
        cd: float = 0.0,
        ca: float = 0.0
    ) -> List[FlatState]:

    N = len(state_history)

    flat_history: List[FlatState] = []

    yaw_prev = 0.0

    for k in range(N):

        state = state_history[k]
        control = control_history[k]

        p = state.pos
        v = state.vel

        # IMPORTANT:
        # acc is NOT true acceleration, but best proxy available
        acc = state.acc

        # jerk is directly your control input
        u = control.value

        # =====================================================
        # physical acceleration (from your model)
        # =====================================================
        a_phys = acc - cd * v

        # =====================================================
        # physical jerk
        # =====================================================
        j_phys = u - (cd + ca) * acc + (cd ** 2) * v

        # yaw from velocity direction
        yaw = compute_yaw_from_velocity(v)

        if k == 0:
            yaw_rate = 0.0
        else:
            yaw_rate = compute_yaw_rate(yaw_prev, yaw, dt)

        yaw_prev = yaw

        flat = FlatState(
            p=p.copy(),
            v=v.copy(),
            a=a_phys,
            j=j_phys,
            yaw=yaw,
            yaw_rate=yaw_rate
        )

        flat_history.append(flat)

    return flat_history


from typing import List
import numpy as np

def double_integrator_history_to_flat(
        state_history: List['DIState3D'],
        control_history: List['ControlInput3D'],
        dt: float,
        cd: float = 0.0,
    ):

    N = len(state_history)

    flat_history = []

    yaw_prev = 0.0
    u_prev = np.zeros(3)

    for k in range(N):

        state = state_history[k]
        control = control_history[k]

        p = state.pos
        v = state.vel

        # =====================================================
        # physical acceleration = control input
        # =====================================================
        a_phys = control.value

        # =====================================================
        # numerical jerk (only meaningful approx)
        # =====================================================
        if k == 0:
            j_phys = np.zeros(3)
        else:
            j_phys = (control.value - u_prev) / max(dt, 1e-6)

        u_prev = control.value.copy()

        # =====================================================
        # yaw from velocity
        # =====================================================
        yaw = compute_yaw_from_velocity(v)

        if k == 0:
            yaw_rate = 0.0
        else:
            yaw_rate = compute_yaw_rate(yaw_prev, yaw, dt)

        yaw_prev = yaw

        flat = FlatState(
            p=p.copy(),
            v=v.copy(),
            a=a_phys,
            j=j_phys,
            yaw=yaw,
            yaw_rate=yaw_rate
        )

        flat_history.append(flat)

    return flat_history

#Converters

class TrajectoryBundleConverter(ABC):

    @abstractmethod
    def convert(
        self,
        sim_result: SimulationResult
    ) -> QuadTrajectoryBundle:
        pass

class DIToQuadTrajBundleConverter(
    TrajectoryBundleConverter
):
    def __init__(self, dynamics_model):
        self.dynamics_model = dynamics_model

    def convert(
        self,
        sim_result: SimulationResult
    ) -> QuadTrajectoryBundle:

        trajectory = []

        N = len(sim_result.state_history)

        for k in range(N):
            state = sim_result.state_history[k]
            control = sim_result.control_history[k]

            # ==========================================
            # Quad state (partial)
            # ==========================================

            quad_state = QuadState(
                p=state.pos.copy(),
                v=state.vel.copy(),

                # unavailable for DI
                R=None,
                q=None,
                omega=None
            )



            # ==========================================
            # Flat representation
            # ==========================================
            x_dot = self.dynamics_model.rhs(state, control)
            v_dot = x_dot[3:6]

            a_phys = np.asarray(v_dot).copy()

            flat_state = FlatState(
                p=state.pos.copy(),
                v=state.vel.copy(),

                # acceleration state
                a=a_phys,

                # jerk input
                j=None,

                yaw=None,
                yaw_rate=None
            )

            # ==========================================
            # Control
            # ==========================================

            quad_control = QuadControl(
                thrust=None,

                # useful generic control signal
                accel=control.copy()
            )

            point = QuadTrajectoryPoint(
                flat=flat_state,
                state=quad_state,
                control=quad_control
            )

            trajectory.append(point)

        return QuadTrajectoryBundle(
            time=sim_result.time_history,
            trajectory=trajectory,

            label=sim_result.label,
            color=sim_result.color or "blue",
            linestyle=sim_result.line_style
        )


# ============================================================
# Flatness to full quadrotor state mapper
# ============================================================

class FlatToQuadMapper:

    def __init__(self, mass=1.0, gravity=9.81):
        self.m = mass
        self.g = gravity
        self.e3 = np.array([0.0, 0.0, 1.0])

    # --------------------------------------------------------
    def map(self, flat: FlatState) -> QuadTrajectoryPoint:

        p, v, a, j = flat.p, flat.v, flat.a, flat.j
        yaw, yaw_rate = flat.yaw, flat.yaw_rate

        # =====================================================
        # 1. Desired force
        # =====================================================

        f_des = self.m * (a + self.g * self.e3)
        thrust = np.linalg.norm(f_des)

        if thrust < 1e-8:
            b3 = self.e3.copy()
        else:
            b3 = f_des / thrust

        # =====================================================
        # 2. Desired heading direction
        # =====================================================

        b1_c = np.array([
            np.cos(yaw),
            np.sin(yaw),
            0.0
        ])

        # avoid singularity if aligned
        b2 = np.cross(b3, b1_c)
        b2_norm = np.linalg.norm(b2)

        if b2_norm < 1e-8:
            b1_c = np.array([1.0, 0.0, 0.0])
            b2 = np.cross(b3, b1_c)
            b2 = b2 / np.linalg.norm(b2)
        else:
            b2 = b2 / b2_norm

        b1 = np.cross(b2, b3)

        Rwb = np.column_stack((b1, b2, b3))

        quat = R.from_matrix(Rwb).as_quat()  # xyzw

        # =====================================================
        # 3. Body rates from jerk
        # =====================================================

        proj = np.dot(b3, j)

        if thrust < 1e-8:
            omega = np.zeros(3)
        else:
            h_omega = (self.m / thrust) * (j - proj * b3)

            omega_x = -np.dot(b2, h_omega)
            omega_y = np.dot(b1, h_omega)
            omega_z = yaw_rate * np.dot(self.e3, b3)

            omega = np.array([omega_x, omega_y, omega_z])

        # =====================================================
        # 4. Build state
        # =====================================================

        state = QuadState(
            p=p,
            v=v,
            R=Rwb,
            q=quat,
            omega=omega
        )

        control = QuadControl(thrust=thrust)

        return QuadTrajectoryPoint(
            flat=flat,
            state=state,
            control=control
        )

    # --------------------------------------------------------
    def map_trajectory(self, flats):
        return [self.map(f) for f in flats]


class TIToQuadTrajBundleConverter(
    TrajectoryBundleConverter
):

    def __init__(
        self,
        mapper: FlatToQuadMapper,
        dynamics_model
    ):
        self.mapper = mapper
        self.dynamics_model = dynamics_model

    def convert(
        self,
        sim_result: SimulationResult
    ) -> QuadTrajectoryBundle:

        trajectory = []

        for state, control in zip(
            sim_result.state_history,
            sim_result.control_history
        ):
            x_dot = self.dynamics_model.rhs(state, control)
            v_dot = x_dot[3:6]

            a_phys = np.asarray(v_dot).copy()

            flat = FlatState(
                p=state.pos,
                v=state.vel,
                a=a_phys,
                j=control.value,
                yaw=0.0,
                yaw_rate=0.0
            )

            trajectory.append(
                self.mapper.map(flat)
            )

        return QuadTrajectoryBundle(
            time=sim_result.time_history,
            trajectory=trajectory,

            label=sim_result.label,
            color=sim_result.color or "red",
            linestyle=sim_result.line_style
        )


class QuadTrajBundleFactory:

    @staticmethod
    def from_simulation_result(
        sim_result: SimulationResult,
        quadrotor: Quadrotor,
    ) -> QuadTrajectoryBundle:

        first_state = (
            sim_result.state_history[0]
        )

        if isinstance(
            first_state,
            DIState3D
        ):
            return (
                DIToQuadTrajBundleConverter(dynamics_model=quadrotor.dynamics_model)
                .convert(sim_result)
            )

        if isinstance(
            first_state,
            State3D
        ):
            return (
                TIToQuadTrajBundleConverter(
                    FlatToQuadMapper(), dynamics_model=quadrotor.dynamics_model
                ).convert(sim_result)
            )

        raise TypeError(
            f"Unsupported state type "
            f"{type(first_state)}"
        )
# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":

    mapper = FlatToQuadMapper(mass=1.0)

    dt = 0.01
    N = 500

    traj = []

    for k in range(N):

        t = k * dt

        # circular trajectory
        r = 2.0
        w = 0.6

        p = np.array([
            r * np.cos(w * t),
            r * np.sin(w * t),
            1.0
        ])

        v = np.array([
            -r * w * np.sin(w * t),
            r * w * np.cos(w * t),
            0.0
        ])

        a = np.array([
            -r * w**2 * np.cos(w * t),
            -r * w**2 * np.sin(w * t),
            0.0
        ])

        j = np.array([
            r * w**3 * np.sin(w * t),
            -r * w**3 * np.cos(w * t),
            0.0
        ])

        yaw = np.arctan2(v[1], v[0])
        yaw_rate = w

        flat = FlatState(
            p=p,
            v=v,
            a=a,
            j=j,
            yaw=yaw,
            yaw_rate=yaw_rate
        )

        traj.append(flat)

    quad_traj = mapper.map_trajectory(traj)

    # ========================================================
    # Example access
    # ========================================================

    print("Position:", quad_traj[0].state.p)
    print("Quaternion:", quad_traj[0].state.q)
    print("Body rates:", quad_traj[0].state.omega)
    print("Thrust:", quad_traj[0].control.thrust)