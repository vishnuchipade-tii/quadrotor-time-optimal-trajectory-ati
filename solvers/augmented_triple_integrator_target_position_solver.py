import numpy as np

from core.targets import SphereTarget
from core.constraints import PositionConstraint, SphereConstraint
from core.planning import PlanningRequest
from core.planning import PlanningResult
from core.state import State3D

#SolverRegistry related imports
from core.registry import SolverRegistry
from core.keys import SolverKey
from core.type import DynamicsType
from core.type import TargetType
from core.type import SolverType

# ===========================================================================
# AUGMENTED TRIPLE INTEGRATOR UNIFIED SOLVER FOR POSITION AND SPHERE TARGETS
# ===========================================================================
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_TRIPLE_INTEGRATOR,
        TargetType.POSITION,
        SolverType.ROOT_FINDING,
    )
)
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_TRIPLE_INTEGRATOR,
        TargetType.SPHERE,
        SolverType.ROOT_FINDING,
    )
)
class AugmentedTripleIntegratorTargetPositionSolver:

    def __init__(self):

        self.solver_id = "AugmentedDoubleIntegratorTargetPositionSolver"

        # Dynamics
        self.ca = None
        self.cd = None
        self.gravity = None

        # Input bounds
        self.j_max = None

        # Optional solver parameters
        self.max_iter = 3000
        self.tol = 1e-5

    def configure(self, planning_request: PlanningRequest):

        dynamics = planning_request.dynamics
        bounds = planning_request.input_bounds

        self.ca = dynamics.ca
        self.cd = dynamics.cd
        self.gravity = dynamics.gravity

        self.j_max = self.a_max = bounds.get("j_max")

        solver_config = planning_request.solver_config

        solver_options = solver_config.options

        if solver_options is not None:
            self.max_iter = solver_options.get(
                "max_iter",
                self.max_iter
            )

            self.tol = solver_options.get(
                "tol",
                self.tol
            )

    # ========================================================
    # BASIS FUNCTIONS
    # ========================================================

    def get_basis(self, t):

        cd = self.cd
        ca = self.ca

        if t < 1e-6:

            return {
                "f1": 0.0,
                "f2": 0.0,
                "f3": 0.0,
                "fg": 0.0,
                "f1d": 1.0,
                "f2d": 0.0,
                "f3d": 0.0,
                "fgd": 0.0
            }

        ea = np.exp(-cd * t)

        eb = np.exp(-ca * t)

        # ----------------------------------------------------
        # Basis
        # ----------------------------------------------------

        f1 = (1.0 - ea) / cd

        f2 = (
                (1.0 / (ca - cd))
                * (
                        f1
                        - (1.0 - eb) / ca
                )
        )

        f3 = (
                (1.0 / ca)
                * (
                        t / cd
                        - (1.0 - ea) / (cd ** 2)
                        - f2
                )
        )

        # ----------------------------------------------------
        # Gravity drift basis
        # ----------------------------------------------------

        fg = (cd * t - (1.0 - ea) ) / (cd ** 2)

        # ----------------------------------------------------
        # Derivatives
        # ----------------------------------------------------

        f1d = ea

        f2d = (ea - eb) / (ca - cd)

        f3d = (1.0 / ca) * (f1 - f2d)

        fgd = (1.0 - ea) / cd

        return {
            "f1": f1,
            "f2": f2,
            "f3": f3,
            "fg": fg,
            "f1d": f1d,
            "f2d": f2d,
            "f3d": f3d,
            "fgd": fgd
        }

    # ========================================================
    # DRIFT COMPUTATION
    # ========================================================

    def compute_drift(
            self,
            state,
            target,
            basis
    ):

        p0 = state.pos
        v0 = state.vel
        a0 = state.acc

        # ----------------------------------------------------
        # Relative position
        # ----------------------------------------------------

        if isinstance(target, SphereConstraint):

            rel_p0 = p0 - target.target.center

        elif isinstance(target, PositionConstraint):

            rel_p0 = p0 -  target.target_position

        else:
            raise ValueError("Unsupported target type")

        # ----------------------------------------------------
        # Drift
        # ----------------------------------------------------

        drift = (
                rel_p0
                + v0 * basis["f1"]
                + a0 * basis["f2"]
                + self.gravity * basis["fg"]
        )

        drift_dot = (
                v0 * basis["f1d"]
                + a0 * basis["f2d"]
                + self.gravity * basis["fgd"]
        )

        return drift, drift_dot

    # ========================================================
    # INITIAL TIME GUESS
    # ========================================================

    def estimate_initial_time(
            self,
            state,
            target
    ):

        p0 = state.pos

        if isinstance(target, SphereConstraint):

            rel = p0 - target.target.center

            dist = abs(
                np.linalg.norm(rel)
                - target.target.radius
            )

        elif isinstance(target, PositionConstraint):

            dist = np.linalg.norm(
                target.target_position - p0
            )

        else:
            raise ValueError("Unsupported target type")

        return max(
            0.1,
            (6.0 * dist / self.j_max) ** (1.0 / 3.0)
        )

    # ========================================================
    # SOLVE
    # ========================================================

    def solve(
            self,
            planning_request: PlanningRequest,
            t_init=None
    ):
        self.configure(planning_request)

        state = State3D.from_vector(planning_request.initial_state)
        target = planning_request.target

        if isinstance(target, SphereTarget):
            target_constraint = SphereConstraint(target)
        elif isinstance(target, PositionConstraint):
            target_constraint = target
        else:
            # assume raw xyz target
            target_constraint = PositionConstraint(target)

        # ----------------------------------------------------
        # Initial guess
        # ----------------------------------------------------

        if t_init is None:

            t = self.estimate_initial_time(
                state,
                target_constraint
            )

        else:
            t = t_init

        # ----------------------------------------------------
        # Newton solve
        # ----------------------------------------------------

        for _ in range(self.max_iter):

            basis = self.get_basis(t)

            drift, drift_dot = self.compute_drift(
                state,
                target_constraint,
                basis
            )

            reach = self.j_max * basis["f3"]

            reach_dot = self.j_max * basis["f3d"]

            G, Gd = target_constraint.residual(
                drift,
                drift_dot,
                reach,
                reach_dot
            )

            if abs(Gd) < 1e-10:
                break

            step = G / Gd

            t = max(1e-4, t - step)

            if abs(step) < self.tol:
                break

        # ----------------------------------------------------
        # Final jerk
        # ----------------------------------------------------

        basis = self.get_basis(t)

        drift, _ = self.compute_drift(
            state,
            target_constraint,
            basis
        )

        j_star = target_constraint.optimal_control(
            drift,
            self.j_max
        )

        return PlanningResult(
                    success=True,
                    final_time=t,
                    times=np.array([0.0, t]),
                    states=np.array([state.as_vector()]),
                    controls=np.array([j_star])
                )


# ============================================================
# EXAMPLE USAGE
# ============================================================

#
# POINT TARGET
#
#
# solver = AugmentedTripleIntegratorMinimumTimeSolver(
#     j_max=15.0,
#     cd=1.2,
#     ca=0.6
# )
#
# target = PositionConstraint(
#     np.array([10.0, 0.0, 0.0])
# )
#
# t, j = solver.solve(state, target)
#
# print(t, j)


#
# HEMISPHERE TARGET
#
#
# hemi_target = SphereTarget(
#     center=np.array([0, 0, 0]),
#     radius=5.0
# )
#
# solver = AugmentedTripleIntegratorMinimumTimeSolver(
#     j_max=15.0,
#     cd=1.2,
#     ca=0.6,
#     gravity=np.array([0, 0, -9.81])
# )
#
# target = SphereConstraint(
#     hemi_target
# )
#
# t, j = solver.solve(state, target)
#
# print(t, j)