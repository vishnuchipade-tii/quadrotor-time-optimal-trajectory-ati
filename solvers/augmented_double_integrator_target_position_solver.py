import numpy as np
from core.targets import SphereTarget
from core.constraints import PositionConstraint, SphereConstraint
from core.planning import PlanningRequest
from core.planning import PlanningResult
from core.state import DIState3D

#SolverRegistry related imports
from core.registry import SolverRegistry
from core.keys import SolverKey
from core.type import DynamicsType
from core.type import TargetType
from core.type import SolverType

# ===========================================================================
# AUGMENTED DOUBLE INTEGRATOR UNIFIED SOLVER FOR POSITION AND SPHERE TARGETS
# ===========================================================================
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_DOUBLE_INTEGRATOR,
        TargetType.POSITION,
        SolverType.ROOT_FINDING,
    )
)
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_DOUBLE_INTEGRATOR,
        TargetType.SPHERE,
        SolverType.ROOT_FINDING,
    )
)
class AugmentedDoubleIntegratorTargetPositionSolver:

    def __init__(self):

        self.solver_id = "AugmentedDoubleIntegratorTargetPositionSolver"
        # Dynamics
        self.cd = None
        self.gravity = None

        # Input bounds
        self.a_max = None

        # Optional solver parameters
        self.max_iter = 3000
        self.tol = 1e-5

    def configure(self, planning_request: PlanningRequest):

        dynamics = planning_request.dynamics
        bounds = planning_request.input_bounds

        self.cd = dynamics.cd
        self.gravity = dynamics.gravity

        self.a_max = bounds.get("a_max")

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

        if t < 1e-6:

            return {
                "f1": 0.0,
                "f2": 0.0,
                "fg": 0.0,
                "f1d": 1.0,
                "f2d": 0.0,
                "fgd": 0.0
            }

        exp_a = np.exp(-self.cd * t)

        # ----------------------------------------------------
        # Velocity drift basis
        # ----------------------------------------------------

        f1 = (1.0 - exp_a) / self.cd

        # ----------------------------------------------------
        # Control reach basis
        # ----------------------------------------------------

        f2 = (
                self.cd * t
                - (1.0 - exp_a)
        ) / (self.cd ** 2)

        # ----------------------------------------------------
        # Gravity basis
        # Same as f2 for DI
        # ----------------------------------------------------

        fg = f2

        # ----------------------------------------------------
        # Derivatives
        # ----------------------------------------------------

        f1d = exp_a

        f2d = f1

        fgd = f1

        return {
            "f1": f1,
            "f2": f2,
            "fg": fg,
            "f1d": f1d,
            "f2d": f2d,
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

        # ----------------------------------------------------
        # Relative position
        # ----------------------------------------------------

        if isinstance(target, SphereConstraint):

            rel_p0 = p0 - target.target.center

        elif isinstance(target, PositionConstraint):

            rel_p0 = p0 - target.target_position

        else:
            raise ValueError("Unsupported target type")

        # ----------------------------------------------------
        # Drift
        # ----------------------------------------------------

        drift = (
                rel_p0
                + v0 * basis["f1"]
                + self.gravity * basis["fg"]
        )

        drift_dot = (
                v0 * basis["f1d"]
                + self.gravity * basis["fgd"]
        )

        return drift, drift_dot

    # ========================================================
    # INITIAL TIME ESTIMATE
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
            dist / (0.5 * self.a_max + 1e-3)
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

        state = DIState3D.from_vector(planning_request.initial_state)
        target = planning_request.target

        # ----------------------------------------------------
        # Normalize target
        # ----------------------------------------------------

        if isinstance(target, SphereTarget):

            target_constraint = SphereConstraint(target)

        elif isinstance(target, PositionConstraint):

            target_constraint = target

        else:
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

            reach = self.a_max * basis["f2"]

            reach_dot = self.a_max * basis["f2d"]

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
        # Final optimal control
        # ----------------------------------------------------

        basis = self.get_basis(t)

        drift, _ = self.compute_drift(
            state,
            target_constraint,
            basis
        )

        u_opt = target_constraint.optimal_control(
            drift,
            self.a_max
        )

        return PlanningResult(
            success=True,
            final_time=t,
            times=np.array([0.0, t]),
            states=np.array([state.as_vector()]),
            controls=np.array([u_opt])
        )


# ============================================================
# EXAMPLES
# ============================================================

#
# POINT TARGET
#

# solver = AugmentedDoubleIntegratorPositionOptimalSolver(
#     a_max=10.0,
#     cd=1.0,
#     use_gravity=False
# )
#
# target = np.array([10.0, 0.0, 0.0])
#
# t, u = solver.solve(state, target)
#
# print(t)
# print(u)


#
# HEMISPHERE TARGET
#

# hemi = SphereTarget(
#     center=np.array([0.0, 0.0, 0.0]),
#     radius=5.0
# )
#
# solver = AugmentedDoubleIntegratorPositionOptimalSolver(
#     a_max=10.0,
#     cd=1.0,
#     use_gravity=True
# )
#
# t, u = solver.solve(state, hemi)
#
# print(t)
# print(u)