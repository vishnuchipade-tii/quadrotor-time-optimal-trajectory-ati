import casadi as ca
import numpy as np

from core.planning import PlanningRequest
from core.planning import PlanningResult

#SolverRegistry related imports
from core.registry.solver_registry import SolverRegistry
from core.keys.solver_key import SolverKey
from core.dynamics.models import DynamicsType
from core.type import TargetType
from core.type.solver_type import SolverType

# ===========================================================================
# AUGMENTED DOUBLE INTEGRATOR SOLVER FOR FULL STATE TARGETS
# ===========================================================================
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_DOUBLE_INTEGRATOR,
        TargetType.STATE,
        SolverType.DIRECT_COLLOCATION,
    )
)
class AugmentedDoubleIntegratorTargetStateCasadiSolver:
    """
    Minimum-time direct transcription solver for:

        p_dot = v
        v_dot = u - cd*v + g

    where:
        ||u||_2 <= a_max

    State:
        x = [p(3), v(3)]  -> 6 states

    Control:
        u = acceleration command (3D)

    Uses:
        - exact STM propagation
        - piecewise constant controls
        - free final time
        - IPOPT
    """

    def __init__(self):

        self.solver_id = "AugmentedDoubleIntegratorTargetStateCasadiSolver"

        # configuration, populated from PlanningRequest
        self.a_max = None
        self.cd = None
        self.g = None

        self.N = 100
        self.t_max = 30.0

        # runtime solution storage
        self.sol = None
        self.x_traj = None
        self.u_traj = None
        self.T = None
        self.t_grid = None

    def configure(self, planning_request):
        self.a_max = planning_request.input_bounds.get("a_max")

        self.cd = planning_request.dynamics.cd
        self.g = planning_request.dynamics.gravity

        solver_config = planning_request.solver_config

        solver_options = solver_config.options

        self.N = solver_options.get(
            "N",
            100
        )

        self.t_max = solver_options.get(
            "t_max",
            30.0
        )

    # ============================================================
    # Scalar helper
    # ============================================================

    def phi_scalar(self, c, t):
        return (1.0 - ca.exp(-c * t)) / c

    # ============================================================
    # Exact STM
    # ============================================================

    def stm(self, dt):

        I = ca.MX.eye(3)

        e_cd = ca.exp(-self.cd * dt)

        phi_d = self.phi_scalar(self.cd, dt)

        Phi = ca.MX.zeros(6, 6)

        # position
        Phi[0:3, 0:3] = I
        Phi[0:3, 3:6] = phi_d * I

        # velocity
        Phi[3:6, 3:6] = e_cd * I

        return Phi

    # ============================================================
    # Exact ZOH input matrix
    # ============================================================

    def gamma_u(self, dt):

        I = ca.MX.eye(3)

        e_cd = ca.exp(-self.cd * dt)

        phi_d = (1.0 - e_cd) / self.cd

        # velocity contribution
        Gv = phi_d

        # position contribution
        Gp = (dt - phi_d) / self.cd

        G = ca.MX.zeros(6, 3)

        G[0:3, :] = Gp * I
        G[3:6, :] = Gv * I

        return G

    # ============================================================
    # Exact gravity offset
    # ============================================================

    def gravity_term(self, dt):

        phi_d = self.phi_scalar(self.cd, dt)

        g_term = ca.MX.zeros(6)

        # position gravity contribution
        g_term[0:3] = ((dt - phi_d) / self.cd) * self.g

        # velocity gravity contribution
        g_term[3:6] = phi_d * self.g

        return g_term

    # ============================================================
    # Numeric STM (for initialization)
    # ============================================================

    def stm_numeric(self, dt):

        e_cd = np.exp(-self.cd * dt)

        phi_d = (1.0 - e_cd) / self.cd

        Phi = np.zeros((6, 6))

        I = np.eye(3)

        Phi[0:3, 0:3] = I
        Phi[0:3, 3:6] = phi_d * I

        Phi[3:6, 3:6] = e_cd * I

        return Phi

    # ============================================================
    # Numeric Gamma
    # ============================================================

    def gamma_u_numeric(self, dt):

        e_cd = np.exp(-self.cd * dt)

        phi_d = (1.0 - e_cd) / self.cd

        Gv = phi_d

        Gp = (dt - phi_d) / self.cd

        G = np.zeros((6, 3))

        I = np.eye(3)

        G[0:3, :] = Gp * I
        G[3:6, :] = Gv * I

        return G

    # ============================================================
    # Numeric gravity
    # ============================================================

    def gravity_term_numeric(self, dt):

        e_cd = np.exp(-self.cd * dt)

        phi_d = (1.0 - e_cd) / self.cd

        g_term = np.zeros(6)

        g_term[0:3] = ((dt - phi_d) / self.cd) * self.g
        g_term[3:6] = phi_d * self.g

        return g_term

    # ============================================================
    # Main planner
    # ============================================================

    def solve(self, planning_request: PlanningRequest):

        self.configure(planning_request)

        x0 = planning_request.initial_state

        xf = planning_request.target.desired_state.as_vector()

        opti = ca.Opti()

        N = self.N

        # --------------------------------------------------------
        # Decision variables
        # --------------------------------------------------------

        X = opti.variable(6, N + 1)
        U = opti.variable(3, N)

        T = opti.variable()

        dt = T / N

        # --------------------------------------------------------
        # Time bounds
        # --------------------------------------------------------

        dist = np.linalg.norm(xf[0:3] - x0[0:3])

        T_min = np.sqrt(dist / max(self.a_max, 1e-3))

        opti.subject_to(T >= 1.5 * T_min)
        opti.subject_to(T <= 50.0)

        # --------------------------------------------------------
        # Initial condition
        # --------------------------------------------------------

        opti.subject_to(X[:, 0] == x0)

        # --------------------------------------------------------
        # Dynamics
        # --------------------------------------------------------

        for k in range(N):

            xk = X[:, k]
            xk1 = X[:, k + 1]
            uk = U[:, k]

            Phi = self.stm(dt)
            G = self.gamma_u(dt)
            g_term = self.gravity_term(dt)

            x_next = Phi @ xk + G @ uk + g_term

            opti.subject_to(xk1 == x_next)

            # control bound
            opti.subject_to(
                ca.sumsqr(uk) <= self.a_max**2
            )

        # --------------------------------------------------------
        # State bounds
        # --------------------------------------------------------

        MAX_POS = 1e4
        MAX_VEL = 200.0

        opti.subject_to(
            opti.bounded(-MAX_POS, X[0:3, :], MAX_POS)
        )

        opti.subject_to(
            opti.bounded(-MAX_VEL, X[3:6, :], MAX_VEL)
        )

        # --------------------------------------------------------
        # Terminal constraint
        # --------------------------------------------------------

        opti.subject_to(X[:, -1] == xf)

        # --------------------------------------------------------
        # Objective
        # --------------------------------------------------------

        cost = T + 1e-4 * ca.sumsqr(U)

        opti.minimize(cost)

        # --------------------------------------------------------
        # IPOPT settings
        # --------------------------------------------------------

        p_opts = {
            "expand": True
        }

        s_opts = {
            "max_iter": 1000,
            "tol": 1e-3,
            "acceptable_tol": 1e-2,
            "mu_strategy": "adaptive",
            "linear_solver": "mumps",
            "hessian_approximation": "limited-memory",
            "print_level": 5,
        }

        opti.solver("ipopt", p_opts, s_opts)

        # ========================================================
        # Initialization
        # ========================================================

        tf_guess = np.clip(dist / 5.0, 2.0, 15.0)

        opti.set_initial(T, tf_guess)

        xk = np.array(x0)

        dt_guess = tf_guess / N

        Phi_num = self.stm_numeric(dt_guess)
        g_num = self.gravity_term_numeric(dt_guess)

        for k in range(N):

            opti.set_initial(X[:, k], xk)

            opti.set_initial(U[:, k], np.zeros(3))

            xk = Phi_num @ xk + g_num

        opti.set_initial(X[:, N], xf)

        # ========================================================
        # Solve
        # ========================================================

        sol = opti.solve()

        #Extract solution
        self.x_traj = sol.value(X)
        self.u_traj = sol.value(U)
        self.T = sol.value(T)

        self.t_grid = np.linspace(0.0, self.T, N + 1)

        return PlanningResult(

            success=True,

            final_time=self.T,

            times=np.linspace(
                0,
                self.T,
                N + 1
            ),

            states=self.x_traj.T,

            controls=self.u_traj.T

        )