import casadi
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
# AUGMENTED TRIPLE INTEGRATOR SOLVER FOR FULL STATE TARGETS
# ===========================================================================
@SolverRegistry.register(
    SolverKey(
        DynamicsType.AUG_TRIPLE_INTEGRATOR,
        TargetType.STATE,
        SolverType.DIRECT_COLLOCATION,
    )
)
class AugmentedTripleIntegratorTargetStateCasadiSolver:

    def __init__(self):

        self.solver_id = "AugmentedDoubleIntegratorTargetStateCasadiSolver"

        # configuration, populated from PlanningRequest
        self.j_max = None
        self.ca = None
        self.cd = None
        self.g = None

        self.N = 100
        self.t_max = 30.0

        # solution
        self.x_traj = None
        self.u_traj = None
        self.T = None
        self.t_grid = None

    def configure(self, planning_request):
        self.j_max = planning_request.input_bounds.get("j_max")

        self.ca = planning_request.dynamics.ca
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

    # ------------------------------------------------------------
    # Analytical STM blocks (CasADi compatible)
    # ------------------------------------------------------------
    def phi_scalar(self, c, t):
        return (1 - casadi.exp(-c * t)) / c

    def stm(self, dt):

        p_v = self.phi_scalar(self.cd, dt)
        p_a = self.phi_scalar(self.ca, dt)

        gamma = (casadi.exp(-self.ca * dt) - casadi.exp(-self.cd * dt)) / (self.cd - self.ca)
        lambd = (p_a - p_v) / (self.cd - self.ca)

        Phi = casadi.MX.zeros(9, 9)
        I = casadi.MX.eye(3)

        Phi[0:3, 0:3] = I
        Phi[3:6, 3:6] = casadi.exp(-self.cd * dt) * I
        Phi[6:9, 6:9] = casadi.exp(-self.ca * dt) * I

        Phi[0:3, 3:6] = p_v * I
        Phi[0:3, 6:9] = lambd * I
        Phi[3:6, 6:9] = gamma * I

        return Phi

    def gamma_u(self, dt):

        I = casadi.MX.eye(3)

        e_ca = casadi.exp(-self.ca * dt)
        e_cd = casadi.exp(-self.cd * dt)

        phi_a = (1 - e_ca) / self.ca
        phi_d = (1 - e_cd) / self.cd

        # --------------------------------------------------
        # Exact discrete input matrix
        # --------------------------------------------------

        Ga = phi_a

        Gv = (phi_d - phi_a) / (self.ca - self.cd)

        Gp = (
                (dt - phi_d) / (self.cd * (self.ca - self.cd))
                -
                (dt - phi_a) / (self.ca * (self.ca - self.cd))
        )

        G = casadi.MX.zeros(9, 3)

        G[0:3, :] = Gp * I
        G[3:6, :] = Gv * I
        G[6:9, :] = Ga * I

        return G

    def gamma_u_bak(self, dt):

        # control influence blocks (exact same structure as your integrator)
        p_v = self.phi_scalar(self.cd, dt)
        p_a = self.phi_scalar(self.ca, dt)

        g1 = (p_a - p_v) / (self.cd - self.ca)
        g2 = (casadi.exp(-self.ca * dt) - casadi.exp(-self.cd * dt)) / (self.cd - self.ca)
        g3 = casadi.exp(-self.ca * dt)

        Gamma_u = casadi.MX.zeros(9, 3)

        I = casadi.MX.eye(3)

        Gamma_u[0:3, :] = g1 * I
        Gamma_u[3:6, :] = g2 * I
        Gamma_u[6:9, :] = g3 * I

        return Gamma_u

    def gamma_gravity(self, dt):
        p_v = self.phi_scalar(self.cd, dt)

        Gamma_g = casadi.MX.zeros(9)

        Gamma_g[0:3] = ((dt - p_v) / self.cd) * self.g
        Gamma_g[3:6] = p_v * self.g

        return Gamma_g

    def stm_numeric(self, dt):

        e_ca = np.exp(-self.ca * dt)
        e_cd = np.exp(-self.cd * dt)

        phi_d = (1 - e_cd) / self.cd
        phi_a = (1 - e_ca) / self.ca

        gamma = (e_ca - e_cd) / (self.cd - self.ca)
        lambd = (phi_a - phi_d) / (self.cd - self.ca)

        Phi = np.zeros((9, 9))
        I = np.eye(3)

        Phi[0:3, 0:3] = I
        Phi[3:6, 3:6] = e_cd * I
        Phi[6:9, 6:9] = e_ca * I

        Phi[0:3, 3:6] = phi_d * I
        Phi[0:3, 6:9] = lambd * I
        Phi[3:6, 6:9] = gamma * I

        return Phi

    def gamma_u_numeric(self, dt):

        e_ca = np.exp(-self.ca * dt)
        e_cd = np.exp(-self.cd * dt)

        phi_a = (1 - e_ca) / self.ca
        phi_d = (1 - e_cd) / self.cd

        Ga = phi_a

        Gv = (phi_d - phi_a) / (self.ca - self.cd)

        Gp = (
                (dt - phi_d) / (self.cd * (self.ca - self.cd))
                -
                (dt - phi_a) / (self.ca * (self.ca - self.cd))
        )

        Gamma_u = np.zeros((9, 3))
        I = np.eye(3)

        Gamma_u[0:3, :] = Gp * I
        Gamma_u[3:6, :] = Gv * I
        Gamma_u[6:9, :] = Ga * I

        return Gamma_u

    def gamma_gravity_numeric(self, dt):

        e_cd = np.exp(-self.cd * dt)

        phi_d = (1 - e_cd) / self.cd

        Gamma_g = np.zeros(9)

        Gamma_g[0:3] = ((dt - phi_d) / self.cd) * self.g
        Gamma_g[3:6] = phi_d * self.g

        return Gamma_g

    def estimate_tf_guess(self, x0, xf):

        p0, v0, a0 = x0[:3], x0[3:6], x0[6:9]
        pf, vf, af = xf[:3], xf[3:6], xf[6:9]

        d = np.linalg.norm(pf - p0)
        dv = np.linalg.norm(vf - v0)
        da = np.linalg.norm(af - a0)

        j = max(self.j_max, 1e-6)

        t_dist = (d / j) ** (1.0 / 3.0)
        t_vel = np.sqrt(dv / j)
        t_acc = da / j

        tf = 2.0 * max(t_dist, t_vel, t_acc)

        return np.clip(tf, 0.5, self.t_max)

    # ------------------------------------------------------------
    # MAIN SOLVER
    # ------------------------------------------------------------
    def solve(self, planning_request: PlanningRequest):

        self.configure(planning_request)

        x0 = planning_request.initial_state

        xf = planning_request.target.desired_state.as_vector()

        N = self.N
        opti = casadi.Opti()

        # decision variables
        X = opti.variable(9, N + 1)
        U = opti.variable(3, N)
        T = opti.variable()

        dt = T / N

        # control bounds
        dist = np.linalg.norm(xf[0:3] - x0[0:3])

        T_min = (dist / self.j_max) ** (1 / 3)

        opti.subject_to(T >= 2.0 * T_min)

        opti.subject_to(T < 50.0)

        # initial condition
        opti.subject_to(X[:, 0] == x0)

        cost = T  # minimum time

        # --------------------------------------------------------
        # dynamics via EXACT STM propagation
        # --------------------------------------------------------
        for k in range(N):

            xk = X[:, k]
            xk1 = X[:, k + 1]
            uk = U[:, k]

            Phi = self.stm(dt)
            Gamma_u = self.gamma_u(dt)
            Gamma_g = self.gamma_gravity(dt)

            x_next = Phi @ xk + Gamma_u @ uk + Gamma_g

            opti.subject_to(xk1 == x_next)

            opti.subject_to(casadi.sumsqr(uk) <= self.j_max**2)

        # state constraints
        MAX_POS = 1e4
        MAX_VEL = 200.0
        MAX_ACC = 30.0

        opti.subject_to(opti.bounded(-MAX_POS, X[0:3, :], MAX_POS))
        opti.subject_to(opti.bounded(-MAX_VEL, X[3:6, :], MAX_VEL))
        opti.subject_to(opti.bounded(-MAX_ACC, X[6:9, :], MAX_ACC))

        # terminal constraint
        terminal_tol = 1e-4
        opti.subject_to(
            casadi.sumsqr(X[:, -1] - xf) <= terminal_tol
        )

        opti.minimize(cost)

        p_opts = {
            "expand": True
        }

        s_opts = {
            "max_iter": 5000,
            "tol": 1e-4,
            "acceptable_tol": 1e-3,
            "mu_strategy": "adaptive",
            "linear_solver": "mumps",
            "print_level": 5,
        }

        opti.solver("ipopt", p_opts, s_opts)


        #Initialize
        tf_guess = self.estimate_tf_guess(x0, xf)

        opti.set_initial(T, tf_guess)
        xk = np.array(x0)
        dt_guess = tf_guess / N

        r = xf[:3] - x0[:3]
        r_norm = np.linalg.norm(r)

        if r_norm > 1e-6:
            u_guess = self.j_max * r / r_norm
        else:
            u_guess = np.zeros(3)

        Phi_num = self.stm_numeric(dt_guess)
        Gamma_u_num = self.gamma_u_numeric(dt_guess)
        Gamma_g_num = self.gamma_gravity_numeric(dt_guess)


        for k in range(N):
            opti.set_initial(X[:, k], xk)
            opti.set_initial(U[:, k], u_guess)
            xk = Phi_num @ xk + Gamma_g_num

        opti.set_initial(X[:, N], xf)

        sol = opti.solve()

        #Extract solution
        self.x_traj = sol.value(X)
        self.u_traj = sol.value(U)
        self.T = sol.value(T)

        self.t_grid = np.linspace(0, self.T, N + 1)

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
