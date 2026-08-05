"""
Module: models.py
Project:  Time-Optimal Control for Quadrotor
"""

import numpy as np

from core.registry.dynamics_registry import DynamicsRegistry
from core.type.dynamics_type import DynamicsType
from core.dynamics.base import BaseDynamicsModel
from core.dynamics.model_info import ModelInfo


@DynamicsRegistry.register(DynamicsType.AUG_DOUBLE_INTEGRATOR)
class AugmentedDoubleIntegratorModel(BaseDynamicsModel):

    def __init__(self, params):
        self.info = ModelInfo(
                        model_id="augmented_double_integrator",
                        display_name=r"DI$^{*}$",
                        directory_name="DI_star"
                    )
        self.params = params
        self.cd = params["cd"]
        self.use_gravity = params.get("use_gravity", True)
        if self.use_gravity:
            self.gravity = np.asarray(
                params.get("gravity", [0.0, 0.0, -9.81]),
                dtype=float,
            )
        else:
            self.gravity = np.zeros(3)

    def dim_state(self):
        return 6

    def rhs(self, state, control):

        a = control.value

        p_dot = state.vel
        v_dot = a - self.cd * state.vel + self.gravity

        x_dot = np.concatenate([p_dot, v_dot])

        return x_dot


    def step(self, state, control, dt):

        x_dot = self.rhs(state, control)

        p_dot = x_dot[0:3]
        v_dot = x_dot[3:6]

        state.pos = state.pos + p_dot * dt
        state.vel = state.vel + v_dot * dt

        return state

@DynamicsRegistry.register(DynamicsType.AUG_TRIPLE_INTEGRATOR)
class AugmentedTripleIntegratorModel(BaseDynamicsModel):

    def __init__(self, params):
        self.info = ModelInfo(
                        model_id="augmented_triple_integrator",
                        display_name=r"TI$^{*}$",
                        directory_name="TI_star"
                    )
        self.params = params
        self.ca = params["ca"]
        self.cd = params["cd"]
        self.use_gravity = params.get("use_gravity", True)
        if self.use_gravity:
            self.gravity = np.asarray(
                params.get("gravity", [0.0, 0.0, -9.81]),
                dtype=float,
            )
        else:
            self.gravity = np.zeros(3)

    def dim_state(self):
        return 9

    def rhs(self, state, control):

        u = control.value

        p_dot = state.vel
        v_dot = state.acc - self.cd * state.vel + self.gravity
        a_dot = u - self.ca * state.acc

        x_dot = np.concatenate([p_dot, v_dot, a_dot])

        return x_dot

    def step(self, state, control, dt):

        x_dot = self.rhs(state, control)

        p_dot = x_dot[0:3]
        v_dot = x_dot[3:6]
        a_dot = x_dot[6:9]

        state.pos = state.pos + p_dot * dt
        state.vel = state.vel + v_dot * dt
        state.acc = state.acc + a_dot * dt

        return state


#Dummy 6DOF model only for bookkeeping
@DynamicsRegistry.register(DynamicsType.SIX_DOF)
class SixDOFModel(BaseDynamicsModel):

    def __init__(self, params):
        self.info = ModelInfo(
                        model_id="6dof",
                        display_name="6DOF",
                        directory_name="6DOF"
                    )
        self.params = params

    def dim_state(self):
        return 12

    def rhs(self, state, control):
        return

    def step(self, state, control, dt):
        return