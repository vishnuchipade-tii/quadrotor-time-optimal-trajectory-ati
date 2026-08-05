"""
Module: quadrotor.py
Project:  Time-Optimal Control for Quadrotor
"""
from core.state import ControlInput3D
import numpy as np

class Quadrotor:

    def __init__(self, id, initial_state, dynamics_model, params, name=None):

        self.id = id
        self.name = name

        self.state = initial_state
        self.dynamics_model = dynamics_model
        self.params = params

        self.state_history = [self.state.copy()]
        self.control = ControlInput3D()
        self.control_history = []
        self.time_history = []

    def set_control(self, u_vec: np.ndarray):
        self.control.value = u_vec
        self.control_history.append(self.control.copy())

    def step(self, dt):

        self.state = self.dynamics_model.step(
            self.state,
            self.control,
            dt
        )

        self.state_history.append(self.state.copy())

    def get_flat_state_history(self, dt):

        return