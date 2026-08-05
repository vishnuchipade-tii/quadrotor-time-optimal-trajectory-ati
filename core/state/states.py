from dataclasses import dataclass, field
import numpy as np

from core.type import StateType

from core.registry.state_registry import StateRegistry


@StateRegistry.register(StateType.AUG_TRIPLE_INTEGRATOR)
@dataclass
class State3D:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_config(cls, cfg):
        return cls(
            pos=np.asarray(cfg["pos"]),
            vel=np.asarray(cfg["vel"]),
            acc=np.asarray(cfg["acc"]),
        )

    @classmethod
    def from_vector(cls, x: np.ndarray):
        x = np.asarray(x)

        if x.shape != (9,):
            raise ValueError(
                f"State3D expects vector of size 9, got {x.shape}"
            )

        return cls(
            pos=x[0:3].copy(),
            vel=x[3:6].copy(),
            acc=x[6:9].copy()
        )

    def to_vec(self):
        return np.concatenate([self.pos, self.vel, self.acc])

    def as_vector(self):
        return np.concatenate([self.pos, self.vel, self.acc])

    def copy(self):
        """
        Returns a new State3D instance with copied numpy arrays.
        Essential for accurate history logging.
        """
        return State3D(
            pos=self.pos.copy(),
            vel=self.vel.copy(),
            acc=self.acc.copy()
        )

@StateRegistry.register(StateType.AUG_DOUBLE_INTEGRATOR)
@dataclass
class DIState3D:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_config(cls, cfg):
        return cls(
            pos=np.asarray(cfg["pos"]),
            vel=np.asarray(cfg["vel"]),
        )

    @classmethod
    def from_vector(cls, x: np.ndarray):
        x = np.asarray(x)

        if x.shape != (6,):
            raise ValueError(
                f"DIState3D expects vector of size 6, got {x.shape}"
            )

        return cls(
            pos=x[0:3].copy(),
            vel=x[3:6].copy()
        )

    def to_vec(self):
        return np.concatenate([self.pos, self.vel])

    def as_vector(self):
        return np.concatenate([self.pos, self.vel])

    def copy(self):
        """
        Returns a new DIState3D instance with copied numpy arrays.
        Essential for accurate history logging.
        """
        return DIState3D(
            pos=self.pos.copy(),
            vel=self.vel.copy()
        )