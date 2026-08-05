from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from core.type import TargetType
from core.registry.target_registry import TargetRegistry
from core.targets.base import BaseTarget
from builders.state_builder import StateBuilder

#======================
# POSIITON TARGET
#======================

@TargetRegistry.register(TargetType.POSITION)
@dataclass
class PositionTarget(BaseTarget):
    target_type = TargetType.POSITION

    desired_position: np.ndarray = field()

    tol: float = 0.1

    @classmethod
    def from_config(cls, cfg):
        return cls(
            desired_position=np.asarray(
                cfg["pos"]
            ),
            tol=cfg.get(
                "tol",
                0.1
            ),
        )

    def is_reached(self, state) -> bool:
        return (
            np.linalg.norm(
                state.pos - self.desired_position
            )
            <= self.tol
        )

#======================
# SPHERE TARGET
#======================
@TargetRegistry.register(TargetType.SPHERE)
@dataclass
class SphereTarget:
    target_type = TargetType.SPHERE

    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    radius: float = 2.0
    tol: Optional[float] = None  # optional override

    def __post_init__(self):
        if self.tol is None:
            # default: no tolerance unless radius is zero
            if self.radius == 0.0:
                self.tol = 0.15  # fallback tolerance
            else:
                self.tol = 0.0

    @classmethod
    def from_config(cls, cfg):

        return cls(
            center=np.asarray(
                cfg.get(
                    "center",
                    [0.0, 0.0, 0.0]
                )
            ),
            radius=cfg.get(
                "radius",
                2.0
            ),
            tol=cfg.get(
                "tol"
            ),
        )

    def is_inside(self, point: np.ndarray) -> bool:
        rel_pos = point - self.center
        dist = np.linalg.norm(rel_pos)
        return dist <= (self.radius + self.tol)

    def is_reached(self, state) -> bool:
        return self.is_inside(state.pos)


#======================
# STATE TARGET
#======================
@TargetRegistry.register(TargetType.STATE)
@dataclass
class StateTarget(BaseTarget):

    target_type = TargetType.STATE

    desired_state: object

    pos_tol: float = 0.1
    vel_tol: float = 0.1
    acc_tol: float = 0.1

    @classmethod
    def from_config(cls, cfg):
        desired_state = StateBuilder.build(
            {
                **cfg,
                "type": cfg["model"],
            }
        )

        return cls(
            desired_state=desired_state,
            pos_tol=cfg.get(
                "pos_tol",
                0.1
            ),
            vel_tol=cfg.get(
                "vel_tol",
                0.1
            ),
            acc_tol=cfg.get(
                "acc_tol",
                0.1
            ),
        )

    def is_reached(self, state) -> bool:

        # -----------------------------
        # position
        # -----------------------------

        pos_ok = (
            np.linalg.norm(
                state.pos - self.desired_state.pos
            )
            <= self.pos_tol
        )

        # -----------------------------
        # velocity
        # -----------------------------

        vel_ok = (
            np.linalg.norm(
                state.vel - self.desired_state.vel
            )
            <= self.vel_tol
        )

        # -----------------------------
        # acceleration
        # only if both states have acc
        # -----------------------------

        acc_ok = True

        if (
            hasattr(state, "acc")
            and hasattr(self.desired_state, "acc")
        ):

            acc_ok = (
                np.linalg.norm(
                    state.acc - self.desired_state.acc
                )
                <= self.acc_tol
            )

        return pos_ok and vel_ok and acc_ok