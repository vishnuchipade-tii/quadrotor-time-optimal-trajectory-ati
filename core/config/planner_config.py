from dataclasses import dataclass, field
from pathlib import Path
import yaml

@dataclass
class PlannerConfig:

    replan: bool = False
    replan_interval: float | None = None

    def validate(self):
        if self.replan and self.replan_interval is None:
            raise ValueError(
                "replan_interval must be provided when replan=True"
            )

        if self.replan_interval is not None and self.replan_interval <= 0:
            raise ValueError(
                "replan_interval must be positive"
            )

    @classmethod
    def from_dict(cls, cfg: dict):
        obj = cls(
            replan=cfg.get("replan", False),
            replan_interval=cfg.get("replan_interval"),
        )

        obj.validate()

        return obj

    @classmethod
    def from_config(cls, cfg):
        """
        Build from either:
        - inline dictionary
        - yaml file path
        """

        if cfg is None:
            return None

        if isinstance(cfg, dict):
            return cls.from_dict(cfg)

        if isinstance(cfg, str):
            path = Path(cfg)

            with open(path, "r") as f:
                yaml_cfg = yaml.safe_load(f)

            return cls.from_dict(yaml_cfg)

        raise TypeError(
            f"Unsupported planner config type: {type(cfg)}"
        )