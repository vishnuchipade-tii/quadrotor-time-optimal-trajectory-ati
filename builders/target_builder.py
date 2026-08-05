
from core.registry.target_registry import TargetRegistry
from core.type import TargetType

class TargetBuilder:
    @staticmethod
    def build(cfg, model):

        target_type = cfg["type"]

        try:
            target_cls = TargetRegistry.get(TargetType.from_string(target_type))
        except KeyError:
            raise ValueError(f"Unknown target '{target_type}'")

        cfg = {
            **cfg,
            "model": model,
        }

        return target_cls.from_config(cfg)