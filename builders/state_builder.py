from core.registry.state_registry import StateRegistry
from core.type import StateType

class StateBuilder:
    @staticmethod
    def build(cfg):
        try:
            state_cls = StateRegistry.get(StateType.from_string(cfg["type"]))
        except KeyError:
            raise ValueError(f"Unknown state type: {cfg['type']}")

        return state_cls.from_config(cfg)