from core.registry.dynamics_registry import DynamicsRegistry

class DynamicsBuilder:

    @staticmethod
    def build(name, params):

        try:
            cls = DynamicsRegistry.get(name)
        except KeyError:
            raise ValueError(f"Unknown dynamics model '{name}'")

        return cls(
            params=params
        )