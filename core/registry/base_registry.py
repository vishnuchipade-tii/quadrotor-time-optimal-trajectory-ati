from typing import Dict, Type, TypeVar, Generic

T = TypeVar("T")


class BaseRegistry(Generic[T]):
    """
    Generic registry for framework components.
    """

    _registry: Dict[object, Type[T]] = {}

    @classmethod
    def register(cls, key):

        def decorator(component_cls: Type[T]):
            if key in cls._registry:
                raise ValueError(
                    f"{cls.__name__}: {key} already registered"
                )

            cls._registry[key] = component_cls
            return component_cls

        return decorator

    @classmethod
    def get(cls, key) -> Type[T]:

        if key not in cls._registry:
            raise ValueError(
                f"{cls.__name__}: unknown key {key}"
            )

        return cls._registry[key]

    @classmethod
    def available(cls):
        return list(cls._registry.keys())

    @classmethod
    def clear(cls):
        cls._registry.clear()