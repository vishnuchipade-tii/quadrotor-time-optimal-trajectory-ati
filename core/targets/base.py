from abc import ABC, abstractmethod
from typing import ClassVar

from core.type import TargetType

class BaseTarget(ABC):

    target_type: ClassVar[TargetType]

    @abstractmethod
    def is_reached(self, state) -> bool:
        pass